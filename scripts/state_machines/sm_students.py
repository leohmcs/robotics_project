#!/usr/bin/env python3

import numpy as np
from numpy import linalg as LA

import rospy
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty, SetBool, SetBoolRequest  
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from robotics_project.srv import MoveHead, MoveHeadRequest, MoveHeadResponse
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from sensor_msgs.msg import JointState

from actionlib import SimpleActionClient
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry

from tf.transformations import euler_from_quaternion

from moveit_msgs.msg import MoveItErrorCodes
moveit_error_dict = {}
for name in MoveItErrorCodes.__dict__.keys():
    if not name[:1] == '_':
        code = MoveItErrorCodes.__dict__[name]
        moveit_error_dict[code] = name

class StateMachine(object):
    def __init__(self):
        
        self.node_name = "Student SM"
        self.states_dict = {0: 'pick object', 1: 'go to place pose', 2: 'place object'}

        self.robot_pose = None
        self.robot_vel = None

        self.pick_pose = None
        self.place_pose = None

        # Access rosparams
        self.cmd_vel_top = rospy.get_param(rospy.get_name() + '/cmd_vel_topic')
        self.mv_head_srv_nm = rospy.get_param(rospy.get_name() + '/move_head_srv')
        self.pick_srv_name = rospy.get_param(rospy.get_name() + '/pick_srv')
        self.place_srv_name = rospy.get_param(rospy.get_name() + '/place_srv')

        # Subscribe to topics
        self.odom_top = '/ground_truth_odom'
        self.pick_pose_top = rospy.get_param(rospy.get_name() + '/pick_pose_topic')
        self.place_pose_top = rospy.get_param(rospy.get_name() + '/place_pose_topic')

        self.odom_sub = rospy.Subscriber(self.odom_top, Odometry, self.odom_cb)
        self.pick_pose_sub = rospy.Subscriber(self.pick_pose_top, PoseStamped, callback=self.pick_pose_cb)
        self.place_pose_sub = rospy.Subscriber(self.place_pose_top, PoseStamped, callback=self.place_pose_cb)

        # Wait for service providers
        rospy.wait_for_service(self.mv_head_srv_nm, timeout=30)
        self.move_head_srv = rospy.ServiceProxy(self.mv_head_srv_nm, MoveHead)
        rospy.wait_for_service(self.pick_srv_name, timeout=30)
        self.pick_srv = rospy.ServiceProxy(self.pick_srv_name, SetBool)
        rospy.wait_for_service(self.place_srv_name, timeout=30)
        self.place_srv = rospy.ServiceProxy(self.place_srv_name, SetBool)

        # Instantiate publishers
        self.cmd_vel_pub = rospy.Publisher(self.cmd_vel_top, Twist, queue_size=10)

        # Set up action clients
        rospy.loginfo("%s: Waiting for play_motion action server...", self.node_name)
        self.play_motion_ac = SimpleActionClient("/play_motion", PlayMotionAction)
        if not self.play_motion_ac.wait_for_server(rospy.Duration(1000)):
            rospy.logerr("%s: Could not connect to /play_motion action server", self.node_name)
            exit()
        rospy.loginfo("%s: Connected to play_motion action server", self.node_name)

        rospy.loginfo("%s: Waiting for move_base action server...", self.node_name)
        self.move_base_ac = SimpleActionClient("/move_base", MoveBaseAction)
        if not self.move_base_ac.wait_for_server(rospy.Duration(1000)):
            rospy.logerr("%s: Could not connect to /move_base action server", self.node_name)
            exit()
        rospy.loginfo("%s: Connected to move_base action server", self.node_name)

        # Init state machine
        self.init_state = 1
        self.state = self.init_state
        rospy.sleep(3)
        self.check_states()

    def move_base_cb(self):
        pass

    def odom_cb(self, msg: Odometry):
        self.robot_pose = msg.pose.pose
        self.robot_vel = msg.twist.twist

    def pick_pose_cb(self, msg: PoseStamped):
        self.pick_pose = msg
    
    def place_pose_cb(self, msg: PoseStamped):
        self.place_pose = msg

    # TODO
    def turn_and_go(self, goal: PoseStamped):
        if self.robot_pose is None or self.robot_vel is None:
            rospy.loginfo('Still waiting for odometry. Will not move.')
            return False
        
        rate = rospy.Rate(10)

        # turn
        robot_euler = euler_from_quaternion([self.robot_pose.orientation.w, self.robot_pose.orientation.x, self.robot_pose.orientation.y, self.robot_pose.orientation.z])
        goal_euler = euler_from_quaternion([goal.pose.orientation.w, goal.pose.orientation.x, goal.pose.orientation.y, goal.pose.orientation.z])

        rospy.loginfo('Turning')
        while robot_euler[2] - goal_euler[2] > 0.1:
            move_msg = Twist()
            move_msg.angular.z = 0.1
            self.cmd_vel_pub.publish(move_msg)
            rate.sleep()

        # go
        robot_pos = np.array([self.robot_pose.position.x, self.robot_pose.position.y, self.robot_pose.position.z])
        goal_pos = np.array([goal.pose.position.x, goal.pose.position.y, goal.pose.position.z])
        rospy.loginfo(f'Going from {robot_pos} to {goal_pos}')
        while np.linalg.norm(robot_pos - goal_pos) > 0.3:
            rospy.loginfo('Position error = %s', str(np.linalg.norm(robot_pos - goal_pos)))
            move_msg = Twist()
            move_msg.linear.x = 0.5
            self.cmd_vel_pub.publish(move_msg)
            rate.sleep()

        return True
    
        # Example
        # move_msg = Twist()
        # move_msg.linear.x = 1

        # rate = rospy.Rate(10)
        # converged = False
        # cnt = 0
        # rospy.loginfo("%s: Moving towards door", self.node_name)
        # while not rospy.is_shutdown() and cnt < 2:
        #     self.cmd_vel_pub.publish(move_msg)
        #     rate.sleep()
        #     cnt = cnt + 1

        # self.state = 1
        # rospy.sleep(1)

    def go_to_next_state(self):
        self.state += 1

    def check_states(self):
        while not rospy.is_shutdown() and self.state != 4:
            # State 0: go to pick pose
            if self.state == 0:
                goal = MoveBaseGoal()
                goal.target_pose = self.pick_pose
                self.move_base_ac.send_goal(goal)
                self.move_base_ac.wait_for_result()
                self.go_to_next_state()
                rospy.sleep(1)

            # State 1: pick object
            if self.state == 1:
                # move head down to detect aruco
                result = self.move_head_srv('down')
                # call pick_srv service (type SetBool)
                result = self.pick_srv()
                while result.success != True:
                    result = self.pick_srv()

                self.go_to_next_state()
                rospy.sleep(1)
                
            # State 2: go to place pose
            if self.state == 2:
                goal = MoveBaseGoal()
                goal.target_pose = self.place_pose
                self.move_base_ac.send_goal(goal)
                self.move_base_ac.wait_for_result()
                self.go_to_next_state()
                rospy.sleep(1)

            # State 3: place object
            if self.state == 3:
                result = self.place_srv()
                while result.success != True:
                    result = self.place_srv()

                self.go_to_next_state()
                rospy.sleep(1)

            # Error handling
            if self.state == 5:
                rospy.logerr("%s: State machine failed. Check your code and try again!", self.node_name)
                return

        rospy.loginfo("%s: State machine finished!", self.node_name)
        return


# import py_trees as pt, py_trees_ros as ptr

# class BehaviourTree(ptr.trees.BehaviourTree):

# 	def __init__(self):

# 		rospy.loginfo("Initialising behaviour tree")

# 		# go to door until at door
# 		b0 = pt.composites.Selector(
# 			name="Go to door fallback", 
# 			children=[Counter(30, "At door?"), Go("Go to door!", 1, 0)]
# 		)

# 		# tuck the arm
# 		b1 = TuckArm()

# 		# go to table
# 		b2 = pt.composites.Selector(
# 			name="Go to table fallback",
# 			children=[Counter(5, "At table?"), Go("Go to table!", 0, -1)]
# 		)

# 		# move to chair
# 		b3 = pt.composites.Selector(
# 			name="Go to chair fallback",
# 			children=[Counter(13, "At chair?"), Go("Go to chair!", 1, 0)]
# 		)

# 		# lower head
# 		b4 = LowerHead()

# 		# become the tree
# 		tree = pt.composites.Sequence(name="Main sequence", children=[b0, b1, b2, b3, b4])
# 		super(BehaviourTree, self).__init__(tree)

# 		# execute the behaviour tree
# 		self.setup(timeout=10000)
# 		while not rospy.is_shutdown(): self.tick_tock(1)


# class Counter(pt.behaviour.Behaviour):

# 	def __init__(self, n, name):

# 		# counter
# 		self.i = 0
# 		self.n = n

# 		# become a behaviour
# 		super(Counter, self).__init__(name)

# 	def update(self):

# 		# count until n
# 		while self.i <= self.n:

# 			# increment count
# 			self.i += 1

# 			# return failure :(
# 			return pt.common.Status.FAILURE

# 		# succeed after counter done :)
# 		return pt.common.Status.SUCCESS


# class Go(pt.behaviour.Behaviour):

# 	def __init__(self, name, linear, angular):

# 		# action space
# 		self.cmd_vel_top = rospy.get_param(rospy.get_name() + '/cmd_vel_topic')
# 		self.cmd_vel_pub = rospy.Publisher(self.cmd_vel_top, Twist, queue_size=10)

# 		# command
# 		self.move_msg = Twist()
# 		self.move_msg.linear.x = linear
# 		self.move_msg.angular.z = angular

# 		# become a behaviour
# 		super(Go, self).__init__(name)

# 	def update(self):

# 		# send the message
# 		rate = rospy.Rate(10)
# 		self.cmd_vel_pub.publish(self.move_msg)
# 		rate.sleep()

# 		# tell the tree that you're running
# 		return pt.common.Status.RUNNING


# class TuckArm(pt.behaviour.Behaviour):

# 	def __init__(self):

# 		# Set up action client
# 		self.play_motion_ac = SimpleActionClient("/play_motion", PlayMotionAction)

# 		# personal goal setting
# 		self.goal = PlayMotionGoal()
# 		self.goal.motion_name = 'home'
# 		self.goal.skip_planning = True

# 		# execution checker
# 		self.sent_goal = False
# 		self.finished = False

# 		# become a behaviour
# 		super(TuckArm, self).__init__("Tuck arm!")

# 	def update(self):

# 		# already tucked the arm
# 		if self.finished: 
# 			return pt.common.Status.SUCCESS
		
# 		# command to tuck arm if haven't already
# 		elif not self.sent_goal:

# 			# send the goal
# 			self.play_motion_ac.send_goal(self.goal)
# 			self.sent_goal = True

# 			# tell the tree you're running
# 			return pt.common.Status.RUNNING

# 		# if I was succesful! :)))))))))
# 		elif self.play_motion_ac.get_result():

# 			# than I'm finished!
# 			self.finished = True
# 			return pt.common.Status.SUCCESS

# 		# if I'm still trying :|
# 		else:
# 			return pt.common.Status.RUNNING
		


# class LowerHead(pt.behaviour.Behaviour):

# 	def __init__(self):

# 		# server
# 		mv_head_srv_nm = rospy.get_param(rospy.get_name() + '/move_head_srv')
# 		self.move_head_srv = rospy.ServiceProxy(mv_head_srv_nm, MoveHead)
# 		rospy.wait_for_service(mv_head_srv_nm, timeout=30)

# 		# execution checker
# 		self.tried = False
# 		self.tucked = False

# 		# become a behaviour
# 		super(LowerHead, self).__init__("Lower head!")

# 	def update(self):

# 		# try to tuck head if haven't already
# 		if not self.tried:

# 			# command
# 			self.move_head_req = self.move_head_srv("down")
# 			self.tried = True

# 			# tell the tree you're running
# 			return pt.common.Status.RUNNING

# 		# react to outcome
# 		else: return pt.common.Status.SUCCESS if self.move_head_req.success else pt.common.Status.FAILURE


	

if __name__ == "__main__":

	rospy.init_node('main_state_machine')
	try:
		#StateMachine()
		StateMachine()
	except rospy.ROSInterruptException:
		pass

	rospy.spin()
