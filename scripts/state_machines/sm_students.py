#!/usr/bin/env python

from tf.transformations import euler_from_quaternion, quaternion_from_euler
import tf2_ros
import tf2_geometry_msgs
import tf
from gazebo_msgs.srv import GetModelState
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

from moveit_msgs.msg import MoveItErrorCodes
moveit_error_dict = {}
for name in MoveItErrorCodes.__dict__.keys():
    if not name[:1] == '_':
        code = MoveItErrorCodes.__dict__[name]
        moveit_error_dict[code] = name


class StateMachine(object):
    def __init__(self):

        self.node_name = "Student SM"

        # Access rosparams
        self.cmd_vel_top = rospy.get_param(
            rospy.get_name() + '/cmd_vel_topic')
        self.move_head_srv_name = rospy.get_param(
            rospy.get_name() + '/move_head_srv')
        self.pick_cube_srv_name = rospy.get_param(
            rospy.get_name() + '/pick_srv')
        self.place_cube_srv_name = rospy.get_param(
            rospy.get_name() + '/place_srv')
        self.place_pose_top_name = rospy.get_param(
            rospy.get_name() + '/place_pose_topic')
        self.pick_pose_top_name = rospy.get_param(
            rospy.get_name() + '/pick_pose_topic')
        # self.move_base_top_name = rospy.get_param(
        #     rospy.get_name() + '/move_base_feedback')

        # Subscribers
        # rospy.Subscriber("/joint_states", JointState, self.store_joint_states)
        rospy.Subscriber(self.place_pose_top_name,
                         PoseStamped, self.store_place_pose)
        rospy.Subscriber(self.pick_pose_top_name,
                         PoseStamped, self.store_pick_pose)
        # rospy.Subscriber(self.move_base_top_name,
        #                  Odometry, self.store_robot_position)

        # Wait for service providers
        rospy.wait_for_service(self.move_head_srv_name, timeout=30)
        rospy.wait_for_service(self.pick_cube_srv_name, timeout=30)
        rospy.wait_for_service(self.place_cube_srv_name, timeout=30)
        rospy.wait_for_service('/global_localization', timeout=30)

        loc = rospy.ServiceProxy('/global_localization', Empty)
        loc()
        loc()
        loc()

        # Service definitions
        self.move_head_srv = rospy.ServiceProxy(
            self.move_head_srv_name, MoveHead)
        self.pick_cube_srv = rospy.ServiceProxy(
            self.pick_cube_srv_name, SetBool)
        self.place_cube_srv = rospy.ServiceProxy(
            self.place_cube_srv_name, SetBool)

        # Instantiate publishers
        self.cmd_vel_pub = rospy.Publisher(
            self.cmd_vel_top, Twist, queue_size=10)

        # Set up action clients
        rospy.loginfo(
            "%s: Waiting for play_motion action server...", self.node_name)
        self.play_motion_ac = SimpleActionClient(
            "/play_motion", PlayMotionAction)
        if not self.play_motion_ac.wait_for_server(rospy.Duration(1000)):
            rospy.logerr(
                "%s: Could not connect to /play_motion action server", self.node_name)
            exit()
        rospy.loginfo(
            "%s: Connected to play_motion action server", self.node_name)

        self.move_to_goal_ac = rospy.Publisher("/move_base_simple/goal", PoseStamped)

        # Dump
        # self.tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0))
        # self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        # cp = rospy.get_param(rospy.get_name() + "/cube_pose")
        # pp = rospy.get_param(rospy.get_name() + "/pick_pose_topic")
        # rospy.wait_for_service('/gazebo/get_model_state')
        # self.cube_state_service = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

        # Init state machine
        self.state = 0
        rospy.sleep(3)
        self.check_states()

    def check_states(self):
        while not rospy.is_shutdown():

            # Lower robot head
            if self.state == 0:
                try:
                    move_head_req = self.move_head_srv("down")

                    if move_head_req.success == True:
                        self.state += 1

                    rospy.sleep(1)
                except rospy.ServiceException, e:
                    print "Service call to move_head failed: %s" % e

            if self.state == 1:
                goal = PlayMotionGoal()
                goal.motion_name = 'home'
                goal.skip_planning = True
                self.play_motion_ac.send_goal(goal)
                success_tucking = self.play_motion_ac.wait_for_result(
                    rospy.Duration(100.0))
                if success_tucking:
                    rospy.loginfo("%s: Arm tucked.", self.node_name)
                else:
                    self.play_motion_ac.cancel_goal()
                rospy.sleep(1)

                rospy.loginfo("%s: SPINNING", self.node_name)

                move_msg = Twist()
                move_msg.angular.z = -1

                rate = rospy.Rate(10)
                cnt = 0
                while not rospy.is_shutdown() and cnt < 60:
                    self.cmd_vel_pub.publish(move_msg)
                    rate.sleep()
                    cnt = cnt + 1

                self.state += 1
                rospy.sleep(1)

                rospy.loginfo("%s: DONE SPINNING", self.node_name)

                self.move_to_goal_ac.publish(self.pick_pose_msg)
                self.state += 1
                rospy.sleep(1)

            # Pick cube
            if self.state == 2:
                # Check if in front of table
                # Only then pick up
                try:
                    pick_cube_req = self.pick_cube_srv(False)

                    if pick_cube_req.success == True:
                        self.state += 1

                    rospy.sleep(1)
                except rospy.ServiceException, e:
                    rospy.logerr(
                        "%s: service failed to pick cube", self.node_name)

            if self.state == 3:
                self.move_to_goal_ac.publish(self.place_pose_msg)
                self.state += 1

            if self.state == 4:
                # Check if in front of table
                # Only then put it down
                try:
                    place_cube_srv = self.place_cube_srv(False)

                    if place_cube_srv.success == True:
                        self.state += 1

                    rospy.sleep(1)
                except rospy.ServiceException, e:
                    rospy.logerr(
                        "%s: service failed to place cube", self.node_name)

            if self.state == 5:
                rospy.loginfo("%s: SUCCESSSSSS", self.node_name)
                exit()

            # Error handling
            # if self.state == 7:
            #     rospy.logerr(
            #         "%s: State machine failed. Check your code and try again!", self.node_name)
            #     return

        rospy.loginfo("%s: State machine finished!", self.node_name)
        return

    def store_joint_states(self, data):
        '''
        arm_1_joint, arm_2_joint, arm_3_joint, arm_4_joint, arm_5_joint, arm_6_joint, arm_7_joint,
        gripper_left_finger_joint, gripper_right_finger_joint, head_1_joint, head_2_joint,
        torso_lift_joint, wheel_left_joint, wheel_right_joint, caster_back_left_1_joint,
        caster_back_left_2_joint, caster_front_left_1_joint, caster_front_left_2_joint,
        caster_back_right_1_joint, caster_back_right_2_joint, caster_front_right_1_joint,
        caster_front_right_2_joint, suspension_left_joint, suspension_right_joint
        '''
        self.joint_names = data.name
        self.joint_positions = data.position

    def store_place_pose(self, data):
        self.place_pose_msg = data

    def store_pick_pose(self, data):
        self.pick_pose_msg = data

    def store_robot_position(self, msg):
        self.robot_position = msg


if __name__ == "__main__":
    rospy.init_node('main_state_machine')
    try:
        StateMachine()
    except rospy.ROSInterruptException:
        pass

    rospy.spin()
