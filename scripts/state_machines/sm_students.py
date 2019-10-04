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
        self.aruco_pose_top = rospy.get_param(
            rospy.get_name() + '/aruco_pose_topic')

        # Subscribers
        rospy.Subscriber(self.place_pose_top_name,
                         PoseStamped, self.store_place_pose)
        rospy.Subscriber(self.pick_pose_top_name,
                         PoseStamped, self.store_pick_pose)
        rospy.Subscriber('/ground_truth_odom', Odometry, self.store_robot_position)

        self.aruco_pose_rcv = False
        def aruco_callback(data):
            self.aruco_pose_rcv = True

        self.aruco_pose_subs = rospy.Subscriber(
            self.aruco_pose_top, PoseStamped, aruco_callback)

        # Wait for service providers
        rospy.wait_for_service(self.move_head_srv_name, timeout=30)
        rospy.wait_for_service(self.pick_cube_srv_name, timeout=30)
        rospy.wait_for_service(self.place_cube_srv_name, timeout=30)
        rospy.wait_for_service('/global_localization', timeout=30)

        self.localize_srv = rospy.ServiceProxy('/global_localization', Empty)

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

        self.move_to_goal_top = rospy.Publisher(
            "/move_base_simple/goal", PoseStamped, queue_size=10)

        self.move_to_goal_ac = SimpleActionClient("/move_base", MoveBaseAction)
        self.move_to_goal_ac.wait_for_server(rospy.Duration(1000))

        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Dump
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
                # Localize
                self.localize_srv()
                self.localize_srv()
                # Head up for localization
                try:
                    move_head_req = self.move_head_srv("up")

                    if move_head_req.success == True:
                        self.state += 1

                    rospy.sleep(1)
                except rospy.ServiceException, e:
                    print "Service call to move_head failed: %s" % e

            if self.state == 1:
                # Tuck arm
                goal = PlayMotionGoal()
                pick_cube_req = self.pick_cube_srv(True)
                    
                rospy.loginfo(pick_cube_req)
                goal.motion_name = 'home'
                goal.skip_planning = True
                self.play_motion_ac.send_goal(goal)
                rospy.loginfo("%s: SPINNING", self.node_name)
                # Spin fo rlocalization algorithm
                move_msg = Twist()
                move_msg.angular.z = -1
                rate = rospy.Rate(10)
                cnt = 0
                while not rospy.is_shutdown() and cnt < 50:
                    self.cmd_vel_pub.publish(move_msg)
                    rate.sleep()
                    cnt = cnt + 1
                # Tuck arm
                success_tucking = self.play_motion_ac.wait_for_result(
                    rospy.Duration(100.0))
                if success_tucking:
                    rospy.loginfo("%s: Arm tucked.", self.node_name)
                else:
                    self.play_motion_ac.cancel_goal()
                rospy.sleep(1)

                rospy.loginfo("%s: DONE SPINNING", self.node_name)
                """
                # Move to pick pose callback (action client)
                def callback(data):
                    pick_x = self.pick_pose_msg.pose.position.x
                    pick_y = self.pick_pose_msg.pose.position.y
                    curr_x = data.base_position.pose.position.x
                    curr_y = data.base_position.pose.position.y

                    pos_err = np.sqrt((pick_x-curr_x)**2 + (pick_y-curr_y)**2)

                    if pos_err > 0.05:
                        return
                """ 
                def callback(data):
                    pick_x = self.pick_pose_msg.pose.position.x
                    pick_y = self.pick_pose_msg.pose.position.y
                    curr_x = data.base_position.pose.position.x
                    curr_y = data.base_position.pose.position.y

                    pos_err = np.sqrt((pick_x-curr_x)**2 + (pick_y-curr_y)**2)
                    
                    if pos_err > 0.05:
                        return

                    self.move_to_goal_ac.cancel_goal()
                    self.state += 1
                    rospy.sleep(1)
                    return

                # Send goal to move
                move_goal = MoveBaseGoal()
                move_goal.target_pose.pose = self.pick_pose_msg.pose
                move_goal.target_pose.header.frame_id = 'map'
                self.move_to_goal_ac.send_goal(move_goal, feedback_cb=callback)
                success = self.move_to_goal_ac.wait_for_result(rospy.Duration(1000))
                if success:
                    rospy.loginfo("%s: Moved to Table1.", self.node_name)
                else:
                    rospy.loginfo("%s: ancel ,moving to table1", self.node_name)
                    self.move_to_goal_ac.cancel_goal()

            if self.state == 2:
                # Position reached
                rospy.logerr("%s: GOAL REACHED", self.node_name)
                rospy.logerr("%s: STATE 2", self.node_name)

                def get_rotation(msg):
                    orientation_q = msg.pose.orientation
                    orientation_list = [
                        orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w
                    ]
                    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
                    return yaw

                msg = Twist()
                err = np.Inf
                while not rospy.is_shutdown() and err > 0.1:
                    kp = 0.5  # ?

                    pick_yaw = get_rotation(self.pick_pose_msg)
                    robot_yaw = get_rotation(self.robot_position.pose)
                    msg.angular.z = kp * (pick_yaw - robot_yaw)
                    err = abs(pick_yaw - robot_yaw)



                    rospy.loginfo('\n')
                    rospy.loginfo(err)
                    rospy.loginfo(pick_yaw)
                    rospy.loginfo(robot_yaw)
                    rospy.loginfo('\n')

                    self.cmd_vel_pub.publish(msg)
                
                # # Pregrasp position
                # goal = PlayMotionGoal()
                # goal.motion_name = 'pregrasp'
                # goal.skip_planning = True
                # self.play_motion_ac.send_goal(goal)
                # success = self.play_motion_ac.wait_for_result(rospy.Duration(100.0))
                # if success:
                #     rospy.loginfo("%s: Pregrasp position.", self.node_name)
                # else:
                #     self.play_motion_ac.cancel_goal()
                # rospy.sleep(1)

                try:
                    move_head_req = self.move_head_srv("down")

                    if move_head_req.success == True:
                        self.state += 1
                    
                    rospy.sleep(1)
                except rospy.ServiceException, e:
                    rospy.logerr(
                        "%s: service failed to pick cube", self.node_name)
pick_cube_req = self.pick_cube_srv(True)
                    
                    rospy.loginfo(pick_cube_req)
                # move_msg = Twist()
                # move_msg.angular.z = -1
                # rate = rospy.Rate(10)
                # cnt = 0
                # # Spin until robot sees see the cube 
                # while not self.aruco_pose_rcv:
                #     self.cmd_vel_pub.publish(move_msg)
                #     rate.sleep()
                #     cnt = cnt + 1

                rospy.sleep(1)

            # Pick cube
            if self.state == 3:
                rospy.logerr("%s: STATE 3", self.node_name)

                # Pick
                try:
                    pick_cube_req = self.pick_cube_srv(True)
                    
                    rospy.loginfo(pick_cube_req)

                    if pick_cube_req.success == True:
                        self.state += 1
                    
                    rospy.sleep(1)
                except rospy.ServiceException, e:
                    rospy.logerr(
                        "%s: service failed to pick cube", self.node_name)
                
            # Go to other table
            if self.state == 4:
                rospy.logerr("%s: STATE 4", self.node_name)

                try: # Head up
                    move_head_req = self.move_head_srv("up")

                    if move_head_req.success == True:
                        self.state += 1
                    
                    rospy.sleep(1)
                except rospy.ServiceException, e:
                    print "Service call to move_head failed: %s" % e

            if self.state == 5:
                rospy.logerr("%s: STATE 5", self.node_name)

                # Fix localization
                self.localize_srv()
                # Spin a bit
                move_msg = Twist()
                move_msg.angular.z = -1
                rate = rospy.Rate(10)
                cnt = 0
                while cnt < 60:
                    self.cmd_vel_pub.publish(move_msg)
                    rate.sleep()
                    cnt = cnt + 1
                # Stop spin
                move_msg.angular.z = 0
                for _ in range(5):
                    self.cmd_vel_pub.publish(move_msg)
                rospy.sleep(1)

                move_goal = MoveBaseGoal()
                move_goal.target_pose.pose = self.place_pose_msg.pose
                move_goal.target_pose.header.frame_id = 'map'
                self.move_to_goal_ac.send_goal(move_goal)
                success = self.move_to_goal_ac.wait_for_result(rospy.Duration(1000))
                if success:
                    rospy.loginfo("%s: Moved to Table2.", self.node_name)
                else:
                    rospy.loginfo("%s: Cancel moving to table2", self.node_name)
                    self.move_to_goal_ac.cancel_goal()

                # Check if in front of table, only then put it down
                try:
                    place_cube_srv = self.place_cube_srv(False)

                    if place_cube_srv.success == True:
                        self.state += 1

                    rospy.sleep(1)
                except rospy.ServiceException, e:
                    rospy.logerr(
                        "%s: service failed to place cube", self.node_name)

            if self.state == 6:
                rospy.loginfo("%s: SUCCESSSSSS", self.node_name)
                exit()

            # Error handling
            # if self.state == 7:
            #     rospy.logerr(
            #         "%s: State machine failed. Check your code and try again!", self.node_name)
            #     return

        rospy.loginfo("%s: State machine finished!", self.node_name)
        returnstat
stat
    def store_joint_statesstat(self, data):
        '''stat
        arm_1_joint, arm_2stat_joint, arm_3_joint, arm_4_joint, arm_5_joint, arm_6_joint, arm_7_joint,
        gripper_left_fingestatr_joint, gripper_right_finger_joint, head_1_joint, head_2_joint,
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
