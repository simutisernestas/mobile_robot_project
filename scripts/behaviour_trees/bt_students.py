#!/usr/bin/env python

import py_trees as pt
import py_trees_ros as ptr
import rospy
from behaviours_student import *
from reactive_sequence import RSequence
from std_srvs.srv import Empty, SetBool, SetBoolRequest
from actionlib import SimpleActionClient
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Pose
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from gazebo_msgs.srv import SetModelState, SpawnModel
from gazebo_msgs.msg import ModelState
from nav_msgs.msg import Odometry


class pickcube(pt.behaviour.Behaviour):

    def __init__(self):
        rospy.loginfo("Initialising move head behaviour.")
        pick_cube_srv_name = rospy.get_param(rospy.get_name() + '/pick_srv')
        self.pick_cube_srv = rospy.ServiceProxy(pick_cube_srv_name, SetBool)
        rospy.wait_for_service(pick_cube_srv_name, timeout=30)
        # execution checker
        self.tried = False
        self.done = False
        # become a behaviour
        super(pickcube, self).__init__("pickcube")

    def update(self):
        # success if done
        if self.done:
            return pt.common.Status.SUCCESS

            # try if not tried
        elif not self.tried:
            # command
            self.pick_cube_req = self.pick_cube_srv(True)

            self.tried = True

            # tell the tree you're running
            return pt.common.Status.RUNNING

            # if succesful
        elif self.pick_cube_req.success:
            self.done = True
            return pt.common.Status.SUCCESS

            # if failed
        elif not self.pick_cube_req.success:
            return pt.common.Status.FAILURE

            # if still trying
        else:
            return pt.common.Status.RUNNING


class placecube(pt.behaviour.Behaviour):

    def __init__(self):
        rospy.loginfo("Initialising move head behaviour.")
        place_cube_srv_name = rospy.get_param(rospy.get_name() + '/place_srv')
        self.place_cube_srv = rospy.ServiceProxy(place_cube_srv_name, SetBool)
        rospy.wait_for_service(place_cube_srv_name, timeout=30)
        # execution checker
        self.tried = False
        self.done = False
        # become a behaviour
        super(placecube, self).__init__("placecube")

    def update(self):
        # success if done
        if self.done:
            return pt.common.Status.SUCCESS

            # try if not tried
        elif not self.tried:
            # command
            self.place_cube_req = self.place_cube_srv(True)

            self.tried = True
            return pt.common.Status.RUNNING

            # if succesful
        elif self.place_cube_req.success:
            self.done = True
            return pt.common.Status.SUCCESS

            # if failed
        elif not self.place_cube_req.success:
            return pt.common.Status.FAILURE

            # if still trying
        else:
            return pt.common.Status.RUNNING


class respawn_cube(pt.behaviour.Behaviour):

    def __init__(self):
        
        super(respawn_cube, self).__init__("respawn_cube")

    def update(self):
        rospy.logerr("Initialising respawn_cube behaviour.")
        respawn_cube_srv_name = '/gazebo/set_model_state'
        self.respawn_cube_srv = rospy.ServiceProxy(
            '/gazebo/set_model_state', SetModelState)
        rospy.wait_for_service(respawn_cube_srv_name, timeout=30)
        data = {'model_name': 'aruco_cube', 'pose': {'position': {'x': -1.130530, 'y': -6.653650, 'z': 1.86250}, 'orientation': {'x': 0, 'y': 0, 'z': 0, 'w': 1}}, 'twist': {'linear': {'x': 0, 'y': 0, 'z': 0}, 'angular': {'x': 0, 'y': 0, 'z': 0}}, 'reference_frame': 'map'}
        msg = ModelState()
        msg.model_name = data['model_name']
        pose = Pose()
        pose.position.x = data['pose']['position']['x']
        pose.position.y = data['pose']['position']['y']
        pose.position.z = data['pose']['position']['z']
        msg.pose = pose
        self.respawn_cube_srv(msg)
        return pt.common.Status.SUCCESS


class is_cube_placed(pt.behaviour.Behaviour):

    def __init__(self):
        self.aruco_pose_top = rospy.get_param(
            rospy.get_name() + '/aruco_pose_topic')

        def aruco_callback(_):
            self.aruco_pose_rcv = True

        self.aruco_pose_subs = rospy.Subscriber(
            self.aruco_pose_top, PoseStamped, aruco_callback)

        super(is_cube_placed, self).__init__("is_cube_placed")

    def update(self):
        self.aruco_pose_rcv = False

        rospy.sleep(5)

        if self.aruco_pose_rcv == True:
            rospy.loginfo("Cube placed return success")
            exit()
            return pt.common.Status.SUCCESS
        else:
            rospy.logerr("Cube missing return fail")
            return pt.common.Status.FAILURE


class localize(pt.behaviour.Behaviour):

    def __init__(self):
        self.localize_srv = rospy.ServiceProxy('/global_localization', Empty)
        rospy.wait_for_service('/global_localization', timeout=30)
        super(localize, self).__init__("localize")

    def update(self):
        self.localize_srv()
        return pt.common.Status.SUCCESS


class move_to_goal(pt.behaviour.Behaviour):

    def __init__(self, goal):
        self.move_to_goal_ac = SimpleActionClient("/move_base", MoveBaseAction)
        self.move_to_goal_ac.wait_for_server(rospy.Duration(1000))
        self.move_goal = MoveBaseGoal()
        self.move_goal.target_pose.pose = goal.pose
        self.move_goal.target_pose.header.frame_id = 'map'
        super(move_to_goal, self).__init__("move_to_goal")

    def update(self):
        self.move_to_goal_ac.send_goal(self.move_goal)
        success = self.move_to_goal_ac.wait_for_result(rospy.Duration(1000))
        if success:
            return pt.common.Status.SUCCESS
        else:
            self.move_to_goal_ac.cancel_goal()
            return pt.common.Status.FAILURE

class fail(pt.behaviour.Behaviour):

    def __init__(self):
        super(fail, self).__init__("fail")

    def update(self):
        return pt.common.Status.FAILURE

class prints(pt.behaviour.Behaviour):

    def __init__(self):
        super(prints, self).__init__("prints")

    def update(self):
        rospy.logerr("printsomething")
        return pt.common.Status.SUCCESS

class tuckarm2(pt.behaviour.Behaviour):

    """
    Sends a goal to the tuck arm action server.
    Returns running whilst awaiting the result,
    success if the action was succesful, and v.v..
    """

    def __init__(self):

        rospy.loginfo("Initialising tuck arm behaviour.")

        # Set up action client
        self.play_motion_ac = SimpleActionClient("/play_motion", PlayMotionAction)

        # personal goal setting
        self.goal = PlayMotionGoal()
        self.goal.motion_name = 'home'
        self.goal.skip_planning = True

        # execution checker
        self.sent_goal = False
        self.finished = False

        # become a behaviour
        super(tuckarm2, self).__init__("Tuck arm!")

    def update(self):
        # already tucked the arm
        if self.finished: 
            return pt.common.Status.SUCCESS
        
        # command to tuck arm if haven't already
        elif not self.sent_goal:

            # send the goal
            self.play_motion_ac.send_goal(self.goal)
            self.sent_goal = True

            # tell the tree you're running
            return pt.common.Status.RUNNING

        # if I was succesful! :)))))))))
        elif self.play_motion_ac.get_result():

            # than I'm finished!
            self.finished = True
            rospy.logerr("Arm tucked")
            rospy.logerr("Success tucking")
            return pt.common.Status.SUCCESS

        # if failed
        elif not self.play_motion_ac.get_result():
            rospy.logerr("fail tucking")
            return pt.common.Status.FAILURE
        
        # try if not triedmon.Status.FAILURE

        # if I'm still trying :|
        else:
            return pt.common.Status.RUNNING



class relocalize(pt.behaviour.Behaviour):

    def __init__(self):
        # TODO consider THIS
        # seems like ground_truth_odom works better for odometry, but it could be that
        # we still need to transform frames , because it is different fomr acml pose
         
        # rospy.Subscriber('/mobile_base_controller/odom', Odometry, self.store_robot_position)
        rospy.Subscriber('/ground_truth_odom', Odometry, self.store_robot_position) 
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.store_amcl_pose)
        
        self.localize_srv = rospy.ServiceProxy('/global_localization', Empty)
        rospy.wait_for_service('/global_localization', timeout=30)
        self.localize_called = False
        
        super(relocalize, self).__init__("relocalize")

    def update(self):
        e_x = abs(self.robot_position.position.x - self.acml_pose.position.x)
        e_y = abs(self.robot_position.position.y - self.acml_pose.position.y)
        e_z = abs(self.robot_position.position.z - self.acml_pose.position.z)
        e_o_x = abs(self.robot_position.orientation.x - self.acml_pose.orientation.x)
        e_o_y = abs(self.robot_position.orientation.y - self.acml_pose.orientation.y)
        e_o_z = abs(self.robot_position.orientation.z - self.acml_pose.orientation.z)
        e_o_w = abs(self.robot_position.orientation.w - self.acml_pose.orientation.w)
        
        total_error = e_x + e_y + e_z + e_o_x + e_o_y + e_o_z + e_o_w
        # rospy.loginfo(total_error)
        
        # For now this is quite big, maybe because of reason mentioned above 
        treshold = 3.0

        if total_error > treshold:
            if not self.localize_called:
                self.localize_srv()
            self.localize_called = True
            return pt.common.Status.RUNNING
        
        if total_error < treshold:
            return pt.common.Status.SUCCESS

    def store_robot_position(self, msg):
        self.robot_position = msg.pose.pose

    def store_amcl_pose(self, msg):
        self.acml_pose = msg.pose.pose


class BehaviourTree(ptr.trees.BehaviourTree):

    def __init__(self):
        self.pick_cube_srv_name = rospy.get_param(
            rospy.get_name() + '/pick_srv')
        self.place_cube_srv_name = rospy.get_param(
            rospy.get_name() + '/place_srv')

        self.place_pose_top_name = rospy.get_param(
            rospy.get_name() + '/place_pose_topic')
        self.pick_pose_top_name = rospy.get_param(
            rospy.get_name() + '/pick_pose_topic')

        rospy.Subscriber(self.place_pose_top_name,
                         PoseStamped, self.store_place_pose)
        rospy.Subscriber(self.pick_pose_top_name,
                         PoseStamped, self.store_pick_pose)

        rospy.wait_for_service(self.pick_cube_srv_name, timeout=300)
        rospy.wait_for_service(self.place_cube_srv_name, timeout=300)

        # TODO (Selector + counter) with each behaviuor
        # because counter terminates the action I guess
        # ---------------------------------------------
        # b0 = pt.composites.Selector(
        #     name="Go to door fallback",
        #     children=[counter(30, "At door?"), go("Go to door!", 1, 0)]
        # )

        spin = pt.composites.Selector(
            name="twist",
            children=[counter(60, "Enough spin"), go("spin", 0, 1)]
        )
        
        localization = pt.composites.Selector(
            name="twist",
            children=[counter(60, "Localized?"), localize()]
        )
        
        # TODO Where should correction go ???
        pose_correction = pt.composites.Selector(
            name="twist",
            children=[counter(60, "Corrected?"), relocalize(), go("spin", 0, 1)]
        )
        
    
        # cube_spawn = pt.composites.Selector(
        #     name="respawn_cube",
        #     children=[counter(60, "Cube spawned?"), respawn_cube()]
        # ) 
        
        home_position = tuckarm()

        primary_sequence = pt.composites.Sequence(
            name="first_sequence",
            children=[
                movehead("up"),
                home_position,
                localization,
                spin,
                move_to_goal(self.pick_pose_msg),
                movehead("down"),
                pickcube(),
                movehead("up"),
                move_to_goal(self.place_pose_msg),
                placecube(),
                movehead("down")
            ]
        )

        # TODO Think aobut second sequence, 
        #      maybe some actions redundant  
        second_sequence = pt.composites.Sequence(
            name="second_sequence",
            children=[
                prints(),
                respawn_cube(),
                movehead("up"),
                tuckarm2(),
                move_to_goal(self.pick_pose_msg),
                movehead("down"),
                pickcube(),
                movehead("up"),
                move_to_goal(self.place_pose_msg),
                placecube(),
                movehead("down")
            ]
        )

        third_sequence = pt.composites.Sequence(
            name="second_sequence",
            children=[
                movehead("up"),
                home_position,
                move_to_goal(self.pick_pose_msg),
                movehead("down"),
                pickcube(),
                movehead("up"),
                move_to_goal(self.place_pose_msg),
                placecube(),
                movehead("down"),
                fail()
            ]
        )

        # fallback_move_to_goal
        fallback = pt.composites.Chooser(
            name="fallback",
            children=[
                primary_sequence,
                is_cube_placed(),
                second_sequence
            ]
        )
        fallback2 = pt.composites.Chooser(
            name="fallback2",
            children=[
                is_cube_placed(),
                third_sequence
            ]
        )

        tree = RSequence(
            name="Main sequence",
            children=[
                fallback,
                fallback2,
                movehead('up'),
                movehead('down'),
                movehead('up')
            ]
        )

        super(BehaviourTree, self).__init__(tree)

        # execute BT
        rospy.sleep(1)
        self.setup(timeout=100)
        while not rospy.is_shutdown():
            self.tick_tock(1)

    def store_place_pose(self, data):
        self.place_pose_msg = data

    def store_pick_pose(self, data):
        self.pick_pose_msg = data


if __name__ == "__main__":

    rospy.init_node('main_state_machine')
    try:
        BehaviourTree()
    except rospy.ROSInterruptException:
        pass

    rospy.spin()
