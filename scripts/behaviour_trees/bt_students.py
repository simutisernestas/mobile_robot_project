#!/usr/bin/env python

import py_trees as pt
import py_trees_ros as ptr
import rospy
from behaviours_student import *
from reactive_sequence import RSequence
from std_srvs.srv import Empty, SetBool, SetBoolRequest
from actionlib import SimpleActionClient
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from gazebo_msgs.srv import SetModelState


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
#Todo: respawn cube
class respawn_cube(pt.behaviour.Behaviour):

    def __init__(self):
        rospy.logerr("Initialising respawn_cube behaviour.")
        respawn_cube_srv_name = '/gazebo/set_model_state'
        self.respawn_cube_srv = rospy.ServiceProxy(respawn_cube_srv_name, SetModelState)
        rospy.wait_for_service(respawn_cube_srv_name, timeout=30)
        # call /gazebo/set_model_state '{model_state: { model_name: aruco_cube, pose: { position: { x: -1.130530, y: -6.653650, z: 0.86250 }, orientation: {x: 0, y: 0, z: 0, w: 1 } }, twist: { linear: {x: 0 , y: 0, z: 0 } , angular: { x: 0, y: 0, z: 0 } } , reference_frame: map } }'
        point = Point()
        point.x = -1.13
        point.x
        self.respawn_cube_srv('aruco_cube', 
            -1.130530, -6.653650, 0.86250, 
            0, 0, 0, 1, 
            0 ,  0,  0,  0,  0, 0, 
            map )
        # become a behaviour
        super(respawn_cube, self).__init__("respawn_cube")
        

    def update(self):
        # success if done
        rospy.logerr("success")
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
        self.localize_srv()
        super(localize, self).__init__("localize")
   
    def update(self):
        return pt.common.Status.SUCCESS

def build_change_table(name):

    timeout1 = pt.composites.Selector(
		name="Chill",
		children=[counter(30, "Enough chill"), go("Turn", 0, 0)])

    timeout2 = pt.composites.Selector(
		name="Chill",
		children=[counter(30, "Enough chill"), go("Turn", 0, 0)])

    turn_around = pt.composites.Selector(
		name="Turn around",
		children=[counter(58, "Turned around?"), go("Turn", 0, -0.5)]
	)

    go_straight = pt.composites.Selector(
		name="Walk",
		children=[counter(20, "At table?"), go("Walk", 0.4, 0)]
	)

    return RSequence(name=name, children=[turn_around, timeout1, go_straight, timeout2])


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

        timeout3 = pt.composites.Selector(
		name="Chill",
		children=[counter(30, "Enough chill"), go("Turn", 0, 0)])
        
        # confirm_placed_cube = pt.composites.Chooser(
        #     name="Confirm cube is on table",
        #     children=[is_cube_placed(), move_to_goal]
        # )

        twist1 = pt.composites.Selector(
		    name="twist",
		    children=[counter(60, "Enough twist"), go("twist", 0, 1)]
	    )

        twist2 = pt.composites.Selector(
		    name="twist",
		    children=[counter(60, "Enough twist"), go("twist", 0, 1)]
	    )

        prim_sequence = pt.composites.Sequence(
            name="first_sequence",
		    children=[
                movehead("up"),
                localize(),
                tuckarm(),
                twist1,
                move_to_goal(self.pick_pose_msg),
                movehead("down"),
                pickcube(), 
                movehead("up"),
                move_to_goal(self.place_pose_msg),
                placecube(),
                movehead("down"),
                timeout3,]
            )

        second_sequence = pt.composites.Sequence(
            name="first_sequence",
		    children=[
                movehead("up"),
                tuckarm(),
                move_to_goal(self.pick_pose_msg),
                movehead("down"),
                pickcube(), 
                movehead("up"),
                move_to_goal(self.place_pose_msg),
                placecube(),
                movehead("down"),
                timeout3
                ]
            )
        # fallback_move_to_goal
        fallback = pt.composites.Chooser(
            name="fallback",
		    children=[is_cube_placed(), second_sequence]
        )
        tree = RSequence(name="Main sequence", children=[
            respawn_cube,
            prim_sequence,
            fallback
            ])

        super(BehaviourTree, self).__init__(tree)

        # execute BT
        rospy.sleep(1)
        self.setup(timeout=10000)
        while not rospy.is_shutdown():
            rospy.logerr('tick')
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
