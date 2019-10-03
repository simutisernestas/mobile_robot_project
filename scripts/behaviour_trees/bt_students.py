#!/usr/bin/env python

import py_trees as pt, py_trees_ros as ptr, rospy
from behaviours_student import *
from reactive_sequence import RSequence
from std_srvs.srv import Empty, SetBool, SetBoolRequest
from actionlib import SimpleActionClient
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped

class pickcube(pt.behaviour.Behaviour):

    
    def __init__(self):
		rospy.loginfo("Initialising move head behaviour.")
		pick_cube_srv_name = rospy.get_param(rospy.get_name() + '/pick_srv')
		rospy.logerr(pick_cube_srv_name)
		self.pick_cube_srv = rospy.ServiceProxy(pick_cube_srv_name, SetBool)
		rospy.wait_for_service(pick_cube_srv_name, timeout=30)
		rospy.logerr("wait done")
        # execution checker
		self.tried = False
		self.done = False
		# become a behaviour
		super(pickcube, self).__init__("Pick cube")

    def update(self):
		#pick_cube_req = self.pick_cube_srv(False)
        # success if done
		if self.done:
			return pt.common.Status.SUCCESS

        # try if not tried
		elif not self.tried:

            # command
			self.pick_cube_req = self.pick_cube_srv(True)
                    
			rospy.loginfo(self.pick_cube_req)
			self.tried = True

            # tell the tree you're running
			return pt.common.Status.RUNNING

        # if succesfu

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
		rospy.logerr(place_cube_srv_name)
		self.place_cube_srv = rospy.ServiceProxy(place_cube_srv_name, SetBool)
		rospy.wait_for_service(place_cube_srv_name, timeout=30)
		rospy.logerr("wait done")
        # execution checker
		self.tried = False
		self.done = False
		# become a behaviour
		super(placecube, self).__init__("place cube")

    def update(self):
        # success if done
		if self.done:
			return pt.common.Status.SUCCESS

        # try if not tried
		elif not self.tried:

            # command
			self.place_cube_req = self.place_cube_srv(True)
                    
			rospy.loginfo(self.place_cube_req)
			self.tried = True

            # tell the tree you're running
			return pt.common.Status.RUNNING

        # if succesfu

		elif self.place_cube_req.success:
			self.done = True
			return pt.common.Status.SUCCESS

        # if failed
		elif not self.place_cube_req.success:
			return pt.common.Status.FAILURE

        # if still trying
		else:
			return pt.common.Status.RUNNING

class checker(pt.behaviour.Behaviour):

    def __init__(self):
		self.aruco_pose_rcv = False
		self.aruco_pose_top = rospy.get_param(rospy.get_name() + '/aruco_pose_topic')
		def aruco_callback(data):
			self.aruco_pose_rcv = True
		self.aruco_pose_subs = rospy.Subscriber(self.aruco_pose_top, PoseStamped, aruco_callback)
		super(checker, self).__init__("Checker")
    def update(self):
		self.aruco_pose_rcv = False
		def aruco_callback(data):
			self.aruco_pose_rcv = True
		self.aruco_pose_subs = rospy.Subscriber(self.aruco_pose_top, PoseStamped, aruco_callback)
		if self.aruco_pose_rcv == True:
			rospy.logerr("checker success")
			return pt.common.Status.SUCCESS
		else:
			rospy.logerr("checker fail")
			return pt.common.Status.FAILURE


class BehaviourTree(ptr.trees.BehaviourTree):

	def __init__(self):

		rospy.loginfo("Initialising behaviour tree")

	
		# lower head
		b0 = movehead("down")

		#bring arm in home position
		b1 = tuckarm()

		# pick cube 
		b2 = pickcube()

		# walk to other table
		# 1. turn
		b3 = pt.composites.Selector(
			name="Turn around",
			children=[counter(31, "Turned around?"), go("Turn", 0, -0.96)]
		)

		#ack to initial state in front of table 1
		# walk straight
		b4 = pt.composites.Selector(
			name="Walk",
			children=[counter(10, "At table?"), go("Walk", 0.9, 0)]
		)

		# chill a bit 
		b6 = pt.composites.Selector(
			name="Chill",
			children=[counter(50, "Enough chilling?"), go("Chill", 0, 0)]
		)

		# chill a bit 
		b10 = pt.composites.Selector(
			name="Chill2",
			children=[counter(50, "Enough chilling2?"), go("Chill2", 0, 0)]
		)

		#Full sequence to go back
		b7 = pt.composites.Sequence(
			name="Go back",
			children=[b3, b4, b6])
		
		#try to place -> if failed should fallback to going back to table
		b5 = pt.composites.Selector(
			name="Try to place cube",
			children=[placecube(), b7]
		)
		
		#Check if placed -> if failed should fallback to going back to table
		b8 = pt.composites.Selector(
			name="Try to place cube",
			children=[checker(), b7])


		# become the tree
		tree = RSequence(name="Main sequence", children=[b1, b0, b2, b7, b5, b10, b8])
		super(BehaviourTree, self).__init__(tree)

		# execute the behaviour tree
		rospy.sleep(5)
		self.setup(timeout=10000)
		while not rospy.is_shutdown(): self.tick_tock(1)	

if __name__ == "__main__":


	rospy.init_node('main_state_machine')
	try:
		BehaviourTree()
	except rospy.ROSInterruptException:
		pass

	rospy.spin()