#!/usr/bin/env python

import py_trees as pt, py_trees_ros as ptr, rospy
from behaviours_student import *
from reactive_sequence import RSequence
from std_srvs.srv import Empty, SetBool, SetBoolRequest
from actionlib import SimpleActionClient
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
"""
class home(pt.behaviour.Behaviour):

    
    Sends a goal to the home action server.
    Returns running whilst awaiting the result,
    success if the action was succesful, and v.v..


    def __init__(self):
		rospy.loginfo("Initialising home behaviour.")
		rospy.logerr("home")
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
		# super(home, self).__init__("Home")

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
            return pt.common.Status.SUCCESS

        # if failed
        elif not self.play_motion_ac.get_result():
            return pt.common.Status.FAILURE
        # try if not triedmofrom actionlib import SimpleActionClient
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoaln.Status.FAILURE

        # if I'm still trying :|
        else:
            return pt.common.Status.RUNNING
"""
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
		if self.aruco_pose_rcv == True:
			return pt.common.Status.SUCCESS
		else:
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
			children=[counter(20, "Enough chilling?"), go("Chill", 0, 0)]
		)

		b7 = pt.composites.Sequence(
			name="Go back",
			children=[b3, b4])
		
		

		#tryz to place
		b5 = pt.composites.Selector(
			name="Try to place cube",
			children=[placecube(), b7])

		b8 = pt.composites.Selector(
			name="Try to place cube",
			children=[checker(), b7])

		
		
		"""
		# go lower 
		b0 = pt.composites.Selector(
			name="Go to door fallback", 
			children=[counter(30, "At door?"), go("Go to door!", 1, 0)]
		)

		# tuck the arm
		b1 = tuckarm()

		# go to table
		b2 = pt.composites.Selector(
			name="Go to table fallback",
			children=[counter(5, "At table?"), go("Go to table!", 0, -1)]
		)

		# move to chair
		b3 = pt.composites.Selector(
			name="Go to chair fallback",
			children=[counter(13, "At chair?"), go("Go to chair!", 1, 0)]
		)

		"""
		# become the tree
		tree = RSequence(name="Main sequence", children=[b1, b0, b2, b3, b4, b6, b5, b6, b8])
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