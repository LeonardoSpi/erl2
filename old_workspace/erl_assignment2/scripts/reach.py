#! /usr/bin/env python

from std_srvs.srv import Trigger
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg


def moveit_reach():
	moveit_commander.roscpp_initialize(sys.argv)

	robot = moveit_commander.RobotCommander()
	scene = moveit_commander.PlanningScenerInterface()
	group = moveit_commander.MoveGroupCommander("arm")
	display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory)

	group.set_named_targer("zero")

	plan1 = group.plan()

	rospy.sleep(5)

	moveit_commander.roscpp_shutdown()

def reach_server():
	rospy.init_node('reach_server') # Initialize the node as goal_server
	s = rospy.Service('reach',Trigger, moveit_reach) # The nodes works as a server for the
								 	# service moveit_reach

	print("Reaching object...") # Once the node is started, print to screen
	rospy.spin() # Continue to cycle


if __name__ == '__main__':
    try:
        reach_server() # Execute function goal_server()
    except rospy.ROSInterruptException: # Keep going until keyboard exception (Ctrl+C)
        pass
