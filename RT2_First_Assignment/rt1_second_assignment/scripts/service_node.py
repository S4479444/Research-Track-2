#! /usr/bin/env python3

## @package service_node
#  \file service_node.py
#  \brief This package implements the service node.
#  \author Emanuele Giordano - 4479444@studenti.unige.it
#  \version 1.0
#  \date 29/01/2025
#
#  \details
#
#  This package implements the service node, which prints the sequence number, as well as the number of reached and canceled goals; it also implements the action server subscriber, which increases the last two numbers by one, everytime the robot is commanded to reach for a target position, or a previously send command is canceled.
#

import rospy
import os
import sys
import assignment_2_2022.msg
from std_srvs.srv import Empty,EmptyResponse

# Initializing variables
reached_goals_counter = 0
canceled_goals_counter = 0
sequence = 1

def serv_node(request):
##
#  \brief Service node
#  \param request
#  \return EmptyResponse() object
#
# The service_node class calls the three variables of reached goals, canceled goals and sequence, as global variables, and increases sequence by 1.
	
	global reached_goals_counter, canceled_goals_counter, sequence

	print("----------------------------------------------------------------\n")
	print("- [NODE] Sequence: ", sequence "				-\n")
	print("- [NODE] Number of reached goals: ", reached_goals_counter,"	-\n")
	print("- [NODE] Number of canceled goals: ", canceled_goals_counter,"	-\n")
	print("----------------------------------------------------------------\n")
	
	sequence = sequence + 1
	
	# Return an empty response
	return EmptyResponse()
	
def action_server_sub(x):
##
#  \brief Action Server Subscriber
#  \param x, status of the robot
#  \return None
#
# This is the action_server_subscriber, which does the following: whenever a goal is canceled, increases the corresponding counter; whenever a goal is reached, does the same thing.
#
	if x.status.status == 2:
		global canceled_goals_counter
		canceled_goals_counter += 1
		
	elif x.status.status == 3:
		global reached_goals_counter
		reached_goals_counter += 1
		
if __name__ == "__main__":
	try:
		rospy.loginfo("Service has ben started.")
		
		# Creating service node and subscribe it to the /current_goal/result topic
		rospy.init_node('service_node')
		rospy.loginfo("Service node succesfully created.")
		
		rospy.Subscriber('/reaching_goal/result', assignment_2_2022.msg.PlanningActionResult, action_server_sub)
		
		# Calls custom service by using the serv_node class
		rospy.Service('reach_cancel_ints', Empty, serv_node)
		
		# Waiting command
		rospy.spin()
	
	# Handling exceptions
	except rospy.ROSInterruptException:
		print("- [ERROR] Program aborted before completion. ", file=sys.stderr)
