#! /usr/bin/env python3

## @package action_client
#  \file action_client.py
#  \brief This node implements the robot's action client.
#  \author Emanuele Giordano - 4479444@studenti.unige.it
#  \version 1.0
#  \date 29/01/2025
#
#  \details
#
#  This node implements both the graphical user interface (GUI) and the action client node of the robot. In particular, the action client handles the behavour of the robot, based on the user's input, between choosing a specified goal position (x,y), or canceling any previous command and stopping the robot.
#


import rospy
import os
import actionlib
import actionlib.msg
import assignment_2_2022.msg
from std_srvs.srv import *
from geometry_msgs.msg import PoseStamped

def GUI():
## 
#  \brief This is the Graphical User Interface, called at any moment in order to have real-time control.
#  \param None
#  \return None
#

	os.system('clear')
	print("************************************************************************\n")
	print("-		   		GUI			 		-\n")
	print("-					      				-\n")
	print("- Usage: press a digit key and then enter to confirm your choice.    	-\n")
	print("-									-\n")
	print("------------------------------------------------------------------------\n")
	print("-    (1) - Input a desired position for the robot;			-\n")
	print("-    (2) - Cancel the current command;				      	-\n")
	print("-									-\n")
	usr_input = input("- --> ")
	
	if usr_input == "1":
		choose_goal()
	
	elif usr_input == "2":
		cancel_cmd()
	
	else:
		print("- [ERROR]: This is not a valid input.				-\n")


def choose_goal():
##
#  \brief Function to choose the goal.
#  \param None
#  \return None
#  
#  This function is called whenever the user selected (1) on the GUI; it prints the GUI section for choosing the (x,y) coordinates from console. It has no input nor output and serves also the purpouse of sending to the server that (x,y) point selected by user.

	print("------------------------------------------------------------------------\n")
	print("- Specify the (x,y) coordinates and press enter to confirm.		-\n")
	print("-									-\n")
	goal_x = int(input("- x --> "))
	goal_y = int(input("- y --> "))
	print("------------------------------------------------------------------------\n")
	print("- The chosen goal position is (x,y): (" , goal_x, goal_y,")\n")
	print("- 									-\n")
	print("- [INFO] Sending the goal position to the server.			-\n")
	print("- [INFO] Waiting for server response...		 			-\n")
	
	action_client.wait_for_server()
	
	# Initializing an instance of object PoseStamped() as the desired goal, whose attributes
	# are set as the user inputs.
	des_goal = PoseStamped()
	des_goal.pose.position.x = goal_x
	des_goal.pose.position.y = goal_y
	
	# Defining the message to be sent to server by calling pre-defined methods
	msg_goal = assignment_2_2022.msg.PlanningGoal(des_goal)
	
	# Sending message to server
	action_client.send_goal(des_goal)
	print("- [INFO] The chosen goal was correctly sent to server. 		-\n")
	print("- [INFO] Now restarting the GUI...					-\n")
	
	rospy.sleep(1.5)
	
	GUI()
	
def cancel_cmd():
##
#  \brief Function for canceling previous commands.
#  \param None
#  \return None
#
# This function is called whenever the user selected (2) on the GUI; it deletes the last goal sent to the robot.
#

	print("------------------------------------------------------------------------\n")
	print("- [INFO] Sending cancel command to the server...			-\n")
	
	# Actual deletion of the goal
	action_client.cancel_goal()
	
	print("- [INFO] Current goal position correctly aborted.			-\n")
	print("- [INFO] Now restarting the GUI...					-\n")
	
	rospy.sleep(1.5)
	
	GUI()
	
	
if __name__ == "__main__":
	try:
		# Initialize action_client node
		rospy.init_node('action_client')
		
		# Setting the action_client to /current_goal topic
		action_client = actionlib.SimpleActionClient('/reaching_goal', assignment_2_2022.msg.PlanningAction)
		GUI()
		
		# Waiting command
		rospy.spin()
		
	# Print error
	except rospy.ROSInterruptException:
		print("- [ERROR] Program aborted befor completion.				-\n", file=sys.stderr)
