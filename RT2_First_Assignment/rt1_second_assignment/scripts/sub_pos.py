#! /usr/bin/env python3

## @package sub_pos
#  \file sub_pos.py
#  \brief This package implements the position subscriber.
#  \author Emanuele Giordano - 4479444@studenti.unige.it
#  \version 1.0
#  \date 29/01/2025
#
#  \details
#
#  This package implements the subscriber node of the robot, which acquires the robot's position and velocity, as well as the desired target position. Then it computes the correct acceleration in order to drive the robot towards the target.
#

import rospy
import os
import sys
import math
from rt1_second_assignment.msg import odom_custom_msg

# Initializing useful variables
counter = 0
vel = 0
avg_vel = 0
dist = 0

def subscriber(data):
##
#  \brief Subscriber function
#  \param data, position and velocity vector of the robot
#  \return None
#
# This function commands the velocity of the robot based on the current cartesian distance to the target position.
#
	global counter, vel, avg_vel, dist
	
	# Getting the x and y parameters from the correct topics
	des_x = rospy.get_param("/des_pos_x")
	des_y = rospy.get_param("/des_pos_y")
	
	# Setting current position
	curr_x = data.x
	curr_y = data.y
	
	# Calculating cartesian distance
	dist = math.sqrt(((des_x - curr_x)**2)+((des_y - curr_y)**2))
	
	# Setting current velocity
	curr_vel_x = data.vel_x
	curr_vel_y = data.vel_y
	
	# Calculating average velocity
	curr_vel = math.sqrt((curr_vel_x**2)+(curr_vel_y**2))
	
	# If the counter is below the queue size
	if counter < 5:
		vel = vel + curr_vel
		counter += 1
	# Else if reaches the max
	elif counter == 5:
		counter = 0
		vel /= 5
		avg_vel = vel
		vel = 0
		
def print_vel():
##
#  \brief Print current velocity function
#  \param None
#  \return None
#
# This fucntion prints the distance from the gosl and the average velocity, at the desired frequency.
#

	while True:
		print("- Distance from goal: ", dist, " 					-\n")
		print("- Average velocity: " avg_vel, " 					-\n")
		rate.sleep()

if __name__ == "__main__":
	try:
		# Instanciating subscriber node
		rospy.init_node('sub_pos')
		rospy.loginfo("Subscriber node succesfully created.")
		rospy.wait_for_message("robot_informations", odom_custom_msg)
		
		# Setting refresh rate
		rate = rospy.Rate(rospy.get_param('/frequency'))
		
		rospy.Subscriber('robot_informations', odom_custom_msg, subscriber)
		
		# Calling the function to print velocity
		print_vel()
		
	except rospy.ROSInterruptException:
		print("- [ERROR] Program aborted before completion.", file = sys.stderr)
