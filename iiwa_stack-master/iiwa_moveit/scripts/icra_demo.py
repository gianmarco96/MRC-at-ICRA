#!/usr/bin/env python
####################################################################################
# Demo for ICRA challenge
# This demo will be run at 75% override
# The code has been simplified for clarity and is therefore not the optimal solution
####################################################################################
from math import radians
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import sys
import copy
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output  as outputMsg

# Initialising the publisher which will control the robotiq gripper
pub = rospy.Publisher('Robotiq2FGripperRobotOutput', outputMsg.Robotiq2FGripper_robot_output, queue_size=10)

# Initialising the command msg to be sent to the gripper
command = outputMsg.Robotiq2FGripper_robot_output()
command.rACT = 1
command.rGTO = 1
command.rATR = 0
command.rPR = 150  # only value you will change to open/close the gripper
command.rSP = 255
command.rFR = 150

# Initilasing the moveit commander    
moveit_commander.roscpp_initialize(sys.argv)
robot = moveit_commander.RobotCommander()

# Initilasing the ROS node
rospy.init_node('icra_demonstration', anonymous=True)

group_name = "manipulator"
group = moveit_commander.MoveGroupCommander(group_name)


rospy.loginfo("Waiting 2 seconds for intial setup...")
rospy.sleep(2)
rospy.loginfo("Ready to go!")

# Let's first move the robot to a convinient start position using a joint command. 

# First we construct a joint goal object by reading the current joint position
joint_goal = group.get_current_joint_values()
# Now we generate the joint goal by expressing the desired position of each joint
joint_goal[0] = radians(-116.07)
joint_goal[1] = radians(-35.44)
joint_goal[2] = radians(107)
joint_goal[3] = radians(-106.27)
joint_goal[4] = radians(37.82)
joint_goal[5] = radians(64.26)
joint_goal[6] = radians(-20.31)
# And finally we move the robot
rospy.loginfo("Move to initial Position")
group.go(joint_goal, wait=True)
rospy.sleep(5)

# Now we will move to the approach position
joint_goal[0] = 1.81    
joint_goal[1] = 0.789
joint_goal[2] = -0.008
joint_goal[3] = -1.37
joint_goal[4] = -0.02391226775944233
joint_goal[5] = 0.957
joint_goal[6] = 0.5505
rospy.loginfo("Approaching pick position")
group.go(joint_goal, wait=True)
rospy.sleep(20)
# This bit of code makes sure there is no residual movement
# group.stop()

# Moving to the pick position
joint_goal[0] = 1.81    
joint_goal[1] = 0.968
joint_goal[2] = -0.008
joint_goal[3] = -1.373
joint_goal[4] = -0.026
joint_goal[5] = 0.775
joint_goal[6] = 0.556
rospy.loginfo("Moving to pick position")
group.go(joint_goal, wait=True)
rospy.sleep(10)

# Now we can close the gripper
pub.publish(command)
rospy.loginfo("Closing the gripper")
rospy.sleep(2)

# Safely moving away from the table
joint_goal[0] = 1.81    
joint_goal[1] = 0.789
joint_goal[2] = -0.008
joint_goal[3] = -1.37
joint_goal[4] = -0.02391226775944233
joint_goal[5] = 0.957
joint_goal[6] = 0.5505
rospy.loginfo("Approaching pick position")
group.go(joint_goal, wait=True)
rospy.sleep(7)

# Moving to the building area
joint_goal[0] = radians(-109)
joint_goal[1] = radians(-15)
joint_goal[2] = radians(107)
joint_goal[3] = radians(-91)
joint_goal[4] = radians(10)
joint_goal[5] = radians(77)
joint_goal[6] = radians(0)
rospy.loginfo("Approaching building position")
group.go(joint_goal, wait=True)
rospy.sleep(19)

# Moving to place position
joint_goal[0] = radians(-119.07)
joint_goal[1] = radians(-67.17)
joint_goal[2] = radians(107)
joint_goal[3] = radians(-111.72)
joint_goal[4] = radians(75.34)
joint_goal[5] = radians(64.53)
joint_goal[6] = radians(-42.31)
rospy.loginfo("Placing the brick down")
group.go(joint_goal, wait=True)
rospy.sleep(5)

# Opening the gripper
command.rPR = 0
pub.publish(command) 
rospy.loginfo("Opening the gripper")
rospy.sleep(2)

# Back to home position
joint_goal[0] = radians(-116.07)
joint_goal[1] = radians(-35.44)
joint_goal[2] = radians(107)
joint_goal[3] = radians(-106.27)
joint_goal[4] = radians(37.82)
joint_goal[5] = radians(64.26)
joint_goal[6] = radians(-20.31)
rospy.loginfo("Moving to start position")
group.go(joint_goal, wait=True)
rospy.sleep(5)
