#!/usr/bin/env python
#######################################################

# run at 75%
########################################################
from math import radians
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import sys
import copy
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output  as outputMsg

pub = rospy.Publisher('Robotiq2FGripperRobotOutput', outputMsg.Robotiq2FGripper_robot_output, queue_size=10)


command = outputMsg.Robotiq2FGripper_robot_output()
command.rACT = 1
command.rGTO = 1
command.rATR = 0
command.rPR = 150  #when fully open please use 150 to close
command.rSP = 255
command.rFR = 150

    
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial',
                anonymous=True)

robot = moveit_commander.RobotCommander()

#scene = moveit_commander.PlanningSceneInterface()


group_name = "manipulator"
group = moveit_commander.MoveGroupCommander(group_name)


rospy.loginfo("Waiting 2 seconds for intial setup...")
rospy.sleep(2)
rospy.loginfo("Ready to go!")

# Let's first move the robot to a convinient start position using a joint command. 
# It's usually a good idea to use a joint goal command as a first motion in order to avoid singularities 
# Also joint commands use FK which is a much easier problem to solve and rarely fails 

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
rospy.loginfo("Start")
group.go(joint_goal, wait=True)
rospy.sleep(5)

joint_goal[0] = 1.81    
joint_goal[1] = 0.789
joint_goal[2] = -0.008
joint_goal[3] = -1.37
joint_goal[4] = -0.02391226775944233
joint_goal[5] = 0.957
joint_goal[6] = 0.5505
# And finally we move the robot
rospy.loginfo("Approaching pick position")
group.go(joint_goal, wait=True)
rospy.sleep(20)
# This bit of code makes sure there is no residual movement
group.stop()

joint_goal[0] = 1.81    
joint_goal[1] = 0.968
joint_goal[2] = -0.008
joint_goal[3] = -1.373
joint_goal[4] = -0.026
joint_goal[5] = 0.775
joint_goal[6] = 0.556
# And finally we move the robot
rospy.loginfo("pick position")
group.go(joint_goal, wait=True)
rospy.sleep(10)
# This bit of code makes sure there is no residual movement
group.stop()

pub.publish(command)
rospy.sleep(2)

joint_goal[0] = 1.81    
joint_goal[1] = 0.789
joint_goal[2] = -0.008
joint_goal[3] = -1.37
joint_goal[4] = -0.02391226775944233
joint_goal[5] = 0.957
joint_goal[6] = 0.5505
# And finally we move the robot
rospy.loginfo("Approaching pick position")
group.go(joint_goal, wait=True)
rospy.sleep(7)
# This bit of code makes sure there is no residual movement
group.stop()

joint_goal[0] = radians(-109)
joint_goal[1] = radians(-15)
joint_goal[2] = radians(107)
joint_goal[3] = radians(-91)
joint_goal[4] = radians(10)
joint_goal[5] = radians(77)
joint_goal[6] = radians(0)
# And finally we move the robot
rospy.loginfo("Moving to start position")
group.go(joint_goal, wait=True)
rospy.sleep(19)
# This bit of code makes sure there is no residual movement
group.stop()



joint_goal[0] = radians(-119.07)
joint_goal[1] = radians(-67.17)
joint_goal[2] = radians(107)
joint_goal[3] = radians(-111.72)
joint_goal[4] = radians(75.34)
joint_goal[5] = radians(64.53)
joint_goal[6] = radians(-42.31)
# And finally we move the robot
rospy.loginfo("Place")
group.go(joint_goal, wait=True)
rospy.sleep(5)
# This bit of code makes sure there is no residual movement
group.stop()

command.rPR = 0
pub.publish(command) 
rospy.sleep(2)


joint_goal[0] = radians(-116.07)
joint_goal[1] = radians(-35.44)
joint_goal[2] = radians(107)
joint_goal[3] = radians(-106.27)
joint_goal[4] = radians(37.82)
joint_goal[5] = radians(64.26)
joint_goal[6] = radians(-20.31)
# And finally we move the robot
rospy.loginfo("Moving to start position")
group.go(joint_goal, wait=True)
rospy.sleep(5)

# waypoints = []
# wpose = group.get_current_pose().pose
# waypoints.append(copy.deepcopy(wpose))

# wpose.position.z -=  15 # Second move forward/backwards in (x)
# waypoints.append(copy.deepcopy(wpose))

# (plan, fraction) = group.compute_cartesian_path(
    # waypoints, 0.01, 0.0  # waypoints to follow  # eef_step
# )  # jump_threshold

# group.execute(plan, wait=True)


# # Now let's move the robot down with a pose goal
# pose_goal = group.get_current_pose().pose
# pose_goal.position.z -= 0.15
# group.set_pose_target(pose_goal)

# # Planning in Rviz
# rospy.loginfo("Generating the plan")
# plan = group.plan()
# # And now executing the motion with the real robot
# rospy.loginfo("Executing  the plan")
# group.execute(plan, wait=True)
# group.clear_pose_targets()
# rospy.sleep(10)

# pose_goal.position.z += 0.15
# group.set_pose_target(pose_goal)

# # Planning in Rviz
# rospy.loginfo("Generating the plan")
# plan = group.plan()
# # And now executing the motion with the real robot
# rospy.loginfo("Executing  the plan")
# group.execute(plan, wait=True)
# group.clear_pose_targets()
# rospy.sleep(10)

# pick approach position 

# pick position 
# 20 cm
# approach place position
# -0.26624730229377747, 0.32953137159347534, -0.008163953199982643, -1.9706501960754395, -0.006647984962910414, 0.8073738217353821, -0.028938308358192444
# down 10
