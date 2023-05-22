#!/usr/bin/env python
from math import radians
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import sys

    
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
rospy.sleep(2)
# This bit of code makes sure there is no residual movement
group.stop()

# Now let's move the robot down with a pose goal
pose_goal = group.get_current_pose().pose
pose_goal.position.z -= 0.2
group.set_pose_target(pose_goal)

# Planning in Rviz
rospy.loginfo("Generating the plan")
plan = group.plan()
rospy.sleep(2)
# And now executing the motion with the real robot
rospy.loginfo("Executing  the plan")
group.execute(plan, wait=True)
group.clear_pose_targets()
