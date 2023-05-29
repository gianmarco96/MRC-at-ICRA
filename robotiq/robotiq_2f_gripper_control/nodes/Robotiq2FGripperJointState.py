#!/usr/bin/env python
import rospy
from robotiq_2f_gripper_control.msg import Robotiq2FGripper_robot_output as outputMsg
import moveit_commander

group_name = "endeffector"
move_group = moveit_commander.MoveGroupCommander(group_name)


def callback(data):
    joint_goal = move_group.get_current_joint_values()
    joint_goal[0] = (data.rPR*0.8)/255
    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    move_group.go(joint_goal, wait=True)
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('Robotiq2FGripperUpdateJS', anonymous=True)

    rospy.Subscriber("Robotiq2FGripperRobotOutput", outputMsg, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()