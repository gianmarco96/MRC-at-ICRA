#!/usr/bin/env python2.7

from robotiq_2f_gripper_control.srv import UnityGripper, UnityGripperResponse
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output  as outputMsg
import rospy


pub = rospy.Publisher('Robotiq2FGripperRobotOutput', outputMsg.Robotiq2FGripper_robot_output, queue_size=10)



def handle_unity_gripper(req):
    
    command = outputMsg.Robotiq2FGripper_robot_output()
    command.rACT = 1
    command.rGTO = 1
    command.rATR = 0
    command.rPR = 0  #when fully open please use 150 to close
    command.rSP = 255
    command.rFR = 150

    if req.open_close == False:
        print("closing gripper")
        command.rPR = 150
        pub.publish(command)
        
        return "Closing gripper"
    elif req.open_close == True:
        command.rPR = 0 
        print("open gripper")
        pub.publish(command)
        return "Opening gripper"

def unity_gripper_server():
    rospy.init_node('gripper_unity_server_node')
    s = rospy.Service('gripper_unity_server', UnityGripper, handle_unity_gripper)
    
    print("Ready to control the gripper via button controls")
    rospy.spin()

if __name__ == "__main__":
    unity_gripper_server()
    