# Set up for the challenge
During the first hour (or so) of the challenge you will be setting up the ROS environment. Most of the stuff you need should have been already configured in the previous steps, if you get any errors or are unsure about anything just ask for support.

## Prerequisite
- You need to have the ROS environment set up and up to date
- You need to have the network set up

## Set up the ROS IP
First we need to add the IP address of the ROS master to the bashrc so that the robot can see the ROS master. You can use any method you like for this, we recommend the following
```
sudo nano ~/.bashrc
```
Scroll at the bottom of the file and add the following 2 lines
```
export ROS_IP=172.31.1.150
export ROS_MASTER_URI=http://$ROS_IP:11311/
```
## Launch the node that controls the robot
This will depend on the gripper that is assigned to you. There are 3 types of grippers:
- robotiq
- robotiq_cam
- zimmer
You can run the follwoing command to start the moveit environment `roslaunch iiwa_moveit moveit_planning_execution.launch sim:=false gripper:=$gripper_you_are_using`. For example:
```
roslaunch iiwa_moveit moveit_planning_execution.launch sim:=false gripper:=$gripper_you_are_using
```
Then you can use the moveit commander to control the robot as shown in the presentation. 
## Gripper control
Controlling the grippers depends on the type of gripper assigned to you. 
### Robotiq gripper
First plug in the gripper to the ROS master laptop. The run the following command to give proper privileges to the user to control the gripper via serial ports.
```
usermod -a -G dialout mrc-icra
```
Now in a new terminal run the node that activates the gripper.
```
rosrun robotiq_2f_gripper_control Robotiq2FGripperRtuNode.py /dev/ttyUSB0
```
And finally, open another terminal and launch the gripper node
```
rosrun robotiq_2f_gripper_control Robotiq2FGripperSimpleController.py 
```
You first need to reset and activate the gripper. You can not control the gripper
**Note: Beware of namespaces if you want to control the gripper and the robot in the same node**. Ask for help if unsure.

### Robotiq wrist camera gripper
This is identical to the previous one, only difference is the name of the device which in this case is `/dev/ttyACM0`

### Zimmer gripper
A ROS driver for the zimmer gripper has been created on the iiwa Sunrise environment. To control the gripper you can simply create a ROS publisher:
- name of topic : "iiwa/command/GripperTrigger"
- msg type: std_msgs.Bool
If unsure, ask for help.