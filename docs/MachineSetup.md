# Set up of Ubuntu 18 machine
This set of instructions will guide you to set up your Ubuntu 18 machine for the Manufacturing Robotics Challenge.
## Install ROS
You first need to install ROS melodic. [Follow these instructions to do so](http://wiki.ros.org/melodic/Installation/Ubuntu).
## Install the camera drivers
There are 3 cameras that will be used during this challenge. Each group will be using only one type, so you might want to wait until you know which camera has been assigned to you before you carry on with this setup.
### Asus Xtion Pro
Open a new terminal (tip: you can press Ctrl + Alt + T) and copy-paste the following command:
```
sudo apt install ros-melodic-openni2-*
```
**Note:** The above assumes you have run `sudo apt update` during the installation of ROS, if you have not please do so before installing the camera driver.
To test the camera has installed correctly, connect the ASUS Xtion camera to your laptop, open a new terminal and run the following
```
roslaunch openni2_launch openni2.launch
```
Then open Rviz
```
rviz
```
Select _camera_link_ as the Fixed Frame. Then click on Add, find the Camera type select it and click OK
A new Camera type object will appera on the Displays panel, expand it and select /camera/rgb/image_raw from the Image Topic. The rgb camera feed should now appera in the Camera box. 
To test the depth image, click on Add again, and select PoinCloud2. A new PointCloud2 object will be generated, expand it and select /camera/depth/points as the Topic. You should now see the point cloud in the RViz environment.
