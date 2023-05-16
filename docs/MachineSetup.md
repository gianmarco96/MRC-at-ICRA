# Set up of Ubuntu 18 machine
This set of instructions will guide you to set up your Ubuntu 18 machine for the Manufacturing Robotics Challenge.
- Install ROS
- Clone and build the iiwa-stack ROS library
- Install the camera drivers
- Install the find-object package
## Install ROS
You first need to install ROS melodic. [Follow these instructions to do so](http://wiki.ros.org/melodic/Installation/Ubuntu).
## Install the camera drivers
There are 3 cameras that will be used during this challenge. Each group will be using only one type, so you might want to wait until you know which camera has been assigned to you before you carry on with this setup.
- [ASUS Xtion Pro](#asus-xtion-pro)
- [Realsense camera](#realsense)
- [Microsoft Azure Kinect](#microsoft-azure-kinect)
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
Select camera_link_ as the Fixed Frame. Then click on Add, find the Camera type select it and click OK

![alt text](img/RvizOpenniFixedFrame.png)

A new Camera type object will appera on the Displays panel, expand it and select /camera/rgb/image_raw from the Image Topic. The rgb camera feed should now appera in the Camera box. 
To test the depth image, click on Add again, and select PoinCloud2. A new PointCloud2 object will be generated, expand it and select /camera/depth/points as the Topic. You should now see the point cloud in the RViz environment.

![alt text](img/RvizOpenniPointClouds.png)

### RealSense 
Open a new terminal (tip: you can press Ctrl + Alt + T) and copy-paste the following command:
```
sudo apt install ros-melodic-realsense2-camera
```
**Note:** The above assumes you have run `sudo apt update` during the installation of ROS, if you have not please do so before installing the camera driver.
To test the camera has installed correctly, connect the realsense camera to your laptop, open a new terminal and run the following
```
roslaunch realsense2_camera rs_camera.launch device_type:=d435i
```
Then open Rviz
```
rviz
```
Select camera_link_ as the Fixed Frame. Then click on Add, find the Camera type select it and click OK

![alt text](img/RvizRealsenseFixedFrame.png)

A new Camera type object will appera on the Displays panel, expand it and select /camera/color/image_raw from the Image Topic. The rgb camera feed should now appera in the Camera box. 
To test the depth image, click on Add again, and select DepthCloud. A new DepthCloud object will be generated, expand it and select /camera/depth/image_raw_rect as the Depth Map Topic. You should now see the point cloud in the RViz environment.

![alt text](img/RvizDepthCloud.png)

### Microsoft Azure Kinect

### Install the Find Object Package
To install the package run the following 
```
sudo apt install ros-melodic-find-object-2d
```
You are done with the installation. Before launching the application you need to check the model of the camera your using. By default, the application subscribes to the '/camera/rgb/image_rect_color' for the rgb image,  '/camera/depth_registered/image_raw' for the depth image and  '/camera/rgb/camera_info' for the camera metadata. Each camera driver uses a slightly different naming for their topics so you will have to remap to the correct one. To see which one is the correct topic, launch the camera driver first then check the avalible topics with 'rostopic list' and note down the name of the topic. Below, you can find an example with the Xtion Pro Camera drive
```
# First we launch the camera driver
roslaunch realsense2_camera rs_camera.launch device_type:=d435i
# Then we open a new terminal and check the available topics
rostopic list
``` 
You will get something similar to this: 
```
/camera/color/camera_info
/camera/color/image_raw
/camera/color/image_raw/compressed
/camera/color/image_raw/compressed/parameter_descriptions
/camera/color/image_raw/compressed/parameter_updates
/camera/color/image_raw/compressedDepth
/camera/color/image_raw/compressedDepth/parameter_descriptions
/camera/color/image_raw/compressedDepth/parameter_updates
/camera/color/image_raw/theora
/camera/color/image_raw/theora/parameter_descriptions
/camera/color/image_raw/theora/parameter_updates
/camera/color/metadata
/camera/depth/camera_info
/camera/depth/image_rect_raw
/camera/depth/image_rect_raw/compressed
/camera/depth/image_rect_raw/compressed/parameter_descriptions
/camera/depth/image_rect_raw/compressed/parameter_updates
/camera/depth/image_rect_raw/compressedDepth
/camera/depth/image_rect_raw/compressedDepth/parameter_descriptions
/camera/depth/image_rect_raw/compressedDepth/parameter_updates
/camera/depth/image_rect_raw/theora
/camera/depth/image_rect_raw/theora/parameter_descriptions
/camera/depth/image_rect_raw/theora/parameter_updates
/camera/depth/metadata
/camera/extrinsics/depth_to_color
/camera/motion_module/parameter_descriptions
/camera/motion_module/parameter_updates
/camera/realsense2_camera_manager/bond
/camera/rgb_camera/auto_exposure_roi/parameter_descriptions
/camera/rgb_camera/auto_exposure_roi/parameter_updates
/camera/rgb_camera/parameter_descriptions
/camera/rgb_camera/parameter_updates
/camera/stereo_module/auto_exposure_roi/parameter_descriptions
/camera/stereo_module/auto_exposure_roi/parameter_updates
/camera/stereo_module/parameter_descriptions
/camera/stereo_module/parameter_updates
/diagnostics
/rosout
/rosout_agg
/tf
/tf_static
```
So you should run the following command:
```
roslaunch find_object_2d find_object_3d.launch rgb_topic:=/camera/color/image_raw depth_topic:=/camera/depth/image_rect_raw camera_info_topic:=/camera/depth/camera_info
```
 More info on how to use the find_object_2d package can be found [here](http://wiki.ros.org/find_object_2d)