# Instructions to set up VM
- Install VirtualBox following this link and select the right version accordion to your operating system: https://www.virtualbox.org/wiki/Downloads

![alt text](img/VirtualBoxInstall.png)

- Once VirtualBox is installed, download the ova file (https://drive.google.com/file/d/1g0XNztTsAcZdYUQSZr6nftOegRRDPGJF/view?usp=sharing) and double-click on it. You will be presented with a Window similar to the below. Feel free to amend the settings to your machines power but be aware that the current settings are the minimum we recommend for a smooth experience

![alt text](img/ImportOVA.png)

- Click on Import and wait for the import to be completed. Once this is done, simply select the Image that has just been created (MRC-ICRA) and Run it. Once the system boots up you will be asked for a password. The password is: mrc-icra

There are a couple of other things you need to install

```
sudo apt install ros-melodic-moveit-commander
sudo apt install ros-melodic-joint-state-publisher-gui
```
The ROS environment has slightly been updated so please download the latets version by following this instructions. Create a new workspace

```
mkdir -p mrc_icra_ws_updated/src
```
Then we need to initialise the workspace:
```
cd ~/mrc_icra_ws_updated
catkin_init_workspace 
```
Now go to the src folder and clone the challenge repo:
```
cd ~/mrc_icra_ws_updated/src
git clone https://github.com/gianmarco96/MRC-at-ICRA
```
This is optional but we would suggest removing the unnecessary files:
```
mv ~/MRC-at-ICRA/iiwa_stack-master/ .
rm -fr MRC-at-ICRA/
```
Now you can build the worksapce
```
cd ~/mrc_icra_ws_updated
rosdep install --from-paths src --ignore-src -r -y
catkin build
```
At the end of the build you can source the environment
```
source devel/setup.bash
```
You will also need to change the network settings. Please ask the organisers for help with this. You are all set up. You can now follow the [Day 1 Setup instructions](Day1Setup.md)

