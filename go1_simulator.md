After installing ROS Noetic, as presented in [ROS Noetic on WSL2](noetic_wsl.md), you may also need to install the following components (although some of these might have been included in the Noetic installation):

```
sudo apt update
sudo apt-get install ros-noetic-controller-interface ros-noetic-gazebo-ros-control ros-noetic-joint-state-controller ros-noetic-effort-controllers ros-noetic-joint-trajectory-controller liblcm-dev ros-noetic-move-base-msgs ros-noetic-move-base
```

Once the ROS packages are set-up the next step would be to set up a workspace in which you can add pre-built packages to control your robot.

# CATKIN Workspace
1.  First install catkin_tools:
```
$ sudo apt install python3-catkin-tools
```
2. The next step is to create, your workspace. This is typically created in the home directory following the standard ROS convention, once built, you should see a `build` and `devel` folder next to your `src` folder:
```console
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/
$ catkin build
```
3. To enable the workspace, add this into the last line of the bashrc file (`$ gedit ~/.bashrc`):
```
source /home/<your_computer_username>/catkin_ws/devel/setup.bash
```
4. To verify that everything is working correctly, type the following into the terminal:
```
$ echo $ROS_PACKAGE_PATH
```

# Simulation and Control Installation
To perform simple high-level simulations, we need `unitree_guide`, which relies on `unitree_ros`. Therefore, in this section, I will explain how to install both `unitree_guide` and `unitree_ros`. First, it is essential to install the C++ packages supporting the console output and logging in roscpp, and that of the XML-RPC protocol
```console
$ sudo apt install libxmlrpcpp-dev librosconsole-dev
```
Next, the abovesaid repos shall be installed as follows

```console
$ cd ~/catkin_ws/src
$ git clone https://github.com/LeoBoticsHub/unitree_ros --depth 1
$ cd unitree_ros
$ git submodule update --init --recursive --depth 1
```

Next, navigate to the file `unitree_gazebo/worlds/starts.world`, and at the end of the document, modify the <uri> to reflect the actual path (specifically, change the name following 'home' to your own username, and adjust the workspace directory name if it does not match):

```bash
<include>
  <uri>model:///home/unitree/catkin_ws/src/unitree_ros/unitree_gazebo/worlds/building_editor_models/stairs</uri>
</include>
```
that should be (if the above-steps are followed)

```bash
<uri>///home/<your_computer_username>/catkin_ws/src/unitree_ros/unitree_gazebo/worlds/building_editor_models/stairs</uri>
```

Afterward, return to the catkin workspace directory and execute `catkin_make` to complete the installation.
```console
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/
$ catkin build
```


To download the `unitree_guide` in your workspace, execute the following command in the src directory:
```console
$ cd src
$ git clone https://github.com/LeoBoticsHub/unitree_guide --depth 1
$ cd ..
$ catkin build
```

# Run the Simulator and Controller

Open a terminal and start Go1 in Gazebo with a preloaded world:
```console
$ roslaunch unitree_guide gazeboSim.launch
```

and the GO1 simulation motion controller can be run by executing in another terminal:
```console
$ ~/catkin_ws$ sudo ./devel/lib/unitree_guide/junior_ctrl
```
so that the robot can be moved using the keys (`2` and `4` for going fixed stand and trotting, `w`, `a`, `s` and `d` for moving towards the four direction, and `J` and `L` for turning left and right).



