
# Motivation:
"In this tutorial we have included a Python script that loads a Panda robot and builds an OmniGraph to publish and subscribe to the ROS topics used to control the robot. The OmniGraph also contains nodes to publish RGB and Depth images from the camera mounted on the hand of the Panda. The RGB image is published to the topic /rgb, the camera info to /camera_info, and the depth image to /depth. The frame ID of the camera frame is /sim_camera. To learn about configuring your Isaac Sim robot to communicate with ROS 2 please see the Joint Control tutorial on Omniverse."

https://moveit.picknik.ai/main/doc/how_to_guides/isaac_panda/isaac_panda_tutorial.html#:~:text=In%20this%20tutorial%20we,Control%20tutorial%20on%20Omniverse.


# Requirements 
(this is what I worked with, will probably work under different setups):

1. Ubuntu 24.04 (again, this is what I worked with, will probably work with different distributions)
2. Isaac sim 4.5 naitve (on pc) installed on home directory. See: https://docs.isaacsim.omniverse.nvidia.com/latest/installation/requirements.html

More details in main tutorial: https://moveit.picknik.ai/main/doc/how_to_guides/isaac_panda/isaac_panda_tutorial.html#running-the-moveit-interactive-marker-demo-with-isaac-sim:~:text=This%20tutorial%20requires%20a%20machine%20with%20Isaac%20Sim%204.5%20(recommended)%20or%20Isaac%20Sim%204.2%20installed.%20For%20Isaac%20Sim%20requirements%20and%20installation%20please%20see%20the%20Omniverse%20documentation.%20To%20configure%20Isaac%20Sim%20to%20work%20with%20ROS%202%20please%20see%20this%20guide.

https://moveit.picknik.ai/main/doc/how_to_guides/isaac_panda/isaac_panda_tutorial.html#running-the-moveit-interactive-marker-demo-with-isaac-sim:~:text=This%20tutorial%20has,configure%20the%20system.

3. docker + nvidia-container toolkit:
More details are in:
https://moveit.picknik.ai/main/doc/how_to_guides/isaac_panda/isaac_panda_tutorial.html#running-the-moveit-interactive-marker-demo-with-isaac-sim:~:text=Docker%20is%20installed.%20If%20you%20plan%20to%20use%20your%20GPU%20with%20MoveIt%2C%20you%20will%20need%20to%20install%20nvidia%2Ddocker.





4. nvidia driver and cuda - I used this one:       
Driver Version: 570.133.07     
CUDA Version: 12.8  


5. Some basic understanding of omnigraph concept:
(tutorial refers to this link
https://docs.omniverse.nvidia.com/isaacsim/latest/gui_tutorials/tutorial_gui_omnigraph.html )


# Setup instructions (only once):
All instructions are based on this link:
https://moveit.picknik.ai/main/doc/how_to_guides/isaac_panda/isaac_panda_tutorial.html
also see: sources below!


## 1. install isaac sim, docker, nvidia-container toolkit:

### docker: search online

### isaac sim (more details above)
https://docs.isaacsim.omniverse.nvidia.com/latest/installation/install_workstation.html

### nvidia container toolkit: note that if you already have a cuda driver, you can probably skip the driver 535 recommended installation...


More info here: https://moveit.picknik.ai/main/doc/how_to_guides/isaac_panda/isaac_panda_tutorial.html#:~:text=This%20tutorial%20has,run%20this%20tutorial.

## 2. 
> First, make a directory on your system, to clone the moveit 2 tutorials into.., (I just used the home directory).
## 3. 
"Perform a shallow clone of the MoveIt 2 Tutorials repo - named moveit2_tutorials (actually, we clone this repo, which is named the same, but has minor changes)".
> git clone https://github.com/Dan7171/moveit2_tutorials.git


NOTE: If for some reason you prefer the original repo, clone as that:
>  git clone https://github.com/moveit/moveit2_tutorials.git -b main

https://moveit.picknik.ai/main/doc/how_to_guides/isaac_panda/isaac_panda_tutorial.html#:~:text=Perform%20a%20shallow,%2Db%20main


## 4. 
"Go to the folder in which you cloned the tutorials and then switch to the following directory."

> cd moveit2_tutorials/doc/how_to_guides/isaac_panda


## 5. 
"Build the Docker image. This docker image also contains pytorch".

> docker compose build base

NOTE: In this command, we use the hidden dockerfile file in this repository. It contains some minor big fixes compared to the original dockerfile from the original repo...
You can compare the files to convice yourself (the changes are done by the help of an llm, to debug and make the docker build work, since the original file failed to build on my setup).

# Test after setup (only once)
> docker compose up demo_mock_components
https://moveit.picknik.ai/main/doc/how_to_guides/isaac_panda/isaac_panda_tutorial.html#:~:text=compose%20build%20base-,Running%20the%20MoveIt%20Interactive%20Marker%20Demo%20with%20Mock%20Components,%EF%83%81,-This%20section%20tests

To convince yourself it succeeded, rviz window in docker will run, and you'll see the franka arm. Make sure this is the image, and there are no errors or missing contents in the rviz (like a missing motion planner for example)

# Running instructions (every time):

## 1. OPEN TERMINAL 1: RUN "NATIVE" (pc installed) ISAAC SIM WITH ROS2 BRIDGE:

### 1. go the the parent dir of moveit2_tutorials again.

### 2. run
> cd ~/moveit2_tutorials/doc/how_to_guides/isaac_panda/launch


### 3. run 
> export ROS_DISTRO=humble && export RMW_IMPLEMENTATION=rmw_fastrtps_cpp && export ROS_DOMAIN_ID=0 && export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/dan/isaacsim/exts/isaacsim.ros2.bridge/humble/lib && ./python.sh isaac_moveit.py

NOTE: This was also a minor modification comapred to the original running command, as it had bugs.
The original comammand was just ./python.sh isaac_moveit.py



## 2. OPEN TERMINAL 2 - Run rviz in docker container:
###  1. go the the parent dir of moveit2_tutorials again.

### 2. Then cd to the isaac_panda dir: 

> cd moveit2_tutorials/doc/how_to_guides/isaac_panda

### 3. Now run the docker compose command starting rviz container:

> docker compose up demo_isaac

Now you should see one window for rviz and one window for isaacsim (took from tutorial):

https://www.youtube.com/watch?embeds_referring_euri=https%3A%2F%2Fmoveit.picknik.ai%2F&source_ve_path=Mjg2NjQsMTY0NTAz&v=EiLaJ7e4M-4&feature=youtu.be

## HOW TO OPERATE?

1. MARK THE GOAL: In rviz: drag the gizmo around the panda head to mark the desired end effector pose at the end of the motion.
2. Plan and execute: In rviz: click plan & execute button (you can choose your motion planner in the 'Context' tab)
3. See how both rviz and isaacsim robots move!!



Sources:
1. Main tutorial to follow:
https://moveit.picknik.ai/main/doc/how_to_guides/isaac_panda/isaac_panda_tutorial.html

2. Isaac sim 4.5 ros setup: (the tutorial refers to ros 4.2 setup guide, this is the updated one):
https://docs.isaacsim.omniverse.nvidia.com/latest/installation/install_ros.html#isaac-sim-app-no-system-installed-ros