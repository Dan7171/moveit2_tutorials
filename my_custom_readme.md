
# to install (only once):
Based on this link:
https://moveit.picknik.ai/main/doc/how_to_guides/isaac_panda/isaac_panda_tutorial.html
also see sources below.


# to run (every time):


## TERMINAL 1: RUN "NATIVE" (pc installed) ISAAC SIM WITH ROS2 BRIDGE:

> cd ~/moveit2_tutorials/doc/how_to_guides/isaac_panda/launch$

> export ROS_DISTRO=humble && export RMW_IMPLEMENTATION=rmw_fastrtps_cpp && export ROS_DOMAIN_ID=0 && export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/dan/isaacsim/exts/isaacsim.ros2.bridge/humble/lib && ./python.sh isaac_moveit.py


## TERMINAL 2 - Run rviz in docker container:

> cd moveit2_tutorials/doc/how_to_guides/isaac_panda
> docker compose up demo_isaac


Sources:
1. Main tutorial to follow:
https://moveit.picknik.ai/main/doc/how_to_guides/isaac_panda/isaac_panda_tutorial.html

2. Isaac sim 4.5 ros setup: (the tutorial refers to ros 4.2 setup guide, this is the updated one):
https://docs.isaacsim.omniverse.nvidia.com/latest/installation/install_ros.html#isaac-sim-app-no-system-installed-ros