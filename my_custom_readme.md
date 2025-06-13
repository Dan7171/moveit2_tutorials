
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




# Tutorial (by o3)


Below is a “deep-dive tour” of everything that comes to life when you run the Isaac-Sim × MoveIt 2 “Panda” tutorial – plus the extras you asked for.  It is written for readers who have only a light background in ROS 2, Docker, or OmniGraph, and it contains enough detail for you to draw a formal system‐diagram later.

────────────────────────────────────────────────────────
1.  The big picture – who runs where?
────────────────────────────────────────────────────────

HOST ( Linux Workstation )
│
├─ Isaac Sim GUI process  
│    • Started by `isaac_moveit.py` (native python on host)  
│    • Spawns USD stage with a Franka-Panda robot and props  
│    • Builds an OmniGraph that publishes & subscribes ROS 2 topics  
│
└─ Docker container `demo_isaac`  
     • Ubuntu 22.04 image built from `docker-compose.yml`  
     • Runs ROS 2 Humble, MoveIt 2, RViz, ros2_control  
     • RViz window forwarded via X11 (or VNC/Wayland)  
     • All ROS nodes in the container share the same DDS domain as Isaac Sim  
       (domain id is supplied by the OmniGraph `ROS2Context` node)

Key idea: the host provides hi-fidelity simulation and vision; the container provides planning, control loops, visualisation, and a clean ROS environment.

────────────────────────────────────────────────────────
2.  Anatomy of `isaac_moveit.py`
────────────────────────────────────────────────────────

Lines you’ll notice:

• Simulation startup  
  ```python
  simulation_app = SimulationApp({"renderer":"RayTracedLighting","headless":False})
  ```
  Creates the Isaac Sim application.  Everything afterwards is executed inside its python interpreter.

• Stage loading  
  Background USD, the Franka arm, and four YCB objects are referenced into the stage with `prims.create_prim(...)`.

• OmniGraph construction (≈ lines 120-450)  
  One call to `og.Controller.edit()` creates ~20 nodes and connects them.  
  The most important nodes and their ROS 2 roles are:

  Node Name                     Purpose (topic)  
  ------------------------------------------------------------------
  ROS2Context               • initialises rclcpp, sets domain id  
  IsaacReadSimulationTime   • reads /clock ticks (sim-time)  
  ROS2PublishClock          • publishes `/clock`  
  ROS2PublishJointState     • publishes `/isaac_joint_states`  
  ROS2SubscribeJointState   • subscribes  `/isaac_joint_commands`  
  IsaacArticulationController • writes joint commands into the USD articulation  
  IsaacCreateViewport/…     • spawns second camera viewport  
  ROS2CameraHelper          • publishes `/rgb` & `/depth` images  
  ROS2CameraInfoHelper      • publishes `/camera_info`

  Execution flow: an `OnImpulseEvent` and an `OnTick` node fire once every physics frame, provoking the publish / subscribe chain.

• Two extra `simulation_app.update()` calls right after the graph is built.  This is an Isaac-Sim quirk: it forces the graph to initialise once targets have been set.

• Main loop
  ```python
  while simulation_app.is_running():
      simulation_context.step(render=True)
      og.Controller.set(...enableImpulse), True)
  ```
  Each physics frame the controller sets an “impulse” flag that pulses the ROS nodes.  (Exactly the same pattern shown in NVIDIA’s [Joint Control tutorial]).

────────────────────────────────────────────────────────
3.  ROS 2 side – what lives in the container?
────────────────────────────────────────────────────────

`docker compose up demo_isaac` launches – via the entry-point script –
these major ROS 2 nodes:

Node/Executable             Role
----------------------------------------------------------------------
ros2_control Node + 
  topic_based_ros2_control  • Converts incoming joint commands on
                              `/isaac_joint_commands` to
                              ros2_control command interfaces.
                            • Publishes `/isaac_joint_states`
move_group (MoveIt 2)       • High-level planner (OMPL is default)
rviz2                       • 3-D visualiser & interactive marker
static_transform_publisher  • Robot’s fixed TF tree (URDF frames)
robot_state_publisher       • Broadcasts TF from joint positions

Data Flow (normal cycle):
1. RViz Interactive Marker → MoveIt (target_pose)  
2. MoveIt → ros2_control (JointTrajectory)  
3. ros2_control (TopicBasedSystem plugin) → `/isaac_joint_commands`  
4. Isaac Sim subscribes, moves the USD articulation, and publishes joint-state, images, clock.  
5. ros2_control receives `/isaac_joint_states`, updates state interfaces → TF & RViz update.

────────────────────────────────────────────────────────
4.  Glossary of the main concepts
────────────────────────────────────────────────────────

• ROS Node – an executable that uses rclcpp/rclpy API.  
• Topic – typed, anonymous publish-subscribe channel (under Fast-DDS).  
• Service – request/response RPC; MoveIt uses `/get_planner_params`, etc.  
• ros2_control – generic controller framework; a *hardware interface*
  chooses whether commands go to a real robot, Gazebo, Isaac Sim …  
• OmniGraph – Isaac Sim’s node-based runtime; nodes can be “simulation”
  (physics, sensors) or “ROS 2 bridge”; an action-graph is saved inside the USD stage.  
• USD – Pixar’s scene database; holds meshes, joints, cameras, metadata.  
• RViz – Qt/OpenGL visualiser that subscribes TF, sensor_msgs, etc.

────────────────────────────────────────────────────────
5.  Step-by-step when you press “Plan & Execute” in RViz
────────────────────────────────────────────────────────

 1. RViz publishes a `moveit_msgs/msg/DisplayTrajectory` goal via the
    Interactive-Marker plugin.  
 2. `move_group` receives it, calls OMPL (or Pilz, STOMP …) and returns
    a time-parameterised `JointTrajectory`.  
 3. A `JointTrajectoryController` inside ros2_control converts it to
    position set-points and publishes them on `/isaac_joint_commands`
    (geometry_msgs/msg/Twist for velocity or efforts if configured).  
 4. Isaac Sim’s `ROS2SubscribeJointState` node writes those arrays to the
    input pins of `IsaacArticulationController`.  
 5. The physics engine integrates, moves the robot, and the
    `ROS2PublishJointState` node samples joint angles/velocities from
    the articulation and fires a sensor_msgs/JointState message.  
 6. ros2_control receives that and closes the feedback loop; RViz & TF
    tree update; MoveIt sees the new state and can re-plan if needed.

────────────────────────────────────────────────────────
6.  Answers to your “missing pieces”
────────────────────────────────────────────────────────

6.1  Loading *any* robot (URDF → USD → OmniGraph)  
   a)  Drag-and-drop the URDF in Isaac Sim or run  
       `isaacsim-python.sh tools/urdf_importer.py --urdf <file>.urdf`  
       A USD stage with physics joints is generated under `/World/<robot>`.  
   b)  Add a ros2_control *hardware* tag in your URDF/Xacro:  

       ```xml
       <hardware>
         <plugin>topic_based_ros2_control/TopicBasedSystem</plugin>
         <param name="joint_commands_topic">/isaac_joint_commands</param>
         <param name="joint_states_topic">/isaac_joint_states</param>
       </hardware>
       ```  
   c)  Re-build your MoveIt package (`moveit_setup_assistant`) so that
       it knows about the new joints and planning groups.  
   d)  In the OmniGraph, change `ArticulationController.inputs:robotPath`
       to the new prim path; change camera paths if needed.

6.2  Specifying a target pose *without* RViz  
   •  Python example in the container:  

      ```python
      from moveit2 import MoveItCpp
      mcpp = MoveItCpp('panda_arm')
      pose = geometry_msgs.msg.PoseStamped()
      pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = 0.3,0.1,0.5
      mcpp.setPoseTarget(pose)
      plan = mcpp.plan()
      mcpp.execute(plan)
      ```  
   •  Or publish directly to a `FollowJointTrajectory` action.  
   •  Inside Isaac Sim you can call
     `IsaacArticulationController.set_joint_position_targets(q)` from a
     custom python node – bypassing MoveIt completely.

6.3  Plugging-in a **custom motion planner**  
   •  MoveIt lets you register a new planner plugin in
     `moveit_core/planning_interface`.  Add the plugin’s `.so`,
     declare it in `<moveit_planners.yaml>`, and set  
     `move_group.launch.py` → `planning_pipelines:=my_planner`.  
   •  To bypass MoveIt entirely, send your own
     `trajectory_msgs/JointTrajectory` to the
     `joint_trajectory_controller` action topic (`/joint_trajectory_controller/follow_joint_trajectory`).

6.4  Editing the OmniGraph  
   •  GUI: Window → Graph Editor → drag nodes, right-click pins, save the stage (`Ctrl-S`).  
   •  Python: use the exact API you saw in `isaac_moveit.py`:  

      ```python
      og.Controller.set(("MyNode.inputs:parameter"), 0.5)
      og.Controller.disconnect(("A.out","B.in"))
      ```  
     Full API reference: [OmniGraph Python Scripting].  
   •  Scripted graphs can be loaded from an extension so they appear every time the stage opens.

6.5  Commanding a *real* robot as well  
   •  Replace TopicBasedSystem with your robot’s real hardware interface
     (e.g. `franka_hardware/FrankaHardwareInterface`).  
   •  Keep MoveIt and ros2_control exactly the same.  Only the *system*
     plugin changes; all controllers stay untouched.  
   •  If you want “digital-twin” operation, launch two hardware
     interfaces simultaneously:  
     ```
     /isaac_joint_commands   → Isaac Sim
     /franka_joint_commands  → Real robot
     ```  
     Use a simple multiplexer node to broadcast each trajectory to both topics.

────────────────────────────────────────────────────────
7.  Proposed flow-diagram (text description)
────────────────────────────────────────────────────────

[Box]  RViz GUI  
  ↳ interactive-marker   ↓    ↑ TF, sensor_msgs  
[Box]  move_group (MoveIt 2)  
  ↳ /execute_trajectory action  
[Box]  ros2_control mgr  
  • JointTrajectoryController  
  ↳ /isaac_joint_commands  (geometry_msgs/JointState feedback loop)  
  ↕  
[Box]  Isaac-Sim Process  
  ├─ OmniGraph / ROS2SubscribeJointState  
  ├─ IsaacArticulationController  
  ├─ Physics Engine  
  ├─ OmniGraph / ROS2PublishJointState  
  ├─ ROS2PublishClock  
  └─ ROS2CameraHelpers (/rgb, /depth, /camera_info)  
  ↕  
[Box]  DDS Domain (Fast-DDS)  

Optionally add:  
[Box]  Real Robot HW Interface  
  ↳ /franka_joint_commands  

(Arrow tails --> arrow heads indicate topic publishers versus subscribers.)

────────────────────────────────────────────────────────
8.  Further reading / references
────────────────────────────────────────────────────────

• MoveIt-Isaac tutorial (primary reference)  
  https://moveit.picknik.ai/main/doc/how_to_guides/isaac_panda/isaac_panda_tutorial.html  

• OmniGraph basics  
  https://docs.omniverse.nvidia.com/isaacsim/latest/gui_tutorials/tutorial_gui_omnigraph.html  

• Native ROS bridge installation  
  https://docs.omniverse.nvidia.com/isaacsim/latest/installation/install_ros.html#running-native-ros  

• ros2_control hardware interfaces  
  https://control.ros.org/rolling/index.html  

• Hardware-access design draft  
  https://github.com/ros-controls/roadmap/blob/master/design_drafts/hardware_access.md  

• Camera, TF, navigation, multi-robot Isaac tutorials (see links you provided).  

────────────────────────────────────────────────────────

With these notes you should be able to ① explain each moving part to a newcomer, ② adapt the project to any robot, and ③ extend it toward custom planners or real-robot execution.  When you convert section 7 into Draw.io, each “Box” becomes a rounded rectangle and each topic a labelled arrow.










# More references:
##  omnigraph:

1. https://docs.omniverse.nvidia.com/extensions/latest/ext_omnigraph/tutorials/gentle_intro.html

2. https://docs.omniverse.nvidia.com/extensions/latest/ext_omnigraph.html

3. https://docs.isaacsim.omniverse.nvidia.com/latest/ros2_tutorials/tutorial_ros2_custom_omnigraph_node_python.html#creating-the-ros-2-custom-omnigraph-short-python-node-template

# Handy tips:
## 1. open action graph:

In isaacsim GUI: 
1. Click :
    ├─ Window
    ├─── Extensions
    
2. In ├─── Extensions, search the word "action" in the search field

3. Then toggle to "ENABLED" (green) next 4:
    1. OMNIVERSE ACTION GRAPH
    2. OMNIVERSE ACTION GRAPH NODESS
    3. OMNIVERSE GRAPH BUNDLE
    4. OMNIVERSE ACTION GRAPH EDITOR

4. Close the ├─── Extensions bar.
5. Click:
    ├─ Window
    ├─── Graph Editors
    ├───── Action Graph

6. Now you are supposed to see the "Action Graph" slide at the buttom of the simulator, next to "Isaac sim assets"

7. click "edit action graph" to see the graph.
Illustration:





# Tutorial (by sonnet 4 with 'thinking')



I'll analyze the action graph created in the `isaac_moveit.py` file to explain its structure and functionality.

Read file: doc/how_to_guides/isaac_panda/launch/isaac_moveit.py

Read file: doc/how_to_guides/isaac_panda/launch/isaac_moveit.py
Based on my analysis of the `isaac_moveit.py` file, I can explain the action graph structure and functionality. The action graph is Isaac Sim's visual programming system that connects different nodes to create a data flow pipeline for ROS 2 integration with the Franka Panda robot simulation.

## Action Graph Overview

The action graph in this file creates a comprehensive ROS 2 bridge that enables:
1. **Robot Control**: Bidirectional communication for joint commands and states
2. **Camera Simulation**: Publishing RGB, depth images, and camera info
3. **Time Synchronization**: Publishing simulation clock to ROS 2
4. **Viewport Management**: Creating and managing camera viewports

## Node Structure and Data Flow

### 1. **Trigger Nodes (Execution Control)**
- **`OnImpulseEvent`**: Main trigger that fires every simulation frame
- **`OnTick`**: Secondary trigger for viewport/camera operations

### 2. **ROS 2 Context and Time**
- **`Context`**: Initializes ROS 2 context with domain ID
- **`ReadSimTime`**: Reads Isaac Sim's simulation time
- **`PublishClock`**: Publishes `/clock` topic for time synchronization

### 3. **Robot Control Pipeline**
```
SubscribeJointState → ArticulationController → Robot USD Prim
                ↓
            PublishJointState → /isaac_joint_states
```

**Key Nodes:**
- **`SubscribeJointState`**: Listens to `/isaac_joint_commands` topic
- **`ArticulationController`**: Applies joint commands to the Franka robot USD prim
- **`PublishJointState`**: Publishes current joint states to `/isaac_joint_states`

### 4. **Camera Pipeline**
```
OnTick → createViewport → getRenderProduct → setCamera → Camera Helpers
```

**Camera Nodes:**
- **`createViewport`**: Creates a second viewport for the RealSense camera
- **`getRenderProduct`**: Gets render product from the viewport
- **`setCamera`**: Associates the RealSense camera with the render product
- **`cameraHelperRgb`**: Publishes RGB images to `/rgb`
- **`cameraHelperInfo`**: Publishes camera info to `/camera_info`
- **`cameraHelperDepth`**: Publishes depth images to `/depth`

## Data Flow Connections

### Execution Flow:
1. **`OnImpulseEvent`** triggers every frame and executes:
   - Joint state publishing
   - Joint command subscription
   - Clock publishing
   - Articulation control

2. **`OnTick`** triggers viewport and camera operations

### Data Connections:
- **ROS Context**: Shared across all ROS 2 nodes for proper initialization
- **Simulation Time**: Fed to both joint state and clock publishers
- **Joint Commands**: Flow from ROS subscription to articulation controller
- **Render Products**: Shared between camera helpers for image publishing

## Configuration Parameters

The graph is configured with specific values:
```python
("Context.inputs:domain_id", ros_domain_id)  # ROS domain
("ArticulationController.inputs:robotPath", "/Franka")  # Robot path
("PublishJointState.inputs:topicName", "isaac_joint_states")
("SubscribeJointState.inputs:topicName", "isaac_joint_commands")
("cameraHelperRgb.inputs:topicName", "rgb")  # Camera topics
```

## Execution Loop

The main simulation loop continuously:
1. Steps the physics simulation
2. Triggers the action graph via `OnImpulseEvent.state:enableImpulse`
3. Processes all node connections and data flow

This action graph essentially creates a complete ROS 2 interface for the Isaac Sim Franka robot, enabling MoveIt 2 to control the simulated robot while receiving sensor feedback and maintaining proper time synchronization.
