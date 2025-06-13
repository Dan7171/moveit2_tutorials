# Summary of omni graph nodes

## Type of input/outputs/nodes in graphs:

### Nodes:
- On Playback Tick 
node to execute other graph nodes every simulation frame.
This is a form of a family of nodes called  "event nodes" and its very common.

- Custom Python ROS 2 Node
custom node.

- To String 
node to convert the output of our custom node to a string.

- Print Text 
node to display the output of our custom node (as string) to the viewport or terminal. Edit this node’s properties, in the Property panel, and check the To Screen attribute to display the text in the viewport.


## Source:

https://docs.isaacsim.omniverse.nvidia.com/latest/ros2_tutorials/tutorial_ros2_custom_omnigraph_node_python.html#creating-the-ros-2-custom-omnigraph-short-python-node-template


https://docs.isaacsim.omniverse.nvidia.com/latest/ros2_tutorials/tutorial_ros2_drive_turtlebot.html

https://docs.isaacsim.omniverse.nvidia.com/4.5.0/omnigraph/omnigraph_tutorial.html#building-the-graph

https://docs.omniverse.nvidia.com/extensions/latest/ext_omnigraph.html

https://www.youtube.com/results?search_query=isaac+sim+omnigraph





# Examples: graphs explained:
## image1.png:
- Was taken from this video (good tutorial): https://docs.omniverse.nvidia.com/extensions/latest/ext_omnigraph.html

- What do we see in the image?
1. The action graph rotates a cube (which is a prim).
2. Nodes: 

- On tick: triggers the node its sending input to every simulation step. Triggered by the simulation step. 

- Read prim attributes: reads the attributes (states) of a specific prim (in this case, the cube).
In this example, it selected a specific attribute: the rotation euler xyz (in radians I guess).



- Costant float: outputs a constat (in this case, 1)


- Break 3 vector: splits a vector in R3 to x,y,z (so you can use a specific index of it). In the exampele, it takes input from the read prim node. We read the cube state.


- Add: just adding two numbers.

- Make 3 vector: used to recompose the old vector we "broke", + change in the y value (we added 1 using the add node with constant 1)



- Write prim attributes: writes the attributes (states) of a specific prim (in this case, the cube).
It's used here to re-write the pose of the cube.
It rotates the cube around the 1 axis by 1 unit of angle (I guess 1 radians) every time step (every simulation step).









