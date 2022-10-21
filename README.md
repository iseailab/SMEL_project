# Composition of all assignments | Ozodjon Kunishev
<br/>

## 1. Turtlesim and RQT installation
<br/>

```
sudo apt update
sudo apt install ros-humble-turtlesim
```
```
ros2 pkg executables turtlesim
```
```
ros2 run turtlesim turtlesim_node
```

<br/>

Output:
```
[INFO] [turtlesim]: Starting turtlesim with node name /turtlesim

[INFO] [turtlesim]: Spawning turtle [turtle1] at x=[5.544445], y=[5.544445], theta=[0.000000]
```
<br/>

```
ros2 run turtlesim turtle_teleop_key
```
![turtle](https://user-images.githubusercontent.com/90167023/192159988-9bac1b27-1dcf-48fe-9315-121852ebf928.png)
<br/>

## 1.2 Install RQT
<br/>

```
sudo apt update
sudo apt install ~nros-humble-rqt*
```
```
rqt
```
```
ros2 run turtlesim turtle_teleop_key --ros-args --remap turtle1/cmd_vel:=turtle2/cmd_vel
```

![turtle2](https://user-images.githubusercontent.com/90167023/192160718-2a46367c-b9af-4a32-9eaf-dd4784b95bdc.png)
<br/>
<br/>
<br/>

## 2. Writing a simple publisher and subscriber
<br/>

```
ros2 pkg create --build-type ament_python py_pubsub
```
```
wget https://raw.githubusercontent.com/ros2/examples/humble/rclpy/topics/minimal_publisher/examples_rclpy_minimal_publisher/publisher_member_function.py
```
<br/>

Some changes are made to the created files, then:
```
wget https://raw.githubusercontent.com/ros2/examples/humble/rclpy/topics/minimal_subscriber/examples_rclpy_minimal_subscriber/subscriber_member_function.py
```
```
rosdep install -i --from-path src --rosdistro humble -y
```
```
. install/setup.bash
```
```
ros2 run py_pubsub talker
```
![image](https://user-images.githubusercontent.com/90167023/197249185-9dba8ed8-d13e-4092-9d6d-155ac744cdf9.png)
```
ros2 run py_pubsub listener
```
![image](https://user-images.githubusercontent.com/90167023/197249256-37c93ffb-af29-4d40-97ff-7ff0342ef09c.png)

<br/>
<br/>
<br/>

## 3. Writing a simple service and client
<br/>

Relevant files are opened and adjusted according to the official website. Some new files will also be created. Then:
```
rosdep install -i --from-path src --rosdistro humble -y
```
```
colcon build --packages-select py_srvcli
```
In a new terminal:
```
. install/setup.bash
```
```
ros2 run py_srvcli service
```
In the third terminal:
```
ros2 run py_srvcli client 2 3
```
Output:
![client](https://user-images.githubusercontent.com/90167023/197254478-bb7b518e-52ac-4250-9deb-9d00008de0bf.png)
<br/>

![service](https://user-images.githubusercontent.com/90167023/197254578-42dab3ed-035d-4446-b879-149db318fe87.png)
<br/>
<br/>
<br/>

## 4. Creating custom msg and srv files
<br/>

```
ros2 pkg create --build-type ament_cmake tutorial_interfaces
```
```
mkdir msg
mkdir srv
```
Several files will be created and edited. Then:
```
colcon build --packages-select tutorial_interfaces
```
In a new terminal:
```
. install/setup.bash
```
```
ros2 interface show tutorial_interfaces/msg/Num
```
```
ros2 interface show tutorial_interfaces/msg/Sphere
```
```
ros2 interface show tutorial_interfaces/srv/AddThreeInts
```
After that changes are made to the publisher and subscriber files. After all the changes:
```
colcon build --packages-select cpp_pubsub
```
Two new terminals will be opened and sourced. Then:
```
ros2 run py_pubsub talker
```

```
ros2 run py_pubsub listener
```
![image](https://user-images.githubusercontent.com/90167023/197255816-dee0b0ad-173b-4fc0-ab05-f6f987482579.png)

<br/>
<br/>
<br/>

## 5. Action server and client
<br/>

This includes creating two python files for action.:
```
gedit fibonacci_action_server.py
```
```
gedit fibonacci_action_client.py
```
After making changes, we run them using python3 in separate terminals:
```
python3 fibonacci_action_server.py
```
![action clent-serv](https://user-images.githubusercontent.com/90167023/197257906-2a55da82-1caa-4e20-b5cc-b88bcefac4df.png)
```
python3 fibonacci_action_client.py
```
![action clent-serv1](https://user-images.githubusercontent.com/90167023/197257965-3972d1ed-f7b1-42d8-982a-344238e00999.png)

<br/>
<br/>
<br/>

## 6. Composing multiple nodes in a single process
<br/>

```
ros2 component types
```
```
ros2 run rclcpp_components component_container
```
```
ros2 component list
```
```
ros2 component load /ComponentManager composition composition::Talker
```
```
ros2 component load /ComponentManager composition composition::Listener
```
```
ros2 component load /ComponentManager composition composition::Server
ros2 component load /ComponentManager composition composition::Client
```
```
ros2 run composition manual_composition
```
```
ros2 component unload /ComponentManager 1 2
```
```
ros2 run rclcpp_components component_container --ros-args -r __node:=MyContainer -r __ns:=/ns
```
```
ros2 component load /ComponentManager composition composition::Talker --node-name talker2
```
```
ros2 component load /ComponentManager composition composition::Talker --node-namespace /ns
```
```
ros2 component load /ComponentManager composition composition::Talker --node-name talker3 --node-namespace /ns2
```
```
ros2 component load /ComponentManager image_tools image_tools::Cam2Image -p burger_mode:=true
```
```
ros2 component load /ComponentManager composition composition::Talker -e use_intra_process_comms:=true
```
![image](https://user-images.githubusercontent.com/90167023/197258665-fa5350af-bc56-48cc-9f25-e61009f58493.png)
<br/>

![image](https://user-images.githubusercontent.com/90167023/197258712-faf95373-b0f9-4c61-a1d0-d815c23fb162.png)

<br/>
<br/>
<br/>

## 7. Launch
<br/>

### 7.1 Creating a launch file
<br/>

```
mkdir launch
```
Creating new files will be included in this task.
```
cd launch
ros2 launch turtlesim_mimic_launch.py
```
```
ros2 topic pub -r 1 /turtlesim1/turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -1.8}}"
```
```
rqt_graph
```
![image](https://user-images.githubusercontent.com/90167023/197259487-48af49da-ff19-4eaa-ac3d-b80af622f310.png)

<br/>
<br/>
<br/>

### 7.2 Integrating launch files into ROS 2 packages
<br/>



```
mkdir -p launch_ws/src
cd launch_ws/src
```
```
ros2 pkg create py_launch_example --build-type ament_python
```
Several changes are made to the auto-created files inside the package
```
colcon build
```
```
ros2 launch py_launch_example my_script_launch.py
```
![image](https://user-images.githubusercontent.com/90167023/197259794-6a28f374-aa8c-4b1b-88a8-4fcc0619c864.png)

<br/>
<br/>
<br/>

### 7.3 Using substitutions
<br/>

```
ros2 pkg create launch_tutorial --build-type ament_python
```
```
mkdir launch_tutorial/launch
```
After creating new files and making changes to some of the existing ones:
```
colcon build
```
```
ros2 launch launch_tutorial example_main.launch.py
```
```
ros2 launch launch_tutorial example_substitutions.launch.py --show-args
```
```
ros2 launch launch_tutorial example_substitutions.launch.py turtlesim_ns:='turtlesim3' use_provided_red:='True' new_background_r:=200
```
![image](https://user-images.githubusercontent.com/90167023/197260311-a49a99bc-a8b6-42de-9d62-0a3d9e9c154c.png)


<br/>
<br/>
<br/>

## 8. Introducing tf2
<br/>


```
sudo apt-get install ros-humble-turtle-tf2-py ros-humble-tf2-tools ros-humble-tf-transformations
```
```
ros2 launch turtle_tf2_py turtle_tf2_demo.launch.py
```
```
ros2 run turtlesim turtle_teleop_key
```
```
ros2 run tf2_tools view_frames
```
```
ros2 run tf2_ros tf2_echo [reference_frame] [target_frame]
```
```
ros2 run tf2_ros tf2_echo turtle2 turtle1
```
![image](https://user-images.githubusercontent.com/90167023/197260663-4463df3b-e8bf-4058-99d5-0aed67d355b3.png)

```

```
```

```



