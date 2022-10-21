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
