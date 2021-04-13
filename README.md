mc_rtc_ros_control
==

Integration package between mc_rtc and ROS control. Provides a node subscribe to a robot's JointState and publishing a message suitable for a position_controllers/JointGroupPositionController controller.

Requirements
--

Build as a catkin package. This has the following required dependencies:

- roscpp
- std_msgs
- sensor_msgs
- [mc_rtc](https://github.com/jrl-umi3218/mc_rtc)

Usage
--

```bash
roslaunch mc_rtc_ros_control control.launch publish_to:=/my/command subscribe_to:=/my/state
```

Where:

- `publish_to` is the topic where the controller is subscribed to a control message (defaults to: `/command`)
- `subscribe_to` is the topic where the robot is publishing its state through a `sensor_msgs/JointState` message (defaults to: `/joint_state`)
