# Grasp service
![humble](https://github.com/Wiktor-99/ros2_grasp_service/actions/workflows/build_and_test_humble.yaml/badge.svg)

Simple ros2 grasp service. The job of the service is to follow sequence of points, then close the gripper and stop and then back to home positions and open gripper.


# Configuration
Configuration is straightforward and example is available in the open_manipulator_bringup package in the config folder. Simple configuration looks like this

```yaml
grasp_service:
  ros__parameters:
    joints_controller_name: joint_trajectory_controller
    joints_names:
      - joint1
      - joint2
      - joint3
      - joint4

    positions_to_target_list:
      - first_pos
      - second_pos

    positions_to_target:
      first_pos:
        positions: [0.50, 0.0, 0.0, 0.0]
        time_to_target: 2
      second_pos:
        positions: [-0.50, 0.0, 0.0, 0.0]
        time_to_target: 4

    positions_to_home_list:
      - first_pos

    positions_to_home:
      first_pos:
        positions: [0.50, 0.0, 0.0, 0.0]
        time_to_target: 2

    time_to_wait_on_target: 5

    gripper_controller_name: gripper_action_controller
    gripper_close: -0.01
    gripper_open: 0.019
```

# Demo
To launch demo just build (and source) repo and use following commands:

```bash
ros2 launch manipulator_simulation manipulator_simulation_bringup.launch.py
```
And to trigger grasp_service just call:

```bash
ros2 service call grasp_service std_srvs/srv/Empty
```

# Setup
You can clone it and run in docker devcontainer or use native setup. Docker container is setup with all dependencies.

To setup with docker devcontainer:

```bash
./build.sh &&
source install/setup.bash
```

To setup it without docker:

```bash
mkdir -p ~/ros_ws/ && cd ~/ros_ws/src
git clone https://github.com/Wiktor-99/ros2_grasp_service.git
cd ..
rosdep install --from-paths src --ignore-src -r -y
./build.sh
. install/setup.bash
```
