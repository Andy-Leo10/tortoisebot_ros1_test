# tortoisebot_ros1_test

## Simulation
**start**
```shell
source ~/simulation_ws/devel/setup.bash
roslaunch tortoisebot_gazebo tortoisebot_playground.launch
```

**reset**
```shell
rosservice call /gazebo/reset_world "{}"
```

## Action
**compile**
```shell
cd simulation_ws/;catkin_make;source devel/setup.bash
```

**start server**
```shell
source ~/simulation_ws/devel/setup.bash
rosrun tortoisebot_waypoints tortoisebot_action_server.py
```

**run client**
```shell
source ~/simulation_ws/devel/setup.bash
rosrun tortoisebot_waypoints exp_action_client.py
```
