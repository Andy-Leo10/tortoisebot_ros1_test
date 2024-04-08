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

## TEST
build
```shell
cd simulation_ws/;catkin_make;source devel/setup.bash
```
run test
```shell
cd simulation_ws/;catkin_make run_tests
```
launch test
```
rostest --text tortoisebot_waypoints waypoints_test.test --reuse-master
```
check log
```shell
vim /home/user/.ros/test_results/tortoisebot_waypoints/rostest-test_waypoints_test.xml
```
clean tests
```shell
rm -r /home/user/simulation_ws/build/test_results/tortoisebot_waypoints
```

## RESULTS
> [!NOTE] test-pass:
tests pass correctly if the goal is reached before and before timeout
![test-pass](pictures/ros1_pass.png)

> [!CAUTION] test-fail:
tests don't pass correctly if the goal is not reached or due to timeout
![test-fail](pictures/ros1_fail.png)
