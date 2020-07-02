# Readme for ddk sim launch.

## Fetch robot 2d frontier exploration demo
required packages:
nav2d, fetch_gazebo

1. ```roslaunch ddk_sim_launch my_fetch_nav.launch```
2. ```roslaunch ddk_sim_launch my_fetch_test.launch```
3. ```rosservice call /StartMapping```
4. ```rosservice call /StartExploration```

## Dragon DDK frontier exploration
1. cd ddk_sim_launch/scripts
2. ./demo_ddk.sh
3. Wait for several seconds, motors on and take off.
4. Change to the third window, hit "ENTER" to start the command: rosservice call /StartMapping
5. Wait until the robot finish a 360 degree turn.
6. Switch to the bottom tab, hit "ENTER" to start the command: rosservice call /StartExploration
