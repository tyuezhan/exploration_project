# Readme for comcast launch.

## Fetch robot 2d frontier exploration demo
required packages:
nav2d, fetch_gazebo

1. ```roslaunch comcast_launch my_fetch_nav.launch```
2. ```roslaunch comcast_launch my_fetch_test.launch```

## Dragon DDK frontier exploration
1. ```roslaunch comcast_launch my_ddk_gazebo.launch```
2. ```roslaunch mrsl_quadrotor_launch spawn.launch mav_type:=dragon_ddk mav_name:=ddk```
3. ```roslaunch mrsl_quadrotor_launch controller.launch mav_type:=dragon_ddk mav_name:=ddk mass:=0.25 odom_topic:=ground_truth/odom```
4. ```roslaunch comcast_launch octomap_mapping.launch```
5. ```roslaunch comcast_launch ddk_sim_tf_pub.launch```
6. ```roslaunch comcast_launch my_ddk_nav.launch```
7. ```rosrun rqt_mav_manager rqt_mav_manager```
Then, motors on and take off.
8. ```rosservice call /StartMapping```
After the robot got straight and turn around
9. ```rosservice call /StartExploration```