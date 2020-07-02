#!/bin/bash

MAV_TYPE=dragon_ddk
MAV_NAME=ddk
WORLD_FRAME_ID=world
ODOM_TOPIC=ground_truth/odom

echo "MAV name: $MAV_NAME MAV Type: $MAV_TYPE"

MASTER_URI=http://localhost:11311
SETUP_ROS_STRING="export ROS_MASTER_URI=${MASTER_URI}"
SESSION_NAME=demo_ddk

CURRENT_DISPLAY=${DISPLAY}
if [ -z ${DISPLAY} ];
then
  echo "DISPLAY is not set"
  CURRENT_DISPLAY=:0
fi

if [ -z ${TMUX} ];
then
  TMUX= tmux new-session -s $SESSION_NAME -d
  echo "Starting new session."
else
  echo "Already in tmux, leave it first."
  exit
fi

# Make mouse useful in copy mode
tmux setw -g mouse on

tmux rename-window -t $SESSION_NAME "Main"
tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; export DISPLAY=${CURRENT_DISPLAY}; roslaunch comcast_launch my_ddk_gazebo.launch" Enter
tmux split-window -t $SESSION_NAME
tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; sleep 3; export DISPLAY=${CURRENT_DISPLAY}; roslaunch mrsl_quadrotor_launch spawn.launch mav_type:=${MAV_TYPE} mav_name:=${MAV_NAME}" Enter
tmux split-window -t $SESSION_NAME
tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; sleep 4; export DISPLAY=${CURRENT_DISPLAY}; roslaunch mrsl_quadrotor_launch controller.launch mav_type:=${MAV_TYPE} mav_name:=${MAV_NAME} odom_topic:=${ODOM_TOPIC} mass:=0.25" Enter
tmux split-window -t $SESSION_NAME
tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; sleep 4; export DISPLAY=${CURRENT_DISPLAY}; rosrun rqt_mav_manager rqt_mav_manager" Enter
tmux select-layout -t $SESSION_NAME tiled

tmux new-window -t $SESSION_NAME
tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; sleep 4; roslaunch comcast_launch octomap_mapping.launch" Enter
tmux split-window -t $SESSION_NAME
tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; sleep 4; roslaunch comcast_launch ddk_sim_tf_pub.launch" Enter
tmux split-window -t $SESSION_NAME
tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; sleep 4; roslaunch comcast_launch my_ddk_nav.launch" Enter
tmux select-layout -t $SESSION_NAME even-horizontal

tmux new-window -t $SESSION_NAME
tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; rosservice call /StartMapping"
tmux split-window -t $SESSION_NAME
tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; rosservice call /StartExploration"


# Add window to easily kill all processes
tmux new-window -t $SESSION_NAME -n "Kill"
tmux send-keys -t $SESSION_NAME "tmux kill-session -t ${SESSION_NAME}"

tmux select-window -t $SESSION_NAME:0
tmux -2 attach-session -t $SESSION_NAME