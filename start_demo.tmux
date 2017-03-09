#!/bin/bash

SESSION=demo

# Set this variable in order to have a development workspace sourced, surplus/instead of the .bashrc one
DEVELOPMENT_WS=/home/strands/muc_ws/devel/setup.bash
_SRC_ENV="tmux send-keys source Space $DEVELOPMENT_WS C-m "

tmux -2 new-session -d -s $SESSION
tmux new-window -t $SESSION:0 -n 'cam'
tmux new-window -t $SESSION:1 -n 'soma'
tmux new-window -t $SESSION:2 -n 'semmap'
tmux new-window -t $SESSION:3 -n 'people'
tmux new-window -t $SESSION:4 -n 'intrud'
tmux new-window -t $SESSION:5 -n 'change'
tmux new-window -t $SESSION:6 -n 'rtalk'
tmux new-window -t $SESSION:7 -n 'foo'
tmux new-window -t $SESSION:8 -n 'bar'

tmux select-window -t $SESSION:0
tmux send-keys "roslaunch openni2_launch openni2.launch camera:=head_xtion" 

tmux select-window -t $SESSION:1
[ -f $DEVELOPMENT_WS ] && `$_SRC_ENV`
tmux split-window -v
tmux select-pane -t 0
tmux send-keys "roslaunch soma_manager soma2_local.launch map_name:=muc16"
tmux select-pane -t 1
tmux send-keys "rosrun soma_roi_manager soma_roi.py muc16"

tmux select-window -t $SESSION:2
[ -f $DEVELOPMENT_WS ] && `$_SRC_ENV`
tmux split-window -v
tmux select-pane -t 0
tmux send-keys "roslaunch semantic_map_launcher semantic_map.launch max_instances:=1000"
tmux select-pane -t 1
tmux send-keys "roslaunch observation_registration_launcher observation_registration.launch"

tmux select-window -t $SESSION:3
[ -f $DEVELOPMENT_WS ] && `$_SRC_ENV`
tmux send-keys "roslaunch perception_people_launch people_tracker_robot.launch"

tmux select-window -t $SESSION:4
[ -f $DEVELOPMENT_WS ] && `$_SRC_ENV`
tmux send-keys "rosrun intrusion_detection_people intrusion_detector.py"

tmux select-window -t $SESSION:5
[ -f $DEVELOPMENT_WS ] && `$_SRC_ENV`
tmux split-window -v
tmux select-pane -t 0
tmux send-keys "rosrun change_detection_objects change_detector.py"
tmux select-pane -t 1
tmux send-keys "rosrun change_detection_objects demand_sweep.py WayPoint1"

tmux select-window -t $SESSION:6
[ -f $DEVELOPMENT_WS ] && `$_SRC_ENV`
tmux send-keys "rosrun robot_talk rtalk.py list"

# Set default window
tmux select-window -t $SESSION:0

# Attach to session
tmux -2 attach-session -t $SESSION
