#!/bin/bash

SESSION=base

# Set this variable in order to have a development workspace sourced, surplus/instead of the .bashrc one
DEVELOPMENT_WS=/home/strands/muc_ws/devel/setup.bash
_SRC_ENV="tmux send-keys source Space $DEVELOPMENT_WS C-m "

tmux -2 new-session -d -s $SESSION
tmux new-window -t $SESSION:0 -n 'roscore'
tmux new-window -t $SESSION:1 -n 'core'
tmux new-window -t $SESSION:2 -n 'robot'
tmux new-window -t $SESSION:3 -n 'ui'
tmux new-window -t $SESSION:4 -n 'navigation'
tmux new-window -t $SESSION:5 -n 'exec'
tmux new-window -t $SESSION:6 -n 'patrol'
tmux new-window -t $SESSION:7 -n 'foo'
tmux new-window -t $SESSION:8 -n 'bar'

tmux select-window -t $SESSION:0
tmux split-window -v
tmux select-pane -t 0
tmux send-keys "roscore" C-m
tmux resize-pane -U 30
tmux select-pane -t 1
tmux send-keys "htop" C-m

tmux select-window -t $SESSION:1
[ -f $DEVELOPMENT_WS ] && `$_SRC_ENV`
tmux send-keys "ssh bobl" C-m
tmux send-keys "roslaunch strands_bringup strands_core.launch db_path:=/data/muc"

tmux select-window -t $SESSION:2
[ -f $DEVELOPMENT_WS ] && `$_SRC_ENV`
tmux send-keys "roslaunch strands_bringup strands_robot.launch with_magnetic_barrier:=false"

tmux select-window -t $SESSION:3
[ -f $DEVELOPMENT_WS ] && `$_SRC_ENV`
tmux send-keys "roslaunch strands_bringup strands_ui.launch"

tmux select-window -t $SESSION:4
[ -f $DEVELOPMENT_WS ] && `$_SRC_ENV`
tmux send-keys "roslaunch strands_bringup strands_navigation.launch map:=/home/strands/muc_ws/maps/muc16/cropped.yaml  topological_map:=muc16"

tmux select-window -t $SESSION:5
[ -f $DEVELOPMENT_WS ] && `$_SRC_ENV`
tmux split-window -v
tmux select-pane -t 0
tmux send-keys "roslaunch task_executor mdp-executor.launch"
tmux select-pane -t 1
tmux send-keys "rosrun task_executer schedule_status.py" 

tmux select-window -t $SESSION:6
[ -f $DEVELOPMENT_WS ] && `$_SRC_ENV`
tmux send-keys "rosrun task_executor continuous_patrolling.py"

# Set default window
tmux select-window -t $SESSION:0

# Attach to session
tmux -2 attach-session -t $SESSION
