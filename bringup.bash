#!/bin/sh

# Set Session Name
SESSION=$hostname
SESSIONEXISTS=$(tmux list-sessions | grep $SESSION)

if [ "$SESSIONEXISTS" = "" ]
then
     # Start New Session with our name
    tmux new-session -d -s $SESSION

	tmux split-window -v

	tmux select-pane -t 0
    tmux send-keys 'docker compose up'
    	tmux select-pane -t 1
    tmux send-keys 'pushd orchestration; bash join.bash; roslaunch mocha_launch jackal.launch robot_name:=phobos'
    	tmux select-pane -t 0
fi 

tmux attach-session -t $SESSION:0
