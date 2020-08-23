#! /bin/bash

tmux new-session -d
tmux split-window -h 'docker run -ti --rm -p 1883:1883 -p 9001:9001 toke/mosquitto'
tmux split-window -v 'sleep 3; docker run -it --rm toke/mosquitto mosquitto_sub -h 192.168.1.8 -t sensor/#'

tmux attach-session -d
