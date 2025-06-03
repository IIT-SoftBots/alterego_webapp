#!/bin/bash
export DISPLAY=:0
notify-send "AlterEgo WebApp is starting. Please wait..." # Show a message while waiting for system to be sure network is ready
source ~/.bashrc sleep 2 # Wait for system to fully initialize
node ~/catkin_ws/src/AlterEGO_v2/alterego_webapp/src/webapp.js & sleep 5 # Wait for webapp to start 
gtk-launch webapp
