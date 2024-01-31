#!/bin/bash
source devel/setup.bash
gnome-terminal --window -e 'bash -c "roscore; exec bash"' \
--tab -e 'bash -c "sleep 5; rosrun track_detection cam_node; exec bash"' \
--tab -e 'bash -c "sleep 5; rosrun track_detection tensorrt_track_node; exec bash"' \
--tab -e 'bash -c "sleep 5; rosrun serial_ros moni; exec bash"' \
--tab -e 'bash -c "sleep 5; rosrun serial_ros serial_node; exec bash"' 


