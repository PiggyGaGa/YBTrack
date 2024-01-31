#!/bin/bash
gnome-terminal --window -e 'bash -c "roscore; exec bash"' \
--tab -e 'bash -c "sleep 5; rosrun fv_tracking web_cam; exec bash"' \
--tab -e 'bash -c "sleep 5; rosrun fv_tracking tracker_kcf; exec bash"' \
--tab -e 'bash -c "sleep 5; rosrun serial_ros serial_node; exec bash"' \
--tab -e 'bash -c "sleep 5; rosrun serial_ros moni; exec bash"' \

