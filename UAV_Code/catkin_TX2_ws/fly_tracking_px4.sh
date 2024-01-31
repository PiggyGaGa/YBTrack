#!/bin/bash
source devel/setup.bash
gnome-terminal --window -e 'bash -c "roscore; exec bash"' \
--tab -e 'bash -c "sleep 5; rosrun track_detection cam_node; exec bash"' \
--tab -e 'bash -c "sleep 5; rosrun track_detection tensorrt_track_node; exec bash"' \
--tab -e 'bash -c "sleep 5; rosrun serial_ros moni; exec bash"' \
--tab -e 'bash -c "sleep 5; rosrun serial_ros serial_node; exec bash"' \
--tab -e 'bash -c "sleep 5; roslaunch mavros px4.launch fcu_url:="/dev/ttyTHS2:921600"; exec bash"' \
--tab -e 'bash -c "sleep 4; roslaunch px4_command px4_pos_controller.launch; exec bash"' \
--tab -e 'bash -c "sleep 4; roslaunch px4_command target_tracking.launch; exec bash"' \
--tab -e 'bash -c "sleep 5; rosrun px4_command set_mode; exec bash"' \


