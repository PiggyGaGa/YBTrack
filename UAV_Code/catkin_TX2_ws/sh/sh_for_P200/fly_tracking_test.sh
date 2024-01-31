#!/bin/bash
gnome-terminal --window -e 'bash -c "roscore; exec bash"' \
--tab -e 'bash -c "sleep 5; rosrun fv_tracking web_cam; exec bash"' \
--tab -e 'bash -c "sleep 5; rosrun fv_tracking tracker_kcf; exec bash"' \
--tab -e 'bash -c "sleep 5; rosrun serial_ros serial_node; exec bash"' \
--tab -e 'bash -c "sleep 5; rosrun serial_ros moni; exec bash"' \
--tab -e 'bash -c "sleep 5; roslaunch mavros px4.launch fcu_url:="/dev/ttyTHS2:921600"; exec bash"' \
--tab -e 'bash -c "sleep 4; roslaunch px4_command px4_pos_controller.launch; exec bash"' \
--tab -e 'bash -c "sleep 4; roslaunch px4_command target_tracking.launch; exec bash"' \
--tab -e 'bash -c "sleep 5; rosrun px4_command set_mode; exec bash"' \
--tab -e 'bash -c "sleep 5; roslaunch rplidar_ros rplidar.launch; exec bash"' \
--tab -e 'bash -c "sleep 5; roslaunch cartographer_ros my_lidar_imu.launch; exec bash"' \
--tab -e 'bash -c "sleep 5; roslaunch realsense2_camera rs_t265.launch; exec bash"' \
--tab -e 'bash -c "sleep 5; roslaunch vision_to_mavros t265_tf_to_mavros.launch; exec bash"' \


