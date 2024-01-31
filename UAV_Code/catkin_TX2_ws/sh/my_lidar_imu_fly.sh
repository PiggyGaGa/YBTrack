##激光雷达自主飞行脚本
gnome-terminal --tab -e 'bash -c " roslaunch rplidar_ros rplidar.launch; exec bash"' \
--tab -e 'bash -c "sleep 5; roslaunch mavros px4.launch fcu_url:="/dev/ttyTHS2:921600"; exec bash"' \
--tab -e 'bash -c "sleep 4; roslaunch px4_command px4_pos_estimator.launch; exec bash"' \
--tab -e 'bash -c "sleep 3; roslaunch px4_command px4_pos_controller.launch; exec bash"' \
--tab -e 'bash -c "sleep 5; roslaunch px4_command tfmini.launch; exec bash"' \
--tab -e 'bash -c "sleep 5; roslaunch cartographer_ros my_lidar_imu.launch; exec bash"' \
--tab -e 'bash -c "sleep 2; rosrun px4_command move; exec bash"' \

