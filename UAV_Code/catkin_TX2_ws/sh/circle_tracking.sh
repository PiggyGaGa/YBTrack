#!/bin/bash

##激光雷达自主飞行脚本


gnome-terminal --window -e 'bash -c "source /opt/ros/melodic/setup.bash; roslaunch mavros px4.launch fcu_url:="/dev/ttyTHS2:921600"; exec bash"' \
--tab -e 'bash -c "sleep 5;source /home/amov/ws_rplidar/devel/setup.bash;roslaunch rplidar_ros rplidar.launch; exec bash"' \
--tab -e 'bash -c "sleep 5;source  ~/px4_ws/devel/setup.bash; roslaunch px4_command tfmini.launch; exec bash"' \
--tab -e 'bash -c "sleep 5;source  ~/px4_ws/devel/setup.bash; roslaunch px4_command px4_pos_estimator.launch; exec bash"' \
--tab -e 'bash -c "sleep 5;source  ~/px4_ws/devel/setup.bash; roslaunch px4_command px4_pos_controller.launch; exec bash"' \
--tab -e 'bash -c "sleep 5;source  ~/px4_ws/devel/setup.bash; rosrun px4_command move; exec bash"' \
--tab -e 'bash -c "sleep 10; source /home/amov/catkin_ws/install_isolated/setup.bash;roslaunch cartographer_ros rplidar.launch; exec bash"' \
--tab -e 'bash -c "sleep 2; rosrun web_cam web_cam; exec bash"' \
--tab -e 'bash -c "sleep 2;source /home/nvidia/vision_ws/devel/setup.bash; roslaunch ellipse_det ellipse_det.launch; exec bash"' \
--tab -e 'bash -c "sleep 2;source  ~/px4_ws/devel/setup.bash; roslaunch px4_command target_tracking.launch; exec bash"' \

