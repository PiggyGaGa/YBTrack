#include <ros/ros.h>
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include <image_transport/image_transport.h>
#include "time.h"
#include "fstream"
#include <sensor_msgs/Imu.h>

using namespace std;


//回调函数 
void write_callback(const serial_ros::Cloud_platform::ConstPtr& msg) 
{ 
    // const std::vector<unsigned char>& 
    unsigned char *com;
    *com = msg->get_data_com[0];
    *(com+1) = msg->get_data_com[1];
    *(com+2) = msg->get_data_com[2];
    *(com+3) = msg->get_data_com[3];
    *(com+4) = msg->get_data_com[4];
    
    cout<<"com_0  "<<com[0]<<"  com_1  "<<com[1]<<"  com_2  "<<com[2]<<"  com_3  "<<com[3]<<"  com_4  "<<com[4]<<std::endl; 
    // ROS_INFO_STREAM("Writing to serial port" << msg->get_data_com); 
    // ser.write(msg->get_data_com,5);   //发送串口数据 
    ser.write(com,5);
} 



int main(int argc, char **argv)
{
  ros::init(argc, argv, "serial_ros");
  ros::NodeHandle nh;


  ros::Subscriber write_sub = nh.subscribe<serial_ros::Cloud_platform>("write", 10, cp_callback); 
  // 频率 [50Hz]
  ros::Rate rate(20.0);

  while(ros::ok())
  {
     cout << " creat ros project test !"<<endl;
     //回调
     ros::spinOnce();
     //挂起一段时间(rate为 50HZ)
     rate.sleep();
  }
}
