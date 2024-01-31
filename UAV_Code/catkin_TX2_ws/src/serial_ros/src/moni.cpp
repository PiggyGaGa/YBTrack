/*********************************************************************
  > File Name: ros_serial
  >  
  >            
  > Author   : lirongw & Xuancen liu
  > Time     : 2018-08-25
  > email    : buaalxc@163.com
  > wechat   : liuxuancen003
**********************************************************************/
#include <ros/ros.h>
#include <serial/serial.h>  
#include <std_msgs/String.h> 
#include <std_msgs/UInt8MultiArray.h>
#include <std_msgs/UInt8.h>
#include <stdio.h>
#include <string.h>
#include "serial_ros/Cloud_platform.h"
  
 using namespace std;

// 获取云台角度数据的命令与控制指令  在命令生成文件中 publish ,串口接收发送统一在 serial_node 中进行！

// this code get ptz position data
serial_ros::Cloud_platform  cloud_platform;
unsigned int gimbal_control[20];


  serial::Serial serr; 
int main (int argc, char** argv) 
{ 
    //初始化节点 
    ros::init(argc, argv, "moni"); 
    //声明节点句柄 
    ros::NodeHandle nh; 
  
    //发布主题 
    ros::Publisher write_pub = nh.advertise<serial_ros::Cloud_platform>("/ptz/write", 10); 
    //指定循环的频率 
    ros::Rate loop_rate(50);
    // 尝试失败
    // char *s; 
    // sprintf(s,"{%d %d %d %d %d}",0x3e,0x3d,0x00,0x3d,0x00);  //sprintf 的使用
    // 尝试 std_msgs::UInt8MultiArray 

    cloud_platform.get_data_com[0]=0x3e; 
    cloud_platform.get_data_com[1]=0x3d;
    cloud_platform.get_data_com[2]=0x00;
    cloud_platform.get_data_com[3]=0x3d;
    cloud_platform.get_data_com[4]=0x00;

//-----------------HOME-----------------
/*
cloud_platform.gimbal_control[0] = 0xff;
cloud_platform.gimbal_control[1] = 0x01;
cloud_platform.gimbal_control[2] = 0x0f;
cloud_platform.gimbal_control[3] = 0x10;
cloud_platform.gimbal_control[4] = 0x00;
cloud_platform.gimbal_control[5] = 0x05;
cloud_platform.gimbal_control[6] = 0x05;
cloud_platform.gimbal_control[7] = 0x00;
cloud_platform.gimbal_control[8] = 0x00;
cloud_platform.gimbal_control[9] = 0x00;
cloud_platform.gimbal_control[10] = 0x00;
cloud_platform.gimbal_control[11] = 0x00;
cloud_platform.gimbal_control[12] = 0x00;
cloud_platform.gimbal_control[13] = 0x00;
cloud_platform.gimbal_control[14] = 0x00;
cloud_platform.gimbal_control[15] = 0x00;
cloud_platform.gimbal_control[16] = 0x00;
cloud_platform.gimbal_control[17] = 0x00;
cloud_platform.gimbal_control[18] = 0x00;
cloud_platform.gimbal_control[19] = 0x0a;
*/

//----pitch to 40deg-----yaw to 20deg---------
/*
cloud_platform.gimbal_control[0] = 0xff;
cloud_platform.gimbal_control[1] = 0x01;
cloud_platform.gimbal_control[2] = 0x0f;
cloud_platform.gimbal_control[3] = 0x10;
cloud_platform.gimbal_control[4] = 0x00;
cloud_platform.gimbal_control[5] = 0x05;
cloud_platform.gimbal_control[6] = 0x05;
cloud_platform.gimbal_control[7] = 0x00;
cloud_platform.gimbal_control[8] = 0x00;
cloud_platform.gimbal_control[9] = 0x00;
cloud_platform.gimbal_control[10] = 0x00;
cloud_platform.gimbal_control[11] = 0x00;
cloud_platform.gimbal_control[12] = 0x00;
cloud_platform.gimbal_control[13] = 0xd0;
cloud_platform.gimbal_control[14] = 0x07;
cloud_platform.gimbal_control[15] = 0x00;
cloud_platform.gimbal_control[16] = 0x00;
cloud_platform.gimbal_control[17] = 0xe8;
cloud_platform.gimbal_control[18] = 0x03;
cloud_platform.gimbal_control[19] = 0xcc;
*/

//----pitch speed 1.2deg/s-----yaw to 0deg---------
/*
cloud_platform.gimbal_control[0] = 0xff;
cloud_platform.gimbal_control[1] = 0x01;
cloud_platform.gimbal_control[2] = 0x0f;
cloud_platform.gimbal_control[3] = 0x10;
cloud_platform.gimbal_control[4] = 0x00;
cloud_platform.gimbal_control[5] = 0x01;
cloud_platform.gimbal_control[6] = 0x05;
cloud_platform.gimbal_control[7] = 0x00;
cloud_platform.gimbal_control[8] = 0x00;
cloud_platform.gimbal_control[9] = 0x00;
cloud_platform.gimbal_control[10] = 0x00;
cloud_platform.gimbal_control[11] = 0x3a;
cloud_platform.gimbal_control[12] = 0x00;
cloud_platform.gimbal_control[13] = 0x00;
cloud_platform.gimbal_control[14] = 0x00;
cloud_platform.gimbal_control[15] = 0x00;
cloud_platform.gimbal_control[16] = 0x00;
cloud_platform.gimbal_control[17] = 0x00;
cloud_platform.gimbal_control[18] = 0x00;
cloud_platform.gimbal_control[19] = 0x40;
*/

//----pitch speed 156deg/s to 40deg-----yaw to 20deg---------
/*
cloud_platform.gimbal_control[0] = 0xff;
cloud_platform.gimbal_control[1] = 0x01;
cloud_platform.gimbal_control[2] = 0x0f;
cloud_platform.gimbal_control[3] = 0x10;
cloud_platform.gimbal_control[4] = 0x00;
cloud_platform.gimbal_control[5] = 0x05;
cloud_platform.gimbal_control[6] = 0x05;
cloud_platform.gimbal_control[7] = 0x00;
cloud_platform.gimbal_control[8] = 0x00;
cloud_platform.gimbal_control[9] = 0x00;
cloud_platform.gimbal_control[10] = 0x00;
cloud_platform.gimbal_control[11] = 0x00;
cloud_platform.gimbal_control[12] = 0x05;
cloud_platform.gimbal_control[13] = 0xd0;
cloud_platform.gimbal_control[14] = 0x07;
cloud_platform.gimbal_control[15] = 0x00;
cloud_platform.gimbal_control[16] = 0x00;
cloud_platform.gimbal_control[17] = 0xe8;
cloud_platform.gimbal_control[18] = 0x03;
cloud_platform.gimbal_control[19] = 0xd1;
*/



/*   cloud_platform.gimbal_control[0] = 0x55;
 cloud_platform.gimbal_control[1] = 0x01;
cloud_platform.gimbal_control[2] = 0x60;
cloud_platform.gimbal_control[3] = 0x00;
cloud_platform.gimbal_control[4] = 0x60;
cloud_platform.gimbal_control[5] = 0x00;
cloud_platform.gimbal_control[6] = 0x02;
cloud_platform.gimbal_control[7] = 0x18;
*/
/*
cloud_platform.gimbal_control[0] = 0x55;
 cloud_platform.gimbal_control[1] = 0x01;
cloud_platform.gimbal_control[2] = 0x08;
cloud_platform.gimbal_control[3] = 0xFF;
cloud_platform.gimbal_control[4] = 0x08;
cloud_platform.gimbal_control[5] = 0xFF;
cloud_platform.gimbal_control[6] = 0x02;
cloud_platform.gimbal_control[7] = 0x66;
*/

    while(ros::ok()) 
    { 
  
        write_pub.publish(cloud_platform); 


	    
        //处理ROS的信息，比如订阅消息,并调用回调函数 
        ros::spinOnce(); 
        loop_rate.sleep(); 
  
    } 
} 





















/* *****************************************************

using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "serial_ros");
  ros::NodeHandle nh;

  // 频率 [20Hz]
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

 * ************************************************************************/
