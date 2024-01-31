// gimbal tracking control & serial communication---advanced by Xuancen Liu -----------------------------------------
// 2019.9.18 at Hunan Changsha.
//  email: buaalxc@163.com
// wechat: liuxuancen003

#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt8MultiArray.h>
#include <geometry_msgs/Point.h>
#include <iostream>
#include "serial_ros/Cloud_platform.h"
#include "serial_ros/Diff.h"
#include "serial_ros/gimbal.h"
#include <iomanip>
#include "fstream"

using namespace std;
serial::Serial ser;
serial_ros::Cloud_platform cloud_platform;
serial_ros::Diff temp;
serial_ros::gimbal gimbaldata;
unsigned char command[8];

float flag_target;
short int yaw_control_last, pitch_control_last;

float imu_yaw_vel, imu_pitch_vel;
float get_ros_time(ros::Time begin);
// 获取当前时间 单位：秒
float get_ros_time(ros::Time begin)
{
    ros::Time time_now = ros::Time::now();
    float currTimeSec = time_now.sec - begin.sec;
    float currTimenSec = time_now.nsec / 1e9 - begin.nsec / 1e9;
    return (currTimeSec + currTimenSec);
}

// 回调函数
void write_callback(const serial_ros::Cloud_platform::ConstPtr &msg)
{
    unsigned char com[5], longcmd[20];

    com[0] = msg->get_data_com[0];
    com[1] = msg->get_data_com[1];
    com[2] = msg->get_data_com[2];
    com[3] = msg->get_data_com[3];
    com[4] = msg->get_data_com[4];

    ser.write(com, 5);
}

void pos_diff_callback(const serial_ros::Diff::ConstPtr &msg)
{

    float pKp = 2.6, yKp = 2.6;
    float pKd = 0.14, yKd = 0.14;
    float pKI = 0.0, yKI = 0.0;

    unsigned char xl, xh, yl, yh;
    short int yaw_control, pitch_control;

    temp.x = msg->x;
    temp.y = msg->y;
    temp.velx = msg->velx;
    temp.vely = msg->vely;
    temp.Ix = msg->Ix;
    temp.Iy = msg->Iy;

    yaw_control = (short int)(yKp * temp.x + yKd * (temp.velx) + yKI * temp.Ix);
    pitch_control = (short int)(pKp * temp.y + pKd * (temp.vely) + pKI * temp.Iy);

    yaw_control_last = yaw_control;
    pitch_control_last = pitch_control;

    if (yaw_control - 500.0 > 0.001)
    {
        yaw_control = 500;
    }
    if (yaw_control + 500.0 < -0.001)
    {
        yaw_control = -500;
    }

    if (yaw_control > 0.001)
    {
        yaw_control = yaw_control + 50.0;
    }
    if (yaw_control < 0.001)
    {
        yaw_control = yaw_control - 50.0;
    }

    if (pitch_control - 500.0 > 0.001)
    {
        pitch_control = 500;
    }
    if (pitch_control + 500.0 < -0.001)
    {
        pitch_control = -500;
    }

    if (pitch_control > 0.001)
    {
        pitch_control = pitch_control + 50.0;
    }
    if (pitch_control < 0.001)
    {
        pitch_control = pitch_control - 50.0;
    }

    xh = (yaw_control & 0xff00) >> 8;
    xl = (yaw_control & 0x00ff);

    yh = (pitch_control & 0xff00) >> 8;
    yl = (pitch_control & 0x00ff);

    // cout<<temp.x<<endl;

    if (flag_target == 0)
    {
        // cout<<"============000000000000000000"<<endl;
        command[0] = 0x55;
        command[1] = 0x01;
        command[2] = 0x00;
        command[3] = 0x00;
        command[4] = 0x00;
        command[5] = 0x00;
        command[6] = 0x02;
        command[7] = 0x58;
    }

    if (flag_target == 1)
    {
        // cout<<"============11111111111111111"<<endl;

        command[0] = 0x55;
        command[1] = 0x01;
        command[2] = xl;
        command[3] = xh;
        command[4] = yl;
        command[5] = yh;
        command[6] = 0x02;
        command[7] = (command[0] + command[1] + command[2] + command[3] + command[4] + command[5] + command[6]) % 256;
    }
}

void pos_diff_flag_cb(const geometry_msgs::Point::ConstPtr &msg)
{
    flag_target = msg->x;
    // cout<<"@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@"<<endl;
}

// 日志】数据保存
ofstream file;
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

int main(int argc, char **argv)
{

    // 初始化节点
    ros::init(argc, argv, "ptz_serial_node");
    // 声明节点句柄
    ros::NodeHandle nh;
    static int p;
    int i;
    uint8_t flag = 0x00;
    float dt;
    float imu_camera[3], imu_camera_last[3], imu_camera_vel[3];
    float last_time;
    float RC_target_camera[3], stator_rel_camera_last[3], stator_rel_camera_vel[3];
    float stator_rel_camera[3];

    // 注意，里面有iOS：：app，表示打开文件后，在写入的文件不会覆盖原文件中的内容，也就是原来文件中的数据会得到保存。
    file.open("./serial_node.txt", ios::binary |ios::in | ios::out);

    file << "cur_time"
         << "  "
         << "dt"
         << "  "
         << "imu_camera[0]"
         << "  "
         << "imu_camera[1]"
         << "  "
         << "imu_camera[2]"
         << "  "
         << "RC_target_camera[0]"
         << "  "
         << "RC_target_camera[1]"
         << "  "
         << "RC_target_camera[2]"
         << "  "
         << "stator_rel_camera[0]"
         << "  "
         << "stator_rel_camera[1]"
         << "  "
         << "stator_rel_camera[2]"
         << "  "
         << "imu_camera_vel[0]"
         << "  "
         << "imu_camera_vel[1]"
         << "  "
         << "imu_camera_vel[2]"
         << "  "
         << "stator_rel_camera_vel[0]"
         << "  "
         << "stator_rel_camera_vel[1]"
         << "  "
         << "stator_rel_camera_vel[2]"
         << "  "
         << "yaw_control_last"
         << "  "
         << "pitch_control_last"
         << "\n";

    //    float pitch_imu;
    //    float pitch_rc;
    //    float pitch_stator_rel;
    std_msgs::UInt8MultiArray r_buffer;
    // 订阅主题，并配置回调函数
    ros::Subscriber write_sub = nh.subscribe<serial_ros::Cloud_platform>("/ptz/write", 10, write_callback);
    ros::Subscriber pos_diff_sub = nh.subscribe<serial_ros::Diff>("/position_diff", 10, pos_diff_callback);
    // 视觉flag
    ros::Subscriber pos_diff_flag_sub = nh.subscribe<geometry_msgs::Point>("/relative_position_flag", 10, pos_diff_flag_cb);
    // 发布主题
    ros::Publisher read_pub = nh.advertise<serial_ros::gimbal>("/readgimbal", 10);
    ros::Time begin_time = ros::Time::now();

    try
    {

        ser.setPort("/dev/ttyUSB0");
        ser.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    }
    catch (serial::IOException &e)
    {
        ROS_ERROR_STREAM("Unable to open port ");
        return -1;
    }

    if (ser.isOpen())
    {
        ROS_INFO_STREAM("Serial Port initialized");
    }
    else
    {
        return -1;
    }
    static unsigned char rx_state = 0x00;
    static unsigned int len = 0;
    // 指定循环的频率
    ros::Rate loop_rate(28);

    while (ros::ok())
    {
        // cout<<"serial is not avaliable"<<endl;
        //  ser.available -- Return the number of characters in the buffer

        if (ser.available())
        {

            float cur_time = get_ros_time(begin_time);
            dt = cur_time - last_time;
            last_time = cur_time;
            ser.write(command, 8);
            std_msgs::UInt8MultiArray serial_data;
            uint8_t rx_byte;
            p = ser.available();

            ser.read(serial_data.data, p);
            r_buffer.data.resize(p + 1);
            r_buffer.data[0] = p;
            std::cout << "get buff len: " << p << std::endl;
            for (i = 0; i < p; i++)
            {
                r_buffer.data[i + 1] = serial_data.data[i];

                if (r_buffer.data[6] > 40)
                {
                    imu_camera[0] = -0.02197 * (256 * (256 - r_buffer.data[6]) - r_buffer.data[5]);
                }
                else if (r_buffer.data[6] < 40)
                {
                    imu_camera[0] = 0.02197 * (256 * r_buffer.data[6] + r_buffer.data[5]);
                }

                imu_camera_vel[0] = (imu_camera[0] - imu_camera_last[0]) / dt;
                imu_camera_last[0] = imu_camera[0];

                // printf("r_buffer.data[6] is %d \n",r_buffer.data[6]);
                // printf("r_buffer.data[5] is %d \n",r_buffer.data[5]);
                // printf("imu_camera[0] is %f \n",imu_camera[0]);

                if (r_buffer.data[8] > 40)
                {
                    RC_target_camera[0] = -0.02197 * (256 * (256 - r_buffer.data[8]) - r_buffer.data[7]);
                }
                else if (r_buffer.data[8] < 40)
                {
                    RC_target_camera[0] = 0.02197 * (256 * r_buffer.data[8] + r_buffer.data[7]);
                }

                if (r_buffer.data[10] > 40)
                {
                    stator_rel_camera[0] = -0.02197 * (256 * (256 - r_buffer.data[10]) - r_buffer.data[9]);
                }
                else if (r_buffer.data[10] < 40)
                {
                    stator_rel_camera[0] = 0.02197 * (256 * r_buffer.data[10] + r_buffer.data[9]);
                }
                // printf("stator_rel_camera[0] is %d \n",(stator_rel_camera[0]));

                stator_rel_camera_vel[0] = (stator_rel_camera[0] - stator_rel_camera_last[0]) / dt;
                stator_rel_camera_last[0] = stator_rel_camera[0];

                //----------------------------------------俯仰PITCH-------------------------------------------
                if (r_buffer.data[24] > 40)
                {
                    imu_camera[1] = -0.02197 * (256 * (256 - r_buffer.data[24]) - r_buffer.data[23]);
                }
                else if (r_buffer.data[24] < 40)
                {
                    imu_camera[1] = 0.02197 * (256 * r_buffer.data[24] + r_buffer.data[23]);
                }

                imu_camera_vel[1] = (imu_camera[1] - imu_camera_last[1]) / dt;
                imu_camera_last[1] = imu_camera[1];

                if (r_buffer.data[26] > 40)
                {
                    RC_target_camera[1] = -0.02197 * (256 * (256 - r_buffer.data[26]) - r_buffer.data[25]);
                }
                else if (r_buffer.data[26] < 40)
                {
                    RC_target_camera[1] = 0.02197 * (256 * r_buffer.data[26] + r_buffer.data[25]);
                }

                if (r_buffer.data[28] > 40)
                {
                    stator_rel_camera[1] = -0.02197 * (256 * (256 - r_buffer.data[28]) - r_buffer.data[27]);
                }
                else if (r_buffer.data[28] < 40)
                {
                    stator_rel_camera[1] = 0.02197 * (256 * r_buffer.data[28] + r_buffer.data[27]);
                }

                stator_rel_camera_vel[1] = (stator_rel_camera[1] - stator_rel_camera_last[1]) / dt;
                stator_rel_camera_last[1] = stator_rel_camera[1];

                //----------------------------------------偏航YAW-------------------------------------------

                if (r_buffer.data[42] > 40)
                {
                    imu_camera[2] = -0.02197 * (256 * (256 - r_buffer.data[42]) - r_buffer.data[41]);
                }
                else if (r_buffer.data[42] < 40)
                {
                    imu_camera[2] = 0.02197 * (256 * r_buffer.data[42] + r_buffer.data[41]);
                }

                imu_camera_vel[2] = (imu_camera[2] - imu_camera_last[2]) / dt;
                imu_camera_last[2] = imu_camera[2];

                // printf("imu_camera[1] is %d \n",(imu_camera[1]));

                if (r_buffer.data[44] > 40)
                {
                    RC_target_camera[2] = -0.02197 * (256 * (256 - r_buffer.data[44]) - r_buffer.data[43]);
                }
                else if (r_buffer.data[44] < 40)
                {
                    RC_target_camera[2] = 0.02197 * (256 * r_buffer.data[44] + r_buffer.data[43]);
                }
                // printf("RC_target_camera[1] is %d \n",(RC_target_camera[1]));

                if (r_buffer.data[46] > 40)
                {
                    stator_rel_camera[2] = -0.02197 * (256 * (256 - r_buffer.data[46]) - r_buffer.data[45]);
                }
                else if (r_buffer.data[46] < 40)
                {
                    stator_rel_camera[2] = 0.02197 * (256 * r_buffer.data[46] + r_buffer.data[45]);
                }
                // printf("stator_rel_camera[1] is %d \n",(stator_rel_camera[1]));

                stator_rel_camera_vel[2] = (stator_rel_camera[2] - stator_rel_camera_last[2]) / dt;
                stator_rel_camera_last[2] = stator_rel_camera[2];

                cout << "-----------------imu_camera----------------------" << endl;
                cout << "roll:" << setprecision(6) << stator_rel_camera[0] << "    "
                     << "pitch:" << setprecision(6) << stator_rel_camera[1] << "    "
                     << "yaw:" << setprecision(6) << stator_rel_camera[2] << endl;
            }

            gimbaldata.imu0 = imu_camera[0];
            gimbaldata.imu1 = imu_camera[1];
            gimbaldata.imu2 = imu_camera[2];
            gimbaldata.rc0 = RC_target_camera[0];
            gimbaldata.rc1 = RC_target_camera[1];
            gimbaldata.rc2 = RC_target_camera[2];
            gimbaldata.rel0 = stator_rel_camera[0];
            gimbaldata.rel1 = stator_rel_camera[1];
            gimbaldata.rel2 = stator_rel_camera[2];
            gimbaldata.imuvel0 = imu_camera_vel[0];
            gimbaldata.imuvel1 = imu_camera_vel[1];
            gimbaldata.imuvel2 = imu_camera_vel[2];
            gimbaldata.relvel0 = stator_rel_camera_vel[0];
            gimbaldata.relvel1 = stator_rel_camera_vel[1];
            gimbaldata.relvel2 = stator_rel_camera_vel[2];

            file << cur_time << "  " << dt << "  " << imu_camera[0] << "  " << imu_camera[1] << "  " << imu_camera[2] << "  " << RC_target_camera[0] << "  " << RC_target_camera[1] << "  " << RC_target_camera[2] << "  " << stator_rel_camera[0] << "  " << stator_rel_camera[1] << "  " << stator_rel_camera[2] << "  " << imu_camera_vel[0] << "  " << imu_camera_vel[1] << "  " << imu_camera_vel[2] << "  " << stator_rel_camera_vel[0] << "  " << stator_rel_camera_vel[1] << "  " << stator_rel_camera_vel[2] << "  " << yaw_control_last << "  " << pitch_control_last << "\n";
            // 此处数据需要完完成分类、创建msg.h、publish出去
            read_pub.publish(gimbaldata);
        }
        else
        {
            std::cout<< "there is no data in buff" << std::endl;
        }

        // 处理ROS的信息，比如订阅消息,并调用回调函数
        ros::spinOnce();
        loop_rate.sleep();
    }
    file.close(); // 关闭文件，保存文件。
}
