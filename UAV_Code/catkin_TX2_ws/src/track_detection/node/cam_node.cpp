#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Pose.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/aruco.hpp>
#include "id_management.h"

using namespace std;
using namespace cv;


// 设置图像大小
cv::Size image_size = Size(640.0, 360.0);

int main(int argc, char **argv)
{

    cv::VideoCapture cap(0); // 设置本机默认摄像头
    //cv::VideoCapture cap("/media/nvidia/3434-6230/UAV/catkin_TX2_ws/materials/deepsort_v1.2/2.mp4"); // 设置本机默认摄像头
                             // 设置摄像头分辨率
    if(!cap.isOpened())
    {
        return -1;
    }
    // 640 * 640 无人机吊舱的分辨率
    cap.set(CAP_PROP_FRAME_WIDTH, 640);
    cap.set(CAP_PROP_FRAME_HEIGHT, 480);

    // 1280 * 720 是电脑相机的分辨率
    // cap.set(CAP_PROP_FRAME_WIDTH, 720);
    // cap.set(CAP_PROP_FRAME_HEIGHT, 1280);

    int cap_flag = 0;
    ros::init(argc, argv, "ptz_usb_camera");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher image_pub;
    ros::Rate loop_rate(10);
    // 在这里修改发布话题名称
    image_pub = it.advertise("/camera/rgb/image", 1);
    // 用系统默认驱动读取摄像头0，使用其他摄像头ID，请在这里修改
    cv::Mat frame;
    // 设置全屏
    // namedWindow("web_cam frame", CV_WINDOW_NORMAL);
    // setWindowProperty("web_cam frame", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);
    sensor_msgs::ImagePtr msg;
    int id = 0;
    while (ros::ok())
    {

        cap >> frame;
        if (!frame.empty())
        {
            if (0 == id)
            {
                cout << "IM RESOLUTION: " << frame.cols << " x " << frame.rows << "  channels: " << frame.channels() <<" frame size: " << frame.size << endl;
            }
            // 添加ID, 这个干啥用？
            // frame = IDManagement::add_id(id, frame);
            id = id + 1;
            msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
            // 将图像通过话题发布出去
            image_pub.publish(msg);
        }
        else
        {
            ROS_INFO("can not read image");
        }
        ros::spinOnce();
        // 按照设定的帧率延时，ros::Rate loop_rate(30)
        loop_rate.sleep();
    }
    cap.release();
}
