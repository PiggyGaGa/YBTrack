/*!
    @Description : https://github.com/shaoshengsong/
    @Author      : shaoshengsong
    @Date        : 2022-09-23 02:52:22
*/
#include <fstream>
#include <sstream>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include "YOLOv5Detector.h"

#include "FeatureTensor.h"
#include "BYTETracker.h" //bytetrack
#include "tracker.h"     //deepsort
#include "track_detection/Diff.h"
#include "derror.h"
// Deep SORT parameter

#include <math.h>
#include <string>
#include <vector>
#include <iostream>
#include <pthread.h>
#include <thread>
#include <chrono>
#include <boost/thread/mutex.hpp>
#include <boost/thread/shared_mutex.hpp>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Pose2D.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/ml.hpp>

const int nn_budget = 100;
const float max_cosine_distance = 0.2;

cv::Mat camImageCopy;
boost::shared_mutex mutexImageCallback;
bool imageStatus = false;
boost::shared_mutex mutexImageStatus;
// 更新error pixels

void cameraCallback(const sensor_msgs::ImageConstPtr &msg)
{
    ROS_DEBUG("[EllipseDetector] USB image received.");
    std::cout << "camera call back success\n";
    cv_bridge::CvImageConstPtr cam_image;
    try
    {
        cam_image = cv_bridge::toCvShare(msg, "bgr8");
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    if (cam_image)
    {
        {
            boost::unique_lock<boost::shared_mutex> lockImageCallback(mutexImageCallback);
            camImageCopy = cam_image->image.clone();
        }
        {
            boost::unique_lock<boost::shared_mutex> lockImageStatus(mutexImageStatus);
            imageStatus = true;
        }
        int width_img = cam_image->image.size().width;
        int height_img = cam_image->image.size().height;
    }
    return;
}

float get_ros_time(ros::Time begin)
{
    ros::Time time_now = ros::Time::now();
    float currTimeSec = time_now.sec - begin.sec;
    float currTimenSec = time_now.nsec / 1e9 - begin.nsec / 1e9;
    return (currTimeSec + currTimenSec);
}

// 用此函数查看是否收到图像话题
bool getImageStatus(void)
{
    boost::shared_lock<boost::shared_mutex> lock(mutexImageStatus);
    return imageStatus;
}

void get_detections(DETECTBOX box, float confidence, DETECTIONS &d)
{
    DETECTION_ROW tmpRow;
    tmpRow.tlwh = box; // DETECTBOX(x, y, w, h);

    tmpRow.confidence = confidence;
    d.push_back(tmpRow);
}

void test_deepsort(cv::Mat &frame, std::vector<detect_result> &results, tracker &mytracker)
{
    std::vector<detect_result> objects;

    DETECTIONS detections;
    for (detect_result dr : results)
    {
        // cv::putText(frame, classes[dr.classId], cv::Point(dr.box.tl().x+10, dr.box.tl().y - 10), cv::FONT_HERSHEY_SIMPLEX, .8, cv::Scalar(0, 255, 0));
        if (dr.classId == 0) // person
        {
            objects.push_back(dr);
            cv::rectangle(frame, dr.box, cv::Scalar(255, 0, 0), 2);
            get_detections(DETECTBOX(dr.box.x, dr.box.y, dr.box.width, dr.box.height), dr.confidence, detections);
        }
    }

    std::cout << "begin track" << std::endl;
    if (FeatureTensor::getInstance()->getRectsFeature(frame, detections))
    {
        std::cout << "get feature succeed!" << std::endl;
        mytracker.predict();
        mytracker.update(detections);
        std::vector<RESULT_DATA> result;
        for (Track &track : mytracker.tracks)
        {
            if (!track.is_confirmed() || track.time_since_update > 1)
                continue;
            result.push_back(std::make_pair(track.track_id, track.to_tlwh()));
        }
        for (unsigned int k = 0; k < detections.size(); k++)
        {
            DETECTBOX tmpbox = detections[k].tlwh;
            cv::Rect rect(tmpbox(0), tmpbox(1), tmpbox(2), tmpbox(3));
            cv::rectangle(frame, rect, cv::Scalar(0, 0, 255), 4);
            // cvScalar的储存顺序是B-G-R，CV_RGB的储存顺序是R-G-B

            for (unsigned int k = 0; k < result.size(); k++)
            {
                DETECTBOX tmp = result[k].second;
                cv::Rect rect = cv::Rect(tmp(0), tmp(1), tmp(2), tmp(3));
                rectangle(frame, rect, cv::Scalar(255, 255, 0), 2);

                std::string label = cv::format("%d", result[k].first);
                cv::putText(frame, label, cv::Point(rect.x, rect.y), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 0), 2);
            }
        }
    }
    std::cout << "end track" << std::endl;
}

std::vector<STrack> inference_bytetrack(cv::Mat &frame, std::vector<detect_result> &results, BYTETracker &tracker)
{
    std::vector<detect_result> objects;
    for (detect_result dr : results)
    {

        if (dr.classId == 0) // person
        {
            objects.push_back(dr);
        }
    }
    std::vector<STrack> output_stracks = tracker.update(objects);
    return output_stracks;
}

cv::Scalar get_color_by_idx(int idx)
{
    idx += 3;
    return cv::Scalar(37 * idx % 255, 17 * idx % 255, 29 * idx % 255);
}

void draw_bytetrack_result(cv::Mat &frame, std::vector<STrack> &tracker_results)
{
    for (unsigned long i = 0; i < tracker_results.size(); i++)
    {
        std::vector<float> tlwh = tracker_results[i].tlwh;
        bool vertical = tlwh[2] / tlwh[3] > 1.6;
        if (tlwh[2] * tlwh[3] > 20 && !vertical)
        {
            cv::Scalar s = get_color_by_idx(tracker_results[i].track_id);
            cv::putText(frame, cv::format("%d", tracker_results[i].track_id), cv::Point(tlwh[0], tlwh[1] - 5),
                        0, 0.6, cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
            cv::rectangle(frame, cv::Rect(tlwh[0], tlwh[1], tlwh[2], tlwh[3]), s, 2);
        }
    }
}
/* 根据目标检测算法检测出来的目标，选择一个云台要跟踪的目标 */
bool get_unique_track_obj(cv::Mat &img, std::vector<STrack> tracker_results, STrack &PTZ_track_obj)
{
    if (tracker_results.size() > 1)
    {
        PTZ_track_obj = tracker_results[0];  /* 暂时取第一个目标，我们只需要让算法跟踪第一个目标 */
        return true;
    }
    else
    {
        return false; /* 不需要做什么 */
    }

}


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "tracker_ros");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    ros::Rate loop_rate(30);
    image_transport::Subscriber sub;
    // 接收图像的话题
    ROS_INFO("init update");
    sub = it.subscribe("/camera/rgb/image", 1, cameraCallback);

    const auto wait_duration = std::chrono::milliseconds(2000);
    float unfilter_vely, unfilter_velx;
    ros::Time begin_time = ros::Time::now();
    // bytetrack
    int fps = 20;
    int num_frames = 0;
    BYTETracker bytetracker(fps, 30);
    /***************************************/
    /*******************error pixels *******/
    track_detection::Diff error_pixels;
    geometry_msgs::Point flag_vision; // Point.x 是整数存储了是否有检测框的标志
    float cur_time;
    float last_time;
    float last_error_x, last_error_y;
    float dt;
    // 发布话题信息，
    ros::Publisher flag_version_pub = nh.advertise<geometry_msgs::Point>("/relative_position_flag", 10); // 标志信息发布
    ros::Publisher position_diff_pub = nh.advertise<track_detection::Diff>("/position_diff", 10);        // Diff信息发布

    /**************************************************************/
    //-----------------------------------------------------------------------
    // 加载类别名称
    std::vector<std::string> classes;
    std::string file = "/home/nvidia/UAV/catkin_TX2_ws/materials/deepsort_v1.2/coco_80_labels_list.txt";
    std::ifstream ifs(file);
    if (!ifs.is_open())
        CV_Error(cv::Error::StsError, "File " + file + " not found");
    std::string line;
    while (std::getline(ifs, line))
    {
        classes.push_back(line);
    }

    std::vector<detect_result> results;
    std::vector<STrack> tracker_results;
    // STrack PTZ_track_obj = ; // 云台要跟踪的目标

    std::shared_ptr<YOLOv5Detector> detector(new YOLOv5Detector());
    detector->init(k_detect_model_path);

    while (ros::ok())
    {
        // ros::spinOnce();
        while (!getImageStatus())
        {
            std::cout << "Waiting for image.\n";
            std::this_thread::sleep_for(wait_duration);
            ros::spinOnce();
        }
        cv::Mat frame;
        {
            boost::unique_lock<boost::shared_mutex> lockImageCallback(mutexImageCallback);
            frame = camImageCopy.clone();
        }
        num_frames++;
        // Second/Millisecond/Microsecond  秒s/毫秒ms/微秒us

        auto start = std::chrono::system_clock::now();
        detector->detect(frame, results);
        auto end = std::chrono::system_clock::now();
        auto detect_time = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count(); // ms
        std::cout << "classes num:" << classes.size() << "| results size: " << results.size() << "| num frames: " << num_frames << std::endl;
        std::cout << "detect time per frame: " << detect_time << std::endl;

        start = std::chrono::system_clock::now();
        tracker_results = inference_bytetrack(frame, results, bytetracker);
        end = std::chrono::system_clock::now();
        detect_time = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count(); // ms
        draw_bytetrack_result(frame, tracker_results);
        std::cout << "bytetrack time: " << detect_time << std::endl;
        cv::imshow("YOLOv5-6.x", frame);
        if (cv::waitKey(30) == 27) // Wait for 'esc' key press to exit
        {
         break;
        }

        /**************************************************/
        /**************************************************/
        /**************************更新 error pixels ******/
        if (tracker_results.size() < 1)//tracker_results.size() == 0
        {
            flag_vision.x = 0x00;
            error_pixels.x = 0.0;
            error_pixels.y = 0.0;
            error_pixels.Ix = 0.0;
            error_pixels.Iy = 0.0;
            error_pixels.velx = 0.0;
            error_pixels.vely = 0.0;
            error_pixels.recsize = 0.0;
            error_pixels.selectrec = 0.0;
        }
        else
        {
            /*********************************************/
            // 在跟踪之前 设置云台跟踪的目标 PTZ 云台相机
            STrack PTZ_track_obj = tracker_results[0];
            bool exist_obj_flag = get_unique_track_obj(frame, tracker_results, PTZ_track_obj);


            DERROR derrorX, derrorY;
            flag_vision.x = 0x01;
            cv::Rect det_box = results[0].box;                                            /*    TBD  找到和目标一致的 */
            cv::Rect track_box(PTZ_track_obj.tlwh[0], PTZ_track_obj.tlwh[1], PTZ_track_obj.tlwh[2], PTZ_track_obj.tlwh[3]); //center x, y, width, heigh
            cur_time = get_ros_time(begin_time);
            dt = (cur_time - last_time);
            if (dt > 1.0 || dt < 0.0)
            {
                dt = 0.05;
            }
            error_pixels.x = track_box.x + track_box.width / 2 - frame.cols / 2;
            error_pixels.y = track_box.y + track_box.height / 2 - frame.rows / 2;

            error_pixels.recsize = track_box.width * track_box.height;
            error_pixels.selectrec = det_box.width * det_box.height;

            float error_x = error_pixels.x;
            float error_y = error_pixels.y;

            derrorX.add_error(error_x, cur_time);
            derrorY.add_error(error_y, cur_time);
            //derrorX.derror_output();
            //derrorY.derror_output();
            //derrorX.show_error();
            //derrorY.show_error();

            error_pixels.velx = derrorX.Output;
            error_pixels.vely = derrorY.Output;

            error_pixels.Ix += error_pixels.x * dt;
            error_pixels.Iy += error_pixels.y * dt;

            unfilter_velx = (error_pixels.x - last_error_x) / dt;
            unfilter_vely = (error_pixels.y - last_error_y) / dt;

            last_time = cur_time;
            last_error_x = error_pixels.x;
            last_error_y = error_pixels.y;
        }

        position_diff_pub.publish(error_pixels);
        flag_version_pub.publish(flag_vision);

        results.clear();
        ros::spinOnce();
        loop_rate.sleep();
    }
    cv::destroyAllWindows();
}
