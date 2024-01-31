#include "cuda_utils.h"
#include "logging.h"
#include "utils.h"
#include "preprocess.h"
#include "postprocess.h"
#include "tensorrt_model.h"

#include "BYTETracker.h" //bytetrack
#include "track_detection/Diff.h"
#include "derror.h"

#include <iostream>
#include <chrono>
#include <cmath>
#include <pthread.h>
#include <thread>
#include <boost/thread/mutex.hpp>
#include <boost/thread/shared_mutex.hpp>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Pose2D.h>

using namespace nvinfer1;

static Logger gLogger;
const static int kOutputSize = kMaxNumOutputBbox * sizeof(Detection) / sizeof(float) + 1;

cv::Mat camImageCopy;
boost::shared_mutex mutexImageCallback;
bool imageStatus = false;
boost::shared_mutex mutexImageStatus;

DERROR derrorX, derrorY;

void cameraCallback(const sensor_msgs::ImageConstPtr &msg)
{
    ROS_DEBUG("[EllipseDetector] USB image received.");
    // std::cout << "camera call back success\n";
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

void prepare_buffers(ICudaEngine *engine, float **gpu_input_buffer, float **gpu_output_buffer, float **cpu_output_buffer)
{
    assert(engine->getNbBindings() == 2);
    // In order to bind the buffers, we need to know the names of the input and output tensors.
    // Note that indices are guaranteed to be less than IEngine::getNbBindings()
    const int inputIndex = engine->getBindingIndex(kInputTensorName);
    const int outputIndex = engine->getBindingIndex(kOutputTensorName);
    assert(inputIndex == 0);
    assert(outputIndex == 1);
    // Create GPU buffers on device
    CUDA_CHECK(cudaMalloc((void **)gpu_input_buffer, kBatchSize * 3 * kInputH * kInputW * sizeof(float)));
    CUDA_CHECK(cudaMalloc((void **)gpu_output_buffer, kBatchSize * kOutputSize * sizeof(float)));

    *cpu_output_buffer = new float[kBatchSize * kOutputSize];
}

void infer(IExecutionContext &context, cudaStream_t &stream, void **gpu_buffers, float *output, int batchsize)
{
    context.enqueue(batchsize, gpu_buffers, stream, nullptr);
    CUDA_CHECK(cudaMemcpyAsync(output, gpu_buffers[1], batchsize * kOutputSize * sizeof(float), cudaMemcpyDeviceToHost, stream));
    cudaStreamSynchronize(stream);
}

void serialize_engine(unsigned int max_batchsize, bool &is_p6, float &gd, float &gw, std::string &wts_name, std::string &engine_name)
{
    // Create builder
    IBuilder *builder = createInferBuilder(gLogger);
    IBuilderConfig *config = builder->createBuilderConfig();

    // Create model to populate the network, then set the outputs and create an engine
    ICudaEngine *engine = nullptr;
    if (is_p6)
    {
        engine = build_det_p6_engine(max_batchsize, builder, config, DataType::kFLOAT, gd, gw, wts_name);
    }
    else
    {
        engine = build_det_engine(max_batchsize, builder, config, DataType::kFLOAT, gd, gw, wts_name);
    }
    assert(engine != nullptr);

    // Serialize the engine
    IHostMemory *serialized_engine = engine->serialize();
    assert(serialized_engine != nullptr);

    // Save engine to file
    std::ofstream p(engine_name, std::ios::binary);
    if (!p)
    {
        std::cerr << "Could not open plan output file" << std::endl;
        assert(false);
    }
    p.write(reinterpret_cast<const char *>(serialized_engine->data()), serialized_engine->size());

    // Close everything down
    engine->destroy();
    builder->destroy();
    config->destroy();
    serialized_engine->destroy();
}

void deserialize_engine(std::string &engine_name, IRuntime **runtime, ICudaEngine **engine, IExecutionContext **context)
{
    std::ifstream file(engine_name, std::ios::binary);
    if (!file.good())
    {
        std::cerr << "read " << engine_name << " error!" << std::endl;
        assert(false);
    }
    size_t size = 0;
    file.seekg(0, file.end);
    size = file.tellg();
    file.seekg(0, file.beg);
    char *serialized_engine = new char[size];
    assert(serialized_engine);
    file.read(serialized_engine, size);
    file.close();

    *runtime = createInferRuntime(gLogger);
    assert(*runtime);
    *engine = (*runtime)->deserializeCudaEngine(serialized_engine, size);
    assert(*engine);
    *context = (*engine)->createExecutionContext();
    assert(*context);
    delete[] serialized_engine;
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

/*

bytetrack
    int classId;
    float confidence;
    cv::Rect_<float> box;

yolov5 - tensorrt
  float bbox[4];  // center_x center_y w h
  float conf;  // bbox_conf * cls_conf
  float class_id;
  float mask[32];
*/
std::vector<detect_result> tensorrt_yolo_2_bytetrack_ojb_detects(cv::Mat &img, std::vector<Detection> yolov5_detects)
{
    std::vector<detect_result> results;
    for (int i = 0; i < yolov5_detects.size(); i++)
    {
        detect_result single_det;
        single_det.classId = yolov5_detects[i].class_id;
        single_det.confidence = yolov5_detects[i].conf;
        single_det.box = get_rect(img, yolov5_detects[i].bbox);
        results.push_back(single_det);
    }
    return results;
}


std::vector<Detection> get_person_detect(std::vector<Detection> yolov5_detects)
{
    std::vector<Detection> results;
    for(int i = 0; i < yolov5_detects.size(); i++)
    {
      if(yolov5_detects[i].class_id == 0)  // person only
      {
         results.push_back(yolov5_detects[i]);
      }
    }
    return results;
}
/* 根据目标检测算法检测出来的目标，选择一个云台要跟踪的目标 */
bool get_unique_track_obj(cv::Mat &img, std::vector<STrack> tracker_results, STrack &PTZ_track_obj)
{
    if (tracker_results.size() > 1)
    {
        PTZ_track_obj = tracker_results[0]; /* 暂时取第一个目标，我们只需要让算法跟踪第一个目标 */
        return true;
    }
    else
    {
        return false; /* 不需要做什么 */
    }
}

int main(int argc, char **argv)
{
    cudaSetDevice(kGpuId);

    std::string engine_name = "/home/nvidia/UAV/catkin_TX2_ws/materials/yolov5_tx2/yolov5s.engine";
    std::string coco_file = "/home/nvidia/UAV/catkin_TX2_ws/materials/deepsort_v1.2/coco_80_labels_list.txt";
    bool is_p6 = false;
    float gd = 0.0f, gw = 0.0f;

    // ros setup and setup subscribe
    ros::init(argc, argv, "detect_tracker_node");
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

    // Deserialize the engine from file
    std::cout << "init cuda device " << std::endl;
    IRuntime *runtime = nullptr;
    ICudaEngine *engine = nullptr;
    IExecutionContext *context = nullptr;
    deserialize_engine(engine_name, &runtime, &engine, &context);
    cudaStream_t stream;
    CUDA_CHECK(cudaStreamCreate(&stream));
    // Init CUDA preprocessing
    cuda_preprocess_init(kMaxInputImageSize);
    // Prepare cpu and gpu buffers
    float *gpu_buffers[2];
    float *cpu_output_buffer = nullptr;
    prepare_buffers(engine, &gpu_buffers[0], &gpu_buffers[1], &cpu_output_buffer);

    while (ros::ok())
    {
        std::vector<cv::Mat> img_batch;
        while (!getImageStatus())
        {
            std::cout << "Waiting for image published.\n";
            std::this_thread::sleep_for(wait_duration);
            ros::spinOnce();
        }
        cv::Mat img;
        {
            boost::unique_lock<boost::shared_mutex> lockImageCallback(mutexImageCallback);
            img = camImageCopy.clone();
        }

        img_batch.push_back(img);
        // Preprocess
        if (num_frames != -1)
        {
            cuda_batch_preprocess(img_batch, gpu_buffers[0], kInputW, kInputH, stream);
        }

        /***********************************************************/
        /**************************Run inference *******************/
        std::vector<cv::Mat> draw_image_detect;
        auto start = std::chrono::system_clock::now();
        infer(*context, stream, (void **)gpu_buffers, cpu_output_buffer, kBatchSize);
        auto end = std::chrono::system_clock::now();
        // std::cout << "inference time: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << "ms" << std::endl;
        // 处理到CPU
        std::vector<std::vector<Detection>> res_batch, one_type_;
        batch_nms(res_batch, cpu_output_buffer, img_batch.size(), kOutputSize, kConfThresh, kNmsThresh);
        /*********************************************************************************/
        /***********detect only one type objecto what we what, such as human-0 ***********/
        std::vector<string> coco_classes;

        std::ifstream ifs(coco_file);
        if (!ifs.is_open())
            CV_Error(cv::Error::StsError, "File " + coco_file + " not found");
        std::string line;
        while (std::getline(ifs, line))
        {
            coco_classes.push_back(line);
        }
        std::vector<Detection> res_detect_;
        res_detect_ = get_person_detect(res_batch[0]);   // only person detect
        draw_image_detect.push_back(img_batch[0].clone());
        draw_bbox(draw_image_detect, res_batch);
        /*
        cv::imshow("_show_image_detect", draw_image_detect[0]);
        if (cv::waitKey(30) == 27) // Wait for 'esc' key press to exit
        {
            break;
        }
        */
        /*************************************************/
        /**************    track *****************************/
        start = std::chrono::system_clock::now();
        cv::Mat frame = img_batch[0];

        std::vector<detect_result> detect_results = tensorrt_yolo_2_bytetrack_ojb_detects(frame, res_detect_);

        std::vector<STrack> tracker_results = inference_bytetrack(frame, detect_results, bytetracker);
        end = std::chrono::system_clock::now();
        // std::cout << "track time: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << "ms" << std::endl;

        draw_bytetrack_result(frame, tracker_results);
        cv::imshow("_show_image_tracker", img_batch[0]);
        if (cv::waitKey(30) == 27) // Wait for 'esc' key press to exit
        {
            break;
        }

        /**************************************************/
        /**************************************************/
        /**************************更新 error pixels ******/
        cur_time = get_ros_time(begin_time);
        dt = (cur_time - last_time);
        if (dt > 1.0 || dt < 0.0)
        {
            dt = 0.05;
        }
        std::cout << "tracker objects num: " << tracker_results.size() << std::endl;
        if (tracker_results.size() == 0)
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
            std::cout << "There is no tracked object" << std::endl;
        }
        else
        {
            /*********************************************/
            // 在跟踪之前 设置云台跟踪的目标 PTZ 云台相机
            STrack PTZ_track_obj = tracker_results[0];
            bool exist_obj_flag = get_unique_track_obj(frame, tracker_results, PTZ_track_obj); // return true if exists obj else return false
            flag_vision.x = 0x01;
            cv::Rect det_box = detect_results[0].box;                                                                       /*    TBD  找到和目标一致的 */
            cv::Rect track_box(PTZ_track_obj.tlwh[0], PTZ_track_obj.tlwh[1], PTZ_track_obj.tlwh[2], PTZ_track_obj.tlwh[3]); // x, y, width, heigh
            
            
            error_pixels.x = track_box.x + track_box.width / 2 - frame.cols / 2;
            error_pixels.y = track_box.y + track_box.height / 2 - frame.rows / 2;

            error_pixels.recsize = track_box.width * track_box.height;
            error_pixels.selectrec = det_box.width * det_box.height;

            float error_x = error_pixels.x;
            float error_y = error_pixels.y;

            derrorX.add_error(error_x, cur_time);
            derrorY.add_error(error_y, cur_time);
            derrorX.derror_output();
            derrorY.derror_output();
            derrorX.show_error();
            derrorY.show_error();

            error_pixels.velx = derrorX.Output;
            error_pixels.vely = derrorY.Output;

            error_pixels.Ix += error_pixels.x * dt;
            error_pixels.Iy += error_pixels.y * dt;

            unfilter_velx = (error_pixels.x - last_error_x) / dt;
            unfilter_vely = (error_pixels.y - last_error_y) / dt;

            last_time = cur_time;
            last_error_x = error_pixels.x;
            last_error_y = error_pixels.y;

            float left_point = frame.cols / 2 - 20;
            float right_point = frame.cols / 2 + 20;
            float up_point = frame.rows / 2 + 20;
            float down_point = frame.rows / 2 - 20;
            // draw
            cv::line(frame, cv::Point(left_point, frame.rows / 2), cv::Point(right_point, frame.rows / 2), cv::Scalar(0, 255, 0), 1, 8);
            cv::line(frame, cv::Point(frame.cols / 2, down_point), cv::Point(frame.cols / 2, up_point), cv::Scalar(0, 255, 0), 1, 8);
            cv::putText(frame, "x:", cv::Point(50, 60), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 23, 0), 3, 8);
            cv::putText(frame, "y:", cv::Point(50, 90), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 23, 0), 3, 8);

            // draw
            char s[20] = "";
            sprintf(s, "%.2f", error_pixels.x);
            cv::putText(frame, s, cv::Point(100, 60), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 23, 0), 2, 8);
            sprintf(s, "%.2f", error_pixels.y);
            cv::putText(frame, s, cv::Point(100, 90), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 23, 0), 2, 8);
        }
        position_diff_pub.publish(error_pixels);
        flag_version_pub.publish(flag_vision);

        num_frames++;
        ros::spinOnce();
    }
    // Release stream and buffers
    cudaStreamDestroy(stream);
    CUDA_CHECK(cudaFree(gpu_buffers[0]));
    CUDA_CHECK(cudaFree(gpu_buffers[1]));
    delete[] cpu_output_buffer;
    cuda_preprocess_destroy();
    // Destroy the engine
    context->destroy();
    engine->destroy();
    runtime->destroy();
    return 0;
}
