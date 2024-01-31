cmake_minimum_required(VERSION 3.5)

project(track_detection LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_definitions(-std=c++11)
add_definitions(-DAPI_EXPORTS)


option(CUDA_USE_STATIC_CUDA_RUNTIME OFF)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE Debug)

set(CMAKE_CUDA_COMPILER /usr/local/cuda/bin/nvcc)
enable_language(CUDA)

# include and link dirs of cuda and tensorrt, you need adapt them if yours are different
# cuda
include_directories(/usr/local/cuda/include)
link_directories(/usr/local/cuda/lib64)
# tensorrt
# TODO(Call for PR): make TRT path configurable from command line
# include_directories(/home/nvidia/TensorRT-8.2.5.1/include/)
# link_directories(/home/nvidia/TensorRT-8.2.5.1/lib/)

include_directories(3rdparty/TensorRT_YOLOv5/src/)
include_directories(3rdparty/TensorRT_YOLOv5/plugin/)
file(GLOB_RECURSE MODEL_SRCS 3rdparty/TensorRT_YOLOv5/src/*.cpp 3rdparty/TensorRT_YOLOv5/src/*.cu)
file(GLOB_RECURSE PLUGIN_SRCS 3rdparty/TensorRT_YOLOv5/plugin/*.cu)



find_package(catkin REQUIRED COMPONENTS
    message_generation
    rostime
    roscpp
    rosconsole
    roscpp_serialization
    sensor_msgs
    image_transport
    cv_bridge
)

find_package(Boost REQUIRED COMPONENTS
    thread
    date_time
)

find_package (Eigen3 3.2 REQUIRED NO_MODULE)
set(OpenCV_DIR "/home/nvidia/SoftWare/OpenCV4.6/lib/cmake/opencv4")
find_package(OpenCV 4.6 REQUIRED )

set(ONNXRUNTIME_DIR "/home/nvidia/SoftWare/onnxruntime-linux-aarch64-1.12.1")


#message(${OpenCV_LIBS})

message(status "OpenCV Version: ${OpenCV_VERSION}")
message(status "OpenCV Include DIR: ${OpenCV_INCLUDE_DIRS}")
message(status "OpenCV LIB_DIR: ${OpenCV_LIB_DIR}")
message(status "OpenCV LIBS: ${OpenCV_LIBS}")

add_message_files(
FILES
Diff.msg
)

generate_messages(
DEPENDENCIES
std_msgs
)

catkin_package(
#INCLUDE_DIRS include
  LIBRARIES cv_bridge
  CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
)

message(eigen3 ${EIGEN3_INCLUDE_DIRS})

include_directories(
    include
    /usr/local/cuda/include
    ${OpenCV_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    ${Boost_INUCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    3rdparty/tracker/deepsort/include
    3rdparty/tracker/bytetrack/include
    3rdparty/OpenCV_YOLOv5/include
    3rdparty/id_management
    "${ONNXRUNTIME_DIR}/include"
)

### 生成相关的动态链接库



add_library(myplugins SHARED ${PLUGIN_SRCS})
target_link_libraries(myplugins nvinfer cudart)


add_library(id_management 
    3rdparty/id_management/id_management.cpp
)
target_link_libraries(id_management ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBS})

add_library(opencv_deepsort_lib
    3rdparty/OpenCV_YOLOv5/src/YOLOv5Detector.cpp
    3rdparty/tracker/deepsort/src/FeatureTensor.cpp
    3rdparty/tracker/deepsort/src/model.cpp
    3rdparty/tracker/deepsort/src/kalmanfilter.cpp
    3rdparty/tracker/deepsort/src/linear_assignment.cpp
    3rdparty/tracker/deepsort/src/nn_matching.cpp
    3rdparty/tracker/deepsort/src/track.cpp
    3rdparty/tracker/deepsort/src/tracker.cpp
    3rdparty/tracker/deepsort/src/munkres.cpp
    3rdparty/tracker/deepsort/src/hungarianoper.cpp
)

add_library(opencv_bytetrack_lib
    3rdparty/tracker/bytetrack/src/BytekalmanFilter.cpp
    3rdparty/tracker/bytetrack/src/BYTETracker.cpp
    3rdparty/tracker/bytetrack/src/lapjv.cpp
    3rdparty/tracker/bytetrack/src/STrack.cpp
    3rdparty/tracker/bytetrack/src/utils.cpp
)



### 生成可执行程序


add_executable(cam_node node/cam_node.cpp)
target_link_libraries(cam_node  id_management)

add_executable(opencv_track_node node/opencv_track.cpp)
target_link_libraries(opencv_track_node opencv_deepsort_lib opencv_bytetrack_lib ${catkin_LIBRARIES} "${ONNXRUNTIME_DIR}/lib/libonnxruntime.so" ${OpenCV_LIBS} ${Boost_LIBRARIES} Eigen3::Eigen)


add_executable(yolov5_opencv_detector_node
    3rdparty/OpenCV_YOLOv5/src/YOLOv5Detector.cpp
    node/yolov5_opencv_detector.cpp)
target_link_libraries(yolov5_opencv_detector_node PRIVATE  ${OpenCV_LIBS} Eigen3::Eigen)

# test_tensorrt_detect
add_executable(tensorrt_detector_node node/tensorrt_detect.cpp ${MODEL_SRCS})
target_link_libraries(tensorrt_detector_node nvinfer cudart myplugins ${OpenCV_LIBS})


# test_tensorrt_track
add_executable(tensorrt_track_node node/tensorrt_track.cpp ${MODEL_SRCS})
target_link_libraries(tensorrt_track_node nvinfer cudart myplugins opencv_bytetrack_lib ${catkin_LIBRARIES} ${OpenCV_LIBS} ${Boost_LIBRARIES} Eigen3::Eigen)
