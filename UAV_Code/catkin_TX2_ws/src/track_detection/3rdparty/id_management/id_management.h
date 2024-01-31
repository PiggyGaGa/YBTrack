#ifndef ID_MANAGEMENT_H
#define ID_MANAGEMENT_H

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/aruco.hpp>

namespace IDManagement {
  int read_id(cv::Mat frame);
  cv::Mat add_id(int id, cv::Mat frame);
}

#endif // ID_MANAGEMENT_H
