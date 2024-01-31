#include "id_management.h"
#include <vector>

namespace IDManagement {
  cv::Mat add_id(int id, cv::Mat frame)
  {
    cv::Mat markerImage;
    std::vector<cv::Mat> channel(3);
    cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_1000);
    channel[0]  = cv::Mat::zeros(24, 24, CV_8UC1);
    channel[1]  = cv::Mat::zeros(24, 24, CV_8UC1);
    channel[2]  = cv::Mat::zeros(24, 24, CV_8UC1);

    channel[0].setTo(cv::Scalar(255));
    channel[1].setTo(cv::Scalar(255));
    channel[2].setTo(cv::Scalar(255));
    cv::Rect myroi = cv::Rect(4, 4, 16, 16);
    cv::Rect idroi = cv::Rect(0, 0, 24, 24);

    int id_m = id/1000/1000;
    int id_k = id/1000-id_m*1000;
    int id_1 = id%1000%1000;

    dictionary.drawMarker(id_1, 16, markerImage, 1);
    markerImage.copyTo(channel[0](myroi));
    dictionary.drawMarker(id_k, 16, markerImage, 1);
    markerImage.copyTo(channel[1](myroi));
    dictionary.drawMarker(id_m, 16, markerImage, 1);
    markerImage.copyTo(channel[2](myroi));

    cv::Mat idmark;
    merge(channel, idmark);
    idmark.copyTo(frame(idroi));
    return frame;
  }

  int read_id(cv::Mat frame)
  {
    int id;
    cv::Mat idmark;
    std::vector<cv::Mat> channel(3);
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_1000);
    cv::Rect idroi = cv::Rect(0, 0, 24, 24);
    frame(idroi).copyTo(idmark);
    split(idmark, channel);

    std::vector< int > id_1;
    std::vector< int > id_k;
    std::vector< int > id_m;
    std::vector< std::vector<cv::Point2f> > markerCorners;
    cv::aruco::detectMarkers(channel[0], dictionary, markerCorners, id_1);
    if(id_1.size()>0) {
        cv::aruco::detectMarkers(channel[1], dictionary, markerCorners, id_k);
        if(id_k.size()>0) {
            cv::aruco::detectMarkers(channel[2], dictionary, markerCorners, id_m);
            if(id_m.size()>0) {
                id = id_m[0]*1000*1000+id_k[0]*1000+id_1[0];
            }
            else id = -1;
        }
        else id = -1;
    }
    else id = -1;

    return id;
  }
}
