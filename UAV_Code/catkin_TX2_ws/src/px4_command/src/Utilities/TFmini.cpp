#include <TFmini.h>
#include "ros/ros.h"
#include <sstream>

using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "TFmini");
  ros::NodeHandle nh("~");
  std::string frame_id;
  std::string portName;
  int baud_rate;
  benewake::TFmini *tfmini_obj;

  nh.param("serial_port", portName, std::string("/dev/ttyUSB0"));
  nh.param("baud_rate", baud_rate, 115200);

  nh.param("frame_id", frame_id, std::string("tfmini_link"));

  tfmini_obj = new benewake::TFmini(portName, baud_rate);
  ros::Publisher pub_range = nh.advertise<sensor_msgs::Range>(frame_id, 1000, true);
  sensor_msgs::Range TFmini_range;
  TFmini_range.radiation_type = sensor_msgs::Range::INFRARED;
  TFmini_range.field_of_view = 0.04;
  TFmini_range.min_range = 0.1;
  TFmini_range.max_range = 12;
  TFmini_range.header.frame_id = frame_id;
  float dist = 0;
  ROS_INFO_STREAM("Start processing ...");

  while(ros::master::check() && ros::ok())
  {
    ros::spinOnce();
    dist = tfmini_obj->getDist();
    if(dist > 0 && dist < TFmini_range.max_range)
    {
      TFmini_range.range = dist;
      TFmini_range.header.stamp = ros::Time::now();
      pub_range.publish(TFmini_range);

      std::cout << "Height: " << dist*100 << " [cm]" << std::endl;
    }
    else if(dist == -1.0)
    {
      ROS_ERROR_STREAM("Failed to read data. TFmini ros node stopped!");
      break;
    }
    else if(dist == 0.0)
    {
      ROS_ERROR_STREAM("Data validation error!Got dist=0 means Strength is low,date isn't reliable.");
    }
  }

  tfmini_obj->closePort();
}
