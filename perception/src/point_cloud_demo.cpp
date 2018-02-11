#include "perception/crop.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "point_cloud_demo");
  ros::NodeHandle nh;
  ros::Publisher crop_pub =
      nh.advertise<sensor_msgs::PointCloud2>("cropped_cloud", 1, true);
  perception::Cropper cropper(crop_pub);
  // cloud_in for now, we will update this to a real cloud source when running
  ros::Subscriber sub =
      nh.subscribe("cloud_in", 1, &perception::Cropper::Callback, &cropper);
  ros::spin();
  return 0;
}
