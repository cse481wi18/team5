#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "perception_crop/crop.h"

namespace perception {
  Cropper::Cropper() {}

  void Cropper::Callback(const sensor_msgs::PointCloud2& msg) {
      ROS_INFO("Got point cloud");
  }
}
