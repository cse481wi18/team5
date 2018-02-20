#include <vector>

#include "perception/segmentation.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "visualization_msgs/Marker.h"
#include "perception/object_recognizer.h"
#include "perception_msgs/ObjectFeatures.h"

int main(int argc, char **argv)
{
  if (argc < 2)
  {
    ROS_INFO("Usage: rosrun perception point_cloud_demo DATA_DIR");
    ros::spinOnce();
  }
  std::string data_dir(argv[1]);
  ros::init(argc, argv, "point_cloud_demo");
  ros::NodeHandle nh;
  ros::Publisher table_pub = nh.advertise<sensor_msgs::PointCloud2>("table_cloud", 1, true);
  ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1, true);
  ros::Publisher above_table_pub = nh.advertise<sensor_msgs::PointCloud2>("above_table_cloud", 1, true);

  // Create the object recognizer.
  std::vector<perception_msgs::ObjectFeatures> dataset;
  perception::LoadData(data_dir, &dataset);
  perception::ObjectRecognizer recognizer(dataset);

  perception::Segmenter segmenter(&table_pub, &marker_pub, &above_table_pub, &recognizer);

  ros::Subscriber sub = nh.subscribe("cloud_in", 1, &perception::Segmenter::Callback, &segmenter);

  ros::spin();
  return 0;
}
