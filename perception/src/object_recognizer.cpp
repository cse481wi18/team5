#include <limits.h>
#include <math.h>
#include <string>
#include <vector>

#include "ros/ros.h"
#include "boost/filesystem.hpp"
#include "rosbag/bag.h"
#include "rosbag/view.h"
#include "perception/feature_extraction.h"
#include "perception_msgs/ObjectFeatures.h"
#include "perception/object_recognizer.h"
#include "perception/crop.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/filters/crop_box.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/common/common.h"

using boost::filesystem::directory_iterator;
using perception_msgs::ObjectFeatures;

namespace perception
{
  namespace
  {
    // This does the 3D Euclidean distance of two 3-dimensional vectors
    double EuclideanDistance(const std::vector<double> &v1,
        const std::vector<double> &v2)
    {
      double sum = 0;
      for (size_t i = 0; i < v1.size(); i++) {
        sum += pow(v1[i] - v2[i], 2);
      }
      return sqrt(sum);
    }
  }

  void LoadData(const std::string &data_dir,
      std::vector<perception_msgs::ObjectFeatures> *dataset)
  {
    directory_iterator end;
    for (directory_iterator file_it(data_dir); file_it != end; ++file_it)
    {
      if (boost::filesystem::is_regular_file(file_it->path()))
      {
        rosbag::Bag bag;
        bag.open(file_it->path().string(), rosbag::bagmode::Read);
        std::vector<std::string> topics;
        topics.push_back("object_features");
        rosbag::View view(bag, rosbag::TopicQuery(topics));

        for (rosbag::View::iterator it = view.begin(); it != view.end(); ++it)
        {
          ObjectFeatures::ConstPtr fp = it->instantiate<ObjectFeatures>();
          if (fp != NULL)
          {
            dataset->push_back(*fp);
          }
        }
      }
    }
  }

  ObjectRecognizer::ObjectRecognizer(const std::vector<ObjectFeatures> &dataset)
    : dataset_(dataset) {}

  void ObjectRecognizer::Recognize(const Object &object, std::string *name,
      double *confidence)
  {
    // TODO: extract features from the object
    geometry_msgs::Vector3 dimensions = object.dimensions;
    std::vector<double> object_dimensions;
    object_dimensions.push_back(dimensions.z);
    object_dimensions.push_back(dimensions.y);
    object_dimensions.push_back(dimensions.x);
    perception_msgs::ObjectFeatures features;

    perception::ExtractFeatures(object, &features);
    //perception::ExtractSizeFeatures(object, &features);

    //if(object.name == "galaxy_nexus") {
      /*ROS_INFO("x: %f", features.values[0]);
      ROS_INFO("y: %f", features.values[1]);
      ROS_INFO("z: %f", features.values[2]);*/
      //ROS_INFO("name: %s", object.name);
   // }

    double min_distance = std::numeric_limits<double>::max();
    double second_min_distance = std::numeric_limits<double>::max();
    for (size_t i = 0; i < dataset_.size(); ++i)
    {
      // TODO: compare the features of the input object to the features of the current dataset object.
      double distance = EuclideanDistance(features.values/*object_dimensions*/, dataset_[i].values);

      if (distance < min_distance)
      {
        second_min_distance = min_distance;
        min_distance = distance;
        *name = dataset_[i].object_name;
        
      }
      else if (distance < second_min_distance)
      {
        second_min_distance = distance;
      }
    }
    //ROS_INFO("name: %s, x: %f, y: %f, z: %f", (*name).c_str(),
    //features.values[0], features.values[1], features.values[2]);
    // Confidence is based on the distance to the two nearest results.
    *confidence = 1 - min_distance / (min_distance + second_min_distance);
  }
} // namespace perception
