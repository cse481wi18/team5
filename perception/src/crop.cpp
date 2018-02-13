#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "perception/crop.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/filters/crop_box.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/common/common.h"

// Add these typedefs after your #includes
typedef pcl::PointXYZRGB PointC;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudC;


namespace perception
{
  Cropper::Cropper(const ros::Publisher& pub) : pub_(pub) {}

  void Cropper::Callback(const sensor_msgs::PointCloud2 &msg)
  {
    PointCloudC::Ptr cloud(new PointCloudC());
    pcl::fromROSMsg(msg, *cloud);
    ROS_INFO("Got point cloud with %ld points", cloud->size());

    PointCloudC::Ptr cropped_cloud(new PointCloudC());
    PointCloudC::Ptr downsampled_cloud(new PointCloudC());
    double min_x, min_y, min_z, max_x, max_y, max_z;
    ros::param::param("crop_min_x", min_x, 0.3);
    ros::param::param("crop_min_y", min_y, -1.0);
    ros::param::param("crop_min_z", min_z, 0.5);
    ros::param::param("crop_max_x", max_x, 0.9);
    ros::param::param("crop_max_y", max_y, 1.0);
    ros::param::param("crop_max_z", max_z, 1.5);
    Eigen::Vector4f min_pt(min_x, min_y, min_z, 1);
    Eigen::Vector4f max_pt(max_x, max_y, max_z, 1);
    pcl::CropBox<PointC> crop;
    crop.setInputCloud(cloud);
    crop.setMin(min_pt);
    crop.setMax(max_pt);
    crop.filter(*cropped_cloud);

    // min max stuff
    PointC min_pcl;
    PointC max_pcl;
    pcl::getMinMax3D<PointC>(*cropped_cloud, min_pcl, max_pcl);
    ROS_INFO("min: %f, max: %f", min_pcl.x, max_pcl.x);

    pcl::VoxelGrid<PointC> vox;
    vox.setInputCloud(cropped_cloud);
    double voxel_size;
    ros::param::param("voxel_size", voxel_size, 0.01);
    vox.setLeafSize(voxel_size, voxel_size, voxel_size);
    vox.filter(*downsampled_cloud);
    ROS_INFO("Cropped to %ld points", cropped_cloud->size());
    ROS_INFO("Cropped and downsampled to %ld points", downsampled_cloud->size());  

    sensor_msgs::PointCloud2 msg_out;
    pcl::toROSMsg(*downsampled_cloud, msg_out);
    pub_.publish(msg_out);
  }
  void GetMinMax(PointCloudC::Ptr input, double* min_x, double* max_x) {
    *min_x = std::numeric_limits<double>::max();
    *max_x = std::numeric_limits<double>::min();
    for (size_t i = 0; i < input->size(); ++i) {
      const PointC& pt = input->at(i);
      if (pt.x < *min_x) {
        *min_x = pt.x;
      }
      if (pt.x > *max_x) {
        *max_x = pt.x;
      }
    }
  }
}
