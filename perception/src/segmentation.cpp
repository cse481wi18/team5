#include "perception/segmentation.h"

#include "pcl/PointIndices.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "pcl/common/angles.h"
#include "pcl/sample_consensus/method_types.h"
#include "pcl/sample_consensus/model_types.h"
#include "pcl/segmentation/sac_segmentation.h"
#include "pcl/segmentation/extract_clusters.h"
#include "pcl/filters/extract_indices.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Vector3.h"
#include "pcl/common/common.h"
#include "visualization_msgs/Marker.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/filters/crop_box.h"

typedef pcl::PointXYZRGB PointC;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudC;

namespace perception {
void SegmentSurface(PointCloudC::Ptr cloud, pcl::PointIndices::Ptr indices, const ros::Publisher& surface_points_pub, const ros::Publisher& marker_pub, const ros::Publisher& above_surface_pub) {
  ROS_INFO("Got point cloud with %ld points", cloud->size());

  PointCloudC::Ptr cropped_cloud(new PointCloudC());
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
  
  pcl::PointIndices indices_internal;
  pcl::SACSegmentation<PointC> seg;
  seg.setOptimizeCoefficients(true);
  // Search for a plane perpendicular to some axis (specified below).
  seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  // Set the distance to the plane for a point to be an inlier.
  seg.setDistanceThreshold(0.01);
  seg.setInputCloud(cropped_cloud);

  // Make sure that the plane is perpendicular to Z-axis, 10 degree tolerance.
  Eigen::Vector3f axis;
  axis << 0, 0, 1;
  seg.setAxis(axis);
  seg.setEpsAngle(pcl::deg2rad(10.0));

  // coeff contains the coefficients of the plane:
  // ax + by + cz + d = 0
  pcl::ModelCoefficients coeff;
  seg.segment(indices_internal, coeff);

  for (size_t i=0; i<indices_internal.indices.size(); ++i) {
    int index = indices_internal.indices[i];
    const PointC& pt = cropped_cloud->points[index];
  }

  

  double distance_above_plane;
  ros::param::param("distance_above_plane", distance_above_plane, 0.005);

  // Build custom indices that ignores points above the plane.
  for (size_t i = 0; i < cropped_cloud->size(); ++i) {
    const PointC& pt = cropped_cloud->points[i];
    float val = coeff.values[0] * pt.x + coeff.values[1] * pt.y +
                coeff.values[2] * pt.z + coeff.values[3];
    if (val <= distance_above_plane) {
      indices->indices.push_back(i);
    }
  }

  //*indices = indices_internal;
 
  if (indices->indices.size() == 0) {
    ROS_ERROR("Unable to find surface.");
    return;
  }  

  PointCloudC::Ptr subset_cloud(new PointCloudC);
  // Extract subset of original_cloud into subset_cloud:
  pcl::ExtractIndices<PointC> extract;
  extract.setInputCloud(cropped_cloud);
  extract.setIndices(indices);
  extract.filter(*subset_cloud);

  sensor_msgs::PointCloud2 msg_out;
  pcl::toROSMsg(*subset_cloud, msg_out);
  surface_points_pub.publish(msg_out);

  extract.setNegative(true); //added for 31
  extract.filter(*subset_cloud); //added for 31

  PointCloudC::Ptr downsampled_cloud(new PointCloudC());
  pcl::VoxelGrid<PointC> vox;
  vox.setInputCloud(subset_cloud);
  double voxel_size;
  ros::param::param("voxel_size", voxel_size, 0.01);
  vox.setLeafSize(voxel_size, voxel_size, voxel_size);
  vox.filter(*downsampled_cloud);


  pcl::PointIndices::Ptr indices2(new pcl::PointIndices());
  for (size_t i = 0; i < downsampled_cloud->size(); ++i) {
    const PointC& pt = downsampled_cloud->points[i];
    float val = coeff.values[0] * pt.x + coeff.values[1] * pt.y +
                coeff.values[2] * pt.z + coeff.values[3];
    if (val <= distance_above_plane) {
      indices2->indices.push_back(i);
    }
  }

  

  visualization_msgs::Marker table_marker;
  table_marker.ns = "table";
  table_marker.header.frame_id = "base_link";
  table_marker.type = visualization_msgs::Marker::CUBE;
  GetAxisAlignedBoundingBox(subset_cloud, &table_marker.pose, &table_marker.scale);
  table_marker.color.r = 1;
  table_marker.color.a = 0.8;
  //marker_pub.publish(table_marker);

  //ROS_INFO("Subset cloud sz: %ld, indices sz: %ld", subset_cloud->size(), *(indices).size());

  std::vector<pcl::PointIndices> object_indices;
  SegmentSurfaceObjects(downsampled_cloud, indices2, &object_indices, above_surface_pub, marker_pub);

}


// Computes the axis-aligned bounding box of a point cloud.
//
// Args:
//  cloud: The point cloud
//  pose: The output pose. Because this is axis-aligned, the orientation is just
//    the identity. The position refers to the center of the box.
//  dimensions: The output dimensions, in meters.
void GetAxisAlignedBoundingBox(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                               geometry_msgs::Pose* pose,
                               geometry_msgs::Vector3* dimensions) {
   PointC min_pcl;
   PointC max_pcl;
   pcl::getMinMax3D<PointC>(*cloud, min_pcl, max_pcl);
   pose->orientation.w = 1;
   pose->position.x = (min_pcl.x + max_pcl.x)/2;
   pose->position.y = (min_pcl.y + max_pcl.y)/2;
   pose->position.z = (min_pcl.z + max_pcl.z)/2;

   dimensions->x = max_pcl.x - min_pcl.z;
   dimensions->y = max_pcl.y - min_pcl.y;
   dimensions->z = max_pcl.z - min_pcl.z;
  
};

void SegmentSurfaceObjects(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                           pcl::PointIndices::Ptr surface_indices,
                           std::vector<pcl::PointIndices>* object_indices, const ros::Publisher& above_surface_pub, const ros::Publisher& marker_pub) {
  pcl::ExtractIndices<PointC> extract;
  pcl::PointIndices::Ptr above_surface_indices(new pcl::PointIndices());
  extract.setInputCloud(cloud);
  extract.setIndices(surface_indices);
  extract.setNegative(true);
  extract.filter(above_surface_indices->indices);

  ROS_INFO("There are %ld points above the table", above_surface_indices->indices.size());

  double cluster_tolerance;
  int min_cluster_size, max_cluster_size;
  ros::param::param("ec_cluster_tolerance", cluster_tolerance, 0.01);
  ros::param::param("ec_min_cluster_size", min_cluster_size, 10);
  ros::param::param("ec_max_cluster_size", max_cluster_size, 10000);

  pcl::EuclideanClusterExtraction<PointC> euclid;
  euclid.setInputCloud(cloud);
  euclid.setIndices(above_surface_indices);
  euclid.setClusterTolerance(cluster_tolerance);
  euclid.setMinClusterSize(min_cluster_size);
  euclid.setMaxClusterSize(max_cluster_size);
  euclid.extract(*object_indices);

  // Find the size of the smallest and the largest object,
  // where size = number of points in the cluster
  size_t min_size = std::numeric_limits<size_t>::max();
  size_t max_size = std::numeric_limits<size_t>::min();

  size_t min, max = (*object_indices)[0].indices.size();
  for (size_t i = 1; i < object_indices->size(); ++i) {
    // TODO: implement this
    size_t curr_sz = (*object_indices)[i].indices.size();
    if(curr_sz > max) {
      max = curr_sz;
    }
    if(curr_sz < min) {
      min = curr_sz;
    }
  }


  ROS_INFO("There are %ld points above the table", above_surface_indices->indices.size());

  sensor_msgs::PointCloud2 msg_out;
  pcl::toROSMsg(*cloud, msg_out);
  above_surface_pub.publish(msg_out);


  for (size_t i = 0; i < object_indices->size(); ++i) {
    // Reify indices into a point cloud of the object.
    pcl::PointIndices::Ptr indices(new pcl::PointIndices);
    *indices = (*object_indices)[i];
    PointCloudC::Ptr object_cloud(new PointCloudC());
    // TODO: fill in object_cloud using indices
    extract.setIndices(indices);
    extract.setNegative(false);
    extract.filter(*object_cloud);
    // Publish a bounding box around it.
    visualization_msgs::Marker object_marker;
    object_marker.ns = "objects";
    object_marker.id = i;
    object_marker.header.frame_id = "base_link";
    object_marker.type = visualization_msgs::Marker::CUBE;
    GetAxisAlignedBoundingBox(object_cloud, &object_marker.pose,
                            &object_marker.scale);
    object_marker.color.g = 1;
    object_marker.color.a = 0.3;
    marker_pub.publish(object_marker);
  }

}




Segmenter::Segmenter(const ros::Publisher& surface_points_pub, const ros::Publisher& marker_pub, const ros::Publisher& above_surface_pub) : 
  surface_points_pub_(surface_points_pub), marker_pub_(marker_pub), above_surface_pub_(above_surface_pub)  {}

void Segmenter::Callback(const sensor_msgs::PointCloud2& msg) {
  PointCloudC::Ptr cloud(new PointCloudC());
  pcl::fromROSMsg(msg, *cloud);

  pcl::PointIndices::Ptr table_inliers(new pcl::PointIndices());
  SegmentSurface(cloud, table_inliers, surface_points_pub_, marker_pub_, above_surface_pub_);

  
}
}  // namespace perception
