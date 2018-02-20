#include "perception/box_fitter.h"
#include "perception/segmentation.h"
#include "pcl/PointIndices.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/common/common.h"
#include "pcl/common/angles.h"
#include "pcl/sample_consensus/method_types.h"
#include "pcl/sample_consensus/model_types.h"
#include "pcl/segmentation/sac_segmentation.h"
#include "pcl/segmentation/extract_clusters.h"
#include "pcl/filters/extract_indices.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/filters/crop_box.h"
#include "pcl_conversions/pcl_conversions.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Vector3.h"
#include "visualization_msgs/Marker.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "shape_msgs/SolidPrimitive.h"
#include "simple_grasping/shape_extraction.h"
#include "perception/object.h"

typedef pcl::PointXYZRGB PointC;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudC;

namespace perception {

  void Segmenter::SegmentSurface(PointCloudC::Ptr cloud,
      pcl::PointIndices::Ptr indices,
      pcl::ModelCoefficients::Ptr coeff,
      std::vector<Object>* objects) {
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
    PointCloudC::Ptr extract_out(new PointCloudC);
    shape_msgs::SolidPrimitive shape;
    geometry_msgs::Pose table_pose;

    // coeff contains the coefficients of the plane: ax + by + cz + d = 0
    seg.segment(indices_internal, *coeff);
    //simple_grasping::extractShape(*cropped_cloud, coeff, *extract_out, shape, table_pose);
    perception::FitBox(*cropped_cloud, coeff, *extract_out, shape, table_pose);
    double distance_above_plane;
    ros::param::param("distance_above_plane", distance_above_plane, 0.005);

    // Build custom indices that ignores points above the plane.
    for (size_t i = 0; i < cropped_cloud->size(); ++i) {
      const PointC& pt = cropped_cloud->points[i];
      float val = coeff->values[0] * pt.x + coeff->values[1] * pt.y +
        coeff->values[2] * pt.z + coeff->values[3];
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
    if (surface_points_pub_) surface_points_pub_->publish(msg_out);

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
      float val = coeff->values[0] * pt.x + coeff->values[1] * pt.y +
        coeff->values[2] * pt.z + coeff->values[3];
      if (val <= distance_above_plane) {
        indices2->indices.push_back(i);
      }
    }

    //simple_grasping::extractShape(*subset_cloud, coeff, *extract_out, shape, table_pose);
    perception::FitBox(*subset_cloud, coeff, *extract_out, shape, table_pose);

    visualization_msgs::Marker table_marker;
    table_marker.ns = "table";
    table_marker.header.frame_id = "base_link";
    table_marker.type = visualization_msgs::Marker::CUBE;
    table_marker.pose = table_pose;
    if (shape_msgs::SolidPrimitive::BOX == shape.type) {
      table_marker.scale.x = shape.dimensions[0];
      table_marker.scale.y = shape.dimensions[1];
      table_marker.scale.z = shape.dimensions[2];
    }
    table_marker.color.r = 1;
    table_marker.color.a = 0.8;
    table_marker.pose.position.z -= table_marker.scale.z;
    if (marker_pub_) marker_pub_->publish(table_marker);

    //std::vector<pcl::PointIndices> object_indices;
    SegmentSurfaceObjects(subset_cloud, indices2, objects, coeff); // TO BE CHANGED
  }

  void Segmenter::GetAxisAlignedBoundingBox(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
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

  }

  void Segmenter::SegmentSurfaceObjects(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
      pcl::PointIndices::Ptr surface_indices,
      std::vector<Object>* objects,
      pcl::ModelCoefficients::Ptr coeff) {

    std::vector<pcl::PointIndices> object_indices; // new line here
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
    euclid.extract(object_indices);

    // Find the size of the smallest and the largest object,
    // where size = number of points in the cluster
    /*size_t min_size = std::numeric_limits<size_t>::max();
      size_t max_size = std::numeric_limits<size_t>::min();

      size_t min, max = (*object_indices)[0].indices.size();
      for (size_t i = 1; i < object_indices->size(); ++i) {
      size_t curr_sz = (*object_indices)[i].indices.size();
      if(curr_sz > max) {
      max = curr_sz;
      }
      if(curr_sz < min) {
      min = curr_sz;
      }
      }*/

    ROS_INFO("There are %ld points above the table", above_surface_indices->indices.size());

    sensor_msgs::PointCloud2 msg_out;
    pcl::toROSMsg(*cloud, msg_out);
    if (above_surface_pub_) above_surface_pub_->publish(msg_out);


    for (size_t i = 0; i < object_indices.size(); ++i) {
      // Reify indices into a point cloud of the object.

      // We need the following fields to be filled up.
      /*  std::string name;
          double confidence;
          $ pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
          $ geometry_msgs::Pose pose;
          $ geometry_msgs::Vector3 dimensions; */


      pcl::PointIndices::Ptr indices(new pcl::PointIndices);
      *indices = object_indices[i];
      PointCloudC::Ptr object_cloud(new PointCloudC());

      extract.setIndices(indices);
      extract.setNegative(false);
      extract.filter(*object_cloud);




      // Publish a bounding box around it.
      /*visualization_msgs::Marker object_marker;
        object_marker.ns = "objects";
        object_marker.id = i;
        object_marker.header.frame_id = "base_link";
        object_marker.type = visualization_msgs::Marker::CUBE;*/

      PointCloudC::Ptr extract_out(new PointCloudC);
      shape_msgs::SolidPrimitive shape;
      geometry_msgs::Pose object_pose;
      perception::FitBox(*object_cloud, coeff, *extract_out, shape, object_pose);

      // new code using object vector

      Object obj;
      obj.cloud = object_cloud;
      obj.pose = object_pose;



      //object_marker.pose = object_pose;
      if (shape_msgs::SolidPrimitive::BOX == shape.type) {
        geometry_msgs::Vector3 obj_dim;
        obj_dim.x = shape.dimensions[0];
        obj_dim.y = shape.dimensions[1];
        obj_dim.z = shape.dimensions[2];
        /*object_marker.scale.x = shape.dimensions[0];
          object_marker.scale.y = shape.dimensions[1];
          object_marker.scale.z = shape.dimensions[2];*/
        obj.dimensions = obj_dim;

      }

      objects->push_back(obj);

      //object_marker.color.g = 1;
      //object_marker.color.a = 0.3;
      //marker_pub_->publish(object_marker);
    }

  }

  void Segmenter::SegmentTabletopScene(PointCloudC::Ptr cloud, std::vector<Object>* objects) {
    // Same as callback, but with visualization code removed.
    pcl::PointIndices::Ptr table_inliers(new pcl::PointIndices());
    pcl::ModelCoefficients::Ptr coeff(new pcl::ModelCoefficients());
    SegmentSurface(cloud, table_inliers, coeff, objects);     
  }

  Segmenter::Segmenter(ros::Publisher *surface_points_pub, ros::Publisher *marker_pub, ros::Publisher *above_surface_pub, ObjectRecognizer* recognizer) : 
    surface_points_pub_(surface_points_pub), marker_pub_(marker_pub), above_surface_pub_(above_surface_pub), recognizer_(recognizer)  {}

  void Segmenter::Callback(const sensor_msgs::PointCloud2& msg) {
    /*PointCloudC::Ptr cloud(new PointCloudC());
      pcl::fromROSMsg(msg, *cloud);

      pcl::PointIndices::Ptr table_inliers(new pcl::PointIndices());
      pcl::ModelCoefficients::Ptr coeff(new pcl::ModelCoefficients());
      SegmentSurface(cloud, table_inliers, coeff); */

    PointCloudC::Ptr cloud_unfiltered(new PointCloudC());
    pcl::fromROSMsg(msg, *cloud_unfiltered);
    PointCloudC::Ptr cloud(new PointCloudC());
    std::vector<int> index;
    pcl::removeNaNFromPointCloud(*cloud_unfiltered, *cloud, index);

    std::vector<Object> objects;
    SegmentTabletopScene(cloud, &objects);

    for (size_t i = 0; i < objects.size(); ++i) {
      const Object& object = objects[i];

      // Publish a bounding box around it.
      visualization_msgs::Marker object_marker;
      object_marker.ns = "objects";
      object_marker.id = i;
      object_marker.header.frame_id = "base_link";
      object_marker.type = visualization_msgs::Marker::CUBE;
      object_marker.pose = object.pose;
      object_marker.scale = object.dimensions;
      object_marker.color.g = 1;
      object_marker.color.a = 0.3;
      if (marker_pub_) marker_pub_->publish(object_marker);

      // Recognize the object
      std::string name;
      double confidence;
      if (recognizer_) recognizer_->Recognize(object, &name, &confidence);
      confidence = round(1000 * confidence) / 1000;

      std::stringstream ss;
      ss << name << " (" << confidence << ")";

      // Publish the recognition result.
      visualization_msgs::Marker name_marker;
      name_marker.ns = "recognition";
      name_marker.id = i;
      name_marker.header.frame_id = "base_link";
      name_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      name_marker.pose.position = object.pose.position;
      name_marker.pose.position.z += 0.1;
      name_marker.pose.orientation.w = 1;
      name_marker.scale.x = 0.025;
      name_marker.scale.y = 0.025;
      name_marker.scale.z = 0.025;
      name_marker.color.r = 0;
      name_marker.color.g = 0;
      name_marker.color.b = 1.0;
      name_marker.color.a = 1.0;
      name_marker.text = ss.str();
      if (marker_pub_) marker_pub_->publish(name_marker);
    }

  }
}  // namespace perception


