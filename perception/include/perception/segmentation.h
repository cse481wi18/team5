#include "pcl/PointIndices.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/ModelCoefficients.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Vector3.h"
#include "perception/object.h"


namespace perception {
  class Segmenter {
    public:
      Segmenter(ros::Publisher *surface_points_pub, 
                ros::Publisher *marker_pub, 
                ros::Publisher *above_surface_pub);

      void Callback(const sensor_msgs::PointCloud2& msg);

      // Finds the largest horizontal surface in the given point cloud.
      // This is useful for adding a collision object to MoveIt.
      //
      // Args:
      //  cloud: The point cloud to extract a surface from.
      //  indices: The indices of points in the point cloud that correspond to the
      //    surface. Empty if no surface was found.
      void SegmentSurface(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                          pcl::PointIndices::Ptr indices,
                          pcl::ModelCoefficients::Ptr coeff,
                          std::vector<Object>* objects);

      // Computes the axis-aligned bounding box of a point cloud.
      //
      // Args:
      //  cloud: The point cloud
      //  pose: The output pose. Because this is axis-aligned, the orientation is just
      //    the identity. The position refers to the center of the box.
      //  dimensions: The output dimensions, in meters.
      void GetAxisAlignedBoundingBox(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                                    geometry_msgs::Pose* pose,
                                    geometry_msgs::Vector3* dimensions);


      // Finds object surfaces in the given point cloud
      //
      // Args:
      //  cloud: The point cloud
      //  surface_indices:
      //  object_indicies:
      void SegmentSurfaceObjects(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                                pcl::PointIndices::Ptr surface_indices,
                                std::vector<Object>* objects,
                                pcl::ModelCoefficients::Ptr coeff);

      // Does a complete tabletop segmentation pipeline.
      //
      // Args:
      //  cloud: The point cloud with the surface and the objects above it.
      //  objects: The output objects.
      void SegmentTabletopScene(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                                std::vector<Object>* objects);
    private:
      ros::Publisher *surface_points_pub_;
      ros::Publisher *marker_pub_;
      ros::Publisher *above_surface_pub_;
  };
} // namespace perception
