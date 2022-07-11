#ifndef PCL_UTIL_H
#define PCL_UTIL_H

#include <ros/ros.h>
#include <vision_msgs/BoundingBox2D.h>
#include <geometry_msgs/Pose2D.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf2_ros/transform_listener.h>

// PCL specific includes
#include <pcl/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/passthrough.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
// for clustering, extracting
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

//PCL ROS includes
#include <pcl_ros/transforms.h>

//pcl conversion includes
#include <pcl_conversions/pcl_conversions.h>

#include <stdlib.h>
#include <stdio.h>
#include <chrono>


typedef pcl::PointXYZRGB PointType_XYZRGB;
typedef pcl::PointCloud<PointType_XYZRGB> Cloud_XYZRGB;
typedef Cloud_XYZRGB::Ptr CloudPtr_XYZRGB;

typedef pcl::PointXYZ PointType_XYZ;
typedef pcl::PointCloud<PointType_XYZ> Cloud_XYZ;
typedef Cloud_XYZ::Ptr CloudPtr_XYZ;

typedef struct{
    int xmin;
    int xmax;
    int ymin;
    int ymax;
} Box2D_PixelCoords;

// depth-image pixel value
// remember image has a width of 640 pixels and a height of 480 pixels (480 x 640)
typedef struct{
    int32_t x;
    int32_t y;
    double z;
} PixelCoords;

typedef struct{
    float xmin;
    float xmax;
    float ymin;
    float ymax;
    float zmin;
    float zmax;
} Box3D_CloudCoords;


Box2D_PixelCoords extractBBoxInfo(const vision_msgs::BoundingBox2DConstPtr &msg);

CloudPtr_XYZRGB filterPoints(const CloudPtr_XYZRGB input,const std::vector<PixelCoords> &pixel_coordinates, Box2D_PixelCoords limits);
CloudPtr_XYZRGB filterPoints(const CloudPtr_XYZRGB input, Box3D_CloudCoords limits);

pcl::PointIndices::Ptr findIndices(const std::vector<PixelCoords> &pixel_coordinates, Box2D_PixelCoords limits);
//TODO why doesn't this input need to have an & in front of it like in pixel_coordinates above?
pcl::PointIndices::Ptr findIndices(const CloudPtr_XYZRGB input, Box3D_CloudCoords limits);

PixelCoords poseToPixel(const PointType_XYZRGB &point,const sensor_msgs::CameraInfo &camera_info);
std::vector<PixelCoords> convertCloudToPixelCoords(const CloudPtr_XYZRGB cloudPtr,
                                        const sensor_msgs::CameraInfo &camera_info);

bool transformPointCloud2(sensor_msgs::PointCloud2 &pointcloud,
                          const std::string target_frame, tf2_ros::Buffer &tf_buffer);

CloudPtr_XYZ filterPlane(const CloudPtr_XYZ input);

Cloud_XYZ getCentroid(const CloudPtr_XYZRGB cloud);




// TODO do i need this?
/*
namespace pcl_util_ns{

class Pcl_Util{

    public:
        //Constructor
        Pcl_Util(int argc, char** argv);
        ~Pcl_Util();

    private:

};

}
*/

#endif