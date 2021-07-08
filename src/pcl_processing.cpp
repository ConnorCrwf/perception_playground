#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <sensor_msgs/CameraInfo.h>
// #include <vision_msgs/Detection2DArray.h>

//no file-specific header necessary. everything is declaread and defined here. not great practice

// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/centroid.h>

#include <chrono>

// macros
#define UNKNOWN_OBJECT_ID -1

// typedefs
typedef pcl::PointXYZRGB PointType;
typedef pcl::PointCloud<PointType> Cloud;
typedef Cloud::Ptr CloudPtr;

// if true, we will publish the FoV and bounding box pointclouds
bool debug_pcl;
float z_max;
std::chrono::high_resolution_clock debug_clock_;

// the optical frame of the RGB camera (not the link frame)
//TOOD go to to rviz and look for available optical frame names it's left something
std::string rgb_optical_frame_;

// ROS Nodehandle
ros::NodeHandle *nh;

// Publishers
//for publishing filtered output whether that be a pointcloud or a different message type
ros::Publisher filtered_pcl_pub_;

// Initialize transform listener 
// TODO ask Chris what this is
std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
tf2_ros::Buffer tf_buffer_;

// caches for callback data
sensor_msgs::CameraInfo camera_info_;

// depth-image pixel value
typedef struct
{
    int32_t x;
    int32_t y;
    double z;
} PixelCoords;


/**
 * @brief Callback function for the RGB camera info
 * @details Specifically, this node needs the width, height,
 *          and intrinsic matrix of the camera.
 * @param msg Camera info
 * @post The message is copied to a local cache
 */
void cameraInfoCb(const sensor_msgs::CameraInfoConstPtr msg)
{
    camera_info_ = *msg;
}


/**
 * @brief Convert a cartesian point in the camera optical frame to (x,y,depth) pixel coordinates.
 * @details Note: Make sure the point is in the optical camera frame, and not the link frame.
 *          We also provide the depth value of the pixel.
 * @param point The cartesian point to convert
 * @param camera_info The camera information. Specifically we use the
 *                    intrinsic matrix.
 * @return The (x,y,depth) pixel coordinates.
 */
inline PixelCoords poseToPixel(const PointType &point,
                               const sensor_msgs::CameraInfo &camera_info)
{
    PixelCoords result;

    // use the projection equation here: https://en.wikipedia.org/wiki/Camera_resectioning
    result.x = camera_info.K[0]*point.x / point.z + camera_info.K[2];
    result.y = camera_info.K[4]*point.y / point.z + camera_info.K[5];
    result.z = point.z;

    return result;
}

//TODO is this an ordered pointcloud that is going into this?
//it was a planar laser right? that was the source

/**
 * @brief Convert a pointcloud into (x,y,depth) pixel coordinate space.
 * @details Note: Make sure the cloud is in the optical camera frame, and not the link frame.
 * @param cloud The cartesian pointcloud to convert
 * @param camera_info The camera information. Specifically we use the
 *                    intrinsic matrix.
 * @return A vector of (x,y,depth) pixel coordinates. Index order is preserved.
 */
std::vector<PixelCoords> convertCloudToPixelCoords(const CloudPtr cloud,
                                                   const sensor_msgs::CameraInfo &camera_info)
{
    std::vector<PixelCoords> output;
    output.reserve(cloud->size());

    for (const PointType &point : cloud->points)
    {
        output.push_back( poseToPixel(point, camera_info) );
    }

    return output;
}


//TODO change name to filtePoints

/**
 * @brief Extract from a pointcloud those points that are within the FoV of the camera.
 * @param input The input pointcloud
 * @param pixel_coordinates A vector of pixelspace coordinates. These correspond by index
 *                          with the points ins input
 * @param height The pixel height of the camera
 * @param width The pixel width of the camera
 * @return A pointcloud containing only the points within the camera FoV.
 */
CloudPtr filterPointsInFoV(const CloudPtr input,
                           const std::vector<PixelCoords> &pixel_coordinates,
                           const int height,
                           const int width)
{
    pcl::PointIndices::Ptr indices_in_fov(new pcl::PointIndices());
    indices_in_fov->indices.reserve(input->size());

    for (int i = 0; i < pixel_coordinates.size(); ++i)
    {
        // TODO verify Z is what is in front of the camera
        if (pixel_coordinates[i].z > 0 &&
            pixel_coordinates[i].z < z_max &&   //Connor added
            pixel_coordinates[i].x >= 0 &&
            pixel_coordinates[i].x <= width &&
            pixel_coordinates[i].y >= 0 &&
            pixel_coordinates[i].y <= height)
        {
            indices_in_fov->indices.push_back(i);
        }
    }

    CloudPtr cloud_in_fov(new Cloud);
    pcl::ExtractIndices<PointType> camera_fov_filter;

    // Extract the inliers  of the ROI
    camera_fov_filter.setInputCloud(input);
    camera_fov_filter.setIndices(indices_in_fov);
    camera_fov_filter.setNegative(false);
    camera_fov_filter.filter(*cloud_in_fov);

    return cloud_in_fov;
}


bool transformPointCloud2(sensor_msgs::PointCloud2 &pointcloud,
                          const std::string target_frame)
{
    geometry_msgs::TransformStamped transform;
    try
    {
      transform = tf_buffer_.lookupTransform(target_frame,
                                             tf2::getFrameId(pointcloud),
                                             ros::Time(0),
                                             ros::Duration(0.1));
    }
    catch (tf2::TransformException &ex)
    {
      ROS_WARN("%s",ex.what());
      return false;
    }

    tf2::doTransform(pointcloud, pointcloud, transform);
    
    return true;
}


/**
 * @brief Callback function for the pointclouds
 * @details This does the core processing to locate objects in the cloud
 * @param input_cloud The pointcloud
 */
void pointCloudCb(sensor_msgs::PointCloud2 input_cloud)
{
    auto t1 = debug_clock_.now();
    // check that we've received bounding boxes
    //TODO

    // check that we've received camera info
    if (camera_info_.height == 0 || camera_info_.width == 0)
    {
        return;
    }

    const ros::Time now = ros::Time::now();

    // transform the pointcloud into the RGB optical frame
    if (tf2::getFrameId(input_cloud) != rgb_optical_frame_)
    {
        if (!transformPointCloud2(input_cloud, rgb_optical_frame_))
        {
            return;
        }  
    }

    // Convert sensor_msgs::PointCloud2 to pcl::PointCloud
    CloudPtr cloud(new Cloud);
    pcl::fromROSMsg(input_cloud, *cloud);

    //TODO: ask Chris what these are
    // remove NaN points from the cloud
    CloudPtr cloud_nan_filtered(new Cloud);
    CloudPtr nanfiltered_cloud(new Cloud);
    std::vector<int> rindices;
    pcl::removeNaNFromPointCloud(*cloud, *cloud_nan_filtered, rindices);

    // produce pixel-space coordinates
    const std::vector<PixelCoords> pixel_coordinates = convertCloudToPixelCoords(cloud_nan_filtered, camera_info_);

    // -------------------Extraction of points in the camera FOV------------------------------
    const CloudPtr cloud_fov = filterPointsInFoV(cloud_nan_filtered, pixel_coordinates, camera_info_.height, camera_info_.width);

    if (cloud_fov->empty())
    {
        ROS_WARN("No pointcloud data found within the camera field of view.");
        return;
    }

    if (debug_pcl)
    {
        sensor_msgs::PointCloud2 pc2;
        pcl::toROSMsg(*cloud_fov, pc2);
        filtered_pcl_pub_.publish(pc2);
    }

    // output
    //initiliaze a custom message and add values to some of its fields

    //TODO Ask Chris why this conversion is done twice. and why it's after the filtered_pcl_pub is published?
    // I'm current not doing anything with pixel_coordinates_fov. Should I be?
    // produce pixel-space coordinates
    const std::vector<PixelCoords> pixel_coordinates_fov = convertCloudToPixelCoords(cloud_fov, camera_info_);

    /////////////////////////////////////////////////////////////

    //for loop that decides what to add to a PointXYZRGB vector as a filtered outputed
    //refer to original code

    // publish resuls at the end of this callback
    // TODO do I publish here or in the main
    // refer to original code. could be two different topics I'm thinking about
 
    auto t2 = debug_clock_.now();
    
    ROS_ERROR_STREAM("TIME PC CB: " << std::chrono::duration_cast<std::chrono::milliseconds>(t2- t1).count());

}



int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init(argc,argv,"pcl_processing");
    nh = new ros::NodeHandle("~");

    if (argc != 2)
    {
        ROS_INFO("usage: rosrun perception_playground pcl_processing rgb_optical_frame");
        return 1;
    }

    rgb_optical_frame_ = std::string(argv[1]);

    nh->param("debug_pcl", debug_pcl, {true});
    nh->param("z_max", z_max, {1.0});

    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(tf_buffer_);

    // Initialize subscribers to darknet detection and pointcloud
    ros::Subscriber cloud_sub = nh->subscribe<sensor_msgs::PointCloud2>("pointcloud", 10, pointCloudCb);
    // ros::Subscriber cloud_sub = nh->subscribe<sensor_msgs::PointCloud2>("/seal_cameras/left_camera/depth/color/points", 10, pointCloudCb);
    ros::Subscriber camera_info_sub = nh->subscribe<sensor_msgs::CameraInfo>("camera_info", 100, cameraInfoCb);
    // ros::Subscriber camera_info_sub = nh->subscribe<sensor_msgs::CameraInfo>("/seal_cameras/left_camera/color/camera_info", 100, cameraInfoCb);

    // Create a ROS publisher for the output point cloud


    if (debug_pcl) {
        filtered_pcl_pub_ = nh->advertise<sensor_msgs::PointCloud2>("filtered_pcl", 1);
    }
        
    ros::spin();

    return 0;
}