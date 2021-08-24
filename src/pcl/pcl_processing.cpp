#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <sensor_msgs/CameraInfo.h>
#include <vision_msgs/BoundingBox2D.h>
#include "geometry_msgs/Pose2D.h"

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
// TODO: does this just being PointXYZ have any effect on the logic?
typedef pcl::PointXYZRGB PointType;
typedef pcl::PointCloud<PointType> Cloud;
typedef Cloud::Ptr CloudPtr;

// if true, we will publish the FoV and bounding box pointclouds
bool debug_pcl;
float z_min;
float z_max;
std::chrono::high_resolution_clock debug_clock_;

// the optical frame of the RGB camera (not the link frame)
// can view available frames on /tf or /tf_static topics
// this partiuclar one is left_camera_color_optical_frame
std::string rgb_optical_frame_;
std::string floor_frame_;

// ROS Nodehandle
ros::NodeHandle *nh;

// Publishers
//for publishing filtered output whether that be a pointcloud or a different message type
//TODO change this to zmin_filter_pcl_pub
ros::Publisher filtered_pcl_pub_;
ros::Publisher seal_edge_bbox_pub_;
ros::Publisher single_point_pub_;

// Initialize transform listener 
// TODO ask Chris what this is
std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
tf2_ros::Buffer tf_buffer_;

// caches for callback data
// vision_msgs::BoundingBox2D current_box_;
int box_xmin_ = 0;
int box_xmax_ = 640;
int box_ymin_ = 180;
int box_ymax_ = 230;

sensor_msgs::CameraInfo camera_info_;

// depth-image pixel value
typedef struct
{
    int32_t x;
    int32_t y;
    double z;
} PixelCoords;

/**
 * @brief Callback function for bounding boxes detected upstream
 * @param msg BoundingBox2D
 * @post The message is copied to a local cache
 */
void bBoxCb(const vision_msgs::BoundingBox2DConstPtr& msg)
{
    // current_box_ = *msg;
    geometry_msgs::Pose2D center;
    center = (*msg).center;
    // box_xmin_ = center.x - (*msg).size_x/2;   // or could use ->
    // box_xmax_ = center.x + (*msg).size_x/2;
    // box_ymin_ = center.y - (*msg).size_y/2;
    // box_ymax_ = center.y + (*msg).size_y/2;
    
    // dummy values
    box_xmin_ = 0;
    box_xmax_ = 640;
    box_ymin_ = 0;
    box_ymax_ = 480;
}


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


//TODO is this really (u,v,depth)
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
        //this is unordered still, but why
        //TOOD ask Chris
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
CloudPtr filterPointsZ(const CloudPtr input, 
                           const int height,
                           const int width)
{
    //instead of passing in input that way could also use const pcl::PointCloud<PointType> &input. Then you wouldn't use ->
    //would also need to pass in cloud as a pointer *cloud_nan_filtered instead of cloud_nan_filtered
    pcl::PointIndices::Ptr indices_in_fov(new pcl::PointIndices());
    indices_in_fov->indices.reserve(input->size());

    for (int i = 0; i < input->size(); ++i)
    {
        // TODO verify Z is what is in front of the camera
        // if it's opticla frame, Z is in front postive, x is positive right, y is positive is down (same as u,v)
        if (input->points[i].z > z_min &&
            input->points[i].z < z_max &&   //Connor added
            input->points[i].x >= 0 &&
            input->points[i].x <= width &&
            input->points[i].y >= 0 &&
            input->points[i].y <= height)
        {
            indices_in_fov->indices.push_back(i);
        }
    }

    CloudPtr cloud_in_fov(new Cloud);
    pcl::ExtractIndices<PointType> camera_fov_filter;

    // Extract the inliers  of the ROI
    camera_fov_filter.setInputCloud(input);
    //this is what applies the indices found above to the new cloud
    camera_fov_filter.setIndices(indices_in_fov);
    camera_fov_filter.setNegative(false);
    camera_fov_filter.filter(*cloud_in_fov);

    return cloud_in_fov;
}

/**
 * @brief Extract from a pointcloud those points that are within a rectangular bounding box.
 * @param input The input pointcloud
 * @param pixel_coordinates A vector of pixelspace coordinates. These correspond by index
 *                          with the points ins input
 * @param xmin The x-pixel lower bound of the rectangle
 * @param xmax The x-pixel upper bound of the rectangle
 * @param ymin The y-pixel lower bound of the rectangle
 * @param ymax The y-pixel upper bound of the rectangle
 * @return A pointcloud containing only the points within the bounding box.
 */
CloudPtr filterPointsInBox(const CloudPtr input,
                           const std::vector<PixelCoords> &pixel_coordinates,
                           const int xmin,
                           const int xmax,
                           const int ymin,
                           const int ymax)
{
    // auto t1 = debug_clock_.now();
    pcl::PointIndices::Ptr indices_in_bbox(new pcl::PointIndices());
    indices_in_bbox->indices.reserve(input->size());


    for (int i = 0; i < pixel_coordinates.size(); ++i)
    {
        if ( pixel_coordinates[i].z > 0 &&
            pixel_coordinates[i].x > xmin &&
            pixel_coordinates[i].x < xmax &&
            pixel_coordinates[i].y > ymin &&
            pixel_coordinates[i].y < ymax)
        {
            indices_in_bbox->indices.push_back(i);
        }
    }

    CloudPtr cloud_in_bbox(new Cloud);
    pcl::ExtractIndices<PointType> bbox_filter;

    // Extract the inliers  of the ROI
    bbox_filter.setInputCloud(input);
    bbox_filter.setIndices(indices_in_bbox);
    bbox_filter.setNegative(false);
    bbox_filter.filter(*cloud_in_bbox);
    // auto t2 = debug_clock_.now();
    // if (std::chrono::duration_cast<std::chrono::milliseconds>(t2- t1).count() > 1) {
        // ROS_ERROR_STREAM("TIME FILTER POINTS IN BOX: " << std::chrono::duration_cast<std::chrono::milliseconds>(t2- t1).count());
    // }
    return cloud_in_bbox;
}

//just because it says it takes a reference to a variable, you still just pass in that variable and it knows what to do
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
    // auto t1 = debug_clock_.now();
    // check that we've received bounding boxes
    // TODO not sure how to do that with BoundingBox2D ROS msg
    // if (current_box_.empty())
    // {
    //     return;
    // }

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
    // filters out any of the garbage in the pointcloud
    // remove NaN points from the cloud
    CloudPtr cloud_nan_filtered(new Cloud);
    std::vector<int> rindices;
    pcl::removeNaNFromPointCloud(*cloud, *cloud_nan_filtered, rindices);

    // -------------------Extraction of points in the camera FOV------------------------------
    const CloudPtr cloud_fov = filterPointsZ(cloud_nan_filtered, camera_info_.height, camera_info_.width);

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

    // produce pixel-space coordinates
    const std::vector<PixelCoords> pixel_coordinates = convertCloudToPixelCoords(cloud, camera_info_);

          
    // ----------------------Extract points in the bounding box-----------
    const CloudPtr cloud_in_bbox = filterPointsInBox(cloud,
                                                        pixel_coordinates,
                                                        box_xmin_,
                                                        box_xmax_,
                                                        box_ymin_,
                                                        box_ymax_);

    if (debug_pcl)
    {
        sensor_msgs::PointCloud2 pc2;
        pcl::toROSMsg(*cloud_in_bbox, pc2);
        seal_edge_bbox_pub_.publish(pc2);
    }

    // ----------------------Compute centroid-----------------------------
    //TODO why is it 4f and not 3f?
    Eigen::Vector4f centroid_out;
    pcl::compute3DCentroid(*cloud_in_bbox, centroid_out); 
    // std::cout<<"Centroid X value is "<<centroid_out[0]<<"\n Centroid Y value is: "<<centroid_out[1]<<"\n Centroid Z value is: "<<centroid_out[2]<<std::endl; 
    // std::cout<<"Gap Measurement is "<<-centroid_out[1]<<"m and "<<-centroid_out[1]*39.3701<<" inches\n"<<std::endl; 
    // centroid_out[0];
    // centroid_out[1];
    // centroid_out[2];

    //PCL method. if this doesn't work, then try the method of using pointers like the line below
    // const CloudPtr cloud_single_point;
    pcl::PointCloud<pcl::PointXYZ> cloud_single_point;
    pcl::PointXYZ newPoint;
    // newPoint.x = centroid_out[2];
    // newPoint.y = -centroid_out[0];
    // newPoint.z = -centroid_out[1];
    newPoint.x = centroid_out[0];
    newPoint.y = centroid_out[1];
    newPoint.z = centroid_out[2];
    cloud_single_point.points.push_back(newPoint);

    sensor_msgs::PointCloud2 pc2a;
    pcl::toROSMsg(cloud_single_point, pc2a);
    pc2a.header.frame_id = rgb_optical_frame_;
    pc2a.header.stamp = ros::Time();
    if (tf2::getFrameId(pc2a) != floor_frame_)
    {   
        //TODO fix this transform so I don't have to manually do it above
        if (!transformPointCloud2(pc2a, floor_frame_))
        {
            ROS_WARN("No need for Transform.");
        }  
    }
    single_point_pub_.publish(pc2a);

    // gap_msg = wombot_msgs::GapData();
    // gap_msg.seq = ;
    // gap_msg.time = ros::Time::now();

    //read pc2a
    //iterate over the One point in the sensor message and then std::out to the console
    //for loop should only go around once
    for (sensor_msgs::PointCloud2ConstIterator<float> it(pc2a, "x"); it != it.end(); ++it) {
        // TODO: do something with the values of x, y, z
        std::cout << "Gap Size is" << ", " << it[2] << " meters or " << it[2]*39.3701 <<" inches \n" ;
        // gap_msg.gap_size = it[2];
        // gap_data_pub_.publish(gap_msg);
    }

    if (debug_pcl)
    {
        
        
    }

    
    // output
    //initiliaze a custom message and add values to some of its fields

    /////////////////////////////////////////////////////////////

    //for loop that decides what to add to a PointXYZRGB vector as a filtered outputed
    //refer to original code

    // publish resuls at the end of this callback
    // TODO do I publish here or in the main
    // refer to original code. could be two different topics I'm thinking about
 
    // auto t2 = debug_clock_.now();
    
    // ROS_ERROR_STREAM("TIME PC CB: " << std::chrono::duration_cast<std::chrono::milliseconds>(t2- t1).count());

}



int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init(argc,argv,"pcl_processing");
    nh = new ros::NodeHandle("~");

    if (argc != 3)
    {
        ROS_INFO("usage: rosrun perception_playground pcl_processing rgb_optical_frame");
        return 1;
    }

    rgb_optical_frame_ = std::string(argv[1]);
    floor_frame_ = std::string(argv[2]);

    //TODO ask why these are done this way and not with ros::param?
    nh->param("debug_pcl", debug_pcl, {true});
    nh->param("z_min", z_min, {0.20});
    nh->param("z_max", z_max, {0.32});

    //wierd blake stuff that might be useful
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(tf_buffer_);

    // Initialize subscribers 
    //hard topic callouts commented for debugging when can't pass in topic remaps
    // ros::Subscriber cloud_sub = nh->subscribe<sensor_msgs::PointCloud2>("pointcloud", 10, pointCloudCb);
    ros::Subscriber cloud_sub = nh->subscribe<sensor_msgs::PointCloud2>("/seal_cameras/left_camera/depth/color/points", 10, pointCloudCb);
    // ros::Subscriber camera_info_sub = nh->subscribe<sensor_msgs::CameraInfo>("camera_info", 100, cameraInfoCb);
    ros::Subscriber camera_info_sub = nh->subscribe<sensor_msgs::CameraInfo>("/seal_cameras/left_camera/color/camera_info", 100, cameraInfoCb);

    // Create a ROS publisher for the output point cloud
    filtered_pcl_pub_ = nh->advertise<sensor_msgs::PointCloud2>("filtered_pcl", 1);
    seal_edge_bbox_pub_ = nh->advertise<sensor_msgs::PointCloud2>("seal_edge_bbox", 1);
    single_point_pub_ = nh->advertise<sensor_msgs::PointCloud2>("single_point", 1);
    // Uncomment this after bringing into main repo
    //remember to do an include above
    // gap_data_pub_ = nh->advertise<wombot_msgs::GapData>("gap_data", 1);
        
    ros::spin();

    return 0;
}