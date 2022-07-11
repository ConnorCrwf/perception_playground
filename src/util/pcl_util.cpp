#include <perception_playground/util/pcl_util.h>

// macros
//#define UNKNOWN_OBJECT_ID -1

// using namespace pcl_util_ns;

/*
using namespace pcl_util_ns;

Pcl_Util::Pcl_Util(){}

Pcl_Util::~Pcl_Util(){}
*/


//used to return an inline
/**
 * @brief extracts information from BoundingBox2D message
 * @details 
 * @param msg BoundingBox2DConstPtr
 */
Box2D_PixelCoords extractBBoxInfo(const vision_msgs::BoundingBox2DConstPtr &msg)
{

    geometry_msgs::Pose2D center;
    center = (*msg).center;
    // Remember that X is positve left to right
    // X is positive top to bottom
    Box2D_PixelCoords result;
    result.xmin = center.x - (*msg).size_x/2;   // or could use ->
    result.xmax = center.x + (*msg).size_x/2;
    result.ymin = center.y - (*msg).size_y/2; //this is what is used for filter
    result.ymax = center.y + (*msg).size_y/2;

    // std::cout<<"Center is "<<center.x<<" , "<<center.y<<"\n Width: "<<(*msg).size_x<<"\t Height: "<<(*msg).size_y<<std::endl; 
    // std::cout<<"Xmin "<<result.xmin<<"\t Xmax is: "<<result.xmax<<"\n Ymin: "<<result.ymin<<"\t Ymax: "<<result.ymax<<std::endl; 
    
    // dummy values
    // box_xmin_ = 0;
    // box_xmax_ = 640;
    // box_ymin_ = 0;
    // box_ymax_ = 480;
	return result;
}

/**
 * @brief Extract from a pointcloud those points that are within the FoV of the camera.
 * @param input The input pointcloud
 * @param pixel_coordinates A vector of pixelspace coordinates. These correspond by index
 *                          with the points ins input
 * @param height The pixel height of the camera
 * @param width The pixel width of the camera
 * @return A pointcloud containing only the points within the camera FoV.
 */
CloudPtr_XYZRGB filterPoints(const CloudPtr_XYZRGB input, Box3D_CloudCoords limits) 
{
	
    //instead of passing in input that way could also use const pcl::PointCloud<PointType> &input. Then you wouldn't use ->
    //would also need to pass in cloud as a pointer *cloud_nan_filtered instead of cloud_nan_filtered
    pcl::PointIndices::Ptr indices_in_fov = findIndices(input, limits);

    CloudPtr_XYZRGB cloud_in_fovPtr(new Cloud_XYZRGB);
    pcl::ExtractIndices<PointType_XYZRGB> camera_fov_filter;

    // Extract the inliers  of the ROI
    camera_fov_filter.setInputCloud(input);
    //this is what applies the indices found above to the new cloud
    camera_fov_filter.setIndices(indices_in_fov);
    camera_fov_filter.setNegative(false);
    camera_fov_filter.filter(*cloud_in_fovPtr);

    return cloud_in_fovPtr;
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
CloudPtr_XYZRGB filterPoints(const CloudPtr_XYZRGB input,const std::vector<PixelCoords> &pixel_coordinates, Box2D_PixelCoords limits)
{
    // auto t1 = debug_clock_.now();
	//TODO I don't think we need to pass it in by reference again since it is already passed in by reference
	//if I don't put an & in front of it, am I making a local copy of it in findIndices?
	pcl::PointIndices::Ptr indices_in_bbox = findIndices(pixel_coordinates, limits);

    CloudPtr_XYZRGB cloud_in_bbox(new Cloud_XYZRGB);
    pcl::ExtractIndices<PointType_XYZRGB> bbox_filter;

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

//TODO ask Blake how I combine the following two functions
pcl::PointIndices::Ptr findIndices(const CloudPtr_XYZRGB input, Box3D_CloudCoords limits)

{
	pcl::PointIndices::Ptr indices_in_fov(new pcl::PointIndices());
    indices_in_fov->indices.reserve(input->size());

    for (int i = 0; i < input->size(); ++i)
    {
        // TODO verify Z is what is in front of the camera
        // if it's opticla frame, Z is in front postive, x is positive right, y is positive is down (same as u,v)
        if (
            input->points[i].x >= limits.xmin &&
            input->points[i].x <= limits.xmax &&
            input->points[i].y >= limits.ymin &&
            input->points[i].y <= limits.ymax &&
			input->points[i].z >= limits.zmin &&
            input->points[i].z <= limits.zmax)
        {
            indices_in_fov->indices.push_back(i);
        }
    }

	return indices_in_fov;
}


pcl::PointIndices::Ptr findIndices(const std::vector<PixelCoords> &pixel_coordinates, Box2D_PixelCoords limits)

{
	pcl::PointIndices::Ptr indices_in_bbox(new pcl::PointIndices());
    indices_in_bbox->indices.reserve(pixel_coordinates.size());
    // std::cout<<"pixel_coordinate size is " << pixel_coordinates.size() <<std::endl; 


    for (int i = 0; i < pixel_coordinates.size(); ++i)
    {
        // TODO verify Z is what is in front of the camera
        // if it's opticla frame, Z is in front postive, x is positive right, y is positive is down (same as u,v)
        if (
            pixel_coordinates[i].x >= limits.xmin &&
            pixel_coordinates[i].x <= limits.xmax &&
            pixel_coordinates[i].y >= limits.ymin &&
            pixel_coordinates[i].y <= limits.ymax)
        {
            indices_in_bbox->indices.push_back(i);
        }
        
    }
    // std::cout<<"xmin is " << limits.xmin <<std::endl; 
    // std::cout<<"xmax is " << limits.xmax <<std::endl; 
    // std::cout<<"ymin is " << limits.ymin <<std::endl; 
    // std::cout<<"ymax is " << limits.ymax <<std::endl; 
        
    // std::cout<<"indices_in_bbox size is " << indices_in_bbox->indices.size() <<std::endl; 

	return indices_in_bbox;
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
PixelCoords poseToPixel(const PointType_XYZRGB &point,const sensor_msgs::CameraInfo &camera_info)
{
    PixelCoords result;

    // use the projection equation here: https://en.wikipedia.org/wiki/Camera_resectioning
    result.x = camera_info.K[0]*point.x / point.z + camera_info.K[2];
    result.y = camera_info.K[4]*point.y / point.z + camera_info.K[5];
    result.z = point.z;

    return result;
}

/**
 * @brief Convert a pointcloud into (x,y,depth) pixel coordinate space.
 * @details Note: Make sure the cloud is in the optical camera frame, and not the link frame.
 * @param cloud The cartesian pointcloud to convert
 * @param camera_info The camera information. Specifically we use the
 *                    intrinsic matrix.
 * @return A vector of (x,y,depth) pixel coordinates. Index order is preserved.
 */
std::vector<PixelCoords> convertCloudToPixelCoords(const CloudPtr_XYZRGB cloudPtr,
										const sensor_msgs::CameraInfo &camera_info)
{
    std::vector<PixelCoords> output;
    output.reserve(cloudPtr->size());

    for (const PointType_XYZRGB &point : cloudPtr->points)
    {
        //this is unordered still, but why
        //TOOD ask Chris
        output.push_back(poseToPixel(point, camera_info) );
    }

    return output;
}

//TODO make sure I'm not copying anything here
//TODO make sure I'm bringing in the proper sensor_msgs
//TODO figure out how to exclude PlatformIO in wombot_sensors from this path
//just because it says it takes a reference to a variable, you still just pass in that variable and it knows what to do
//TODO do I even need to return anything here since we are passing by reference?
//TODO is this an ordered pointcloud that is going into this?
//it was a planar laser right? that was the source
/**
 * @brief adfa
 * @param pointcloud takes a reference to a PointCloud2 message
 * @return adfa
 */
bool transformPointCloud2(sensor_msgs::PointCloud2 &pointcloud,
                          const std::string target_frame, tf2_ros::Buffer &tf_buffer)
{
    geometry_msgs::TransformStamped transform;
    try
    {
		//TODO make sure this is not a copy
        //getFrameId takes a reference to the PointCloud2 message
      transform = tf_buffer.lookupTransform(target_frame,
                                             tf2::getFrameId(pointcloud),
                                             ros::Time(0),
                                             ros::Duration(0.1));
    }
    catch (tf2::TransformException &ex)
    {
      ROS_WARN("%s",ex.what());
	  ROS_WARN("Transform Failed");
      return false;
    }

    //thes function accepts references to the data, but if the second output argument is same as the first data in argument,
    //then it should be mutable
    tf2::doTransform(pointcloud, pointcloud, transform);
    
    //don't need to return the pointcloud since it was pased into the function by reference
    return true;
}


CloudPtr_XYZ filterPlane(const CloudPtr_XYZ input){
    //instead of passing in input that way could also use const pcl::PointCloud<PointType> &input. Then you wouldn't use ->
    //would also need to pass in cloud as a pointer *cloud_nan_filtered instead of cloud_nan_filtered
    pcl::PointIndices::Ptr indices_in_fov(new pcl::PointIndices());
    indices_in_fov->indices.reserve(input->size());

    for (int i = 0; i < input->size(); ++i)
    {
        // TODO verify Z is what is in front of the camera
        // if it's opticla frame, Z is in front postive, x is positive right, y is positive is down (same as u,v)
        if (input->points[i].z >=0.34 &&
            input->points[i].z <=0.50 &&
            input->points[i].y <=-0.04)
        {
            indices_in_fov->indices.push_back(i);
        }
    }

    CloudPtr_XYZ filtered_cloud(new Cloud_XYZ);
    pcl::ExtractIndices<PointType_XYZ> cloud_filter;

    // Extract the inliers  of the ROI
    cloud_filter.setInputCloud(input);
    //this is what applies the indices found above to the new cloud
    cloud_filter.setIndices(indices_in_fov);
    cloud_filter.setNegative(false);
    cloud_filter.filter(*filtered_cloud);

    return filtered_cloud;
}

Cloud_XYZ getCentroid(const CloudPtr_XYZRGB cloud) {
    Cloud_XYZ centroid;

    Eigen::Vector4f centroid_out;
    pcl::compute3DCentroid(*cloud, centroid_out);
    PointType_XYZ center;
    center.x = centroid_out[0];
    center.y = centroid_out[1];
    center.z = centroid_out[2];
    centroid.points.push_back(center);

    return centroid;
}