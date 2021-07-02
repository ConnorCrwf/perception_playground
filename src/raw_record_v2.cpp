#include "ros/ros.h"
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>


#include <boost/algorithm/string/predicate.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread/thread.hpp> 
#include <boost/thread/recursive_mutex.hpp>

#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
#include <librealsense2/hpp/rs_processing.hpp>
#include <librealsense2/rs_advanced_mode.hpp>

rs2::device dev_handle;
rs2::stream_profile color_stream, depth_stream;
ros::Publisher depth_pub, color_pub;
uint64_t color_frame_idx = 0, depth_frame_idx = 0;
boost::recursive_mutex color_mutex, depth_mutex;
bool publish_depth = true, publish_color = true;

//TODO: Add filter for other camera. Get only D435 as output here
rs2::device match_serial_no(rs2::device dev, std::string serial_no){
	rs2::device tmp_device;
	if(dev.supports(RS2_CAMERA_INFO_NAME)){
		 	ROS_INFO("Name %s", dev.get_info(RS2_CAMERA_INFO_NAME));
	}
	if(dev.supports(RS2_CAMERA_INFO_SERIAL_NUMBER)){
		ROS_INFO("Serial No: %s", dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));
		if(boost::iequals(serial_no.c_str(), dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER))){
			ROS_INFO("Device found with given serial no %s", serial_no.c_str());
			return dev;
		}
	}
	return tmp_device;
}

rs2::device get_device(std::string serial_no){
	rs2::context ctx;
	auto devices = ctx.query_devices();
	rs2::device tmp_device;
	if(devices.size() == 0){//No device found. Wait for connection
	 	 rs2::device_hub device_hub(ctx);
		 tmp_device = match_serial_no(device_hub.wait_for_device(), serial_no);
		 return tmp_device;
	}else{
		for(auto dev: devices){
			tmp_device =  match_serial_no(dev, serial_no);
			if(tmp_device != NULL){
				return tmp_device;
			}
	    }
	}
	return tmp_device;
}

void set_stream(rs2::device dev, ros::NodeHandle n){
	for (auto sen: dev.query_sensors()){
		auto name = sen.get_info(RS2_CAMERA_INFO_NAME);
		if(boost::iequals(name, "RGB Camera")){
			auto width = 1280,
		 		 height = 720,
			 	 fps = 30;
			n.getParam("color_width", width);
			n.getParam("color_height", height);
			n.getParam("color_fps", fps);
			for (auto profile: sen.get_stream_profiles()){
				auto video_profile = profile.as<rs2::video_stream_profile>();
				color_stream = video_profile;
				if(video_profile.stream_type() == RS2_STREAM_COLOR &&
				  video_profile.width() == width && video_profile.height() == height && video_profile.fps() == fps)
				{
					color_stream = video_profile;
					ROS_INFO("Color Stream set to values %dx%d @ %d", width, height, fps);
					break;
				}
			}
		}else if(boost::iequals(name, "Stereo Module")){
			auto width = 1280,
		 		 height = 720,
			 	 fps = 30;
			n.getParam("depth_width", width);
			n.getParam("depth_height", height);
			n.getParam("depth_fps", fps);
			for (auto profile: sen.get_stream_profiles()){
				auto video_profile = profile.as<rs2::video_stream_profile>();
				depth_stream = video_profile;
				if(video_profile.stream_type() == RS2_STREAM_DEPTH &&
				  video_profile.width() == width && video_profile.height() == height && video_profile.fps() == fps)
				{
					depth_stream = video_profile;
					ROS_INFO("Stereo Stream set to values %dx%d @ %d", width, height, fps);
					break;
				}
			}
		}
	}
}

//TODO: Make it generic. So that any control can be updated
//Or Use dynamic configuration module
void set_controls(rs2::device dev, ros::NodeHandle n){
	bool  auto_expo = false;
	n.getParam("enable_auto_expo", auto_expo);
	for (auto sen: dev.query_sensors()){
		auto name = sen.get_info(RS2_CAMERA_INFO_NAME);
		if(boost::iequals(name, "RGB Camera")){
			sen.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, auto_expo);
			ROS_INFO("Controls set");
		}
	}
}

void publish_frame(rs2::frame f, std::string encoding, ros::Publisher pub, unsigned int idx){
	ros::Time timestamp = ros::Time::now();
	sensor_msgs::Image::Ptr image(new sensor_msgs::Image());
	auto f_image = f.as<rs2::video_frame>();
	auto size = f_image.get_width() * f_image.get_height() * f_image.get_bytes_per_pixel();
	image->encoding = encoding;
	image->header.frame_id = idx;
	image->header.stamp = timestamp;
	image->width = f_image.get_width();
	image->height = f_image.get_height();
	image->step = image->width * f_image.get_bytes_per_pixel();
	image->data.resize(size);
	memcpy(&(image->data[0]),  (void*)f.get_data(), size);
	pub.publish(image);
}

void start_stream(rs2::device dev){
	for (auto sen: dev.query_sensors()){
		auto name = sen.get_info(RS2_CAMERA_INFO_NAME);
		if(boost::iequals(name, "RGB Camera")){
			sen.open(color_stream);
			if(publish_color){
				sen.start([&](rs2::frame f){
					boost::recursive_mutex::scoped_lock(color_mutex);
					publish_frame(f, sensor_msgs::image_encodings::RGB8, color_pub, color_frame_idx++);
				});
			}else{
				sen.start([&](rs2::frame f){});
			}
		}else if (boost::iequals(name, "Stereo Module")){
			sen.open(depth_stream);
			if(publish_depth){
				sen.start([&](rs2::frame f){
					boost::recursive_mutex::scoped_lock(depth_mutex);
					publish_frame(f, sensor_msgs::image_encodings::TYPE_16UC1, depth_pub, depth_frame_idx++);
				});
			}else{
				sen.start([&](rs2::frame f){});
			}
		}
	}
}

void stop_stream(rs2::device dev){
	for (auto sen: dev.query_sensors()){
		sen.stop();
		sen.close();
	}
}

int main(int argc, char *argv[])
{

	ros::init(argc, argv, "raw_record");

	ros::NodeHandle n("~");

	std::string serial_no = "752112070971";//TODO: Add the serial no. here
	n.getParam("serial_no", serial_no);
	//Bag file to record
	std::string fileName = "raw_record_v2_test.bag";
	n.getParam("bag_file", fileName);

	//Need to publish while recording
	n.getParam("pub_color", publish_color);
	n.getParam("pub_depth", publish_depth);

	if(publish_color)
		color_pub = n.advertise<sensor_msgs::Image>("/color/image_raw", 1);
	if(publish_depth)
		depth_pub = n.advertise<sensor_msgs::Image>("/depth/image_rect/raw", 1);

	dev_handle = get_device(serial_no);
	try
	{
		ROS_INFO("My device name %s", dev_handle.get_info(RS2_CAMERA_INFO_NAME));
	}
	catch (...)
	{
		ROS_INFO("Specified device not found. exiting");
		return -1;
	 }
	rs2::recorder recorder(fileName, dev_handle);
	ROS_INFO("File will be recorded @ %s", fileName.c_str());
	set_controls(dev_handle, n);
	set_stream(dev_handle, n);
	start_stream(dev_handle);
	while(ros::ok()){
		boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
		ros::spinOnce();
	}
	ROS_INFO("Stopping recording Node");
	stop_stream(dev_handle);

	return 0;
}