
//	Include needed libraries

#include <librealsense2/rs.hpp> 
#include <librealsense2/rs_advanced_mode.hpp>

#include <iostream>
#include <cmath>
#include <fstream>
#include <algorithm>
#include <vector>
#include <cstdint>


#include <pcl/io/pcd_io.h>
#define NOMINMAX
#include <Windows.h>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_types.h>

//	Define function to transfer from rs2::points to pcl::PointCloud

using pcl_ptr = pcl::PointCloud<pcl::PointXYZ>::Ptr;

pcl::PointCloud<pcl::PointXYZ>::Ptr points_to_pcl(const rs2::points& points)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	auto sp = points.get_profile().as<rs2::video_stream_profile>();
	cloud->width = sp.width();
	cloud->height = sp.height();
	cloud->is_dense = false;
	cloud->points.resize(points.size());
	auto ptr = points.get_vertices();
	for (auto& p : cloud->points)
	{
		p.x = ptr->x;
		p.y = ptr->y;
		p.z = ptr->z;
		ptr++;
	}

	return cloud;
}

//	main function	
int main()
{
	
	//	Parameters of translation and rotation
	float dangle = 27.9 * 3.14 /180;
	float angle = 0;
	float dy = 0.025;
	float sin_ang = sin(angle);
	float cos_ang = cos(angle);

	uint32_t color_width = 1280;
	uint32_t color_height = 720;
	uint32_t color_fps = 30;

	uint32_t depth_width = 1280;
	uint32_t depth_height = 720;
	uint32_t depth_fps = 30; 

	// Create Point Clouds
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cam1;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cam2;
	
	cloud_cam1 = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
	cloud_cam1->width = static_cast<uint32_t>(depth_width);
	cloud_cam1->height = static_cast<uint32_t>(depth_height);
	cloud_cam1->points.resize(cloud_cam1->height * cloud_cam1->width);
	cloud_cam1->is_dense = false;
	
	cloud_cam2 = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
	cloud_cam2->width = static_cast<uint32_t>(depth_width);
	cloud_cam2->height = static_cast<uint32_t>(depth_height);
	cloud_cam2->points.resize(cloud_cam2->height * cloud_cam2->width);
	cloud_cam2->is_dense = false;

	//	PCL Visualizer
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
	viewer = boost::make_shared<pcl::visualization::PCLVisualizer>("Multi Cam Test");
	viewer->setCameraPosition(0.0, 0.0, -2.5, 0.0, -1.0, 0.0);
	viewer->addCoordinateSystem(0.1);
	
	//	Create vector for camras pipelines
	std::vector<rs2::pipeline> pipeline;	//one for each camera
	std::vector<rs2::pipeline_profile> pipeline_profile;	//one for each camera

	rs2::frameset frameset;
	rs2::pointcloud pc;
	rs2::points points;
	rs2::colorizer color_map;
	rs2::frame color_frame;
	rs2::frame depth_frame;

	rs2::frameset aligned_frames;

	rs2::frameset frameset_filtered;


	rs2::spatial_filter spat_filter;
	spat_filter.set_option(rs2_option::RS2_OPTION_FILTER_MAGNITUDE, 5.0f);
	spat_filter.set_option(rs2_option::RS2_OPTION_FILTER_SMOOTH_DELTA, 50.0f);
	spat_filter.set_option(rs2_option::RS2_OPTION_FILTER_SMOOTH_ALPHA, 0.3f);
	rs2::temporal_filter temp_filter;
	temp_filter.set_option(rs2_option::RS2_OPTION_FILTER_SMOOTH_DELTA, 100.0f);
	temp_filter.set_option(rs2_option::RS2_OPTION_FILTER_SMOOTH_ALPHA, 0.1f);

	// Retrive Connected Sensors List
	rs2::context context;
	const rs2::device_list device_list = context.query_devices();

	int device_count = -1;

	// Initialize Connected Sensors
	for (const rs2::device& device : device_list) {		

		// Retrive Serial Number
		const std::string serial_number = device.get_info(rs2_camera_info::RS2_CAMERA_INFO_SERIAL_NUMBER);

		// Set Device Config
		rs2::config config;
		config.enable_device(serial_number);
		config.enable_stream(rs2_stream::RS2_STREAM_COLOR, color_width, color_height, rs2_format::RS2_FORMAT_BGR8, color_fps);
		config.enable_stream(rs2_stream::RS2_STREAM_DEPTH, depth_width, depth_height, rs2_format::RS2_FORMAT_Z16, depth_fps);

		rs2::pipeline_profile pipeline_prof;
		rs2::pipeline pipel;

		pipeline_profile.push_back(pipeline_prof);
		pipeline.push_back(pipel);

		device_count = device_count + 1;

		// Start Pipeline
		pipeline_profile[device_count] = pipeline[device_count].start(config);

		std::cout << "Realsense device with serial number: " << serial_number << " is enabled" << std::endl;

	}

	device_count = device_count + 1;
	std::cout << "Number of connected devices: " << device_count << std::endl;
	char c = getchar();


	//	Loop for acquisition and display the result on the viewer
	while (!viewer->wasStopped()) {

		//	get frame set from first sensor
		frameset = pipeline[0].wait_for_frames();

		//	Calculate the point cloud
		depth_frame = frameset.get_depth_frame();		
		points = pc.calculate(depth_frame);
		auto pcl_points_cam1 = points_to_pcl(points);

		//	get color data
		rs2::align align(RS2_STREAM_DEPTH);
		aligned_frames = align.process(frameset);
		rs2::video_frame color_frame = aligned_frames.get_color_frame();
		auto p_color_frame = static_cast<uint8_t*>(const_cast<void*>(color_frame.get_data()));
		uint32_t color_width = color_frame.as<rs2::video_frame>().get_width();
		uint32_t color_height = color_frame.as<rs2::video_frame>().get_height();

		//	Fill the PLC
		int j = 0;
		for (int i = 0; i < color_width * color_height; i++) {
			cloud_cam1->points[i].x = pcl_points_cam1->points[i].x;
			cloud_cam1->points[i].y = pcl_points_cam1->points[i].y;
			cloud_cam1->points[i].z = pcl_points_cam1->points[i].z;
			cloud_cam1->points[i].b = p_color_frame[j];
			cloud_cam1->points[i].g = p_color_frame[j + 1];
			cloud_cam1->points[i].r = p_color_frame[j + 2];
			j = j + 3;
		}

		//rotate around x
		angle =  dangle/2;
		sin_ang = sin(angle);
		cos_ang = cos(angle);
		for (int i = 0; i < color_width * color_height; i++) {
			float py = cloud_cam1->points[i].y - (dy/2);
			float pz = cloud_cam1->points[i].z + 0.0;

			// rotate point
			float rot_point_y = (py * cos_ang - pz * sin_ang);
			float rot_point_z = (py * sin_ang + pz * cos_ang);

			cloud_cam1->points[i].y = rot_point_y;
			cloud_cam1->points[i].z = rot_point_z;
		}


		//	get frame set from second sensor
		frameset = pipeline[1].wait_for_frames();

		//	Calculate the point cloud
		depth_frame = frameset.get_depth_frame();
		points = pc.calculate(depth_frame);
		auto pcl_points_cam2 = points_to_pcl(points);

		//	get color data
		aligned_frames = align.process(frameset);
		color_frame = aligned_frames.get_color_frame();
		p_color_frame = static_cast<uint8_t*>(const_cast<void*>(color_frame.get_data()));
		color_width = color_frame.as<rs2::video_frame>().get_width();
		color_height = color_frame.as<rs2::video_frame>().get_height();

		//	Fill the PLC
		j = 0;
		for (int i = 0; i < color_width * color_height; i++) {
			cloud_cam2->points[i].x = pcl_points_cam2->points[i].x;
			cloud_cam2->points[i].y = pcl_points_cam2->points[i].y;
			cloud_cam2->points[i].z = pcl_points_cam2->points[i].z;
			cloud_cam2->points[i].b = p_color_frame[j];
			cloud_cam2->points[i].g = p_color_frame[j + 1];
			cloud_cam2->points[i].r = p_color_frame[j + 2];
			j = j + 3;
		}

		//rotate around x
		angle = - dangle/2;
		sin_ang = sin(angle);
		cos_ang = cos(angle);
		for (int i = 0; i < color_width * color_height; i++) {
			float py = cloud_cam2->points[i].y + (dy/2);
			float pz = cloud_cam2->points[i].z + 0.0;

			// rotate point
			float rot_point_y = (py * cos_ang - pz * sin_ang);
			float rot_point_z = (py * sin_ang + pz * cos_ang);

			cloud_cam2->points[i].y = rot_point_y;
			cloud_cam2->points[i].z = rot_point_z;
		}

		//	Display the result on the viewer
		if (!viewer->updatePointCloud(cloud_cam1, "cloud1")) {
			viewer->addPointCloud(cloud_cam1, "cloud1");
			viewer->addPointCloud(cloud_cam2, "cloud2");
		}
		viewer->updatePointCloud(cloud_cam2, "cloud2");

		viewer->spinOnce();
		
	}

	// if the viewer is closed stop the pipelines
	for (int i = 0; i < device_count; ++i) {
			pipeline[i].stop();
	}

	return 0;

}