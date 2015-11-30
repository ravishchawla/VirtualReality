#pragma once

#include <iostream>
#include <string>

#include <boost/format.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/thread/mutex.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/io_exception.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/segmentation/segment_differences.h>
#include "real_sense_grabber.h"

typedef pcl::PointXYZRGBA Point3DC;

enum View {
	VIEW_0,
	VIEW_1,
	VIEW_2,
	VIEW_3,
	CAD,
	NIL
};

template <typename PointT> class Viewpoint {
public:
	typedef pcl::PointCloud<PointT> PointCloudT;
	pcl::visualization::PCLVisualizer viewer;
	Viewpoint();
	~Viewpoint();
	void run();

private:
	/*
	std::string view0Name = "view0.pcd";
	std::string view1Name = "view1.pcd";
	std::string view2Name = "view2.pcd";
	std::string view3Name = "view3.pcd";
	std::string cadName =   "box2.pcd";
	*/

	std::string view0Name = "view00.ply";
	std::string view1Name = "view01.ply";
	std::string view2Name = "view02.ply";
	std::string view3Name = "view03.ply";
	std::string cadName = "box2.pcd";

	typename PointCloudT::Ptr view0Cloud;
	typename PointCloudT::Ptr view1Cloud;
	typename PointCloudT::Ptr view2Cloud;
	typename PointCloudT::Ptr view3Cloud;
	typename PointCloudT::Ptr cadCloud;

	void keyboardCallback(const pcl::visualization::KeyboardEvent & event, void*);
	void loadPointCloud(typename PointCloudT::Ptr cloud, std::string filename);
	pcl::PointCloud<Point3DC> subtractPointClouds(typename PointCloudT::Ptr cloud1,
												  typename PointCloudT::Ptr cloud2);
	View viewMode = View::NIL;
	View lastViewMode = View::NIL;

};
