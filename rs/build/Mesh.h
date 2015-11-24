#pragma once
#include <iostream>
#include <string>


#include <boost/format.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/common/time.h>
#include <pcl/common/transforms.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/io/io_exception.h>
#include <pcl/filters/fast_bilateral.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/keypoints/harris_3d.h>
#include <pcl/segmentation/segment_differences.h>

#include <Eigen/Geometry>
#include "real_sense_grabber.h"

namespace pcl
{
	template<>
	struct SIFTKeypointFieldSelector<pcl::PointXYZ>
	{
		inline float operator() (const pcl::PointXYZ &p) const
		{
			return p.z;
		}
	};
}

using namespace pcl::console;

enum MODE
{
	USE_GRABBER,
	USE_CAD
};

typedef pcl::PointXYZRGBA Point3DC;

template <typename PointT> class Mesh
{
public:
	typedef pcl::PointCloud<PointT> PointCloudT;
	Mesh(pcl::RealSenseGrabber& grabber);
	~Mesh();
	void run();

private:
	pcl::RealSenseGrabber& grabber;
	pcl::visualization::PCLVisualizer viewer;
	boost::signals2::connection connection;
	int window;
	int threshold;
	
	std::string cad_filename = "box2.pcd";

	mutable boost::mutex new_cloud_mutex;
	typename PointCloudT::ConstPtr new_cloud;
	typename PointCloudT::Ptr filter_cloud;
	typename PointCloudT::ConstPtr last_cloud;
	typename PointCloudT::Ptr key_cloud;
	typename PointCloudT::Ptr cad_cloud;
	bool keys_found = false;

	Eigen::Matrix4f *affine_transformation = NULL;
	
	void cloudCallback(typename PointCloudT::ConstPtr cloud);
	void keyboardCallback(const pcl::visualization::KeyboardEvent & event, void*);

	void findKeyPoints(typename PointCloudT::ConstPtr cloud);
	void loadCADFromFile(std::string filename);
	void loadTransformFromFile(int dim = 3);
	void visualizeCADFile();
	void transformPointCloud(typename PointCloudT::Ptr cloud, Eigen::Matrix4f *transform);
	pcl::PointCloud<Point3DC> subtractPointClouds(typename PointCloudT::Ptr cloud_a, typename PointCloudT::Ptr cloud_b);

	bool filtering_on = false;
	MODE VIEWER_MODE = MODE::USE_GRABBER;

	pcl::StatisticalOutlierRemoval<PointT> sor;
	
};	