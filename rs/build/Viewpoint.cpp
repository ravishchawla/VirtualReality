#include "Viewpoint.h"

Viewpoint<Point3DC>::Viewpoint() : viewer("Point Cloud Viewer") {
	view0Cloud = boost::make_shared < pcl::PointCloud<Point3DC>>();
	view1Cloud = boost::make_shared < pcl::PointCloud<Point3DC>>();
	view2Cloud = boost::make_shared < pcl::PointCloud<Point3DC>>();
	view3Cloud = boost::make_shared < pcl::PointCloud<Point3DC>>();
	cadCloud = boost::make_shared < pcl::PointCloud<Point3DC>>();
	std::cout << "Constructor" << std::endl;
	loadPointCloud(view0Cloud, view0Name);
	std::cout << "View 0 cloud: " << *view0Cloud << std::endl;
	loadPointCloud(view1Cloud, view1Name);
	std::cout << "View 1 cloud: " << *view1Cloud << std::endl;
	loadPointCloud(view2Cloud, view2Name);
	std::cout << "View 2 cloud: " << *view2Cloud << std::endl;
	loadPointCloud(view3Cloud, view3Name);
	std::cout << "View 3 cloud: " << *view3Cloud << std::endl;
	//loadPointCloud(cadCloud, cadName);
	//std::cout << "CAD cloud: " << *cadCloud << std::endl;
	viewMode = View::VIEW_3;

	viewer.setCameraFieldOfView(0.785398); //approximately 45 degrees
	viewer.setCameraPosition(0, 0, 0, 0, 0, 1, 0, 1, 0);
	viewer.registerKeyboardCallback(&Viewpoint::keyboardCallback, *this);
}

template<typename PointT>
Viewpoint<PointT>::~Viewpoint()
{
}

template<> void
Viewpoint<Point3DC>::run() {

	pcl::PointCloud<Point3DC>::Ptr currentCloud;


	while (!viewer.wasStopped()) {
		if (viewMode != lastViewMode && viewMode != View::NIL) {
			switch (viewMode) {
			case View::VIEW_0:
				currentCloud = view0Cloud;
				break;
			case View::VIEW_1:
				currentCloud = view1Cloud;
				break;
			case View::VIEW_2:
				currentCloud = view2Cloud;
				break;
			case View::VIEW_3:
				currentCloud = view3Cloud;
				break;
			case View::CAD:
				currentCloud = cadCloud;
				break;
			}
			std::cout << "viewer updating cloud " << std::endl;
			if (currentCloud && !viewer.updatePointCloud(currentCloud, "cloud")) {
				viewer.addPointCloud(currentCloud, "cloud");
				//viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "cloud");
				viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");
			}
			lastViewMode = viewMode;
		}
		viewer.spinOnce(1, true);
	}
}

template<> void
Viewpoint<Point3DC>::loadPointCloud(typename PointCloudT::Ptr cloud, std::string filename) {
	int success;
	try {
		pcl::PointCloud<Point3DC>::Ptr temp_cloud;
		temp_cloud = boost::make_shared<pcl::PointCloud<Point3DC>>();

		//success = pcl::io::loadPCDFile<Point3DC>(filename, *temp_cloud);
		success = pcl::io::loadPLYFile<Point3DC>(filename, *temp_cloud);
		if (success < 0) {
			std::cout << "Unable to load CAD model from file. Make sure that file exists is openable" << std::endl;
		}

		pcl::copyPointCloud(*temp_cloud, *cloud);
		std::cout << "Completed conversion : " << (*cloud) << std::endl;
	}
	catch (std::exception exe) {
		std::cout << "Exception thrown in loadCADFromFile: " << exe.what() << std::endl;
	}
}

template<> pcl::PointCloud<Point3DC>
Viewpoint<Point3DC>::subtractPointClouds(typename PointCloudT::Ptr cloud1,
	typename PointCloudT::Ptr cloud2) {

	return *cloud2;
}

template<> void
Viewpoint<Point3DC>::keyboardCallback(const pcl::visualization::KeyboardEvent &event, void*) {
	if (event.keyDown()) {
		if (event.isShiftPressed()) {
			std::cout << "key down: ";
			switch (event.getKeyCode()) {
			case '!': //1
				std::cout << "view 0" << std::endl;
				viewMode = View::VIEW_0;
				break;
			case '@': //2
				std::cout << "view 1" << std::endl;
				viewMode = View::VIEW_1;
				break;
			case '#': //3
				std::cout << "view 2" << std::endl;
				viewMode = View::VIEW_2;
				break;
			case '$': //4
				std::cout << "view 3" << std::endl;
				viewMode = View::VIEW_3;
				break;
			case '%': //5
				std::cout << "view c" << std::endl;
				viewMode = View::CAD;
				break;
			default:
				std::cout << "code: " << event.getKeyCode() << std::endl;
			}
		}
	}
}

int
main(int argx, char** argv) {
	std::string device_id = "";
	try
	{
		std::cout << "Good Morning!" << std::endl;
		Viewpoint<pcl::PointXYZRGBA> point_viewer = Viewpoint<pcl::PointXYZRGBA>();
		point_viewer.run();
	}
	catch (pcl::io::IOException& e)
	{
		std::cout << "Exception thrown when instantiating viewer: " << e.what() << std::endl;
		OutputDebugStringA("Exception thrown: ");
		OutputDebugStringA(e.what());
		return(1);
	}
	std::system("pause");
	return(0);
}