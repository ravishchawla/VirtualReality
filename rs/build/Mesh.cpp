#include "Mesh.h"

Mesh<Point3DC>::Mesh(pcl::RealSenseGrabber& _grabber) :
	grabber(_grabber),
	viewer("Point Cloud viewer"),
	window(3),
	threshold(6)
{
	viewer.setCameraFieldOfView(0.785398); //approximately 45 degrees
	viewer.setCameraPosition(0, 0, 0, 0, 0, 1, 0, 1, 0);
	viewer.registerKeyboardCallback(&Mesh::keyboardCallback, *this);

	sor.setMeanK(50);
	sor.setStddevMulThresh(1.0);
	filter_cloud = boost::make_shared<pcl::PointCloud<Point3DC>>();
	cad_cloud = boost::make_shared < pcl::PointCloud<Point3DC>>();
}

Mesh<Point3DC>::~Mesh()
{
	connection.disconnect();
}

template<> void
Mesh<Point3DC>::run() {
	boost::function<void(const typename PointCloudT::ConstPtr&)> f = boost::bind(&Mesh<Point3DC>::cloudCallback, this, _1);
	std::cout << grabber.getName() << std::endl;
	std::cout << f << std::endl;
	if (VIEWER_MODE == MODE::USE_GRABBER)
	{
	connection = grabber.registerCallback(f);
	grabber.start();
	}
	
	else if (VIEWER_MODE == MODE::USE_CAD)
	{
	if (affine_transformation == NULL)
	{
	loadTransformFromFile(3);
	}

	transformPointCloud(cad_cloud, affine_transformation);

	}
	
	while (!viewer.wasStopped())
	{
		if (new_cloud)
		{
			boost::mutex::scoped_lock lock(new_cloud_mutex);

			if (filtering_on == true)
			{
				sor.setInputCloud(new_cloud);
				sor.filter(*filter_cloud);

				if (!viewer.updatePointCloud(filter_cloud, "cloud"))
				{
					viewer.addPointCloud(filter_cloud, "cloud");
					viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");
				}
			}
			else
			{
				if (!viewer.updatePointCloud(new_cloud, "cloud"))
				{
					viewer.addPointCloud(new_cloud, "cloud");
					viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");
				}
			}

			if (keys_found && !viewer.updatePointCloud(key_cloud, "keypoints"))
			{
				viewer.addPointCloud(key_cloud, "keypoints");
				viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "keypoints");
				viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "keypoints");

			}

			last_cloud = new_cloud;
			new_cloud.reset();
		}
		viewer.spinOnce(1, true);
	}

	if (grabber.isRunning())
	{
		grabber.stop();
	}
}

template<> void
Mesh<Point3DC>::cloudCallback(typename PointCloudT::ConstPtr cloud) {
	if (!viewer.wasStopped())
	{
		boost::mutex::scoped_lock lock(new_cloud_mutex);
		new_cloud = cloud;
	}
}

template<> void
Mesh<Point3DC>::keyboardCallback(const pcl::visualization::KeyboardEvent &event, void*) {
	if (event.keyDown())
	{
		if (event.getKeyCode() == 's' || event.getKeyCode() == 'S')
		{
			boost::format fmt("RS_%s_%u.pcd");
			std::string fn = boost::str(fmt % grabber.getDeviceSerialNumber().c_str() % last_cloud->header.stamp);
			pcl::io::savePCDFileBinaryCompressed(fn, *last_cloud);
			std::cout << "Saved point cloud " << fn.c_str() << std::endl;
		}

		else if (event.getKeyCode() == 'k' || event.getKeyCode() == 'K')
		{
			Mesh<Point3DC>::findKeyPoints(last_cloud);
		}
		else if (event.getKeyCode() == 'f' || event.getKeyCode() == 'F')
		{
			filtering_on = !filtering_on;
			std::string status = (filtering_on == true) ? "on" : "off";
			std::cout << "filtering turned to " << status << std::endl;
		}
		else if (event.getKeyCode() == 'v' || event.getKeyCode() == 'V')
		{
			Mesh<Point3DC>::visualizeCADFile();
		}

	}
}

template<> void
Mesh<Point3DC>::findKeyPoints(typename PointCloudT::ConstPtr cloud) {

	pcl::SIFTKeypoint<Point3DC, pcl::PointWithScale> detector;
	detector.setInputCloud(cloud);
	detector.setScales(1.0f, 10, 4);
	detector.setMinimumContrast(0.005f);



	pcl::search::KdTree<Point3DC>::Ptr tree(new pcl::search::KdTree<Point3DC>());
	detector.setSearchMethod(tree);

	pcl::PointCloud<pcl::PointWithScale>::Ptr keypoints(new pcl::PointCloud<pcl::PointWithScale>());
	std::cout << "starting keypoint detection with " << cloud->size() << " points" << std::endl;
	detector.compute(*keypoints);
	std::cout << "detection completed with total " << keypoints->size() << " points found" << std::endl;

	pcl::PointCloud<pcl::PointWithScale>::iterator i;
	key_cloud = boost::make_shared<pcl::PointCloud<Point3DC>>();
	std::cout << "size: " << (*key_cloud).size() << std::endl;

	//Uncomment for using Harris detector, and comment out the upper code
	/*
	pcl::HarrisKeypoint3D<Point3DC, pcl::PointXYZI> detector;
	detector.setInputCloud(cloud);
	detector.setRadius(20);
	key_cloud.reset();

	pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints(new pcl::PointCloud<pcl::PointXYZI>());
	std::cout << "starting keypoint detection with " << cloud->size() << " points" << std::endl;
	detector.compute(*keypoints);
	std::cout << "detection completed with total " << keypoints->size() << " points found" << std::endl;

	pcl::PointCloud<pcl::PointXYZI>::iterator i;
	key_cloud = boost::make_shared<pcl::PointCloud<Point3DC>>();
	*/
	for (i = keypoints->begin(); i != keypoints->end(); i++)
	{
		std::cout << "KeyPoint " << (*i).x << ", " << (*i).y << ", " << (*i).z << std::endl;
		Point3DC pptr;
		pptr.x = (*i).x;
		pptr.y = (*i).y;
		pptr.z = (*i).z;
		key_cloud->push_back(pptr);
	}
	keys_found = true;
}

template<> void
Mesh<Point3DC>::loadCADFromFile(std::string filename) {
	int success;
	try
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud;
		temp_cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

		success = pcl::io::loadPCDFile<pcl::PointXYZ>(filename, *temp_cloud);


		if (success < 0)
		{
			std::cout << "Unable to load CAD model from file. Make sure that file exists is openable" << std::endl;
		}

		pcl::copyPointCloud(*temp_cloud, *cad_cloud);
		std::cout << "Completed conversion : " << (*cad_cloud);
	}
	catch (std::exception exe)
	{
		std::cout << "Exception thrown in loadCADFromFile: " << exe.what() << std::endl;
	}
	return;
}

template<> void
Mesh<Point3DC>::loadTransformFromFile(int dim) {
	std::string filename = "transform.txt";
	std::ifstream dfile(filename);
	std::string line;
	int n;
	std::stringstream stream;
	affine_transformation = &Eigen::Affine3d();
	int row = 0;
	int col = 0;
	if (dfile.is_open()) {
		while (getline(dfile, line)) {
			while (col <= dim)
			{
				stream = std::stringstream(line);
				stream >> n;
				(*affine_transformation)(row, col++) = n;
			}
			col = 0;
			row++;
		}
		dfile.close();
	}
}

template<> void
Mesh<Point3DC>::visualizeCADFile() {
	if (cad_cloud->empty()) {
		loadCADFromFile(cad_filename);
	}	
	
	if (grabber.isRunning() == true) {
		grabber.stop();
	}

	new_cloud = cad_cloud;
}

template<> void
Mesh<Point3DC>::transformPointCloud(typename PointCloudT::Ptr cloud, Eigen::Affine3d *transform) {
	PointCloudT::Ptr transform_cloud;
	transform_cloud = boost::make_shared<pcl::PointCloud<Point3DC>>();
	pcl::transformPointCloud(*cloud, *transform_cloud, *transform);

	*cloud = *transform_cloud;
}



int main(int argx, char** argv)
{
	std::string device_id = "";
	try
	{
		std::cout << "Good Morning!" << std::endl;
		pcl::RealSenseGrabber grabber(device_id);
		std::vector<pcl::RealSenseGrabber::Mode> xyzrgba_modes = grabber.getAvailableModes(false);
		Mesh<pcl::PointXYZRGBA> mesh_viewer(grabber);
		mesh_viewer.run();
	}
	catch (pcl::io::IOException& e)
	{
		std::cout << "Exception thrown when creating grabber: " << e.what() << std::endl;
		OutputDebugStringA("Exception thrown: ");
		OutputDebugStringA(e.what());
		return(1);
	}
	return(0);
}
