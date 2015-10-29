#include <opencv2\calib3d\calib3d.hpp>
#include <opencv2\features2d\features2d.hpp>

#include <opencv2\opencv.hpp>
#include "CSVReader.h"
#include <Windows.h>
#include "pxcsensemanager.h"
#include "util_render.h"
#include "conio.h"


class Model
{
public:
	Model();
	void load(const std::string path);
	std::vector<cv::Point3f> list_points3d_in;

};

class Mesh
{
public:

	Mesh();
	void load(const std::string path);
	std::vector<cv::Point3f> list_vertex;
	std::vector<std::vector<int>> list_triangles;

	int num_vertices;
	int num_triangles;
};

void Model::load(const std::string path)
{
	cv::Mat points3d_mat;
	cv::Mat descriptors;

	cv::FileStorage storage(path, cv::FileStorage::READ);
	storage["points_3d"] >> points3d_mat;
	storage["descriptors"] >> descriptors;

	points3d_mat.copyTo(list_points3d_in);
	storage.release();
}

Model::Model() : list_points3d_in(0)
{

}

Mesh::Mesh() : list_vertex(0), list_triangles(0)
{
	num_vertices = 0;
	num_triangles = 0;
}

void Mesh::load(const std::string path)
{
	CsvReader csvReader(path);
	list_vertex.clear();
	list_triangles.clear();
	csvReader.readPLY(list_vertex, list_triangles);

	num_vertices = (int)list_vertex.size();
	num_triangles = (int)list_triangles.size();
}

int wmain3(int argc, WCHAR* argvW[]) {
	PXCSenseManager *session = PXCSenseManager::CreateInstance();
	


	if (!session) {
		std::cout << "Unable to create instance of PXCSenseManager\n";
		return 3;
	}

	
	//get capture manager instance if file record is set to true (m_brecord)
	PXCCaptureManager *captureManager = session->QueryCaptureManager();

	UtilRender renderc(L"Color");
	UtilRender renderd(L"Depth");

	pxcStatus status;

	PXCVideoModule::DataDesc streams = {};
	if (captureManager->QueryCapture()) {
		captureManager->QueryCapture()->QueryDeviceInfo(0, &streams.deviceInfo);
	}
	else {
		streams.deviceInfo.streams = PXCCapture::STREAM_TYPE_COLOR | PXCCapture::STREAM_TYPE_DEPTH;
		//streams.deviceInfo.streams = PXCCapture::STREAM_TYPE_DEPTH;
	}

	session->EnableStreams(&streams);


	//initialize pipeline
	status = session->Init();
	if (status < PXC_STATUS_NO_ERROR) {
		std::cout << "Failed to locate any video stream(s)\n";
		session->Release();
		return status;
	}

	//get device and reset it
	PXCCapture::Device *device = session->QueryCaptureManager()->QueryDevice();

	//stream data
	PXCImage::ImageData data;
	PXCImage::ImageData data_depth;

	unsigned char *rgb_data;
	unsigned char *depth_data;
	
	cv::Mat image(cv::Size(640, 480), CV_32F);
	cv::Mat depth(cv::Size(320, 240), CV_32F);

	
	for (;;) {
		//wait until new frame is availaible and locks it
		status = session->AcquireFrame(false);

		if (status < PXC_STATUS_NO_ERROR) {
			if (status == PXC_STATUS_STREAM_CONFIG_CHANGED) {
				std::cout << "Stream Config changed. Reinitializing\n";
				session->Close();
			}
			break;
		}

		//Render streams
		const PXCCapture::Sample *sample = session->QuerySample();
		PXCImage *color_image = sample->color;
		PXCImage *depth_image = sample->depth;
		
		color_image->AcquireAccess(PXCImage::ACCESS_READ_WRITE, &data);
		depth_image->AcquireAccess(PXCImage::ACCESS_READ_WRITE, &data_depth);

		rgb_data = data.planes[0];
		for (int y = 0; y<480; y++)
		{
			for (int x = 0; x<640; x++)
				{
					image.at<float>(y, x) = rgb_data[y * 640 + x];
				}
		}
	
		depth_data = data_depth.planes[0];
		for (int y = 0; y<240; y++)
		{
			for (int x = 0; x<320; x++)
			{
				depth.at<float>(y, x) = depth_data[y * 640 + x];
			}
		}

		//std::cout << image << std::endl;
		//std::cout << depth << std::endl;

		color_image->ReleaseAccess(&data);
		depth_image->ReleaseAccess(&data_depth);
		
		imshow("Color image", image);
		imshow("Depth image", depth);
		
		//release frame
		//if (!renderc.RenderFrame(color_image)) break;
		//if (!renderd.RenderFrame(depth_image)) break;
		session->ReleaseFrame();

		cv::Mat corners, corners_norm, corners_scaled;
		cv::cornerHarris(image, corners, 7, 5, .05, 4);
		cv::normalize(corners, corners_norm, 0, 255, cv::NORM_MINMAX, CV_32FC1, cv::Mat());
		cv::convertScaleAbs(corners_norm, corners_scaled);

		int count = 0;
		for (int i = 0; i < corners_norm.rows; i++) {
			for (int j = 0; j < corners_norm.cols; j++) {

				if ((int)corners_norm.at<float>(i, j) > 200) {
					count++;
				}
			}
		}
		std::cout << "eof " << count << std::endl;
	}

	std::cout << "Exiting";

	session->Release();
	return 0;




	std::system("pause");
	return 0;
}