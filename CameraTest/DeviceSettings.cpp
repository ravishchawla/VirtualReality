#include "stdafx.h"
#include "pxcsensemanager.h"
#include <iostream>
#include <Windows.h>
#include "util_cmdline.h"
#include "util_render.h"
#include <conio.h>
#include "DeviceSettings.h"
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/mat.hpp>
#include <time.h>


void wmain2(int argc, WCHAR* argvW[]) {

		int c;
		c = 0;
		std::cout << "c: " << c;

		
	/*
	
	vector<vector<Point2f>> image1Points;
	vector<vector<Point2f>> image2Points;
	vector<vector<Point3f>> objectPoints;

	image1Points.push_back(vector<Point2f>());
	image1Points.push_back(vector<Point2f>());
	image2Points.push_back(vector<Point2f>());
	image2Points.push_back(vector<Point2f>());
	objectPoints.push_back(vector<Point3f>());
	objectPoints.push_back(vector<Point3f>());

	float xt = 1.45;
	float yt = 1.45;
	float zt = 2;
	objectPoints[0].push_back(Point3f(-1 + xt, 1 + yt, 0 + zt));
	objectPoints[0].push_back(Point3f(1 + xt, 1 + yt, 0 + zt));
	objectPoints[0].push_back(Point3f(1 + xt, -1 + yt, 0 + zt));
	objectPoints[0].push_back(Point3f(1 + xt, -1 + yt, -2 + zt));
	objectPoints[0].push_back(Point3f(-1 + xt, -1 + yt, -2 + zt));
	objectPoints[0].push_back(Point3f(-1 + xt, -1 + yt, 0 + zt));

	xt = -3.1;
	yt = 0.6;
	objectPoints[1].push_back(Point3f(-1 + xt, 1 + yt, 0 + zt));
	objectPoints[1].push_back(Point3f(1 + xt, 1 + yt, 0 + zt));
	objectPoints[1].push_back(Point3f(1 + xt, -1 + yt, 0 + zt));
	objectPoints[1].push_back(Point3f(1 + xt, -1 + yt, -2 + zt));
	objectPoints[1].push_back(Point3f(-1 + xt, -1 + yt, -2 + zt));
	objectPoints[1].push_back(Point3f(-1 + xt, -1 + yt, 0 + zt));

	image1Points[0].push_back(Point2f(279, 292));
	image1Points[0].push_back(Point2f(359, 284));
	image1Points[0].push_back(Point2f(384, 313));
	image1Points[0].push_back(Point2f(386, 398));
	image1Points[0].push_back(Point2f(299, 406));
	image1Points[0].push_back(Point2f(297, 324));

	image1Points[1].push_back(Point2f(374, 298));
	image1Points[1].push_back(Point2f(452, 290));
	image1Points[1].push_back(Point2f(489, 316));
	image1Points[1].push_back(Point2f(487, 395));
	image1Points[1].push_back(Point2f(397, 402));
	image1Points[1].push_back(Point2f(399, 320));

	image2Points[0].push_back(Point2f(65, 342));
	image2Points[0].push_back(Point2f(147, 312));
	image2Points[0].push_back(Point2f(207, 330));
	image2Points[0].push_back(Point2f(218, 418));
	image2Points[0].push_back(Point2f(135, 458));
	image2Points[0].push_back(Point2f(122, 367));

	image2Points[1].push_back(Point2f(167, 341));
	image2Points[1].push_back(Point2f(233, 312));
	image2Points[1].push_back(Point2f(318, 334));
	image2Points[1].push_back(Point2f(309, 427));
	image2Points[1].push_back(Point2f(230, 460));
	image2Points[1].push_back(Point2f(219, 370));

	time_t t = time(0);
	struct tm *now = localtime(&t);
	char filename[80];
	strftime(filename, 80, "%Y-%m-%d-%H-%M-%S", now);
	string name = std::string(filename) + "-calib-results.txt";

	fstream file;
	file.open(name, fstream::out);

	DeviceSettings::StereoCalibrate(image1Points, image2Points, objectPoints, cv::Size(320, 240), &file);
	file.flush();
	file.close();
	*/
	std::system("pause");

}

void DeviceSettings::StereoCalibrate(std::vector<std::vector<cv::Point2f>> imagePoints, std::vector<std::vector<cv::Point2f>> imagePoints2, std::vector<std::vector<cv::Point3f>> objectPoints, cv::Size size, std::fstream *outfile) {
	
	cv::Mat cameraMatrix[2];
	cameraMatrix[0] = cv::Mat::eye(3, 3, CV_32F);
	cameraMatrix[1] = cv::Mat::eye(3, 3, CV_32F);

	cv::Mat distortionCoefficients[2];
	cv::Mat rotationMatrix;
	cv::Mat translationVector;
	cv::Mat essentialMatrix;
	cv::Mat fundamentalMatrix;

	cv::TermCriteria term_crit = cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 1e-6);
	int flags = cv::CALIB_FIX_INTRINSIC;
	double rms;
	try {
		rms = stereoCalibrate(objectPoints, imagePoints, imagePoints2, cameraMatrix[0], distortionCoefficients[0],
			cameraMatrix[1], distortionCoefficients[1], size, rotationMatrix,
			translationVector, essentialMatrix, fundamentalMatrix,
			flags, term_crit);
	}
	catch (cv::Exception e) {
		OutputDebugStringA("EXCEPTION THROWN. CAUGHT: ");
		OutputDebugStringA(e.what());
	}

	std::cout << "rms: " << rms << std::endl;
	*outfile << "rms: " << rms << std::endl;
	std::cout << "distortion matrix[0]: " << distortionCoefficients[0] << std::endl;
	*outfile << "distortion matrix[0]: " << distortionCoefficients[0] << std::endl;
	std::cout << "distortion matrix[1]: " << distortionCoefficients[1] << std::endl;
	*outfile << "distortion matrix[1]: " << distortionCoefficients[1] << std::endl;
	std::cout << "rotation matrix: " << rotationMatrix << std::endl;
	*outfile << "rotation matrix: " << rotationMatrix << std::endl;
	std::cout << "translation matrix: " << translationVector << std::endl;
	*outfile << "translation matrix: " << translationVector << std::endl;
	std::cout << "essential matrix: " << essentialMatrix << std::endl;
	*outfile << "essential matrix: " << essentialMatrix << std::endl;
	std::cout << "fundamental matrix: " << fundamentalMatrix << std::endl;
	*outfile << "fundamental matrix: " << fundamentalMatrix << std::endl;
}

void DeviceSettings::SetDeviceDepthSetting(PXCCapture::Device *device, Modality mode) {

	switch (mode) {
	case Modality::FACE_TRACKING:
		DeviceSettings::SetDepthSettingValues(device, 1, 6, 16, 21, PXCCapture::Device::IVCAM_ACCURACY_MEDIAN);
		break;
	case Modality::HAND_TRACKING:
		DeviceSettings::SetDepthSettingValues(device, 1, 6, 16, 0, PXCCapture::Device::IVCAM_ACCURACY_MEDIAN);
		break;
	case Modality::USER_SEGMENTATION:
		DeviceSettings::SetDepthSettingValues(device, 0, 6, 16, 21, PXCCapture::Device::IVCAM_ACCURACY_COARSE);
		break;
	case Modality::OBJECT_TRACKING:
		DeviceSettings::SetDepthSettingValues(device, 6, 5, 0, 16, PXCCapture::Device::IVCAM_ACCURACY_MEDIAN);
		break;
	case Modality::TOUCHLESS_CONTROLLER:
		DeviceSettings::SetDepthSettingValues(device, 1, 6, 16, 0, PXCCapture::Device::IVCAM_ACCURACY_MEDIAN);
		break;
	case Modality::FACTORY_DEFAULT:
	default:
		DeviceSettings::SetDepthSettingValues(device, 6, 5, 16, 0, PXCCapture::Device::IVCAM_ACCURACY_MEDIAN);
		break;
	}

	
}

bool DeviceSettings::SolvePose(std::vector<PXCPoint3DF32> objectPoints, std::vector<PXCPointF32> imagePoints) {
	
	std::vector<cv::Point3f> objPoints(objectPoints.size());
	std::vector<cv::Point2f> imgPoints(imagePoints.size());

	for (int i = 0; i < objectPoints.size(); i++) {
		objPoints[i].x = objectPoints[i].x;
		objPoints[i].y = objectPoints[i].y;
		objPoints[i].z = objectPoints[i].z;

		imgPoints[i].x = imagePoints[i].x;
		imgPoints[i].y = imagePoints[i].y;
	}
	
	cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_32F);
	cv::Mat distCoeffs = cv::Mat::zeros(1, 4, CV_32F);


	cv::Mat rvec;
	cv::Mat tvec;

	cv::Mat o = cv::Mat(objPoints);
	cv::Mat i = cv::Mat(imgPoints);

	cv::InputArray oinn = o;
	cv::InputArray iin = i;

	cv::Mat opoints = oinn.getMat();
	cv::Mat ipoints = iin.getMat();

	bool result = cv::solvePnP(opoints, ipoints, cameraMatrix, distCoeffs, rvec, tvec, false, 0);

	std::cout << "Rotation Matrix: " << rvec << std::endl << std::endl;

	std::cout << "Translation Matrix: " << tvec << std::endl << std::endl;

	return result;
}
/*do not call this method, its risky*/
void DeviceSettings::SetDepthSettingValues(PXCCapture::Device *device, short confidenceThreshold, short filterOption, short laserPower, short motionRangeTradeoff, PXCCapture::Device::IVCAMAccuracy accuracy) {
	device->SetDepthConfidenceThreshold(confidenceThreshold);
	device->SetIVCAMFilterOption(filterOption);
	device->SetIVCAMLaserPower(laserPower);
	device->SetIVCAMMotionRangeTradeOff(motionRangeTradeoff);
	device->SetIVCAMAccuracy(accuracy);

}