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
using namespace std;
using namespace cv;


void DeviceSettings::StereoCalibrate(vector<vector<Point2f>> imagePoints, vector<vector<Point2f>> imagePoints2, vector<vector<Point3f>> objectPoints, Size size) {
	
	Mat cameraMatrix[2];
	
	
	cameraMatrix[0] = Mat::eye(3, 3, CV_32F);
	cameraMatrix[1] = Mat::eye(3, 3, CV_32F);

	Mat distortionCoefficients[2];
	Mat rotationMatrix;
	Mat translationVector;
	Mat essentialMatrix;
	Mat fundamentalMatrix;

	TermCriteria term_crit = TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 30, 1e-6);
	int flags = CALIB_FIX_INTRINSIC;
	double rms;
	try {
		rms = stereoCalibrate(objectPoints, imagePoints, imagePoints2, cameraMatrix[0], distortionCoefficients[0],
			cameraMatrix[1], distortionCoefficients[1], size, rotationMatrix,
			translationVector, essentialMatrix, fundamentalMatrix,
			flags, term_crit);
	}
	catch (exception e) {
		OutputDebugStringA("EXCEPTION THROWN. CAUGHT: ");
		OutputDebugStringA(e.what());
	}

	cout << "calibration complete: " << rms << endl;
	cout << "distortion matrix: " << distortionCoefficients << endl;
	cout << "rotation matrix: " << rotationMatrix << endl;
	cout << "translation matrix: " << translationVector << endl;
	cout << "essential matrix: " << essentialMatrix << endl;
	cout << "fundamental matrix: " << fundamentalMatrix << endl;
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

void DeviceSettings::SetDepthSettingValues(PXCCapture::Device *device, short confidenceThreshold, short filterOption, short laserPower, short motionRangeTradeoff, PXCCapture::Device::IVCAMAccuracy accuracy) {
	device->SetDepthConfidenceThreshold(confidenceThreshold);
	device->SetIVCAMFilterOption(filterOption);
	device->SetIVCAMLaserPower(laserPower);
	device->SetIVCAMMotionRangeTradeOff(motionRangeTradeoff);
	device->SetIVCAMAccuracy(accuracy);

}