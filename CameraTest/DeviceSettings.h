#pragma once
#include <vector>
#include <opencv2\core\types.hpp>

enum Modality {
	FACTORY_DEFAULT,
	FACE_TRACKING,
	HAND_TRACKING,
	USER_SEGMENTATION,
	OBJECT_TRACKING,
	TOUCHLESS_CONTROLLER
};

typedef struct Point {
	int x;
	int y;
	int z;
} Point;


class DeviceSettings {
public:
	static void SetDeviceDepthSetting(PXCCapture::Device *device, Modality mode);
	static void SetDepthSettingValues(PXCCapture::Device *device,
		short confidenceThreshold,
		short filterOption,
		short laserPower,
		short motionRangeTradeoff,
		PXCCapture::Device::IVCAMAccuracy accuracy);
	static void DeviceSettings::StereoCalibrate(std::vector<std::vector<cv::Point2f>> imagePoints, std::vector<std::vector<cv::Point2f>> imagePoints2, std::vector<std::vector<cv::Point3f>> objectPoints, cv::Size size);
};