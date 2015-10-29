#pragma once
#include <vector>
#include <opencv2\core\types.hpp>
#include <fstream>

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
	static void StereoCalibrate(std::vector<std::vector<cv::Point2f>> imagePoints, std::vector<std::vector<cv::Point2f>> imagePoints2, std::vector<std::vector<cv::Point3f>> objectPoints, cv::Size size, std::fstream *outfile);
	static bool SolvePose(std::vector<PXCPoint3DF32> objectPoints, std::vector<PXCPointF32> imagePoints);
private:
	static void SetDeviceDepthSetting(PXCCapture::Device *device, Modality mode);
	static void SetDepthSettingValues(PXCCapture::Device *device,
		short confidenceThreshold,
		short filterOption,
		short laserPower,
		short motionRangeTradeoff,
		PXCCapture::Device::IVCAMAccuracy accuracy);

};