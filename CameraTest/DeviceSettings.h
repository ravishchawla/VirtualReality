#pragma once

enum Modality {
	FACTORY_DEFAULT,
	FACE_TRACKING,
	HAND_TRACKING,
	USER_SEGMENTATION,
	OBJECT_TRACKING,
	TOUCHLESS_CONTROLLER
};

class DeviceSettings {
public:
	static void SetDeviceDepthSetting(PXCCapture::Device *device, Modality mode);
	static void SetDepthSettingValues(PXCCapture::Device *device,
		short confidenceThreshold,
		short filterOption,
		short laserPower,
		short motionRangeTradeoff,
		PXCCapture::Device::IVCAMAccuracy accuracy);
};