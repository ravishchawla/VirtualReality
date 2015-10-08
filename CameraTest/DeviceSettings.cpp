#include "stdafx.h"
#include "pxcsensemanager.h"
#include <iostream>
#include <Windows.h>
#include "util_cmdline.h"
#include "util_render.h"
#include <conio.h>
#include "DeviceSettings.h"
using namespace std;


static void SetDeviceDepthSetting(PXCCapture::Device *device, Modality mode) {

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

static void SetDepthSettingValues(PXCCapture::Device *device, short confidenceThreshold, short filterOption, short laserPower, short motionRangeTradeoff, PXCCapture::Device::IVCAMAccuracy accuracy) {
	device->SetDepthConfidenceThreshold(confidenceThreshold);
	device->SetIVCAMFilterOption(filterOption);
	device->SetIVCAMLaserPower(laserPower);
	device->SetIVCAMMotionRangeTradeOff(motionRangeTradeoff);
	device->SetIVCAMAccuracy(accuracy);

}