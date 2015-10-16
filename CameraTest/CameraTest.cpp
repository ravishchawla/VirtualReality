/* The purpose of this project is to get the depth and camera feeds
*/

#include "stdafx.h"
#include "pxcsensemanager.h"
#include <iostream>
#include <Windows.h>
#include "util_cmdline.h"
#include "util_render.h"
#include <conio.h>
#include "DeviceSettings.h"

#define NFRAMES 200

using namespace std;


int wmin(int argc, WCHAR* argv[]) {
	cout << "Hello, World!\n";

	PXCSenseManager *session = PXCSenseManager::CreateInstance();



	if (!session) {
		cout << "Unable to create instance of PXCSenseManager\n";
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
		//streams.deviceInfo.streams = PXCCapture::STREAM_TYPE_COLOR | PXCCapture:::STREAM_TYPE_DEPTH;
		streams.deviceInfo.streams = PXCCapture::STREAM_TYPE_DEPTH;
	}

	session->EnableStreams(&streams);

	//initialize pipeline
	status = session->Init();
	if (status < PXC_STATUS_NO_ERROR) {
		cout << "Failed to locate any video stream(s)\n";
		session->Release();
		return status;
	}

	//get device and reset it
	PXCCapture::Device *device = session->QueryCaptureManager()->QueryDevice();
	DeviceSettings::SetDeviceDepthSetting(device, Modality::FACTORY_DEFAULT);
	device->ResetProperties(PXCCapture::STREAM_TYPE_ANY);

	//stream data

	for (int nframes = 0; nframes < NFRAMES; nframes++) {
		//wait until new frame is availaible and locks it
		status = session->AcquireFrame(false);

		if (status < PXC_STATUS_NO_ERROR) {
			if (status == PXC_STATUS_STREAM_CONFIG_CHANGED) {
				cout << "Stream Config changed. Reinitializing\n";
				session->Close();
			}
			break;
		}

		//Render streams
		const PXCCapture::Sample *sample = session->QuerySample();
		if (sample) {
			if (sample->depth && !renderd.RenderFrame(sample->depth)) break;
			if (sample->color && !renderc.RenderFrame(sample->color)) break;
		}

		//release frame
		session->ReleaseFrame();

		//check for keystroke press
		if (_kbhit()) {
			int key = _getch() & 255;
			if (key == 27 || key == 'q' || key == 'Q') {
				break;
			}
		}
	} while (status == PXC_STATUS_STREAM_CONFIG_CHANGED);

	cout << "Exiting";

	session->Release();
	return 0;




	system("pause");
	return 0;
}

