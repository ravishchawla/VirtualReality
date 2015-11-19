#include "CameraCalib.h"

void
CameraCalib::clbReshape(int width, int height) {
	glutReshapeWindow(windowRect[0], windowRect[1]);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho(0, width, height, 0, -1, 1);
}

void
CameraCalib::exitFunc(int returnCode) {
	if (mainWnd >= 1)
	{
		glutDestroyWindow(mainWnd);
	}

	camBgraImage.reset(nullptr);
	camDepImage.reset(nullptr);
	verticesImage.reset(nullptr);
	depthFromVerticesImage.reset(nullptr);

	std::wcout << L"CameraCalib: Ends" << std::endl;
	if (currentFile.is_open())
		currentFile.flush();
		currentFile.close();

	exit(returnCode);
}

void
CameraCalib::clbExitFunc() {
	exitFunc(0);
}


void
CameraCalib::Reset() {
	trackingStatus = PXCScenePerception::TrackingAccuracy::FAILED;
	pScenePerceptionController->PauseScenePerception(true);
	points.clear();
	camera1Points.clear();
	camera2Points.clear();
	camera1Points.push_back(std::vector<cv::Point2f>());
	camera1Points.push_back(std::vector<cv::Point2f>());
	camera2Points.push_back(std::vector<cv::Point2f>());
	camera2Points.push_back(std::vector<cv::Point2f>());

}

std::string
CameraCalib::getTimeName(Feed feed) {
	time_t t = time(0);
	struct tm *now = localtime(&t);
	char filename[80];
	strftime(filename, 80, "%Y-%m-%d-%H-%M-%S", now);
	std::string name = std::string(filename);

	if (feed == COLOR1)
		name += "-color1";
	else if (feed == DEPTH1)
		name += "-depth1";
	else if (feed == COLOR2)
		name += "-color2";
	else if (feed == DEPTH2)
		name += "-depth2";
	else if (feed == NIL) {
		name += "-calib";
	}
	
	return name;
}

void
CameraCalib::snapshot() {
	const int WINDOW_WIDTH = 320;
	const int WINDOW_HEIGHT = 240;
	byte *buffer = (byte *)malloc(WINDOW_WIDTH*WINDOW_HEIGHT * 3);
	if (!buffer)
		return;


	glReadPixels((GLint)0, (GLint)0, (GLint)WINDOW_WIDTH - 1, (GLint)WINDOW_HEIGHT - 1,
		GL_RGB, GL_UNSIGNED_BYTE, buffer);

	std::string imagename = getTimeName(feedType) + ".bmp";
	const char * filename = imagename.c_str();
	FILE *file = fopen(filename, "wb");
	if (!file)
		return;

	BITMAPFILEHEADER bitmapFileHeader;
	BITMAPINFOHEADER bitmapInfoHeader;

	bitmapFileHeader.bfType = 0x4D42;
	bitmapFileHeader.bfSize = WINDOW_WIDTH * WINDOW_HEIGHT * 3;
	bitmapFileHeader.bfReserved1 = 0;
	bitmapFileHeader.bfReserved2 = 0;
	bitmapFileHeader.bfOffBits = sizeof(BITMAPFILEHEADER) + sizeof(BITMAPINFOHEADER);

	bitmapInfoHeader.biSize = sizeof(BITMAPINFOHEADER);
	bitmapInfoHeader.biWidth = WINDOW_WIDTH - 1;
	bitmapInfoHeader.biHeight = WINDOW_HEIGHT - 1;
	bitmapInfoHeader.biPlanes = 1;
	bitmapInfoHeader.biBitCount = 24;
	bitmapInfoHeader.biCompression = BI_RGB;
	bitmapInfoHeader.biSizeImage = 0;
	bitmapInfoHeader.biXPelsPerMeter = 0;
	bitmapInfoHeader.biYPelsPerMeter = 0;
	bitmapInfoHeader.biClrUsed = 0;
	bitmapInfoHeader.biClrImportant = 0;

	fwrite(&bitmapFileHeader, sizeof(BITMAPFILEHEADER), 1, file);
	fwrite(&bitmapInfoHeader, sizeof(BITMAPINFOHEADER), 1, file);
	fwrite(buffer, WINDOW_HEIGHT * WINDOW_WIDTH * 3, 1, file);
	fclose(file);
	free(buffer);



}

void
CameraCalib::read3dPoints(std::string filename, std::vector<cv::Point3f> *list) {
	std::ifstream dfile(filename);
	std::string line;
	int n;
	std::stringstream stream;
	if (dfile.is_open()) {
		while (getline(dfile, line)) {
			cv::Point3f point;
			stream = std::stringstream(line);
			stream >> n;
			point.x = n;

			stream >> n;
			point.y = n;

			stream >> n;
			point.z = n;
			list->push_back(point);
		}
	dfile.close();
	}
}

void
CameraCalib::read3dPoints() {
	objectPoints.push_back(std::vector<cv::Point3f>());
	objectPoints.push_back(std::vector<cv::Point3f>());

	read3dPoints("3dpoints1.txt", &objectPoints[0]);
	read3dPoints("3dpoints2.txt", &objectPoints[1]);
}

void
translateColorToDepth(std::vector<cv::Point>) {
	PXCProjection *projection = device->CreateProjection();

}

void
CameraCalib::translatePoints(Feed type)
{
	PXCProjection *projection = device->CreateProjection();	

	PXCImage::ImageInfo dInfo = depthImage->QueryInfo();
	PXCImage::ImageInfo cInfo = colorImage->QueryInfo();
	std::vector<PXCPoint3DF32> translatedPoints(points.size());
	std::vector<PXCPoint3DF32> pointsDepth = std::vector<PXCPoint3DF32>(points.size());
	if (type == DEPTH1 || type == DEPTH2) {

		PXCImage::ImageData depthData;
		if (depthImage->AcquireAccess(PXCImage::ACCESS_READ, PXCImage::PIXEL_FORMAT_DEPTH, &depthData) >= PXC_STATUS_NO_ERROR) {
			unsigned short *depthBuffer = (unsigned short*)depthData.planes[0];


			for (int i = 0; i < points.size(); i++) {
				pointsDepth[i].x = points[i].x;
				pointsDepth[i].y = points[i].y;

				int zloc = points[i].y * IMG_WIDTH + points[i].x;
				pointsDepth[i].z = depthBuffer[zloc];
				std::cout << "D: " << depthBuffer[zloc] << std::endl;
			}
			
			depthImage->ReleaseAccess(&depthData);
			//pxcStatus status = projection->MapDepthToColor(points.size(), &pointsDepth[0], &translatedPoints[0]);
			pxcStatus status = projection->ProjectColorToCamera(points.size(), &pointsDepth[0], &translatedPoints[0]);
			if (status >= PXC_STATUS_NO_ERROR) {
				DeviceSettings::SolvePose(translatedPoints, points);
			}
			else {
				std::cout << "unsuccessful translation" << std::endl;
			}
		}
		else {
			std::cout << "incorrect mode - switch to depth" << std::endl;
		}
	}
		
	for (int i = 0; i < points.size(); i++) {
		//std::cout << pointsDepth[i].x << ", " << pointsDepth[i].y << ", " << pointsDepth[i].z << std::endl;
		std::cout << translatedPoints[i].x << ", " << translatedPoints[i].y << ", " << translatedPoints[i].z<< std::endl;
	}
		
}

void
CameraCalib::switchMode(Feed type, boolean writeToFile) {
	std::cout << "Change mode to " << type << std::endl;
	feedType = type;

	if (!writeToFile) {
		translatePoints(type);
		return;
	}
	
	Reset();
	if (currentFile.is_open()) {
		currentFile.flush();
		currentFile.close();
	}
	
	if (type == COLOR1) {
		currentFile.open(colorFile1, std::fstream::out);
		currentList = &camera1Points[0];
	}
	else if (type == DEPTH1) {
		currentFile.open(depthFile1, std::fstream::out);
		currentList = &camera1Points[1];
	}
	else if (type == COLOR2) {
		currentFile.open(colorFile2, std::fstream::out);
		currentList = &camera2Points[0];
	}
	else if (type == DEPTH2) {
		currentFile.open(depthFile2, std::fstream::out);
		currentList = &camera2Points[1];
	}
}

void
CameraCalib::clbKeys(unsigned char key, int x, int y) {
	switch (key)
	{
	case 'q': // quit
	case 27: // ESC to quit
		glutLeaveMainLoop();
		break;

	case 'r': // reset tracking and accumulation
		CameraCalib::Reset();
		break;
	case 'd': //switch to depth mode
		CameraCalib::switchMode(DEPTH1, true);
		break;
	case 'p':
		CameraCalib::translatePoints(feedType);
		break;
	case 'c': //switch to color mode
		CameraCalib::switchMode(COLOR1, true);
		break;
	case 'k': //take snapshot
		std::cout << "Taking screenshot\n";
		CameraCalib::snapshot();
		break;
	case 's': //start calibration
		CameraCalib::switchMode(COLOR1, true);
		std::cout << "Start calibration process" << std::endl;
		std::cout << "3d points should be in a local file 3dpoints1.txt and 3depoints2.txt" << std::endl;
		std::cout << "Press n to go to next image/feed" << std::endl;
		break;
	case 'n': //move to next step
		CameraCalib::snapshot();
		if (feedType == COLOR1) {
			std::cout << "select the same points without moving the camera in Depth View" << std::endl;
			switchMode(DEPTH1, true);
		}
		else if (feedType == DEPTH1) {
			std::cout << "select the same points from a different perspective" << std::endl;
			CameraCalib::switchMode(COLOR2, true);
		}
		else if (feedType == COLOR2) {
			std::cout << "select the same points without moving the camera in Depth View" << std::endl;
			CameraCalib::switchMode(DEPTH2, true);
		}
		else if (feedType == DEPTH2) {
			std::cout << "Ensure 3d points are in local file 3dpoints(1,2).txt before pressing enter" << std::endl;
			system("pause");
			CameraCalib::read3dPoints();
			cv::Size size(320, 240);
			calibFile.open(calibFileStr, std::fstream::out);
			DeviceSettings::StereoCalibrate(camera1Points, camera2Points, objectPoints, size, &calibFile);
			calibFile.close();
		}
		else {
			std::cout << "Invalid state" << std::endl;
			break;
		}
		break;
	}
}

void
CameraCalib::StartScenePerception() {
	pScenePerceptionController->ResetScenePerception();
	pScenePerceptionController->PauseScenePerception(false);
	imageQuality = 0.0f;
}

void
CameraCalib::clbDisplay(void) {
	StartScenePerception();

	if (!pScenePerceptionController->ProcessNextFrame(&sample, curPose, trackingStatus, imageQuality))
	{
		glutLeaveMainLoop();
			return;
		}
		
		PXCImage *imageType;
		depthImage = sample.depth;
		colorImage = sample.color;

		if (feedType == DEPTH1 || feedType == DEPTH2) {
			imageType = depthImage;
		}
		else {
			imageType = colorImage;
		}

		CopyColorPxcImageToBuffer(imageType, camBgraImage.get(), COLOR_CAPTURE_WIDTH, COLOR_CAPTURE_HEIGHT);
		GetAlignedDepth(pScenePerceptionController->QueryProjection(), sample.depth, sample.color, camDepImage.get(), IMG_WIDTH, IMG_HEIGHT);
		pScenePerceptionController->QueryScenePerception()->GetVertices((PXCPoint3DF32 *)verticesImage.get());
	

	 //Start OpenGL rendering by reshaping and clearing buffer
	 glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	 
	
	glLineWidth(1.0f);
	
	std::vector<PXCPointF32>::iterator it;
	float win = 2.0;
	for (it = points.begin(); it < points.end(); it++) {
		glBegin(GL_QUADS);
		glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
		glVertex2f(it->x - win, it->y - win);
		glVertex2f(it->x + win, it->y - win);
		glVertex2f(it->x + win, it->y + win);
		glVertex2f(it->x - win, it->y + win);
		glEnd();
	}

	
	colorRenderer.Draw(camBgraImage.get());

	glutSwapBuffers();
	pScenePerceptionController->CleanupFrame();
	glutPostRedisplay();
}

void
CameraCalib::clbMouse(int clickedbutton, int clickState, int x, int y) {
	if (GLUT_LEFT_BUTTON == clickedbutton && GLUT_DOWN == clickState)
	{
		std::cout << "X: " << x << " Y: " << y << "\n";	
		PXCPointF32 p;
		p.x = (float)x;
		p.y = (float)y;
		
		points.push_back(p);
		if (currentFile.is_open()) {
			currentFile << x << " " << y << std::endl;
		}

		if (currentList) {
			currentList->push_back(cv::Point2f(x, y));
		}
	}
}

bool
CameraCalib::InitGlutUI(int argc, WCHAR* argvW[]) {
	srand(unsigned int(time(NULL)));

	const size_t len = wcsnlen(argvW[0], 1024);
	std::unique_ptr<char[]> argvuq(new char[len + 1]);
	char *pStr = (argvuq.get());
	size_t lastChar = 0;
	wcstombs_s(&lastChar, pStr, len, argvW[0], len - 1);
	pStr[lastChar] = '\0';
	char **argv = &pStr;
	argc = 1;

	glutInit(&argc, argv);
	glutInitWindowSize(windowRect[0], windowRect[1]);
	glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH); //
	mainWnd = glutCreateWindow("CameraCalib");

	GLenum err = glewInit();
	if (GLEW_OK != err)
	{
		std::wcout << L"Camera Calib" << glewGetErrorString(err) << std::endl;
		return false;
	}
	
	glutDisplayFunc(clbDisplay);
	glutKeyboardFunc(clbKeys);
	glutReshapeFunc(clbReshape);
	glutMouseFunc(clbMouse);
	glutCloseFunc(clbExitFunc);

	glEnable(GL_DEPTH_TEST);
	return true;
}

bool
CameraCalib::AllocateLocalBuffers() {
	camBgraImage.reset(new unsigned char[4 * IMG_WIDTH * IMG_HEIGHT]);
	camDepImage.reset(new unsigned short[IMG_WIDTH * IMG_HEIGHT]);
	verticesImage.reset(new float[3 * IMG_WIDTH * IMG_HEIGHT]);
	depthFromVerticesImage.reset(new float[IMG_WIDTH * IMG_HEIGHT]);
	return true;
}

int
main(int argc, WCHAR* argvW[]) {
	std::wcout << "Depth/Color Opengl Rendering" << std::endl;
	
	session = PXCSenseManager::CreateInstance();

	if (!session) {
		std::cout << "Unable to create instance of PXCSenseManager\n";
		return 3;
	}

	PXCCaptureManager *captureManager = session->QueryCaptureManager();
	pxcStatus status;

	PXCVideoModule::DataDesc streams = {};
	if (session->QueryCaptureManager()->QueryCapture()) {
		session->QueryCaptureManager()->QueryCapture()->QueryDeviceInfo(0, &streams.deviceInfo);
	}
	else {
		streams.deviceInfo.streams = PXCCapture::STREAM_TYPE_COLOR | PXCCapture::STREAM_TYPE_DEPTH;
		//streams.deviceInfo.streams = PXCCapture::STREAM_TYPE_DEPTH;
	}

	session->EnableStreams(&streams);

	colorFile1 = CameraCalib::getTimeName(COLOR1) + ".txt";
	depthFile1 = CameraCalib::getTimeName(DEPTH1) + ".txt";
	colorFile2 = CameraCalib::getTimeName(COLOR2) + ".txt";
	depthFile2 = CameraCalib::getTimeName(DEPTH2) + ".txt";
	calibFileStr = CameraCalib::getTimeName(NIL) + ".txt";

	pScenePerceptionController.reset(new ScenePerceptionController(L"Depth/Color Opengl Rendering", argc, argvW, L"",
		COLOR_CAPTURE_WIDTH, COLOR_CAPTURE_HEIGHT, COLOR_FRAMERATE,
		IMG_WIDTH, IMG_HEIGHT, DEPTH_FRAMERATE));

	if ((*pScenePerceptionController.get()) == false)
	{
		return 1;
	}

	pScenePerceptionController->PauseScenePerception(true);

	if (!pScenePerceptionController->InitPipeline())
	{
		return 1;
	}

	status = session->Init();
	device = session->QueryCaptureManager()->QueryDevice();

	feedType = DEPTH1;
	if (pScenePerceptionController->InitializeProjection() < PXC_STATUS_NO_ERROR)
	{
		std::wcout << L"CameraCalib: Create Projection Failed" << std::endl;
		return 1;
	}

	if (!CameraCalib::InitGlutUI(argc, argvW))
	{
		CameraCalib::exitFunc(2);
	}

	if (!CameraCalib::AllocateLocalBuffers())
	{
		CameraCalib::exitFunc(4);
	}
	colorRenderer.SetImageSize(COLOR_CAPTURE_WIDTH, COLOR_CAPTURE_HEIGHT);

	glutMainLoop();


	return 0;
}
