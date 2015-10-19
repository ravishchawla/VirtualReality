/********************************************************************************

INTEL CORPORATION PROPRIETARY INFORMATION This software is supplied under the
terms of a license agreement or nondisclosure agreement with Intel Corporation
and may not be copied or disclosed except in accordance with the terms of that
agreement.
Copyright(c) 2011-2015 Intel Corporation. All Rights Reserved.

*********************************************************************************/

#define NOMINMAX
#include <windows.h>
#define _USE_MATH_DEFINES
#include "math.h"
#include <algorithm>
#include <conio.h>
#include <fstream>
#include <sstream>
#include<ctime>

#include "sp_controller.h"
#include "sp_color_renderer.h"
#include "sp_fps_counter.h"

#include <vector>
#include <future>
#include <GL/glew.h>
#include <GL/GL.h>
#include <GL/glut.h>
#include <GL/freeglut_ext.h>
#include "sp_glut_ui_utils.h"
#include "DeviceSettings.h"

using namespace std;
using namespace AppUtils;




enum Feed
{
	COLOR1,
	DEPTH1,
	COLOR2,
	DEPTH2
};

const int IMG_WIDTH = 320;
const int IMG_HEIGHT = 240;
const int COLOR_CAPTURE_WIDTH = 320;
const int COLOR_CAPTURE_HEIGHT = 240;
const float COLOR_FRAMERATE = 60.0f;
const float DEPTH_FRAMERATE = 60.0f;
vector<Point> points;
// Default values, matrices and vectors 
PXCScenePerception::VoxelResolution voxelResolution = PXCScenePerception::VoxelResolution::LOW_RESOLUTION;

Feed feedType;
fstream currentFile;
string depthFile1;
string colorFile1;
string depthFile2;
string colorFile2;

int mainWnd = 0;
int windowRect[2] = { 2 * IMG_WIDTH, 2 * IMG_HEIGHT};

float curPose[12] = { 0 };
PXCScenePerception::TrackingAccuracy trackingStatus = PXCScenePerception::TrackingAccuracy::FAILED;
ColorRenderer colorRenderer(0, 0, 640, 480);

std::unique_ptr<ScenePerceptionController> pScenePerceptionController;
// Images captured and images displayed
std::unique_ptr<unsigned char[]> camBgraImage(nullptr); // the captured BGRA image 
std::unique_ptr<unsigned short[]> camDepImage(nullptr); // the captured depth image
std::unique_ptr<float[]> verticesImage(nullptr); // Vertices image 
std::unique_ptr<float[]> depthFromVerticesImage(nullptr); // vertices image converted to depth image 

std::vector<std::vector<cv::Point2f>> camera1Points;
std::vector<std::vector<cv::Point2f>> camera2Points;
std::vector<std::vector<cv::Point3f>> objectPoints;

std::vector<cv::Point2f> *currentList;

float imageQuality = 0.0f;
PXCCapture::Sample sample;

static GLuint sdl = -1;


void clbReshape(int width, int height)
{
	glutReshapeWindow(windowRect[0], windowRect[1]);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho(0, width, height, 0, -1, 1);
}

void exitFunc(int returnCode)
{
	if (mainWnd >= 1)
	{
		glutDestroyWindow(mainWnd);
	}

	camBgraImage.reset(nullptr);
	camDepImage.reset(nullptr);
	verticesImage.reset(nullptr);
	depthFromVerticesImage.reset(nullptr);

	wcout << L"RF_AugmentedRealitySP: Ends" << endl;
	if (currentFile.is_open())
		currentFile.flush();
		currentFile.close();

	exit(returnCode);
}

void clbExitFunc()
{
	exitFunc(0);
}


void Reset()
{
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

string getTimeName(Feed feed)
{
	time_t t = time(0);
	struct tm *now = localtime(&t);
	char filename[80];
	strftime(filename, 80, "%Y-%m-%d-%H-%M-%S", now);
	string name = std::string(filename);

	if (feed == COLOR1)
		name += "-color1";
	else if (feed == DEPTH1)
		name += "-depth1";
	else if (feed == COLOR2)
		name += "-color2";
	else if (feed == DEPTH2)
		name += "-depth2";

	return name;
}

void snapshot()
{
	const int WINDOW_WIDTH = 320;
	const int WINDOW_HEIGHT = 240;
	byte *buffer = (byte *)malloc(WINDOW_WIDTH*WINDOW_HEIGHT * 3);
	if (!buffer)
		return;


	glReadPixels((GLint)0, (GLint)0, (GLint)WINDOW_WIDTH - 1, (GLint)WINDOW_HEIGHT - 1,
		GL_RGB, GL_UNSIGNED_BYTE, buffer);

	string imagename = getTimeName(feedType) + ".bmp";
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

void read3dPoints(string filename, std::vector<cv::Point3f> *list) {
	ifstream dfile(filename);
	string line;
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

void read3dPoints() {
	objectPoints.push_back(std::vector<cv::Point3f>());
	objectPoints.push_back(std::vector<cv::Point3f>());

	read3dPoints("3dpoints1.txt", &objectPoints[0]);
	read3dPoints("3dpoints2.txt", &objectPoints[1]);
}

void switchMode(Feed type, boolean writeToFile) {
	cout << "Change mode to " << type;
	feedType = type;
	Reset();

	if (!writeToFile) {
		return;
	}
	
	if (currentFile.is_open()) {
		currentFile.flush();
		currentFile.close();
	}
	
	if (type == COLOR1) {
		currentFile.open(colorFile1, fstream::out);
		currentList = &camera1Points[0];
	}
	else if (type == DEPTH1) {
		currentFile.open(depthFile1, fstream::out);
		currentList = &camera1Points[1];
	}
	else if (type == COLOR2) {
		currentFile.open(colorFile2, fstream::out);
		currentList = &camera2Points[0];
	}
	else if (type == DEPTH2) {
		currentFile.open(depthFile2, fstream::out);
		currentList = &camera2Points[1];
	}
}

void clbKeys(unsigned char key, int x, int y)
{
	switch (key)
	{
	case 'q': // quit
	case 27: // ESC to quit
		glutLeaveMainLoop();
		break;

	case 'r': // reset tracking and accumulation
		Reset();
		break;
	case 'd': //switch to depth mode
		switchMode(DEPTH1, false);
		break;
	case 'c': //switch to color mode
		switchMode(COLOR1, false);
		break;
	case 'k': //take snapshot
		cout << "Taking screenshot\n";
		snapshot();
		break;
	case 's': //start calibration
		switchMode(COLOR1, true);
		cout << "Start calibration process" << endl;
		cout << "3d points should be in a local file 3dpoints1.txt and 3depoints2.txt" << endl;
		cout << "Press n to go to next image/feed" << endl;
		break;
	case 'n': //move to next step
		snapshot();
		if (feedType == COLOR1) {
			cout << "select the same points without moving the camera in Depth View" << endl;
			switchMode(DEPTH1, true);
		}
		else if (feedType == DEPTH1) {
			cout << "select the same points from a different perspective" << endl;
			switchMode(COLOR2, true);
		}
		else if (feedType == COLOR2) {
			cout << "select the same points without moving the camera in Depth View" << endl;
			switchMode(DEPTH2, true);
		}
		else if (feedType == DEPTH2) {
			cout << "Ensure 3d points are in local file 3dpoints(1,2).txt before pressing enter" << endl;
			system("pause");
			read3dPoints();
			cv::Size size(320, 240);
			DeviceSettings::StereoCalibrate(camera1Points, camera2Points, objectPoints, size);
		}
		else {
			cout << "Invalid state" << endl;
			break;
		}
		break;
	}
}

void StartScenePerception()
{
	pScenePerceptionController->ResetScenePerception();
	pScenePerceptionController->PauseScenePerception(false);
	imageQuality = 0.0f;
}

void clbDisplay(void)
{
	StartScenePerception();

	if (!pScenePerceptionController->ProcessNextFrame(&sample, curPose, trackingStatus, imageQuality))
	{
		glutLeaveMainLoop();
			return;
		}
		
		PXCImage *imageType;
		if (feedType == DEPTH1 || feedType == DEPTH2) {
			imageType = sample.depth;
		}
		else {
			imageType = sample.color;
		}
		CopyColorPxcImageToBuffer(imageType, camBgraImage.get(), COLOR_CAPTURE_WIDTH, COLOR_CAPTURE_HEIGHT);
		GetAlignedDepth(pScenePerceptionController->QueryProjection(), sample.depth, sample.color, camDepImage.get(), IMG_WIDTH, IMG_HEIGHT);
		pScenePerceptionController->QueryScenePerception()->GetVertices((PXCPoint3DF32 *)verticesImage.get());
	

	 //Start OpenGL rendering by reshaping and clearing buffer
	 glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	 
	
	glLineWidth(1.0f);
	
	vector<Point>::iterator it;
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

void clbMouse(int clickedbutton, int clickState, int x, int y)
{
	if (GLUT_LEFT_BUTTON == clickedbutton && GLUT_DOWN == clickState)
	{
		cout << "X: " << x << " Y: " << y << "\n";	
		Point p = { (float)x, (float)y};
		points.push_back(p);
		if (currentFile.is_open()) {
			currentFile << x << " " << y << endl;
		}

		if (currentList) {
			currentList->push_back(cv::Point2f(x, y));
		}
	}
}

bool InitGlutUI(int argc, WCHAR* argvW[])
{
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
	mainWnd = glutCreateWindow("Intel(R) RealSense(TM) SDK: SP Augmented Reality");

	GLenum err = glewInit();
	if (GLEW_OK != err)
	{
		wcout << L"RF_AugmentedRealitySP: Error - " << glewGetErrorString(err) << endl;
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

bool AllocateLocalBuffers()
{
	camBgraImage.reset(new unsigned char[4 * IMG_WIDTH * IMG_HEIGHT]);
	camDepImage.reset(new unsigned short[IMG_WIDTH * IMG_HEIGHT]);
	verticesImage.reset(new float[3 * IMG_WIDTH * IMG_HEIGHT]);
	depthFromVerticesImage.reset(new float[IMG_WIDTH * IMG_HEIGHT]);
	return true;
}

int main(int argc, WCHAR* argvW[])
{
	wcout << "RF_AugmentedRealitySP: Starts." << endl;
	
	
	colorFile1 = getTimeName(COLOR1) + ".txt";
	depthFile1 = getTimeName(DEPTH1) + ".txt";
	colorFile2 = getTimeName(COLOR2) + ".txt";
	depthFile2 = getTimeName(DEPTH2) + ".txt";
	
	pScenePerceptionController.reset(new ScenePerceptionController(L"Augmented Reality SP", argc, argvW, L"",
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

	

	if (pScenePerceptionController->InitializeProjection() < PXC_STATUS_NO_ERROR)
	{
		wcout << L"RF_AugmentedRealitySP: Create Projection Failed" << endl;
		return 1;
	}

	if (!InitGlutUI(argc, argvW))
	{
		exitFunc(2);
	}

	if (!AllocateLocalBuffers())
	{
		exitFunc(4);
	}
	colorRenderer.SetImageSize(COLOR_CAPTURE_WIDTH, COLOR_CAPTURE_HEIGHT);

	glutMainLoop();


	return 0;
}
