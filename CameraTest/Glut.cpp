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
	COLOR,
	DEPTH
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
ofstream file;

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

float imageQuality = 0.0f;
PXCCapture::Sample sample;

static GLuint sdl = -1;
void writeHeaderToFile();

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
	if (file.is_open())
		file.flush();
		file.close();

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
	writeHeaderToFile();
}

void writeHeaderToFile() {
	if (!file.is_open()) {
		return;
	}
	
	if (feedType == COLOR) {
		file << "COLOR" << endl;
	}
	else if (feedType == DEPTH) {
		file << "DEPTH" << endl;
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
		cout << "Change mode to Depth\n";
		feedType = DEPTH;
		Reset();
		break;
	case 'c': //switch to color mode
		cout << "Change mode to Color\n";
		feedType = COLOR;
		Reset();
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
		if (feedType == DEPTH) {
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
		if (file.is_open()) {
			file << x << "," << y << endl;
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
	time_t t = time(0);
	struct tm *now = localtime(&t);
	char filename[80];
	strftime(filename, 80, "%Y-%m-%d-%H-%M-%S.txt", now);
	file.open(filename, fstream::out);
	if (!file.is_open()) {
		cout << "Log file not created";
	}
	else {
		cout << "log file created? " << filename;
	}

	writeHeaderToFile();

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
