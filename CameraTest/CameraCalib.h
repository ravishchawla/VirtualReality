#pragma once
#define NOMINMAX
#include <windows.h>
#define _USE_MATH_DEFINES
#include "math.h"
#include <algorithm>
#include <conio.h>
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
#include <pxcdefs.h>

/*Image size, both depth and color are on 320 x 240 because thats the maximum shared res by 
both cameras, although they should both be changed to larger resolutions for pc registration*/
const int IMG_WIDTH = 320;
const int IMG_HEIGHT = 240;
const int COLOR_CAPTURE_WIDTH = 320;
const int COLOR_CAPTURE_HEIGHT = 240;
const float COLOR_FRAMERATE = 60.0f;
const float DEPTH_FRAMERATE = 60.0f;
// Default values, matrices and vectors 
PXCScenePerception::VoxelResolution voxelResolution = PXCScenePerception::VoxelResolution::LOW_RESOLUTION;

/*What feed is currently being shown.
The numerics after COLOR and DEPTH are for calibration when 2 images from different
perspectives are needed*/
enum Feed
{
	COLOR1,
	DEPTH1,
	COLOR2,
	DEPTH2,
	NIL
};


/*Files for saving data, there are too many here, could definately be consilidated into 1 file only*/
Feed feedType;
std::fstream currentFile;
std::fstream calibFile;
std::string depthFile1;
std::string colorFile1;
std::string depthFile2;
std::string colorFile2;
std::string calibFileStr;

int mainWnd = 0;
int windowRect[2] = { 2 * IMG_WIDTH, 2 * IMG_HEIGHT };

float curPose[12] = { 0 };
PXCScenePerception::TrackingAccuracy trackingStatus = PXCScenePerception::TrackingAccuracy::FAILED;
ColorRenderer colorRenderer(0, 0, 640, 480);

/*OpenGL buffers*/
std::unique_ptr<ScenePerceptionController> pScenePerceptionController;
// Images captured and images displayed
std::unique_ptr<unsigned char[]> camBgraImage(nullptr); // the captured BGRA image 
std::unique_ptr<unsigned short[]> camDepImage(nullptr); // the captured depth image
std::unique_ptr<float[]> verticesImage(nullptr); // Vertices image 
std::unique_ptr<float[]> depthFromVerticesImage(nullptr); // vertices image converted to depth image 

/*Points drawn during calibration*/
std::vector<std::vector<cv::Point2f>> camera1Points;
std::vector<std::vector<cv::Point2f>> camera2Points;
std::vector<std::vector<cv::Point3f>> objectPoints;

/*Points drawn at any other time, including pose estimation*/
std::vector<PXCPointF32> points;

/*which of the above lists is currently being used. unclean.*/
std::vector<cv::Point2f> *currentList;

/*Intel Session variables*/
float imageQuality = 0.0f;
PXCCapture::Sample sample;
PXCSenseManager *session;
PXCCapture::Device *device;

/*Current frame*/
PXCImage *depthImage;
PXCImage *colorImage;

static GLuint sdl = -1;

class CameraCalib {
public:
	static void CameraCalib::exitFunc(int returnCode);
	static void CameraCalib::Reset();

	/*Takes the current frame and saves it to file*/
	static void CameraCalib::snapshot();

	/*For camera calibration, reads 3d points from a file to store them in a list*/
	static void CameraCalib::read3dPoints();
	static void CameraCalib::read3dPoints(std::string filename, std::vector<cv::Point3f> *list);
	
	/*Takes the current drawn pointson on screen and converts them to real world points to estimate pose*/
	static void CameraCalib::translatePoints(Feed type);
	
	/*Switches between depth and camera modes and images*/
	static void CameraCalib::switchMode(Feed type, boolean writeToFile);

	/*OpenGL initialialization functions. Sets local variables, clears them based on current window size*/
	static void CameraCalib::StartScenePerception();
	static bool CameraCalib::InitGlutUI(int argc, WCHAR* argvW[]);
	static bool CameraCalib::AllocateLocalBuffers();

	/*OpenGL Function handlers*/
	static void CameraCalib::clbDisplay(void);

	//Draws points on the GL screen
	static void CameraCalib::clbMouse(int clickedbutton, int clickState, int x, int y);

	//Handles key check, used for switching between different modes.
	static void CameraCalib::clbKeys(unsigned char key, int x, int y);
	static void CameraCalib::clbReshape(int width, int height);
	static void CameraCalib::clbExitFunc();

	//returns a string based on the current time as filename.
	static std::string CameraCalib::getTimeName(Feed feed);

};