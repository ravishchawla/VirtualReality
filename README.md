# VirtualReality
Using the Intel RealSense Camera to detect 3D Differences from the Depth Feed to a CAD object.

Project Flow
1. Calibrate the Camera
2. Estimating the Pose
3. Get Point Cloud and visualize it
4. Register the model with the Point Cloud
5. Find 3D Differences

The Project is mostly organized in the following classes:

The following classes are used for initial camera calibration and device setup

CameraCalib:
  - Uses OpenGL to visualize the Depth and RGB streams
  - Uses a flow-based system to calibrate the cameras with objects in different poses.
  - Uses Point Selection from OpenGL to get 2D points and Intel's RSSDK for certain functions - including conversion from 2D to 3D
  - Contains main()

DeviceSettings
  - Originally intended for updating settings in the Device Firmware for calibration, but also contains methods for pose estimation
  - Uses OpenCV to calibrate the camera from different set of image and world points
  - Uses OpenCV to etimate pose given 2d points and calculated 3d points.
  - Unrelated methodds will be refractored to a different class later.

The following classes are used for Point Cloud processing

Mesh
  - Uses PCLViewer to visualize a Point Cloud, captured from the live streams of the camera
  - Point Clouds are grabbed from a blackbox function, which combines the Depth and Color streams to create a Point Cloud
  - Uses different PCL Library functions to process point clouds, such as calculating Interst Key points, applying transformations, and calculating 3D Differences.
  - Uses PCL Library to save captured Point Clouds to file.

ViewPoint
  - Uses PCLViewer to visualize Point Clouds (and including viewing overlayed clouds), loaded from a file containing point coordinates
  - Uses the same generic type from Mesh class
  - Use PCL Library to load Point clouds from disk into Point Cloud objects.
  - Implements different approaches to fitting Point Clouds, including Plane fitting and K-means.

##Dependencies

This project relies on many libraries
- R200 Real Sense SDK by Intel(R)
- OpenGL/FreeGlut - to display the streams and for 2d point selection
- OpenCV - for computer vision functions 
- OpenCV-contrib - used later for Keypoint detection
- PCL 1.7.2 - for displaying Point Cloud
- VTK 6.3 - visualization
- (project taketwo/rs) - github project that displays PointCloud uses PCL.

##Project Configuration and Installation

Because this project relises on different Computer Vision libraries, they have to be installed first before the project can be built. Many of the libraries have to be compiled and built from source, including PCL and OpenCV.

The project is built on an x64 Configuration on Windows only. 

Installation steps:
1. Visual Studios
  -The project is built on Visual Studios 2015, which can be obtained from Microsoft. A version that supports C++ compiling is needed.
2. Cmake
  - CMake (GUI recommended) is needed to compile different libraries.
  - For CMAKE, the following properties were used:
    - Visual Studio 14 2015 Win64
    - Use Default native compilers
    - Run cmake using Admin properties
3. Real Sense SDK 
  - The Real Sense SDK can be obtained at: https://software.intel.com/en-us/intel-realsense-sdk
4. OpenGL/FreeGLUT
  - The library can be obtained here: http://freeglut.sourceforge.net/index.php#download
  - use the latest stable release
5. OpenCV
  - OpenCV has to be compiled and built from source, because the latest release is not availaible as binaries
  - The source can be obtained at: https://github.com/Itseez/opencv and compiled with CMAKE
  - Open the compiled project file with Visual Studio and build the libraries.
6. OpenCV-contrib
  - OpenCV-contrib packages are also needed, however, they are already included in the project as source files, so they do not need to be downloaded.
7. PCL 1.7.2
  - PCL also has to be compiled from source, because the version 1.7.2 is not availaible as binaries
  - The source can be downloaded from: https://github.com/PointCloudLibrary/pcl 
  - PCL also has many dependencies, some of which can be installed directly using installers:
    - BOOST - installable from http://www.boost.org/
    - Eigen - need to be compiled from source because latest version is needed
    - FLANN - need to be compiled from source because latest version is needed
    - VTK - installable from http://www.vtk.org/
    - QHull - installable from http://www.qhull.org/
    - OpenNI and OpenNI2 - both versions are needed by PCL - need to be comipled from source
8. (Project taketwo/rs)
  - This project is used for its Point Cloud grabber, but it has already been compiled and the project files are included.
  - If a more updated version of the repository than the one used (last downloaded Nov. 2015) is needed, it will have to be re-compiled, and rebuilt with Visual Studios. 
  - The project requires all above libraries, and CMAKE will check for the corresponding paths.
  
Build instructions:

After completing installation of all dependencies, the project can be opened using Visual Studios. Open the solution CameraTest.sln (located in the root folder). By default, the solution should have included projects from the rs repository (real_sense, real_sense_viewer, ZERO_CHECK, and ALL_BUILD). If these do not show on the project explorer, add the corresponding solutions to the  project (these are located in the folder rs/build).

To build, use x64 build configuration, with either Debug or Release. 
