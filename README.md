# VirtualReality
Using the Intel RealSense Camera to detect 3D Differences from the Depth Feed to a CAD object.

Project Flow
1. Calibrate the Camera
2. Estimating the Pose
3. Get Point Cloud and visualize it
4. Register the model with the Point Cloud
5. Find 3D Differences

The Project is mostly organized in CameraCalib and DeviceSettings class

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

##Dependencies

This project relies on many libraries
- OpenGL/FreeGlut - to display the streams and for 2d point selection
- OpenCV - for computer vision functions 
- (OpenCV-contrib) - may be used later for Keypoint detection
- PCL 1.7.2 - for displaying Point Cloud
- VTK 6.3 - visualization
- (project taketwo/rs) - github project that displays PointCloud uses PCL.

It is being built on an x64 Configuration on Windows only.
