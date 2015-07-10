#ifndef _SOFTKINETIC_CALIBRATION_H_
#define _SOFTKINETIC_CALIBRATION_H_

#include <iostream>
#include <stdio.h>
#include <opencv2/opencv.hpp>

#define MAX_DISTANCE_MM       32001
#define TRANSFORM_BASED_ON_MATRIX 1

using namespace std;
using namespace cv;

class softkineticCalibration
{
public:
	softkineticCalibration();
	~softkineticCalibration();

	Mat mapDepthToColor(const Mat& srcDepthImage);
	Mat mapColorToDepth(const Mat& srcDepthImage, const Mat& srcColorImage);

private:
	Mat smoothMappedDepthImage(Mat& srcMappedDepthImage);

private:
	Mat R;               //rotation matrix
	Mat T;               //translation matrix

	Mat R_inv;           ///inverse matrix of R

	Mat K_depth;         //intrinsic parameters of depth camera
	Mat K_rgb;           //intrinsic parameters of color camera
	Mat Dist_depth;      //distortion coefficients of depth camera
	Mat Dist_rgb;        //distortion coefficients of color camera

	Mat K_depth_inv;     //inverse matrix of depth camera
	Mat K_rgb_inv;       //inverse matrix of color camera

	Mat P_depth;         //3D Position of a pixel in the depth camera coordinate system
	Mat P_rgb;           //3D Position of a pixel in the color camera coordinate system
};

#endif
