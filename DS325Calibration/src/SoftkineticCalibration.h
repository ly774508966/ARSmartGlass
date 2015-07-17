/*********************************************************
/Align the depth and color images for softKinect using OpenCV
/
/                 [x_rgb]           [X_camera_rgb]
/  Z_camera_rgb * |y_rgb| = k_rgb * |Y_camera_rgb|            -- (1)
/                 [  1  ]           [Z_camera_rgb]
/
/                   [x_depth]             [X_camera_depth]
/  Z_camera_depth * |y_depth| = k_depth * |Y_camera_depth|    -- (2)
/                   [   1   ]             [Z_camera_depth]
/
/  [X_camera_rgb]       [X_camera_depth]
/  |Y_camera_rgb| = R * |Y_camera_depth|  + T                 -- (3)
/  [Z_camera_rgb]       [Z_camera_depth]
/
/  From above (1), (2), (3), we can get:
/
/                 [x_rgb]                                              [x_depth]
/  Z_camera_rgb * |y_rgb| = k_rgb * R * K_depth_inv * Z_camera_depth * |y_depth| + k_rgb * T
/                 [  1  ]                                              [   1   ]
/
/  then make M1 = k_rgb * R * k_depth_inv
/            M2 = k_rgb * T
/*********************************************************/

#ifndef _SOFTKINETIC_CALIBRATION_H_
#define _SOFTKINETIC_CALIBRATION_H_

#include <iostream>
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <DepthSense.hxx>
#include <Eigen/Eigen>

#define MAX_DISTANCE_MM       32002

using namespace std;
using namespace cv;
using namespace DepthSense;
using namespace Eigen;

class softkineticCalibration
{
public:
	softkineticCalibration();                        //Read DS325 parameters by using DepthSense Driver
	softkineticCalibration(const string filename);   //Read DS325 parameters from a .yml file
	~softkineticCalibration();

	Mat mapDepthToColor(const Mat& srcDepthImage);                            //map the depth image to the color image
	Mat mapColorToDepth(const Mat& srcDepthImage, const Mat& srcColorImage);  //map the color image to the depth image

private:
	Mat smoothMappedDepthImage(Mat& srcMappedDepthImage);

private:
	Mat R;               //rotation matrix
	Mat T;               //translation matrix

	Mat K_depth;         //intrinsic parameters of depth camera
	Mat K_rgb;           //intrinsic parameters of color camera
	Mat Dist_depth;      //distortion coefficients of depth camera
	//Mat Dist_rgb;      //distortion coefficients of color camera

	Mat K_depth_inv;     //inverse matrix of depth camera
	//Mat K_rgb_inv;     //inverse matrix of color camera

	//Mat P_depth;       //3D Position of a pixel in the depth camera coordinate system
	//Mat P_rgb;         //3D Position of a pixel in the color camera coordinate system

	Mat M1;              //k_rgb*R*k_depth_inv
	Mat M2;              //k_rgb*T
    double M2_array[3];
};

#endif
