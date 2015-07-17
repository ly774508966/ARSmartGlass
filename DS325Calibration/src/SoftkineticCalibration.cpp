#include "SoftkineticCalibration.h"

softkineticCalibration::softkineticCalibration()
{
	//Read all parameters based on SoftKenitc DepthSense SDK
	Context context = Context::createStandalone();
	vector<Device> devices = context.getDevices();
	StereoCameraParameters stereo;

	if (devices.size() != 0)
	{
	 	for(int i = 0; i < devices.size(); i++)
	 	{
	 		stereo=devices[i].getStereoCameraParameters();

	 	}
	}

	ExtrinsicParameters extParams = stereo.extrinsics;
	IntrinsicParameters intDepthParams = stereo.depthIntrinsics;
	IntrinsicParameters intColorParams = stereo.colorIntrinsics;

	//Set R
	R = Mat::zeros(3, 3, CV_64F);
	R.at<double>(0, 0) = extParams.r11; R.at<double>(0, 1) = extParams.r12; R.at<double>(0, 2) = extParams.r13;
	R.at<double>(1, 0) = extParams.r21; R.at<double>(1, 1) = extParams.r22; R.at<double>(1, 2) = extParams.r23;
	R.at<double>(2, 0) = extParams.r31; R.at<double>(2, 1) = extParams.r32; R.at<double>(2, 2) = extParams.r33;

	//Set T
	T = Mat::zeros(3, 1, CV_64F);
	T.at<double>(0, 0) = extParams.t1; T.at<double>(1, 0) = extParams.t2; T.at<double>(2, 0) = extParams.t3;

	//Set K_depth
	K_depth = Mat::zeros(3, 3, CV_64F);
	K_depth.at<double>(0, 0) =  intDepthParams.fx; K_depth.at<double>(0, 2) = intDepthParams.cx;
	K_depth.at<double>(1, 1) = -intDepthParams.fy; K_depth.at<double>(1, 2) = intDepthParams.cy;
	K_depth.at<double>(2, 2) = 1;

	//Set K_rgb
	K_rgb = Mat::zeros(3, 3, CV_64F);
	K_rgb.at<double>(0, 0) = intColorParams.fx/2; K_rgb.at<double>(0, 2) = intColorParams.cx/2;
	K_rgb.at<double>(1, 1) = intColorParams.fy/2; K_rgb.at<double>(1, 2) = intColorParams.cy/2;
	K_rgb.at<double>(2, 2) = 1;

	//Set Dist_depth
	Dist_depth = Mat::zeros(5, 1, CV_64F);
	Dist_depth.at<double>(0, 0) = intDepthParams.k1;
	Dist_depth.at<double>(1, 0) = intDepthParams.k2;
	Dist_depth.at<double>(2, 0) = intDepthParams.p1;
	Dist_depth.at<double>(3, 0) = intDepthParams.p2;
	Dist_depth.at<double>(4, 0) = intDepthParams.k3;

	//Set Dist_rgb
	//Dist_rgb = Mat::zeros(5, 1, CV_64F);
	//Dist_rgb.at<double>(0, 0) = intColorParams.k1;
	//Dist_rgb.at<double>(1, 0) = intColorParams.k2;
	//Dist_rgb.at<double>(2, 0) = intColorParams.p1;
	//Dist_rgb.at<double>(3, 0) = intColorParams.p2;
	//Dist_rgb.at<double>(4, 0) = intColorParams.k3;

	invert(K_depth, K_depth_inv);
	//invert(K_rgb, K_rgb_inv);

	//P_depth.create(3, 1, CV_64F);
	//P_rgb.create(3, 1, CV_64F);

	M1 = Mat::zeros(3, 3, CV_64F);
	M2 = Mat::zeros(3, 1, CV_64F);
	Mat temp(3, 3, CV_64F);
	//M1 = k_rgb*R*k_depth_inv
	gemm(K_rgb, R, 1, NULL, NULL, temp, 0);
	gemm(temp, K_depth_inv, 1, NULL, NULL, M1, 0);
	//M2 = k_rgb*t
	gemm(K_rgb, T, 1, NULL, NULL, M2, 0);
}

softkineticCalibration::softkineticCalibration(const string filename)
{
	R = Mat::zeros(3, 3, CV_64F);
	T = Mat::zeros(3, 1, CV_64F);
	K_depth = Mat::zeros(3, 3, CV_64F);
	K_rgb   = Mat::zeros(3, 3, CV_64F);
	Dist_depth = Mat::zeros(5, 1, CV_64F);
	//Dist_rgb   = Mat::zeros(5, 1, CV_64F);

	//All based on Softkinetic 325 sensor
	FileStorage fs;
	fs.open(filename, FileStorage::READ);

	if (!fs.isOpened())
	{
		cerr << "Failed to open DS325 matrix file" << filename << endl;
	}

	fs["R"] >> R;
	fs["T"] >> T;
	fs["Intrinsics of Depth camera"] >> K_depth;
	fs["Intrinsics of Color camera"] >> K_rgb;
	fs["Distortion of Depth camera"] >> Dist_depth;
	//fs["Distortion of Color camera"] >> Dist_rgb;

	fs.release();

	invert(K_depth, K_depth_inv);
	//invert(K_rgb, K_rgb_inv);

	//P_depth.create(3, 1, CV_64F);
	//P_rgb.create(3, 1, CV_64F);

	M1 = Mat::zeros(3, 3, CV_64F);
	M2 = Mat::zeros(3, 1, CV_64F);
	Mat temp(3, 3, CV_64F);
	//M1 = k_rgb*R*k_depth_inv
	gemm(K_rgb, R, 1, NULL, NULL, temp, 0);
	gemm(temp, K_depth_inv, 1, NULL, NULL, M1, 0);
	//M2 = k_rgb*t
	gemm(K_rgb, T, 1, NULL, NULL, M2, 0);
}

softkineticCalibration::~softkineticCalibration()
{
}

Mat softkineticCalibration::mapColorToDepth(const Mat& srcDepthImage, const Mat& srcColorImage)
{
	Mat dstImage(srcDepthImage.rows, srcDepthImage.cols, CV_8UC3);
	Mat p_depth(3, 1, CV_64F);
	Mat p_rgb(3, 1, CV_64F);

	Mat src;
	undistort(srcDepthImage, src, K_depth, Dist_depth);
	//srcDepthImage.copyTo(src);

	dstImage.setTo(0);

	for(int row=0; row<src.rows; row++)
	{
		ushort* depthSrc     = src.ptr<ushort>(row);
		Vec3b*  dstImageData = dstImage.ptr<Vec3b>(row);
		for(int col=0; col<src.cols; col++)
		{

			double depth = (double)(depthSrc[col]/1000.0);
			if(depth > 1.0) continue;

			p_depth.at<double>(0,0) = col * depth;
			p_depth.at<double>(1,0) = row * depth;
			p_depth.at<double>(2,0) = depth;

			gemm(M1, p_depth, 1, M2, 1, p_rgb, 0);

			ushort row_rgb = cvRound(p_rgb.at<double>(1,0)/p_rgb.at<double>(2,0));
			ushort col_rgb = cvRound(p_rgb.at<double>(0,0)/p_rgb.at<double>(2,0));

			if(row_rgb>=0 && row_rgb<dstImage.rows && col_rgb>=0 && col_rgb<dstImage.cols)
			{
				Vec3b tempColor = srcColorImage.at<Vec3b>(row_rgb, col_rgb);
				dstImageData[col][0] = tempColor[0];
				dstImageData[col][1] = tempColor[1];
				dstImageData[col][2] = tempColor[2];
			}
		}
	}

	return dstImage.clone();
}

Mat softkineticCalibration::mapDepthToColor(const Mat& srcDepthImage)
{

	Mat dstImage(srcDepthImage.rows, srcDepthImage.cols, CV_16U);
	Mat p_depth(3, 1, CV_64F);
	Mat p_rgb(3, 1, CV_64F);

	Mat src;
	undistort(srcDepthImage, src, K_depth, Dist_depth);

	dstImage.setTo(MAX_DISTANCE_MM);
	for(int row=0; row<src.rows; row++)
	{
		ushort* depthSrc = src.ptr<ushort>(row);
		for(int col=0; col<src.cols; col++)
		{
			double depth = (double)(depthSrc[col]/1000.0);
			if(depth > 1.0) continue;

#ifdef TRANSFORM_BASED_ON_MATRIX

			p_depth.at<double>(0,0) = col * depth;
			p_depth.at<double>(1,0) = row * depth;
			p_depth.at<double>(2,0) = depth;

			gemm(M1, p_depth, 1, M2, 1, p_rgb, 0);

			ushort row_rgb = cvRound(p_rgb.at<double>(1,0)/p_rgb.at<double>(2,0));
			ushort col_rgb = cvRound(p_rgb.at<double>(0,0)/p_rgb.at<double>(2,0));
			ushort depthDst = cvRound(p_rgb.at<double>(2,0)*1000);

			if(row_rgb>=0 && row_rgb<dstImage.rows && col_rgb>=0 && col_rgb<dstImage.cols)
			{
				dstImage.at<ushort>(row_rgb, col_rgb) = depthDst;
			}

#else
			P_depth.at<double>(0,0) = (col - K_depth.at<double>(0, 2)) * (double)(depthSrc[col]/1000.0) / K_depth.at<double>(0, 0);
			P_depth.at<double>(1,0) = (row - K_depth.at<double>(1, 2)) * (double)(depthSrc[col]/1000.0) / K_depth.at<double>(1, 1);
			P_depth.at<double>(2,0) = (double)(depthSrc[col]/1000.0);

			temp.at<double>(0,0)  = R.at<double>(0,0)*P_depth.at<double>(0,0) + R.at<double>(0,1)*P_depth.at<double>(1,0) + R.at<double>(0,2)*P_depth.at<double>(2,0);
			temp.at<double>(1,0)  = R.at<double>(1,0)*P_depth.at<double>(0,0) + R.at<double>(1,1)*P_depth.at<double>(1,0) + R.at<double>(1,2)*P_depth.at<double>(2,0);
			temp.at<double>(2,0)  = R.at<double>(2,0)*P_depth.at<double>(0,0) + R.at<double>(2,1)*P_depth.at<double>(1,0) + R.at<double>(2,2)*P_depth.at<double>(2,0);
			P_rgb.at<double>(0,0) = temp.at<double>(0,0) + T.at<double>(0,0);
			P_rgb.at<double>(1,0) = temp.at<double>(1,0) + T.at<double>(1,0);
			P_rgb.at<double>(2,0) = temp.at<double>(2,0) + T.at<double>(2,0);

			p_rgb.at<double>(0,0) = (P_rgb.at<double>(0,0) * K_rgb.at<double>(0, 0) / P_rgb.at<double>(2,0)) + K_rgb.at<double>(0, 2);
			p_rgb.at<double>(1,0) = (P_rgb.at<double>(1,0) * K_rgb.at<double>(1, 1) / P_rgb.at<double>(2,0)) + K_rgb.at<double>(1, 2);

			ushort row_rgb = cvRound(p_rgb.at<double>(1,0));
			ushort col_rgb = cvRound(p_rgb.at<double>(0,0));
			if(row_rgb>=0 && row_rgb<dstImage.rows && col_rgb>=0 && col_rgb<dstImage.cols)
			{
				dstImage.at<ushort>(row_rgb, col_rgb) = cvRound(P_rgb.at<double>(2,0)*1000);
			}
#endif
		}
	}

	return smoothMappedDepthImage(dstImage);
}

Mat softkineticCalibration::smoothMappedDepthImage(Mat& srcMappedDepthImage)
{
	for(int row=0; row<srcMappedDepthImage.rows; row++)
	{
		ushort* depthSrc = srcMappedDepthImage.ptr<ushort>(row);
		for(int col=0; col<srcMappedDepthImage.cols; col++)
		{
			if(depthSrc[col] == MAX_DISTANCE_MM && row>0 && row<srcMappedDepthImage.rows-1 && col>0 && col<srcMappedDepthImage.cols-1)
			{
				int neighbor_with_depth_num = 0;
				int distanc_sum = 0;

				if(srcMappedDepthImage.at<ushort>(row-1, col-1) != MAX_DISTANCE_MM) {neighbor_with_depth_num++; distanc_sum += srcMappedDepthImage.at<ushort>(row-1, col-1);}
				if(srcMappedDepthImage.at<ushort>(row-1, col  ) != MAX_DISTANCE_MM) {neighbor_with_depth_num++; distanc_sum += srcMappedDepthImage.at<ushort>(row-1, col  );}
				if(srcMappedDepthImage.at<ushort>(row-1, col+1) != MAX_DISTANCE_MM) {neighbor_with_depth_num++; distanc_sum += srcMappedDepthImage.at<ushort>(row-1, col+1);}
				if(srcMappedDepthImage.at<ushort>(row  , col-1) != MAX_DISTANCE_MM) {neighbor_with_depth_num++; distanc_sum += srcMappedDepthImage.at<ushort>(row  , col-1);}
				if(srcMappedDepthImage.at<ushort>(row  , col+1) != MAX_DISTANCE_MM) {neighbor_with_depth_num++; distanc_sum += srcMappedDepthImage.at<ushort>(row  , col+1);}
				if(srcMappedDepthImage.at<ushort>(row+1, col-1) != MAX_DISTANCE_MM) {neighbor_with_depth_num++; distanc_sum += srcMappedDepthImage.at<ushort>(row+1, col-1);}
				if(srcMappedDepthImage.at<ushort>(row+1, col  ) != MAX_DISTANCE_MM) {neighbor_with_depth_num++; distanc_sum += srcMappedDepthImage.at<ushort>(row+1, col  );}
				if(srcMappedDepthImage.at<ushort>(row+1, col+1) != MAX_DISTANCE_MM) {neighbor_with_depth_num++; distanc_sum += srcMappedDepthImage.at<ushort>(row+1, col+1);}

				if(neighbor_with_depth_num >= 5)
					depthSrc[col] = distanc_sum / neighbor_with_depth_num;
			}
		}
	}

	return srcMappedDepthImage;
}
