#include "SoftkineticCalibration.h"

softkineticCalibration::softkineticCalibration()
{
	//All based on Softkinetic 325 sensor
	R = Mat::zeros(3, 3, CV_64F);
	R.at<double>(0, 0) = 0.999999;
	R.at<double>(0, 1) = -0.00125877;
	R.at<double>(0, 2) = -0.000802669;
	R.at<double>(1, 0) = -0.00126216;
	R.at<double>(1, 1) = -0.99999;
	R.at<double>(1, 2) = -0.00423948;
	R.at<double>(2, 0) = 0.000797325;
	R.at<double>(2, 1) = -0.00424049;
	R.at<double>(2, 2) = 0.999991;

	invert(R, R_inv);

	T = Mat::zeros(3, 1, CV_64F);
	T.at<double>(0, 0) = 0.026;
	T.at<double>(1, 0) = -0.000507992;
	T.at<double>(2, 0) = -0.000862588;

	K_depth = Mat::zeros(3, 3, CV_64F);
	K_depth.at<double>(0, 0) = 224.502;
	K_depth.at<double>(0, 2) = 160;
	K_depth.at<double>(1, 1) = -230.494;
	K_depth.at<double>(1, 2) = 120;
	K_depth.at<double>(2, 2) = 1;

	K_rgb = Mat::zeros(3, 3, CV_64F);
	K_rgb.at<double>(0, 0) = 587.452/2;
	K_rgb.at<double>(0, 2) = 320/2;
	K_rgb.at<double>(1, 1) = 600.675/2;
	K_rgb.at<double>(1, 2) = 240/2;
	K_rgb.at<double>(2, 2) = 1;

	invert(K_depth, K_depth_inv);
	invert(K_rgb, K_rgb_inv);

	Dist_depth = Mat::zeros(5, 1, CV_64F);
	Dist_depth.at<double>(0, 0) = -0.170103;
	Dist_depth.at<double>(1, 0) = 0.144064;
	Dist_depth.at<double>(2, 0) = 0.0;
	Dist_depth.at<double>(3, 0) = 0.0;
	Dist_depth.at<double>(4, 0) = -0.0476994;

	Dist_rgb = Mat::zeros(5, 1, CV_64F);
	Dist_rgb.at<double>(0, 0) = 0.0225752;
	Dist_rgb.at<double>(1, 0) = -0.162668;
	Dist_rgb.at<double>(2, 0) = 0.0;
	Dist_rgb.at<double>(3, 0) = 0.0;
	Dist_rgb.at<double>(4, 0) = 0.186138;

	P_depth.create(3, 1, CV_64F);
	P_rgb.create(3, 1, CV_64F);
}

softkineticCalibration::~softkineticCalibration()
{
}

Mat softkineticCalibration::mapColorToDepth(const Mat& srcDepthImage, const Mat& srcColorImage)
{
	Mat dstImage(srcDepthImage.rows, srcDepthImage.cols, CV_8UC3);
	Mat p_depth(3, 1, CV_64F);         //pixel coordinate in depth image
	Mat p_rgb(3, 1, CV_64F);           //pixel coordinate in color image
	Mat temp(3, 1, CV_64F);

	Mat src;
	undistort(srcDepthImage, src, K_depth, Dist_depth);

	dstImage.setTo(0);
	for(int row=0; row<src.rows; row++)
	{
		ushort* depthSrc     = src.ptr<ushort>(row);
		Vec3b*  dstImageData = dstImage.ptr<Vec3b>(row);
		for(int col=0; col<src.cols; col++)
		{

#ifdef TRANSFORM_BASED_ON_MATRIX

			p_depth.at<double>(0,0) = col;
			p_depth.at<double>(1,0) = row;
			p_depth.at<double>(2,0) = 1;
			gemm(K_depth_inv, p_depth, 1, NULL, NULL, P_depth, 0);

			if(depthSrc[col] > 1000) continue;
			P_depth.at<double>(2,0) = (double)(depthSrc[col]/1000.0);
			P_depth.at<double>(0,0) = P_depth.at<double>(0,0) * P_depth.at<double>(2,0);
			P_depth.at<double>(1,0) = P_depth.at<double>(1,0) * P_depth.at<double>(2,0);

			gemm(R, P_depth, 1, T, 1, P_rgb, 0);

			ushort depthDst = cvRound(P_rgb.at<double>(2,0)*1000);
			P_rgb.at<double>(0,0) = P_rgb.at<double>(0,0) / P_rgb.at<double>(2,0);
			P_rgb.at<double>(1,0) = P_rgb.at<double>(1,0) / P_rgb.at<double>(2,0);
			P_rgb.at<double>(2,0) = 1;

			gemm(K_rgb, P_rgb, 1, NULL, NULL, p_rgb);


			ushort row_rgb = cvRound(p_rgb.at<double>(1,0));
			ushort col_rgb = cvRound(p_rgb.at<double>(0,0));
			if(row_rgb>=0 && row_rgb<dstImage.rows && col_rgb>=0 && col_rgb<dstImage.cols)
			{
				dstImageData[col][0] = srcColorImage.at<Vec3b>(row_rgb, col_rgb)[0];
				dstImageData[col][1] = srcColorImage.at<Vec3b>(row_rgb, col_rgb)[1];
				dstImageData[col][2] = srcColorImage.at<Vec3b>(row_rgb, col_rgb)[2];
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

	return (dstImage);
}

Mat softkineticCalibration::mapDepthToColor(const Mat& srcDepthImage)
{
	//Mat resizeDepthImage;
	//resize(srcDepthImage, resizeDepthImage, cv::Size(srcDepthImage.cols*2, srcDepthImage.rows*2));

	Mat dstImage(srcDepthImage.rows, srcDepthImage.cols, CV_16U);
	//Mat dstOverlaidPixelImage(srcDepthImage.rows, srcDepthImage.cols, CV_16U); //for smoothing the mapped depth image
	Mat p_depth(3, 1, CV_64F);         //pixel coordinate in depth image
	Mat p_rgb(3, 1, CV_64F);           //pixel coordinate in color image
	Mat temp(3, 1, CV_64F);

	Mat src;
	undistort(srcDepthImage, src, K_depth, Dist_depth);
	//undistort(resizeDepthImage, src, K_depth, Dist_depth);

	dstImage.setTo(MAX_DISTANCE_MM);
	//dstOverlaidPixelImage.setTo(MAX_DISTANCE_MM);
	for(int row=0; row<src.rows; row++)
	{
		ushort* depthSrc = src.ptr<ushort>(row);
		for(int col=0; col<src.cols; col++)
		{

#ifdef TRANSFORM_BASED_ON_MATRIX

			p_depth.at<double>(0,0) = col;
			p_depth.at<double>(1,0) = row;
			p_depth.at<double>(2,0) = 1;
			gemm(K_depth_inv, p_depth, 1, NULL, NULL, P_depth, 0);

			if(depthSrc[col] > 1000) continue;
			P_depth.at<double>(2,0) = (double)(depthSrc[col]/1000.0);
			P_depth.at<double>(0,0) = P_depth.at<double>(0,0) * P_depth.at<double>(2,0);
			P_depth.at<double>(1,0) = P_depth.at<double>(1,0) * P_depth.at<double>(2,0);

			gemm(R, P_depth, 1, T, 1, P_rgb, 0);

			ushort depthDst = cvRound(P_rgb.at<double>(2,0)*1000);
			P_rgb.at<double>(0,0) = P_rgb.at<double>(0,0) / P_rgb.at<double>(2,0);
			P_rgb.at<double>(1,0) = P_rgb.at<double>(1,0) / P_rgb.at<double>(2,0);
			P_rgb.at<double>(2,0) = 1;

			gemm(K_rgb, P_rgb, 1, NULL, NULL, p_rgb);


			ushort row_rgb = cvRound(p_rgb.at<double>(1,0));
			ushort col_rgb = cvRound(p_rgb.at<double>(0,0));
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

	return (dstImage);
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
