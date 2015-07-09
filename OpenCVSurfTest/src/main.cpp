#include <iostream>
#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/nonfree/gpu.hpp>
#include <opencv2/gpu/gpu.hpp>

using namespace std;
using namespace cv;

double getRunningTime(double start)
{
	return ((double)getTickCount()-start)*1000/getTickFrequency();
}

int main()
{
	 Mat inImg;
	 vector<cv::KeyPoint> src_keypoints;
	 vector<float> src_descriptors;

	 vector<cv::KeyPoint> keypoints;
	 Mat descriptors;

	 gpu::GpuMat inImg_g;
	 gpu::GpuMat src_keypoints_gpu, src_descriptors_gpu;

	 //image load
	 inImg = imread("church.jpg",0);
//CPU SURF
	 SURF FeatureFinder(3000);
	 double start = getTickCount();
	 FeatureFinder(inImg, Mat(), keypoints, descriptors, false);
     double cost = getRunningTime(start);

     Mat img_keypoint;
     drawKeypoints( inImg, keypoints, img_keypoint, Scalar::all(-1), DrawMatchesFlags::DEFAULT );

     imshow("CPU SurfShow", img_keypoint);
     cout<<"CPU Processing time = "<<cost<<" (ms)"<<endl;
     cout<<"CPU Features: "<<keypoints.size()<<endl;

//GPU
	 //FeatureFinder
	 gpu::SURF_GPU FeatureFinder_gpu(3000);
	 inImg_g.upload(inImg);

	 //processing time measure
	 start = getTickCount();
	 FeatureFinder_gpu(inImg_g, gpu::GpuMat(), src_keypoints_gpu, src_descriptors_gpu, false);
	 cost = getRunningTime(start);

	 //descriptor down
	 FeatureFinder_gpu.downloadKeypoints(src_keypoints_gpu, src_keypoints);
	 FeatureFinder_gpu.downloadDescriptors(src_descriptors_gpu, src_descriptors);

	 //Features Draw
	 drawKeypoints(inImg, src_keypoints, inImg, Scalar::all(-1), DrawMatchesFlags::DEFAULT );

	 imshow("GPU Show", inImg);

	 cout<<"GPU Processing time = "<<cost<<" (ms)"<<endl;
	 cout<<"GPU Features: "<<src_keypoints.size()<<endl;

	  waitKey(0);
	  return 1;
}
