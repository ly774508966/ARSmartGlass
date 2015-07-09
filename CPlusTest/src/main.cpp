#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/gpu/gpu.hpp>

using namespace std;
using namespace cv;

double getRunningTime(double start)
{
	return ((double)getTickCount()-start)*1000/getTickFrequency();
}

int main()
{
	cout<<"Hello World! From host by using GPU support!"<<endl;

	double startTime;

	Mat img = imread("1.png");

	Mat imgGray, imgEdge;

	//normal cvtColor
	startTime = getTickCount();
	cvtColor(img, imgGray, CV_BGR2GRAY);
	Canny(imgGray,imgEdge, 50, 150, 3);
	cout<<"Normal time cost: "<<getRunningTime(startTime)<<endl;
	imshow("Normal Gray", imgEdge);

    gpu::GpuMat imgGpuSrc, imgGpuGray, imgGpuEdge;
    imgGpuSrc.upload(img);

    //gpu cvtColor
    startTime = getTickCount();
    gpu::cvtColor(imgGpuSrc, imgGpuGray, CV_BGR2GRAY);
    gpu::Canny(imgGpuGray,imgGpuEdge, 50, 150, 3);
    cout<<"GPU time cost: "<<getRunningTime(startTime)<<endl;

    imgGpuEdge.download(imgEdge);
	imshow("GPU Gray", imgEdge);

	waitKey(0);

	return 1;
}
