#include <iostream>
#include <vector>
#include <openni/OpenNI.h>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/nonfree/gpu.hpp>
#include <opencv2/gpu/gpu.hpp>

#include "SoftkineticCalibration.h"

using namespace std;

cv::Mat getColorImage(openni::VideoFrameRef& color_frame)
{
  if(!color_frame.isValid())
  {
    return cv::Mat();
  }
  openni::VideoMode video_mode = color_frame.getVideoMode();
  cv::Mat color_img = cv::Mat(video_mode.getResolutionY(),
                              video_mode.getResolutionX(),
                              CV_8UC3, (char*)color_frame.getData());
  cv::Mat ret_img;
  cv::cvtColor(color_img, ret_img, CV_RGB2BGR);
  return ret_img;
}


// CV_16U
cv::Mat getDepthImage(openni::VideoFrameRef& depth_frame)
{
  if(!depth_frame.isValid())
  {
    return cv::Mat();
  }

  openni::VideoMode video_mode = depth_frame.getVideoMode();
  cv::Mat depth_img = cv::Mat(video_mode.getResolutionY(),
                              video_mode.getResolutionX(),
                              CV_16U, (char*)depth_frame.getData());

  return depth_img.clone();
}


cv::Mat getDepthDrawableImage(cv::Mat depth_image)
{

  cv::Mat drawable;
  depth_image.convertTo(drawable, CV_8UC1, 255.0/1000);
  drawable = 255 - drawable;

  return drawable;
}

int main()
{
  softkineticCalibration DS325Cali;

  openni::Device device;
  openni::VideoStream color_stream, depth_stream;
  const char* deviceURI = openni::ANY_DEVICE;

  openni::Status rc = openni::STATUS_OK;

  rc = openni::OpenNI::initialize();

  //cout << "After initialization:" << openni::OpenNI::getExtendedError();

  rc = device.open(deviceURI);
  if (rc != openni::STATUS_OK)
  {
    cout << "SimpleViewer: Device open failed:" << openni::OpenNI::getExtendedError();
    openni::OpenNI::shutdown();
    return 1;
  }

  rc = depth_stream.create(device, openni::SENSOR_DEPTH);
  if (rc == openni::STATUS_OK)
  {
    rc = depth_stream.start();
    if (rc != openni::STATUS_OK)
    {
      cerr << "SimpleViewer: Couldn't start depth stream:" << openni::OpenNI::getExtendedError();
      depth_stream.destroy();
    }
  }
  else
  {
    cerr << "SimpleViewer: Couldn't find depth stream:" << openni::OpenNI::getExtendedError();
  }

  rc = color_stream.create(device, openni::SENSOR_COLOR);
  if (rc == openni::STATUS_OK)
  {
    rc = color_stream.start();
    if (rc != openni::STATUS_OK)
    {
      cerr << "SimpleViewer: Couldn't find color stream:" << openni::OpenNI::getExtendedError();
      color_stream.destroy();
    }
  }
  else
  {
    cerr << "SimpleViewer: Couldn't find color stream:" << openni::OpenNI::getExtendedError();
  }


  if (!depth_stream.isValid() || !color_stream.isValid())
  {
    cerr << "SimpleViewer: No valid streams. Exiting" << std::endl;
    openni::OpenNI::shutdown();
    return 2;
  }


  std::vector<openni::VideoStream*> streams;
  streams.push_back(&color_stream);
  streams.push_back(&depth_stream);

  openni::VideoFrameRef color_frame;
  openni::VideoFrameRef depth_frame;

  device.setImageRegistrationMode(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR);

  cv::Mat color_img, depth_img, depth_img_show;
  cv::Mat color_to_depth;

  time_t start, end;
  char fps[50];
  int counter = 0;

  while(true)
  {
	if (counter == 0){
	    time(&start);
	}

    if(color_stream.readFrame(&color_frame) == openni::STATUS_OK)
    if(color_frame.isValid())
    {
      color_img = getColorImage(color_frame);
    }

    if(depth_stream.readFrame(&depth_frame) == openni::STATUS_OK)
    if(depth_frame.isValid())
    {
      depth_img = getDepthImage(depth_frame);
      depth_img_show = getDepthDrawableImage(depth_img);
    }

    color_to_depth = DS325Cali.mapColorToDepth(depth_img, color_img);
    cv::imshow("color to depth", color_to_depth);

    // fps
    time(&end);
    counter++;
    sprintf(fps, "Fps: %.2f", (double)counter/difftime(end, start));
    if (counter == (INT_MAX - 1000))
        counter = 0;

    putText(color_img, fps , cvPoint(30,30), FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(0,0,250), 1);

    cv::imshow("depth", depth_img_show);
    cv::imshow("color", color_img);

    int key = cv::waitKey(1);
    if( key == 'q' || key == 27)
    {
      break;
    }
  }
  return 0;
}
