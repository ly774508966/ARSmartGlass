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
#include <SDL2/SDL.h>

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
  //SDL2
  SDL_Event event;
  if (SDL_Init(SDL_INIT_EVERYTHING) == 1 )
  {
	 cout << "error: " << SDL_GetError() << endl;
	 return 1;
  }

  SDL_Window* window = SDL_CreateWindow("hello", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, 320, 240, SDL_WINDOW_SHOWN);
  if (window == NULL)
  {
	 cout << "Error: " << SDL_GetError() << endl;
	 return 1;
  }

  SDL_Renderer* renderer = SDL_CreateRenderer(window, 1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
  if (renderer == NULL)
  {
	 cout << "Error: " << SDL_GetError() << endl;
	 return 1;
  }

  //color to depth map
  softkineticCalibration DS325Cali;

  //openni
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
      //depth_img_show = getDepthDrawableImage(depth_img);
    }

    color_to_depth = DS325Cali.mapColorToDepth(depth_img, color_img);

    // fps
    time(&end);
    counter++;
    sprintf(fps, "Fps: %.2f", (double)counter/difftime(end, start));
    if (counter == (INT_MAX - 1000))
        counter = 0;

    putText(color_to_depth, fps , cvPoint(30,30), FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(0,0,250), 1);

    SDL_Surface* surface = SDL_CreateRGBSurfaceFrom((void*)color_to_depth.data,
    		color_to_depth.size().width, color_to_depth.size().height,
            8 * color_to_depth.channels(),
            color_to_depth.step, 0xff0000, 0x00ff00, 0x0000ff, 0);
    SDL_Texture* texture = SDL_CreateTextureFromSurface(renderer, surface);

    SDL_FreeSurface(surface);
    SDL_RenderClear(renderer);
    SDL_RenderCopy(renderer, texture, 0, 0);
    SDL_RenderPresent(renderer);

    //cv::imshow("color to depth", color_to_depth);
    //cv::imshow("depth", depth_img_show);
    //cv::imshow("color", color_img);

    /*
    if(!color_to_depth.empty() && !depth_img_show.empty())
    {
       cv::cvtColor(depth_img_show, depth_img_show, cv::COLOR_GRAY2BGR);
       cv::Mat debug_img = color_to_depth * 0.5 + depth_img_show * 0.5;
       cv::imshow("blend", debug_img);
    }
    */

    int key = 0;
    while(SDL_PollEvent(&event))
    {
    	switch (event.type)
    	{
    		case SDL_QUIT:
    		{
    			key = 1;
    			break;
    		}

    		case SDL_KEYDOWN:
            {
                    switch (event.key.keysym.sym)
                    {
                       case SDLK_ESCAPE:
                          key = 1;
                          break;
                    }
                    break;
            }
    	}
    }

    if(key == 1)
    	break;
  }

  SDL_DestroyWindow(window);
  SDL_DestroyRenderer(renderer);
  SDL_Quit();

  return 0;
}
