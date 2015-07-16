#include <iostream>
#include <SDL2/SDL.h>
#include <opencv2/opencv.hpp>

#define WIDTH 640
#define HEIGHT 480
#define BPP 4
#define DEPTH 32

using namespace std;
using namespace cv;

int main(int argc, char* argv[])
{
	SDL_Surface *screen;
	SDL_Event event;
	int keypress = 0;
	int h = 0;

	Mat img = imread("church.jpg");

	if (SDL_Init(SDL_INIT_EVERYTHING) == 1 )
	{
		cout << "error: " << SDL_GetError() << endl;
		return 1;
	}

	SDL_Window* window = SDL_CreateWindow("hello", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, img.cols, img.rows, SDL_WINDOW_SHOWN);
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

	//SDL_Surface* surface = SDL_LoadBMP("wolf.bmp");
	SDL_Surface* surface = SDL_CreateRGBSurfaceFrom((void*)img.data,
			img.size().width, img.size().height,
            8 * img.channels(),
            img.step, 0xff0000, 0x00ff00, 0x0000ff, 0);
	SDL_Texture* texture = SDL_CreateTextureFromSurface(renderer, surface);

	SDL_FreeSurface(surface);
	SDL_RenderClear(renderer);
	SDL_RenderCopy(renderer, texture, 0, 0);
	SDL_RenderPresent(renderer);

	while(!keypress)
	{
		//DrawScreen(screen, h++);
		//imshow("hello", img);
		//waitKey(1);

		while(SDL_PollEvent(&event))
		{
			switch (event.type)
			{
			case SDL_QUIT:
				keypress = 1;
				break;
			case SDL_KEYDOWN:
				keypress = 1;
				break;
			}
		}
	}

	SDL_DestroyWindow(window);
	SDL_DestroyRenderer(renderer);
	SDL_Quit();
	return 0;
}

