// Copyright 2010 Isis Innovation Limited
// This video source uses Point Grey's FlyCapture 2 API
// It is based on their example code
// Parameters are hard-coded here - adapt to your specific camera.

#include <fstream>
#include <iomanip>

#include "VideoSource.h"
#include <opencv2/highgui/highgui.hpp>
#include "util/settings.h"

#ifdef WIN32
#include <direct.h>
#include <io.h>
#else
#include <time.h>
#include <stdio.h>
#include <pwd.h>
#define _sleep sleep
#endif
namespace VideoSourceNS
{
	using namespace std;

	VideoSource::VideoSource() {

	}

	VideoSource::VideoSource(int channel, bool skip_frame) : 
		channel(channel),
		skip_frame(skip_frame)
	{
		image = NULL;
		numCameras = 0;

		num_captured_images = 0;
		stop_capture = false;
	}

	VideoSource::~VideoSource()
	{
		if (image) delete image;

		mtx.lock();
		for (std::map<int, TrackingImage*>::iterator it = images.begin(); it != images.end(); it++)
			delete it->second->getImage();
		mtx.unlock();
	}

	TrackingImage* VideoSource::GetAndFillFrameBW(int& id)
	{
		mtx.lock();
		TrackingImage* img = NULL;
		if (!skip_frame)
		{
			std::map<int, TrackingImage*>::iterator it = images.begin();
			if (it != images.end())
			{
				id = it->first;
				img = images[id];
				images.erase(it);
			}
		}
		else
		{
			id = num_captured_images - 1;
			img = image;
		}
		mtx.unlock();

		return img;
	}

	void VideoSource::close()
	{
		stop_capture = true;
	}
}

