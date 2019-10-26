// -*- c++ *--
// Copyright 2008 Isis Innovation Limited
//
// VideoSource.h
// Declares the VideoSource class
// 
// This is a very simple class to provide video input; this can be
// replaced with whatever form of video input that is needed.  It
// should open the video input on construction, and provide two
// function calls after construction: Size() must return the video
// format as an ImageRef, and GetAndFillFrameBWandRGB should wait for
// a new frame and then overwrite the passed-as-reference images with
// GreyScale and Colour versions of the new frame.
//
#ifndef __VIDEOSOURCE_H__
#define __VIDEOSOURCE_H__
#pragma once
#include <stdio.h>
#include <mutex>

#include "IOWrapper/OutputWrapper/SampleOutputWrapper.h"

#if defined WIN32 || _WIN32
#define sleep Sleep
#endif
namespace VideoSourceNS {

using namespace std;

class TrackingImage
{
public:
	TrackingImage() {}
	TrackingImage(dso::MinimalImageB* minImage, double timestamp, float exposure) : minImage(minImage), timestamp(timestamp), exposure(exposure) {}
	TrackingImage(int w, int h) { minImage = new dso::MinimalImageB(w, h); timestamp = 0.0; exposure = 0.0f; }
	~TrackingImage() { if(minImage) delete minImage; }

	dso::MinimalImageB* getImage() const { return minImage; }
	double getTimeStamp() const { return timestamp; }
	float getExposure() const { return exposure; }
	int getFrameID() const { return frameID; }

	void setImage(dso::MinimalImageB* val) { minImage = val; }
	void setTimeStamp(double val) { timestamp = val; }
	void setExposure(float val) { exposure = val; }
	void setFrameID(int val) { frameID = val; }

	int w() { return minImage->w; }
	int h() { return minImage->h; }

	TrackingImage* copy()
	{
		TrackingImage* img = new TrackingImage();
		img->setImage(new dso::MinimalImageB(minImage->w, minImage->h));
		memcpy(img->getImage()->data, minImage->data, minImage->w * minImage->h);
		img->setTimeStamp(timestamp);
		img->setExposure(exposure);
		img->setFrameID(frameID);
		return img;
	}

private:
	dso::MinimalImageB* minImage;
	double timestamp;
	float exposure;
	int frameID;
};

class VideoSource
{
 public:
	VideoSource();
	VideoSource(int channel, bool skip_frame);
	virtual ~VideoSource();

	TrackingImage* GetAndFillFrameBW(int& id);
	TrackingImage* getNewestImage() { return image; }
	void close();

	int channel;

	std::map<int, TrackingImage*> images;
	TrackingImage* image;
	unsigned int numCameras;

	int num_captured_images;
	bool skip_frame;
	int numSkipFrame;
	bool stop_capture;

	std::mutex mtx;

	// Rongen testing skip frame notification
	virtual bool getSkipFrame() { return false; };
	virtual void setSkipFrame(bool isSkipFrame) {};
	virtual void setNumSkipFrame(int numFrame) {};
	virtual int getNumSkipFrame() { return 0; };

	// Rongen testing DSO status transmission to engine via VideoSource instead of FullSystem
	virtual int getCameraCableStatus() { return -1; };

	// Rongen testing device temperature
	virtual void setDeviceTemp(float temp) {};
	virtual float getDeviceTemp() { return 0.; };
};

}
#endif
