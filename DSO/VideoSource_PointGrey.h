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
#ifndef __VIDEOSOURCE_PGR_H__
#define __VIDEOSOURCE_PGR_H__
#include <VideoSource.h>
#include <pgr/FlyCapture2.h>
namespace VideoSourceNS {
	namespace PGR {
		using namespace FlyCapture2;
		static const unsigned int pgr_outVoltageEnable = 0x19D0;
		class VideoSource_PGR : public VideoSource
		{
		public:
			VideoSource_PGR(string path, bool save_enabled, bool skip_frame, bool hard_trigger, string camSerial);
			virtual ~VideoSource_PGR();

			Camera cam;
			Property pp;
			Error error;
		};
	}
}
#endif