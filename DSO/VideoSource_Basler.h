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
#ifndef __VIDEOSOURCE_BASLER_H__
#define __VIDEOSOURCE_BASLER_H__
#include <VideoSource.h>
// Include files to use the PYLON API
#include <pylon/PylonIncludes.h>
// Settings for using Basler GigE Vision cameras.
#include <pylon/gige/BaslerGigEInstantCamera.h>

#include <boost/thread.hpp>
namespace VideoSourceNS {
	namespace BASLER {
		using namespace Pylon;
		using namespace Basler_GigECameraParams;
		using namespace Basler_GigEStreamParams;
		typedef CBaslerGigEInstantCamera Camera_t;
		typedef CBaslerGigEGrabResultPtr GrabResultPtr_t; // Or use Camera_t::GrabResultPtr_t
		#define EVENTTHREAD_WAIT_TIMEOUT    1000
		class VideoSource_Basler : public VideoSource, public CImageEventHandler
		{
		public:
			VideoSource_Basler(string path, bool save_enabled, bool skip_frame, bool hard_trigger, string camSerial);
			~VideoSource_Basler();
			virtual void OnImageGrabbed(CInstantCamera& camera, const CGrabResultPtr& ptrGrabResult);
		private:
			Camera_t* bcamera;
			CPylonImage capImage;
			int image_size;
			void captureThread();
			boost::thread thread_;
#ifndef __linux__
			/* on windows we need an Event handle member */
			HANDLE m_hEvent;
#endif
		};
	}
}
#endif