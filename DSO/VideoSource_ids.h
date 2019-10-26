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
#ifndef __VIDEOSOURCE_IDS_H__
#define __VIDEOSOURCE_IDS_H__

#include <VideoSource.h>
#include "ids/uEye.h"
#include <boost/thread.hpp>
namespace VideoSourceNS {
	namespace uEye {
		struct uEyeException : public std::runtime_error
		{
			int error_code;
			uEyeException(int code, const char* msg) :
				std::runtime_error(msg), error_code(code)
			{
			}
		};
		

		class VideoSource_IDS : public VideoSource
		{
		public:
			VideoSource_IDS(string path, bool save_enabled, bool skip_frame, bool hard_trigger);
			virtual ~VideoSource_IDS();

			// Initialization functions in order they should be called.
			INT InitCamera(HIDS *hCam, HWND hWnd);
			void GetMaxImageSize(INT *pnSizeX, INT *pnSizeY);

			typedef boost::function<void(const char *, const void*)> CamCaptureCB;
			void startVideoCapture(CamCaptureCB);
			void stopVideoCapture();
			void closeCamera();

			// Aug 1, 2017 Rongen, for monitoring switch status 
			bool m_lastSwitchStatus;
		private:
			//ids camera variables
			HIDS	m_hCam;			// handle to camera
			char*	m_pcImageMemory;// grabber memory - pointer to buffer
			INT		m_lMemoryId;	// grabber memory - buffer ID
			SENSORINFO m_sInfo;	    // sensor information struct
			INT		m_nSizeX;		// width of video 
			INT		m_nSizeY;		// height of video
			INT		m_imageSize;
			INT		m_nColorMode;	// Y8/RGB16/RGB24/REG32
			INT		m_nBitsPerPixel;// number of bits needed store one pixel
			void captureThread(CamCaptureCB callback);
			boost::thread thread_;
#ifndef __linux__
			/* on windows we need an Event handle member */
			HANDLE m_hEvent;
#endif
			////////////
			inline void checkError(INT err) const {
				INT err2 = IS_SUCCESS;
				IS_CHAR* msg;
				if (err != IS_SUCCESS) {
					if (m_hCam != 0) {
						is_GetError(m_hCam, &err2, &msg);
						if (err2 != IS_SUCCESS) {
							throw uEyeException(err, msg);
						}
					}
					else {
						throw uEyeException(err, "Camera failed to initialize");
					}
				}
			}
			void initIDSParameter(bool hard_trigger);

			//callback
			void DataHandler(const char *img_data, const void* pCallbackData);

			// Rongen testing
			double currentTime = 0., previousTime = 0., timeDiff = 0.;
			int currID = 0, prevID = 0;
		};
	}
}
#endif