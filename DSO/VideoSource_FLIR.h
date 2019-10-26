#ifndef __VIDEOSOURCE_FLIR_H__
#define __VIDEOSOURCE_FLIR_H__

#include <VideoSource.h>
#include "Spinnaker.h"
#include "SpinGenApi/SpinnakerGenApi.h"

namespace VideoSourceNS {

	namespace FLIR {
		using namespace Spinnaker;
		using namespace Spinnaker::GenApi;
		using namespace Spinnaker::GenICam;

		#define EXP_LOWER_LIMIT 11
		#define MAX_MANUALEXPOSURE 20000 //20ms
		//#define MAX_MANUALEXPOSURE 6000 //6ms for 50fps operation
		#define EXP_LOWER_LIMIT 11
		#define EXP_UPPER_LIMIT MAX_MANUALEXPOSURE
		/*#define CAMERA_FPS 25*/
		
		class VideoSource_FLIR;
		class ImageEventHandler : public ImageEvent
		{
		public:

			// The constructor retrieves the serial number and initializes the image 
			// counter to 0.
			ImageEventHandler(Spinnaker::CameraPtr pCam, VideoSource_FLIR* vs);
			~ImageEventHandler() { /*delete cam;  delete videoFlir;*/ }

			// This method defines an image event. In it, the image that triggered the 
			// event is converted and saved before incrementing the count. Please see 
			// Acquisition_CSharp example for more in-depth comments on the acquisition 
			// of images.
			void OnImageEvent(Spinnaker::ImagePtr image);		
		private:
			string m_deviceSerialNumber;
			VideoSource_FLIR *videoFlir;
			Spinnaker::Camera* cam;
			double currentTime = 0., previousTime = 0., timeDiff = 0.;
			int frameDiff;
			float exposure = 0.;
			int currFrameID = 0, prevFrameID = 0, currCamFrameID = 0, prevCamFrameID = 0;
			int skipFrameCount = 0;
			int waitCounter = 0;
			int lastFrameOffset = 0;
			bool frameIdReset = true;
			double startTime = 0.;
			int startCamFrameID = 0;
			bool previousPause = false; // true = pausing
		};

		// -------------- Start of InterfaceEventHandler
		class InterfaceEventHandler : public InterfaceEvent
		{
		public:

			InterfaceEventHandler(InterfacePtr iface, unsigned int interfaceNum, SystemPtr system, VideoSource_FLIR* vs, bool trigger) : m_interface(iface), m_interfaceNum(interfaceNum), m_system(system), videoFlir(vs), hard_trigger(trigger){};
			//InterfaceEventHandler(InterfacePtr iface, unsigned int interfaceNum, VideoSource_FLIR* vs) : m_interface(iface), m_interfaceNum(interfaceNum), videoFlir(vs) {};
			~InterfaceEventHandler() { m_interface = NULL; /*m_system = NULL; m_Cam = NULL; m_imageEventHandler = NULL; videoFlir = NULL;/*delete m_interface; delete m_system; delete m_Cam; delete videoFlir; */ };

		private:

			int m_interfaceNum;
			InterfacePtr m_interface;
			SystemPtr m_system;
			ImageEventHandler* m_imageEventHandler;
			VideoSource_FLIR *videoFlir;
			bool hard_trigger;
		};
		// -------------- End of InterfaceEventHandler
		class VideoSource_FLIR : public VideoSource
		{
		public:
			//VideoSource_FLIR(int channel, bool skip_frame, bool hard_trigger, string camSerial, Spinnaker::SystemPtr system, Spinnaker::CameraList camList);
			VideoSource_FLIR(int channel, bool skip_frame, bool hard_trigger, string camSerial, float camFPS, float camGain);
			virtual ~VideoSource_FLIR();

			int ConfigureImageEvents(Spinnaker::CameraPtr pCam, ImageEventHandler*& imageEventHandler);
			int ResetImageEvents(Spinnaker::CameraPtr pCam, ImageEventHandler*& imageEventHandler);
			int AcquireImages(Spinnaker::CameraPtr pCam, INodeMap & nodeMap, INodeMap & nodeMapTLDevice, ImageEventHandler*& imageEventHandler);
			int WaitForImages(ImageEventHandler*& imageEventHandler);
			int PrintDeviceInfo(INodeMap & nodeMap);

			//Spinnaker::Camera* cam;
			Spinnaker::Error error;

			// Retrieve singleton reference to system object
			Spinnaker::SystemPtr system;

			// Retrieve list of cameras from the system
			Spinnaker::CameraList camList;

			Spinnaker::CameraPtr pCam;
			Spinnaker::InterfaceList interfaceList;
			//InterfaceEventHandler* m_interfaceEventHandler;
			ImageEventHandler* imageEventHandler;

			// Aug 1, 2017 Rongen, for monitoring switch status 
			bool m_lastSwitchStatus;
			bool m_currentSwitchStatus;
			void checkSwitchStatus(Spinnaker::Camera* cam);

			// Rongen Test
			virtual bool getSkipFrame() override;
			virtual void setSkipFrame(bool isSkipFrame) override;
			virtual void setNumSkipFrame(int numFrame) override;
			virtual int getNumSkipFrame() override;
			virtual int getCameraCableStatus() override;
			void setCameraCableStatus(int cable_status);
			////
			int setHardTrigger(INodeMap & nodeMap, bool onOff);
			int setGamma(INodeMap &nodeMap, bool enableGamma);
			int setAutoGain(INodeMap & nodeMap, bool autoGain);
			int setAutoExposure(INodeMap & nodeMap, bool autoExp);
			int setFrameRate(INodeMap & nodeMap, float fps);
			int setChunkEnable(INodeMap & nodeMap, bool onOff);
			int setGIGEPacketSize(INodeMap & nodeMap);
			int setGain(INodeMap & nodeMap);
			int setBuffer(INodeMap & nodeMap);
			void setLastFrameID(int frameID);
			int getLastFrameID();
			int setImageBinning(INodeMap & nodeMap, bool doBinning);

			bool isCameraUnplugged = false;
			bool isFirstTimeInit = true;
			float camFPS = 25;
			float camGain = 0.;

			void setSerialNumber(string serial_num) { m_deviceSerialNumber = serial_num; };
			string getSerialNumber() { return m_deviceSerialNumber; };

			std::chrono::time_point<std::chrono::system_clock> m_startTimeVideoSource = std::chrono::system_clock::now();
		private:
			string m_deviceSerialNumber;
			bool skipFrame = false;
			int numSkipFrame = 0;
			int lastFrameID = 0;
			int interfaceID = -1;
			int CameraCableStatus = -1;
		};

	}
}
#endif
