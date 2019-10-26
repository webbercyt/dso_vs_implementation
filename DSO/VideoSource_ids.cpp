// Copyright 2010 Isis Innovation Limited
// This video source uses Point Grey's FlyCapture 2 API
// It is based on their example code
// Parameters are hard-coded here - adapt to your specific camera.
#include "VideoSource_ids.h"

namespace VideoSourceNS {
	namespace uEye {
#define EVENTTHREAD_WAIT_TIMEOUT    1000
		VideoSource_IDS::VideoSource_IDS(string path, bool save_enabled, bool skip_frame, bool hard_trigger) : VideoSource(path, save_enabled, skip_frame)
		{

			//ids camera init 

			if (m_hCam != 0)
			{
				//free old image mem.
				is_FreeImageMem(m_hCam, m_pcImageMemory, m_lMemoryId);
				is_ExitCamera(m_hCam);
			}
			// init camera
			m_hCam = (HIDS)0; // open next camera
			//checkError(InitCamera(&m_hCam, NULL)); // init camera - no window handle required
			if(InitCamera(&m_hCam, NULL) != IS_SUCCESS)
			{
				cout << "Not enough cameras!" << endl;
				return;
			}
			numCameras = 1;
			
			GetMaxImageSize(&m_nSizeX, &m_nSizeY);
			m_imageSize = m_nSizeX*m_nSizeY;
			initIDSParameter(hard_trigger);

			image = new TrackingImage(m_nSizeX, m_nSizeY);
			startVideoCapture(boost::bind(&VideoSource_IDS::DataHandler, this, _1, _2));
		}
		VideoSource_IDS::~VideoSource_IDS()
		{
			closeCamera();
		}
		/**/

		void VideoSource_IDS::initIDSParameter(bool hard_trigger) {
			///////////////
			checkError(is_GetSensorInfo(m_hCam, &m_sInfo));
			BOARDINFO		m_cameraInfo;
			checkError(is_GetCameraInfo(m_hCam, &m_cameraInfo));
			m_nBitsPerPixel = 8;
			printf(
				"\n*** CAMERA INFORMATION ***\n"
				"Serial number - %s\n"
				"Camera model - %d\n"
				"Camera vendor - %s\n"
				"Sensor - %s\n"
				"Resolution - %d x %d\n"
				"Firmware version - %s\n",
				m_cameraInfo.SerNo,
				m_cameraInfo.Type,
				m_cameraInfo.ID,
				m_sInfo.strSensorName,
				m_nSizeX, m_nSizeY,
				m_cameraInfo.Version);

			// setup the color depth to the current windows setting
			checkError(is_SetColorMode(m_hCam, IS_CM_MONO8));
			// memory initialization
			if (is_AllocImageMem(m_hCam, m_nSizeX, m_nSizeY, m_nBitsPerPixel, &m_pcImageMemory, &m_lMemoryId) != IS_SUCCESS) {
				printf("[IDS Camera]Memory allocation failed!\n");
			}
			is_SetImageMem(m_hCam, m_pcImageMemory, m_lMemoryId); // set memory active
			/////////////////
			INT blacklevel = IS_AUTO_BLACKLEVEL_ON;
			is_Blacklevel(m_hCam, IS_BLACKLEVEL_CMD_SET_MODE, (void*)&blacklevel, sizeof(blacklevel));
			// Get pixel clock range
			UINT nRange[3];
			ZeroMemory(nRange, sizeof(nRange));
			//is_PixelClock(m_hCam, IS_PIXELCLOCK_CMD_GET_RANGE, (void*)nRange, sizeof(nRange));
			UINT nPixelClock = 50;
			INT ret = is_PixelClock(m_hCam, IS_PIXELCLOCK_CMD_SET, (void*)&nPixelClock, sizeof(nPixelClock));
			if (ret == IS_SUCCESS)
				printf("[IDS Camera]is_PixelClock Success!\n");
			double nFPS = 25.0;
			ret = is_SetFrameRate(m_hCam, nFPS, &nFPS);
			if (ret == IS_SUCCESS)
				printf("[IDS Camera]is_SetFrameRate Success!\n");
			// set exposure = 1/framerate
			double dExposure = 8.0;
			ret = is_Exposure(m_hCam, IS_EXPOSURE_CMD_SET_EXPOSURE, (void*)&dExposure, sizeof(dExposure));
			if (ret == IS_SUCCESS)
				printf("[IDS Camera]is_Exposure Success!\n");
			//enable auto exposure
			double dEnable = 1;
			ret = is_SetAutoParameter(m_hCam, IS_SET_ENABLE_AUTO_SHUTTER, &dEnable, 0);
			if (ret == IS_SUCCESS)
				printf("[IDS Camera]Auto Exposure Enabled!\n");
			//hardware gamma
			//is_SetHardwareGamma(m_hCam, IS_SET_HW_GAMMA_ON);
			/* Set gamma to 1.6 */
			INT nGamma = 160;
			ret = is_Gamma(m_hCam, IS_GAMMA_CMD_SET, (void*)&nGamma, sizeof(nGamma));
			if (ret == IS_SUCCESS)
				printf("[IDS Camera]Gamma set to 1.6!\n");
			// Disable Auto Gain
			dEnable = 0;
			ret = is_SetAutoParameter(m_hCam, IS_SET_ENABLE_AUTO_GAIN, &dEnable, 0);
			if (ret == IS_SUCCESS)
				printf("[IDS Camera]Auto Gain Disabled!\n");
			// Set Master Gain to 0
			SENSORINFO sInfo;
			ret = is_GetSensorInfo(m_hCam, &sInfo);
			if (sInfo.bMasterGain)
				printf("[IDS Camera]bMasterGain!\n");
			ret = is_SetHardwareGain(m_hCam, 0, 0, 0, 0);
			if (ret == IS_SUCCESS)
				printf("[IDS Camera]Hardware Gain Disabled!\n");
			/* AES configuration */
			INT nEnable = IS_AUTOPARAMETER_ENABLE;
			ret = is_AutoParameter(m_hCam, IS_AES_CMD_SET_ENABLE, &nEnable, sizeof(nEnable));
			if (ret == IS_SUCCESS)
				printf("[IDS Camera]Set Auto Parameter!\n");

			ret = is_AutoParameter(m_hCam, IS_AES_CMD_GET_ENABLE, &nEnable, sizeof(nEnable));
			if (ret == IS_SUCCESS)
				printf("[IDS Camera]Auto Parameter Enabled!\n", nEnable);

			UINT nSizeOfParam = sizeof(AES_CONFIGURATION) - sizeof(CHAR) + sizeof(AES_PEAK_CONFIGURATION);
			CHAR *pBuffer = new char[nSizeOfParam]; 
			memset(pBuffer, 0, nSizeOfParam);

			AES_CONFIGURATION *pAesConfiguration = (AES_CONFIGURATION*)pBuffer;
			pAesConfiguration->nMode = IS_AES_MODE_PEAK;

			//ret = is_AutoParameter(m_hCam, IS_AES_CMD_GET_CONFIGURATION, pAesConfiguration, nSizeOfParam);

			//if (ret == IS_SUCCESS)
			//	printf("[IDS Camera]Auto Parameter Config Received!\n");
			//else
			//	printf("Auto Parameter Config err:%d\n", ret);
			while (!is_AutoParameter(m_hCam, IS_AES_CMD_GET_CONFIGURATION, pAesConfiguration, nSizeOfParam));
			printf("[IDS Camera]Auto Parameter Config Received!\n");

			pAesConfiguration->nMode = IS_AES_MODE_PEAK;

			AES_PEAK_WHITE_CONFIGURATION *pPeakConfiguration = (AES_PEAK_WHITE_CONFIGURATION*)pAesConfiguration->pConfiguration;
			pPeakConfiguration->nHysteresis = 10;
			pPeakConfiguration->nReference = 10;
			pPeakConfiguration->nFrameSkip = 1;
			//set configuration
			//ret = is_AutoParameter(m_hCam, IS_AES_CMD_SET_CONFIGURATION, pAesConfiguration, nSizeOfParam);
			//if (ret == IS_SUCCESS)
			//	printf("[IDS Camera]is_AutoParameter Success!\n");
			//else
			//{
			//	INT nSupported = 0;
			//	printf("ret is:%d\n ", ret);
			//	ret = is_AutoParameter(m_hCam, IS_AES_CMD_GET_SUPPORTED_TYPES, &nSupported, sizeof(nSupported));
			//	printf("nSupported :%d\n", nSupported);
			//}
			while (!is_AutoParameter(m_hCam, IS_AES_CMD_SET_CONFIGURATION, pAesConfiguration, nSizeOfParam));
			printf("[IDS Camera]is_AutoParameter Success!\n");
			
			/* setup hardware trigger*/
			if (hard_trigger) {
				int nRet = IS_SUCCESS;
				while (!nRet == is_SetExternalTrigger(m_hCam, IS_SET_TRIGGER_HI_LO));
				//set gpio 2 to high 3.3v
				IO_GPIO_CONFIGURATION gpioConfiguration;
				gpioConfiguration.u32Gpio = IO_GPIO_2;
				gpioConfiguration.u32Configuration = IS_GPIO_OUTPUT;
				gpioConfiguration.u32State = 1;
				while (!nRet == is_IO(m_hCam, IS_IO_CMD_GPIOS_SET_CONFIGURATION, (void*)&gpioConfiguration, sizeof(gpioConfiguration)));
				gpioConfiguration.u32Gpio = IO_GPIO_1;
				gpioConfiguration.u32Configuration = IS_GPIO_INPUT;
				gpioConfiguration.u32State = 1;
				while (!nRet == is_IO(m_hCam, IS_IO_CMD_GPIOS_SET_CONFIGURATION, (void*)&gpioConfiguration, sizeof(gpioConfiguration)));
				//set timestamp reset
				m_lastSwitchStatus = false;
				IS_TIMESTAMP_CONFIGURATION timestampConfiguration;
				while (!nRet == is_DeviceFeature(m_hCam, IS_DEVICE_FEATURE_CMD_GET_TIMESTAMP_CONFIGURATION,
					(void*)&timestampConfiguration, sizeof(timestampConfiguration)));
				timestampConfiguration.s32Mode = IS_RESET_TIMESTAMP_ONCE;
				timestampConfiguration.s32Edge = TIMESTAMP_CONFIGURATION_EDGE_FALLING; timestampConfiguration.s32Pin = TIMESTAMP_CONFIGURATION_PIN_GPIO_1;

				while (!nRet == is_DeviceFeature(m_hCam, IS_DEVICE_FEATURE_CMD_SET_TIMESTAMP_CONFIGURATION,
					(void*)&timestampConfiguration, sizeof(timestampConfiguration)));
			}
			else
			{
				//use software default trigger
				is_SetExternalTrigger(m_hCam, IS_SET_TRIGGER_SOFTWARE);
			}
		}
		/**/
		void VideoSource_IDS::GetMaxImageSize(INT *pnSizeX, INT *pnSizeY)
		{
			// Check if the camera supports an arbitrary AOI
			// Only the ueye xs does not support an arbitrary AOI
			INT nAOISupported = 0;
			BOOL bAOISupported = TRUE;
			if (is_ImageFormat(m_hCam,
				IMGFRMT_CMD_GET_ARBITRARY_AOI_SUPPORTED,
				(void*)&nAOISupported,
				sizeof(nAOISupported)) == IS_SUCCESS)
			{
				bAOISupported = (nAOISupported != 0);
			}

			if (bAOISupported)
			{
				// All other sensors
				// Get maximum image size
				SENSORINFO sInfo;
				is_GetSensorInfo(m_hCam, &sInfo);
				*pnSizeX = sInfo.nMaxWidth;
				*pnSizeY = sInfo.nMaxHeight;
			}
			else
			{
				// Only ueye xs
				// Get image size of the current format
				IS_SIZE_2D imageSize;
				is_AOI(m_hCam, IS_AOI_IMAGE_GET_SIZE, (void*)&imageSize, sizeof(imageSize));

				*pnSizeX = imageSize.s32Width;
				*pnSizeY = imageSize.s32Height;
			}
		}
		/**/
		INT VideoSource_IDS::InitCamera(HIDS *hCam, HWND hWnd)
		{
			INT nRet = is_InitCamera(hCam, hWnd);
			/************************************************************************************************/
			/*                                                                                              */
			/*  If the camera returns with "IS_STARTER_FW_UPLOAD_NEEDED", an upload of a new firmware       */
			/*  is necessary. This upload can take several seconds. We recommend to check the required      */
			/*  time with the function is_GetDuration().                                                    */
			/*                                                                                              */
			/*  In this case, the camera can only be opened if the flag "IS_ALLOW_STARTER_FW_UPLOAD"        */
			/*  is "OR"-ed to m_hCam. This flag allows an automatic upload of the firmware.                 */
			/*                                                                                              */
			/************************************************************************************************/
			if (nRet == IS_STARTER_FW_UPLOAD_NEEDED)
			{
				// Time for the firmware upload = 25 seconds by default
				INT nUploadTime = 25000;
				is_GetDuration(*hCam, IS_STARTER_FW_UPLOAD, &nUploadTime);
				printf("[IDS Camera]This camera requires a new firmware. The upload will take about\n seconds.Please wait ...%i sec\n", nUploadTime / 1000);

				// Try again to open the camera. This time we allow the automatic upload of the firmware by
				// specifying "IS_ALLOW_STARTER_FIRMWARE_UPLOAD"
				*hCam = (HIDS)(((INT)*hCam) | IS_ALLOW_STARTER_FW_UPLOAD);
				nRet = is_InitCamera(hCam, hWnd);
			}

			return nRet;
		}
		/**/
		void VideoSource_IDS::captureThread(CamCaptureCB callback)
		{
			stop_capture = false;

			// Setup video frame event
#ifndef __linux__
			m_hEvent = CreateEvent(NULL, FALSE, FALSE, NULL);
			if (m_hEvent == NULL) return;
			checkError(is_InitEvent(m_hCam, m_hEvent, IS_SET_EVENT_FRAME));
#endif
			checkError(is_EnableEvent(m_hCam, IS_SET_EVENT_FRAME));

			// There is a weird condition with the uEye SDK 4.30 where
			// this never returns when using IS_WAIT.
			// We are using IS_DONT_WAIT and retry every 0.1s for 2s instead
			bool capture = false;
			for (int i = 0; i < 20; ++i) {
				if (is_CaptureVideo(m_hCam, IS_DONT_WAIT) == IS_SUCCESS) {
					capture = true;
					break;
				}
			}
			if (!capture) {
				throw uEyeException(-1, "Capture could not be started.");
			}

			char *img_mem;
			while (!stop_capture) {
				// Wait for image. Timeout after 2*FramePeriod = (2000ms/FrameRate)
#ifdef __linux__
				if (is_WaitEvent(m_hCam, IS_SET_EVENT_FRAME, EVENTTHREAD_WAIT_TIMEOUT) == IS_SUCCESS)
#else
				if (WaitForSingleObject(m_hEvent, EVENTTHREAD_WAIT_TIMEOUT) == WAIT_OBJECT_0)
#endif
					if (is_GetImageMem(m_hCam, (void**)&img_mem) == IS_SUCCESS) {
						callback(img_mem,this);
					}
			}
			
			stop_capture = true;
			// Stop video event
			checkError(is_DisableEvent(m_hCam, IS_SET_EVENT_FRAME));
			checkError(is_StopLiveVideo(m_hCam, IS_WAIT));

		}

		void VideoSource_IDS::startVideoCapture(CamCaptureCB callback)
		{
			thread_ = boost::thread(&VideoSource_IDS::captureThread, this, callback);
		}
		/**/
		void VideoSource_IDS::DataHandler(const char *img_data,const void* pCallbackData)
		{
			VideoSource_IDS *pVS = (VideoSource_IDS *)pCallbackData;
			if (!dso::setting_debugout_runquiet)
				printf("pVS->num_captured_images=%d size:%d img_mem_id_:%d\n", pVS->num_captured_images, pVS->m_imageSize, pVS->m_lMemoryId);

			if (pVS->stop_capture) {
				pVS->closeCamera();
				return;
			}
			
			// Aug 1, 2017, Rongen ------------
			// Check GPIO1 status, when switch is not pressed, it should be HIGH
			int nRet = IS_SUCCESS;
			IO_GPIO_CONFIGURATION gpioConfiguration;
			gpioConfiguration.u32Gpio = IO_GPIO_1;
			while (!nRet == is_IO(m_hCam, IS_IO_CMD_GPIOS_GET_CONFIGURATION, (void*)&gpioConfiguration, sizeof(gpioConfiguration)));
			if (!m_lastSwitchStatus && !gpioConfiguration.u32State) // Switch is pressed
			{
				m_lastSwitchStatus = true;
				cout << "Timestamp reset..." << endl;
				//set timestamp reset
				IS_TIMESTAMP_CONFIGURATION timestampConfiguration;
				while (!nRet == is_DeviceFeature(m_hCam, IS_DEVICE_FEATURE_CMD_GET_TIMESTAMP_CONFIGURATION,
					(void*)&timestampConfiguration, sizeof(timestampConfiguration)));
				timestampConfiguration.s32Mode = IS_RESET_TIMESTAMP_ONCE;
				timestampConfiguration.s32Edge = TIMESTAMP_CONFIGURATION_EDGE_FALLING; timestampConfiguration.s32Pin = TIMESTAMP_CONFIGURATION_PIN_GPIO_1;

				while (!nRet == is_DeviceFeature(m_hCam, IS_DEVICE_FEATURE_CMD_SET_TIMESTAMP_CONFIGURATION,
					(void*)&timestampConfiguration, sizeof(timestampConfiguration)));
			}
			else if (gpioConfiguration.u32State) // switch not pressed
				m_lastSwitchStatus = false;
			//------------------


			// timestamp
			UEYEIMAGEINFO ImageInfo;
			is_GetImageInfo(m_hCam, pVS->m_lMemoryId, &ImageInfo, sizeof(ImageInfo));
			unsigned int seconds = ImageInfo.TimestampSystem.wHour * 3600 + ImageInfo.TimestampSystem.wMinute * 60 + ImageInfo.TimestampSystem.wSecond;
			unsigned int millsec = ImageInfo.TimestampSystem.wMilliseconds;

			// Rongen Skip Frame test
			currID = ImageInfo.u64FrameNumber;
			currentTime = seconds + millsec*0.001;
			timeDiff = currentTime - previousTime;
			//printf("CurrID: %d\n", currID);
			if (((currID - prevID) > 1) || timeDiff > 0.045)
			{
				printf("Skipped Frame! Time Diff: %f currentID:%d previousID: %d\n", timeDiff, currID, prevID);
			}
			previousTime = currentTime;
			prevID = currID;
			// exposure
			double dexpose;
			is_Exposure(m_hCam, IS_EXPOSURE_CMD_GET_EXPOSURE, (void*)&dexpose, sizeof(dexpose));
			///copy image
			mtx.lock();
			memcpy(pVS->image->getImage()->data, img_data, pVS->m_imageSize);
			pVS->image->setTimeStamp(seconds + millsec*0.001);
			pVS->image->setExposure(dexpose);
			mtx.unlock();
			if (save_image || !skip_frame)
			{
				mtx.lock();
				pVS->images[pVS->num_captured_images] = pVS->image->copy();
				mtx.unlock();
			}
			num_captured_images++;
			//printf("ids->num_captured_images=%d exposure:%f time:%.3f\n", num_captured_images, dexpose, timestamp);
			
		}
		/////////////////////////////////////
		//other ueye functions

		void VideoSource_IDS::closeCamera()
		{
			if (m_hCam > 0) {
				// Release camera and all associated memory
				checkError(IS_SUCCESS != is_ExitCamera(m_hCam));
			}
		}

		void VideoSource_IDS::stopVideoCapture()
		{
			stop_capture = true;
			if (thread_.joinable()) {
				thread_.join();
			}
		}
	}
}
