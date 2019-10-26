// Copyright 2010 Isis Innovation Limited
// This video source uses Point Grey's FlyCapture 2 API
// It is based on their example code
// Parameters are hard-coded here - adapt to your specific camera.

#include <fstream>
#include <iomanip>

#include "VideoSource_PointGrey.h"
// Rongen testing
double currentTime = 0., previousTime = 0., timeDiff = 0.;
int currID = 0, prevID = 0;
namespace VideoSourceNS {
	namespace PGR
	{
		static void PrintErrorAndExit(Error error)
		{
			error.PrintErrorTrace();
			exit(0);
		}

		void DataHandler(class Image* pImage, const void* pCallbackData)
		{
			VideoSource_PGR *pVS = (VideoSource_PGR *)pCallbackData;
			if (!dso::setting_debugout_runquiet)
				printf("pVS->num_captured_images=%d\n", pVS->num_captured_images);

			if (pVS->stop_capture) {
				pVS->cam.StopCapture();
				return;
			}

			if (!pImage) return;
			ImageMetadata pImgMeta = pImage->GetMetadata();
			unsigned int exposure = pImgMeta.embeddedExposure;
			int w = pImage->GetCols();
			if (pVS->image->w() != w) return;

			int h = pImage->GetRows();
			if (pVS->image->h() != h) return;
			int image_size = w * h;

			// timestamp
			TimeStamp embeddedTime = pImage->GetTimeStamp();
			currentTime = embeddedTime.seconds + embeddedTime.microSeconds*0.000001;
			pVS->image->setTimeStamp(currentTime);

			//shutter
			pVS->pp.type = SHUTTER;
			// exposure
			pVS->error = pVS->cam.GetProperty(&pVS->pp);
			pVS->image->setExposure(pVS->pp.absValue);
			
			// Rongen Skip Frame test
			currID = pImgMeta.embeddedFrameCounter;
			timeDiff = currentTime - previousTime;
			if (((currID - prevID) > 1) || timeDiff > 0.045)
			{
				printf("Skipped Frame! Time Diff: %f currentID:%d previousID: %d\n", timeDiff, currID, prevID);
			}
			previousTime = currentTime;
			prevID = currID;
			if (pImage->GetPixelFormat() != FlyCapture2::PIXEL_FORMAT_MONO8)
			{
				// convert to r
				Image rImage;
				Error error = pImage->Convert(FlyCapture2::PIXEL_FORMAT_MONO8, &rImage);
				if (error != PGRERROR_OK)
				{
					printf("GetAndFillFrameBW(): Image type convert error!\n");
					return;
				}
				pVS->mtx.lock();
				memcpy(pVS->image->getImage()->data, (unsigned char*)rImage.GetData(), image_size);
				pVS->mtx.unlock();
				rImage.ReleaseBuffer();
			}
			else
			{
				pVS->mtx.lock();
				memcpy(pVS->image->getImage()->data, (unsigned char*)pImage->GetData(), image_size);
				pVS->mtx.unlock();
			}

			if (pVS->save_image || !pVS->skip_frame)
			{
				pVS->mtx.lock();
				pVS->images[pVS->num_captured_images] = pVS->image->copy();
				pVS->mtx.unlock();
			}
			pVS->num_captured_images++;
		}

		bool PollForTriggerReady(Camera *pCam)
		{
			const unsigned int k_softwareTrigger = 0x62C;
			Error error;
			unsigned int regVal = 0;

			do
			{
				error = pCam->ReadRegister(k_softwareTrigger, &regVal);
				if (error != PGRERROR_OK)
					PrintErrorAndExit(error);

			} while ((regVal >> 31) != 0);

			return true;
		}

		///////////////////
		VideoSource_PGR::VideoSource_PGR(string path, bool save_enabled, bool skip_frame,bool hard_trigger,string camSerial) : VideoSource(path, save_enabled, skip_frame)
		{
			pp.type = SHUTTER;
			Error error;
			BusManager busMgr;
			error = busMgr.GetNumOfCameras(&numCameras);
			if (error != PGRERROR_OK)
				PrintErrorAndExit(error);

			printf("Number of cameras detected: %u\n", numCameras);
			//assert(numCameras > 0);
			if (numCameras == 0)
			{
				cout << "Not enough cameras!" << endl;
				return;
			}
			
			PGRGuid guid;

			if (camSerial == "")
			{//use default id to open camera
				error = busMgr.GetCameraFromIndex(0, &guid);
				//try next camera if not controlable
				bool isControlable = true;
				error = busMgr.IsCameraControlable(&guid, &isControlable);
				if (error != PGRERROR_OK)
					PrintErrorAndExit(error);
				if (!isControlable && numCameras > 1)
				{
					printf("Camera 0 is already used, using camera 1!\n");
					error = busMgr.GetCameraFromIndex(1, &guid);
					if (error != PGRERROR_OK)
						PrintErrorAndExit(error);

				}
			}
			else
			{
				error = busMgr.GetCameraFromSerialNumber(std::stoi(camSerial), &guid);
				if (error != PGRERROR_OK)
					PrintErrorAndExit(error);
			}
			
			error = cam.Connect(&guid);
			
			if (error != PGRERROR_OK)
				PrintErrorAndExit(error);

			CameraInfo camInfo;
			error = cam.GetCameraInfo(&camInfo);
			if (error != PGRERROR_OK)
				PrintErrorAndExit(error);

			////////////////
			///
			printf(
				"\n*** CAMERA INFORMATION ***\n"
				"Serial number - %u\n"
				"Camera model - %s\n"
				"Camera vendor - %s\n"
				"Sensor - %s\n"
				"Resolution - %s\n"
				"Firmware version - %s\n"
				"Firmware build time - %s\n\n",
				//
				camInfo.serialNumber,
				camInfo.modelName,
				camInfo.vendorName,
				camInfo.sensorInfo,
				camInfo.sensorResolution,
				camInfo.firmwareVersion,
				camInfo.firmwareBuildTime);
			///////
			const char* pgrNum_default = "000000000";


			const Mode k_fmt7Mode = MODE_0;
			const PixelFormat k_fmt7PixFmt = PIXEL_FORMAT_MONO8;

			Format7Info fmt7Info;
			bool supported;

			error = cam.GetFormat7Info(&fmt7Info, &supported);
			if (error != PGRERROR_OK)
				PrintErrorAndExit(error);

			Format7ImageSettings fmt7ImageSettings;
			fmt7ImageSettings.mode = k_fmt7Mode;
			fmt7ImageSettings.offsetX = 0;
			fmt7ImageSettings.offsetY = 0;
			fmt7ImageSettings.width = fmt7Info.maxWidth;
			fmt7ImageSettings.height = fmt7Info.maxHeight;
			fmt7ImageSettings.pixelFormat = k_fmt7PixFmt;

			bool valid;
			Format7PacketInfo fmt7PacketInfo;
			// Validate the settings to make sure that they are valid
			error = cam.ValidateFormat7Settings(
				&fmt7ImageSettings,
				&valid,
				&fmt7PacketInfo);
			if (error != PGRERROR_OK)
				PrintErrorAndExit(error);

			// Set the settings to the camera
			error = cam.SetFormat7Configuration(&fmt7ImageSettings,
				fmt7PacketInfo.recommendedBytesPerPacket
				);

			if (error != PGRERROR_OK)
				PrintErrorAndExit(error);

			Property prop;
			//frame rate hardware trigger or maximum run
			if (hard_trigger)
			{
				cout << "Setting hardware trigger.." << endl;
				//enable output voltage high
				cam.WriteRegister(pgr_outVoltageEnable,1);
				TriggerModeInfo triggerModeInfo;
				error = cam.GetTriggerModeInfo(&triggerModeInfo);
				if (error != PGRERROR_OK)
					PrintErrorAndExit(error);
				if (triggerModeInfo.present != true)
				{
					cout << "Camera does not support external trigger! Exiting..." << endl;
					return;
				}
				TriggerMode triggerMode;
				error = cam.GetTriggerMode(&triggerMode);
				if (error != PGRERROR_OK)
					PrintErrorAndExit(error);
				// Set camera to trigger mode 0
				triggerMode.onOff = true;
				triggerMode.mode = 0;
				triggerMode.parameter = 0;
				triggerMode.source = 0;
				error = cam.SetTriggerMode(&triggerMode);
				if (error != PGRERROR_OK)
					PrintErrorAndExit(error);

				// Poll to ensure camera is ready
				bool retVal = PollForTriggerReady(&cam);
				if (!retVal)
				{
					cout << endl;
					cout << "Error polling for trigger ready!" << endl;
					return;
				}
			}
			else
			{
				//output voltage low
				cam.WriteRegister(pgr_outVoltageEnable, 0);

				TriggerMode triggerMode;
				error = cam.GetTriggerMode(&triggerMode);
				if (error != PGRERROR_OK)
					PrintErrorAndExit(error);
				// Set camera to trigger mode 0
				triggerMode.onOff = false;
				triggerMode.mode = 0;
				triggerMode.parameter = 0;
				triggerMode.source = 0;
				error = cam.SetTriggerMode(&triggerMode);
				if (error != PGRERROR_OK)
					PrintErrorAndExit(error);
				prop.type = FRAME_RATE;
				prop.absValue = 25.0;
				prop.autoManualMode = false;
				prop.onOff = true;
				prop.absControl = true;
				error = cam.SetProperty(&prop);
				if (error != PGRERROR_OK)
					PrintErrorAndExit(error);
			}

			//////////////////////
			//shutter
			prop.type = SHUTTER;
			error = cam.GetProperty(&prop);
			prop.autoManualMode = true;
			prop.onOff = true;
			error = cam.SetProperty(&prop);
			if (error != PGRERROR_OK)
				PrintErrorAndExit(error);
			//brightness
			prop.type = BRIGHTNESS;
			error = cam.GetProperty(&prop);
			prop.autoManualMode = false;
			error = cam.SetProperty(&prop);
			if (error != PGRERROR_OK)
				PrintErrorAndExit(error);
			//gamma
			prop.type = GAMMA;
			error = cam.GetProperty(&prop);
			prop.absValue = 1.;
			prop.onOff = true;
			error = cam.SetProperty(&prop);
			if (error != PGRERROR_OK)
				PrintErrorAndExit(error);
			//exposure
			prop.type = AUTO_EXPOSURE;
			error = cam.GetProperty(&prop);
			prop.autoManualMode = true;
			prop.onOff = true;
			error = cam.SetProperty(&prop);
			if (error != PGRERROR_OK)
				PrintErrorAndExit(error);
			//Gain
			prop.type = GAIN;
			error = cam.GetProperty(&prop);
			prop.absValue = 0.;
			prop.autoManualMode = false;
			prop.onOff = false;
			error = cam.SetProperty(&prop);
			if (error != PGRERROR_OK)
				PrintErrorAndExit(error);
			//Sharpness
			prop.type = SHARPNESS;
			error = cam.GetProperty(&prop);
			prop.autoManualMode = false;
			prop.onOff = false;
			error = cam.SetProperty(&prop);
			if (error != PGRERROR_OK)
				PrintErrorAndExit(error);
			/////////////////////////
			cam.StartCapture();
			cam.StopCapture();

			unsigned int pPacketSize;
			float pPercentage;
			cam.GetFormat7Configuration(&fmt7ImageSettings, &pPacketSize, &pPercentage);
			image = new TrackingImage(fmt7ImageSettings.width, fmt7ImageSettings.height);

			error = cam.StartCapture(DataHandler, this);

			if (error != PGRERROR_OK)
				PrintErrorAndExit(error);
			////////////////
		};

		VideoSource_PGR::~VideoSource_PGR()
		{

		}

	}
}
