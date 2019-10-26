// Copyright 2010 Isis Innovation Limited
// This video source uses Point Grey's FlyCapture 2 API
// It is based on their example code
// Parameters are hard-coded here - adapt to your specific camera.
#include "VideoSource_Basler.h"
namespace VideoSourceNS {
	namespace BASLER
	{

		///////////////////
		VideoSource_Basler::VideoSource_Basler(std::string path, bool save_enabled, bool skip_frame,bool hard_trigger,std::string camSerial) : VideoSource(path, save_enabled, skip_frame)
		{
			// The exit code of the sample application.
			int exitCode = 0;
			// Before using any pylon methods, the pylon runtime must be initialized. 
			PylonInitialize();
			try
			{
				IPylonDevice* firstDevice = CTlFactory::GetInstance().CreateFirstDevice();
				if (!firstDevice)
				{
					cout << "Not enough cameras!" << endl;
					return;
				}
				numCameras = 1;

				// Only look for GigE cameras.
				CDeviceInfo camInfo;
				camInfo.SetDeviceClass(BaslerGigEDeviceClass);
				// Create an instant camera object for the GigE camera found first.
				bcamera = new Camera_t(CTlFactory::GetInstance().CreateFirstDevice(camInfo));
				bcamera->RegisterImageEventHandler(this, RegistrationMode_Append, Cleanup_Delete);
				bcamera->GrabCameraEvents = true;
				bcamera->Open();
				// Print the model name of the camera.
				cout << "Using Basler " << bcamera->GetDeviceInfo().GetModelName() << endl;
				cout << "Camera Device Information" << endl
					<< "=========================" << endl;
				cout << "Serial Number		: "
					<< bcamera->GetDeviceInfo().GetSerialNumber() << endl;
				cout << "Camera Model		: "
					<< bcamera->DeviceModelName.GetValue() << endl;
				cout << "SensorRes		: "
					<< bcamera->SensorWidth.GetValue() << "x" << bcamera->SensorHeight.GetValue() << endl;
				cout << "Vendor			: "
					<< bcamera->DeviceVendorName.GetValue() << endl;
				cout << "Model			: "
					<< bcamera->DeviceModelName.GetValue() << endl;
				cout << "Firmware version: "
					<< bcamera->DeviceFirmwareVersion.GetValue() << endl << endl;
				capImage = CPylonImage::Create(PixelType_Mono8, bcamera->Width.GetMax(), bcamera->Height.GetMax());
				if (IsWritable(bcamera->SequenceEnable))
				{
					// Turn configuration mode on
					if (IsWritable(bcamera->SequenceConfigurationMode))
					{
						bcamera->SequenceConfigurationMode.SetValue(SequenceConfigurationMode_On);
					}

					// Maximize the image area of interest (Image AOI).
					if (IsWritable(bcamera->OffsetX))
					{
						bcamera->OffsetX.SetValue(bcamera->OffsetX.GetMin());
					}
					if (IsWritable(bcamera->OffsetY))
					{
						bcamera->OffsetY.SetValue(bcamera->OffsetY.GetMin());
					}
					bcamera->Width.SetValue(bcamera->Width.GetMax());
					bcamera->Height.SetValue(bcamera->Height.GetMax());

					// Set the pixel data format.
					bcamera->PixelFormat.SetValue(PixelFormat_Mono8);
					// Start the grabbing of c_countOfImagesToGrab images.
					// The camera device is parameterized with a default configuration which
					// sets up free-running continuous acquisition.
					//bcamera->StartGrabbing();
					capImage = CPylonImage::Create(PixelType_Mono8, bcamera->Width.GetMax(), bcamera->Height.GetMax());
					image_size = bcamera->Width.GetMax()*bcamera->Height.GetMax();
					bcamera->StartGrabbing();

					thread_=boost::thread(&VideoSource_Basler::captureThread, this);
					cout << "started basler thread....." << endl;
				}
				
			}
			catch (const GenericException &e)
			{
				// Error handling.
				cerr << "An exception occurred." << endl
					<< e.GetDescription() << endl;
				exitCode = 1;
			}
		};

		VideoSource_Basler::~VideoSource_Basler()
		{

		}
		void VideoSource_Basler::OnImageGrabbed(CInstantCamera& camera, const CGrabResultPtr& ptrGrabResult)
		{
			cout << "CSampleImageEventHandler::OnImageGrabbed called." << std::endl;
			if (ptrGrabResult->GrabSucceeded())
			{
				double timestamp=bcamera->ExposureEndEventTimestamp.GetValue();
				image->setTimeStamp(timestamp*0.000001);
				float expTime=bcamera->ExposureTimeAbs.GetValue();
				image->setExposure(expTime);
				cout << "time stamp:" << timestamp << " exposure:"<< expTime<< std::endl;
				const uint8_t *pImageBuffer = (uint8_t *)ptrGrabResult->GetBuffer();
				mtx.lock();
				memcpy(image->getImage()->data, pImageBuffer, image_size);
				mtx.unlock();

				num_captured_images++;
			}
			else
			{
				cout << "Error: " << ptrGrabResult->GetErrorCode() << " " << ptrGrabResult->GetErrorDescription() << endl;
			}
		}
		/**/
		void VideoSource_Basler::captureThread()
		{
			stop_capture = false;
			GrabResultPtr_t ptrGrabResult;
			while (bcamera->IsGrabbing()&& !stop_capture) {
				bcamera->RetrieveResult(1000, ptrGrabResult, TimeoutHandling_ThrowException);
			}

			stop_capture = true;
			bcamera->Close();
			PylonTerminate();
		}
	
	}
}
