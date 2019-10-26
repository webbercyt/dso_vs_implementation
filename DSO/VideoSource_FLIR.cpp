#include "VideoSource_FLIR.h"
#include <chrono>
#include <inttypes.h>

namespace VideoSourceNS {

	namespace FLIR {
		//////////////
		ImageEventHandler::ImageEventHandler(Spinnaker::CameraPtr pCam, VideoSource_FLIR* vs)
		{
			// Retrieve device serial number
			INodeMap & nodeMap = pCam->GetTLDeviceNodeMap();

			m_deviceSerialNumber = "";
			CStringPtr ptrDeviceSerialNumber = nodeMap.GetNode("DeviceSerialNumber");
			if (IsAvailable(ptrDeviceSerialNumber) && IsReadable(ptrDeviceSerialNumber))
			{
				m_deviceSerialNumber = ptrDeviceSerialNumber->GetValue();
				cout << "ImageEventHandler - DeviceSerialNumber: " << m_deviceSerialNumber << endl;
			}

			// Release reference to camera
			cam = pCam;
			videoFlir = vs;
			pCam = NULL;
			vs = NULL;

		}
		void ImageEventHandler::OnImageEvent(Spinnaker::ImagePtr image)
		{
			//std::chrono::time_point<std::chrono::system_clock> m_startTime = std::chrono::system_clock::now();

			// Check image retrieval status
			if (image && !image->IsIncomplete() && videoFlir != NULL && !videoFlir->stop_capture /*&& !dso::setting_pauseRequested[videoFlir->channel]*/)
			{
				if (waitCounter >= videoFlir->camFPS) //wait 1 second until camera exposure is properly adjusted
				{
					// Test Rongen
					// Retrieve GenICam nodemap
					//INodeMap & nodeMap = videoFlir->pCam->GetNodeMap();
					//m_Time7 = std::chrono::system_clock::now();

					ChunkData m_chunkData = image->GetChunkData();
					int w = image->GetWidth();
					int h = image->GetHeight();
					//cout << "image->GetWidth(): " << w << " image->GetHeight(): " << h << endl;
					if (videoFlir->image->w() != w)
					{
						cout << "videoFlir->image->w(): " << videoFlir->image->w() << " w: " << w << endl;
						cin.ignore();
					}
						//return;
					if (videoFlir->image->h() != h)
					{
						cout << "videoFlir->image->h(): " << videoFlir->image->h() << " h: " << h << endl;
						cin.ignore();
					}
						
						//return;
					int image_size = w * h;

					currCamFrameID = m_chunkData.GetFrameID();
					currentTime = static_cast<double>(m_chunkData.GetTimestamp()) *1E-9; // in seconds
					if (waitCounter == videoFlir->camFPS)
					{
						startTime = currentTime;
						previousTime = currentTime;
						startCamFrameID = currCamFrameID;
						prevCamFrameID = currCamFrameID;
					}
					double exposureTime = static_cast<double>(m_chunkData.GetExposureTime()) * 0.001f;
					exposure = (float)exposureTime;

					videoFlir->image->setTimeStamp(currentTime - startTime);
					currFrameID = currCamFrameID - startCamFrameID;
					videoFlir->image->setFrameID(currFrameID);

					frameDiff = currCamFrameID - prevCamFrameID - 1;
					timeDiff = currentTime - previousTime;
					int timeFrameDiff = (int)(timeDiff * videoFlir->camFPS + 0.5f) - 1;
					//cout << "timeFrameDiff: " << timeFrameDiff << endl;
					if ((frameDiff > 0)/* || (timeFrameDiff > 0)*/)
					{
						if (!previousPause) 
						{
							printf("OnImageEvent(): Skipped Frame! Camera Serial:%s Time Diff: %f currentID: %d previousID: %d exposure:%f timeFrameDiff:%d currCamFrameID:%d prevCamFrameID:%d frameDiff:%d\n", m_deviceSerialNumber, timeDiff, currFrameID, prevFrameID, exposure, timeFrameDiff, currCamFrameID, prevCamFrameID, frameDiff);
							videoFlir->setSkipFrame(true);
							if (videoFlir->getNumSkipFrame() == 0) // system reseting
								skipFrameCount = 0;
							if (frameDiff > 0)
							{
								skipFrameCount += frameDiff;
								//currFrameID += frameDiff;
							}
									
							else
							{
								//skipFrameCount++;
								
								//currFrameID += timeFrameDiff;

								//Rongen 2019 Jan
								//skipFrameCount += timeFrameDiff;
							}
									
							videoFlir->setNumSkipFrame(skipFrameCount);
							//currFrameID++;
							//videoFlir->image->setFrameID(currFrameID);
						}
					}
					else
						videoFlir->setSkipFrame(false);
					prevFrameID = currFrameID;
					previousTime = currentTime;
					prevCamFrameID = currCamFrameID;

					videoFlir->image->setExposure(exposure);

					Spinnaker::ImagePtr convertedImage = image;
					if (image->GetPixelFormat() != Spinnaker::PixelFormat_Mono8)
						convertedImage = image->Convert(Spinnaker::PixelFormat_Mono8, Spinnaker::HQ_LINEAR);

					if (videoFlir->skip_frame)
					{
						videoFlir->mtx.lock();
						memcpy(videoFlir->image->getImage()->data, (unsigned char*)image->GetData(), image_size);
						videoFlir->mtx.unlock();
					}						

					//save the image
					if (!videoFlir->skip_frame)
					{
						TrackingImage* img = new TrackingImage();
						img->setImage(new dso::MinimalImageB(w, h));
						memcpy(img->getImage()->data, (unsigned char*)image->GetData(), w * h);
						img->setTimeStamp(currentTime - startTime);
						img->setExposure(exposure);
						img->setFrameID(currFrameID);

						videoFlir->mtx.lock();
						videoFlir->images[videoFlir->num_captured_images] = img;
						videoFlir->mtx.unlock();
					}
					videoFlir->num_captured_images++;
					previousPause = false;
				}
				waitCounter++;
			}
			else if (image->IsIncomplete())
			{
				// Retreive and print the image status description
				cout << "Image incomplete: " << Image::GetImageStatusDescription(image->GetImageStatus())
					<< "..." << endl << endl;
			}
			//else if (dso::setting_pauseRequested[videoFlir->channel])
			//{
			//	previousPause = true;
			//}
			//std::chrono::time_point<std::chrono::system_clock> m_endTime = std::chrono::system_clock::now();
			//double timeUsed = std::chrono::duration_cast<std::chrono::milliseconds>(m_endTime - m_startTime).count();
			//double timeUsedVideoSource = std::chrono::duration_cast<std::chrono::milliseconds>(m_endTime - videoFlir->m_startTimeVideoSource).count();
			//if (timeUsed > 10)
			//{
			//	printf("OnImageEvent - Camera::%s timeUsed:%f timeUsedVideoSource:%f frameID:%d\n", m_deviceSerialNumber, timeUsed, timeUsedVideoSource, currCamFrameID);
			//	printf("timeUsed1:%f timeUsed2:%f timeUsed3:%f timeUsed4:%f timeUsed5:%f timeUsed6:%f timeUsed7:%f timeUsed8:%f\n", timeUsed1, timeUsed2, timeUsed3, timeUsed4, timeUsed5, timeUsed6, timeUsed7, timeUsed8);
			//}
				

		}
		////////////////////////////////
		VideoSource_FLIR::VideoSource_FLIR(int channel, bool skip_frame, bool hard_trigger, string camSerial, float _camFPS, float _camGain) :
			VideoSource(channel, skip_frame), camFPS(_camFPS), camGain(_camGain)
		{
			try 
			{
				try {
					system->ReleaseInstance();
				}
				catch (...) {
					//cout << "[VideoSource_FLIR]ReleaseInstance(): error!\n" << endl;
				}
				system = System::GetInstance();

				// Retrieve list of interfaces from the system
				interfaceList = system->GetInterfaces();
				unsigned int numInterfaces = interfaceList.GetSize();
				cout << "Number of interfaces detected: " << numInterfaces << endl << endl;
				cout << endl << "*** CONFIGURING ENUMERATION EVENTS ***" << endl << endl;
				try
				{
					for (unsigned int i = 0; i < numInterfaces; i++)
					{
						// Select interface
						InterfacePtr pInterface = interfaceList.GetByIndex(i);
						cout << "pInterface: " << i << endl;
						if (pInterface->GetCameras().GetSize() != 0)
						{
							if (pInterface->TLInterface.DeviceID.ToString().c_str() == camSerial)
							{
								cout << "Create interface event" << endl;
								//// Create interface event
								//m_interfaceEventHandler = new InterfaceEventHandler(pInterface, i, system, this, hard_trigger);
								//// Register interface event
								//pInterface->RegisterEvent(*m_interfaceEventHandler);
								interfaceID = i;
							}
						}
					}
				}
				catch (Spinnaker::Exception &e)
				{
					cout << "Error message: " << e.GetError() << " - " << e.GetErrorMessage() << endl;
					//cout << "Press enter to quit\n";
					//cin.ignore();
				}
				camList = system->GetCameras();
				numCameras = camList.GetSize();
				cout << "Number of cameras detected: " << numCameras << endl << endl;
				if (numCameras == 0)
				{
					// Clear camera list before releasing system
					camList.Clear();
					system->ReleaseInstance();
					cout << "Not enough cameras!" << endl;
					return;
				}
				try
				{
					if (camSerial == "")
					{//use default id to open camera
						pCam = camList.GetByIndex(0);
					}
					else
					{
						pCam = camList.GetBySerial(camSerial);
						setSerialNumber(camSerial);
					}
				}
				catch (Spinnaker::Exception &e)
				{
					cout << "******************************************************************************************************" << endl;
					cout << "******************************************************************************************************" << endl;
					cout << "********* Current Camera #" << camSerial << " is not connected, check SpinView or other running DSO ********************" << endl;
					cout << "Error message: " << e.GetError() << " - " << e.GetErrorMessage() << endl;
					//cout << "Press enter to quit\n";
					//cin.ignore();
					return;
				}
				// Retrieve device serial number
				INodeMap & nodeMapTLDevice = pCam->GetTLDeviceNodeMap();
				PrintDeviceInfo(nodeMapTLDevice);
				try
				{
					// Initialize camera
					pCam->Init();
				}
				catch (Spinnaker::Exception &e)
				{
					cout << "******************************************************************************************************" << endl;
					cout << "******************************************************************************************************" << endl;
					cout << "********* Current Camera #" << camSerial << " is in use, check SpinView or other running DSO ********************" << endl;
					cout << "Error message: " << e.GetError() << " - " << e.GetErrorMessage() << endl;
					//cout << "Press enter to quit\n";
					//cin.ignore();
					//exit(0);
				}

				// Retrieve GenICam nodemap
				INodeMap & nodeMap = pCam->GetNodeMap();
				// setup Buffer
				//setBuffer(nodeMap);
				// set Image Size
				setImageBinning(nodeMap, true);
				// setup Packet Size
				setGIGEPacketSize(nodeMap);
				// setup chunk data
				setChunkEnable(nodeMap, true);
				// setup streaming buffer
				INodeMap & sNodeMap = pCam->GetTLStreamNodeMap();
				setBuffer(sNodeMap);
				//// testing by Rongen
				//CIntegerPtr width = nodeMap.GetNode("Width");
				//width->SetValue(320);
				//CIntegerPtr height = nodeMap.GetNode("Height");
				//height->SetValue(256);

				//setup trigger mode
				if (hard_trigger)
				{
					setFrameRate(nodeMap, 70.0);
					setHardTrigger(nodeMap, 1);
				}
				else
				{
					setHardTrigger(nodeMap, 0);
					setFrameRate(nodeMap, camFPS);
				}
				setGamma(nodeMap, 1);
				setAutoGain(nodeMap,0);
				setGain(nodeMap);
				setAutoExposure(nodeMap, 1);
				image = new TrackingImage(pCam->Width.GetValue(), pCam->Height.GetValue());
				cout << "pCam->Width.GetValue(): " << pCam->Width.GetValue() << " pCam->Height.GetValue(): " << pCam->Height.GetValue() << endl;
				ConfigureImageEvents(pCam, imageEventHandler);
				AcquireImages(pCam, nodeMap, nodeMapTLDevice, imageEventHandler);
			}
			catch (Spinnaker::Exception &e)
			{
				cout << "Error message: " << e.GetError() << " - " << e.GetErrorMessage() << endl;
				//cout << "Press enter to quit\n";
				//cin.ignore();
			}
		}

		VideoSource_FLIR::~VideoSource_FLIR()
		{
			if (numCameras == 0) return;
			try
			{
				pCam->EndAcquisition();
				INodeMap & nodeMap = pCam->GetNodeMap();
				setImageBinning(nodeMap, false);
				setHardTrigger(nodeMap, 0);
				setFrameRate(nodeMap, camFPS);
			}
			catch (Spinnaker::Exception &e)
			{
				cout << "pCam->EndAcquisition(); Error message: " << e.GetError() << " - " << e.GetErrorMessage() << endl;
				//cout << "Press enter to quit\n";
				//cin.ignore();
			}
			try {
				ResetImageEvents(pCam, imageEventHandler);
			}
			catch (Spinnaker::Exception &e)
			{
				cout << "ResetImageEvents Error message: " << e.GetError() << " - " << e.GetErrorMessage() << endl;
				//cout << "Press enter to quit\n";
				//cin.ignore();
			}
			//try {
			//	interfaceList.GetByIndex(interfaceID)->UnregisterEvent(*m_interfaceEventHandler);
			//	delete m_interfaceEventHandler;
			//	//cout << "m_interfaceEventHandler deleted" << endl;
			//}
			//catch (Spinnaker::Exception &e)
			//{
			//	cout << "interfaceList Error message: " << e.GetError() << " - " << e.GetErrorMessage() << endl;
			//	cout << "Press enter to quit\n";
			//	cin.ignore();
			//}
			try {
				interfaceList.Clear();
				//cout << "interfaceList" << endl;
			}
			catch (Spinnaker::Exception &e)
			{
				cout << "interfaceList.Clear(); Error message: " << e.GetError() << " - " << e.GetErrorMessage() << endl;
				//cout << "Press enter to quit\n";
				//cin.ignore();
			}
			try {
				pCam->DeInit();
				pCam = NULL;
				//cout << "pCam->DeInit()" << endl;
			}
			catch (Spinnaker::Exception &e)
			{
				cout << "DeInit Error message: " << e.GetError() << " - " << e.GetErrorMessage() << endl;
				//cout << "Press enter to quit\n";
				//cin.ignore();
			}
			try {
				camList.Clear();
				//cout << "camList.Clear()" << endl;
			}
			catch (Spinnaker::Exception &e)
			{
				cout << "camList Error message: " << e.GetError() << " - " << e.GetErrorMessage() << endl;
				//cout << "Press enter to quit\n";
				//cin.ignore();
			}
			try {
				system->ReleaseInstance();
				//cout << "system->ReleaseInstance();" << endl;
			}
			catch (Spinnaker::Exception &e)
			{
				cout << "system->ReleaseInstance(); Error message: " << e.GetError() << " - " << e.GetErrorMessage() << endl;
				//cout << "Press enter to quit\n";
				//cin.ignore();
			}
		}

		static void PrintErrorAndExit(Error error)
		{
			
		}

		int VideoSource_FLIR::ConfigureImageEvents(CameraPtr pCam, ImageEventHandler*& imageEventHandler)
		{
			int result = 0;
			try
			{
				if (pCam->IsValid())
				{
					imageEventHandler = new ImageEventHandler(pCam, this);
					pCam->RegisterEvent(*imageEventHandler);
				}
				else
				{
					cout << "[VideoSource_FLIR]Error: camera is invalid!" << endl;
				}
			}
			catch (Spinnaker::Exception &e)
			{
				cout << "Error: " << e.what() << endl;
				result = -1;
			}
			return result;
		}

		int VideoSource_FLIR::ResetImageEvents(CameraPtr pCam, ImageEventHandler*& imageEventHandler)
		{
			int result = 0;
			try
			{
				if (imageEventHandler && pCam->IsValid())
				{
					pCam->UnregisterEvent(*imageEventHandler);
					delete imageEventHandler;
				}

				cout << "Image events unregistered..." << endl << endl;
			}
			catch (Spinnaker::Exception &e)
			{
				cout << "Error: " << e.what() << endl;
				result = -1;
			}

			return result;
		}

		int VideoSource_FLIR::AcquireImages(CameraPtr pCam, INodeMap & nodeMap, INodeMap & nodeMapTLDevice, ImageEventHandler*& imageEventHandler)
		{
			int result = 0;
			try
			{
				// Set acquisition mode to continuous
				CEnumerationPtr ptrAcquisitionMode = nodeMap.GetNode("AcquisitionMode");
				if (!IsAvailable(ptrAcquisitionMode) || !IsWritable(ptrAcquisitionMode))
				{
					cout << "Unable to set acquisition mode to continuous (node retrieval). Aborting..." << endl << endl;
					return -1;
				}

				CEnumEntryPtr ptrAcquisitionModeContinuous = ptrAcquisitionMode->GetEntryByName("Continuous");
				if (!IsAvailable(ptrAcquisitionModeContinuous) || !IsReadable(ptrAcquisitionModeContinuous))
				{
					cout << "Unable to set acquisition mode to continuous (enum entry retrieval). Aborting..." << endl << endl;
					return -1;
				}

				int64_t acquisitionModeContinuous = ptrAcquisitionModeContinuous->GetValue();

				ptrAcquisitionMode->SetIntValue(acquisitionModeContinuous);

				cout << "Acquisition mode set to continuous..." << endl;

				// Begin acquiring images
				pCam->BeginAcquisition();

				cout << "Acquiring images..." << endl;
			}
			catch (Spinnaker::Exception &e)
			{
				cout << "Error: " << e.what() << endl;
				result = -1;
			}

			return result;
		}

		int VideoSource_FLIR::WaitForImages(ImageEventHandler*& imageEventHandler)
		{
			int result = 0;

			try
			{
				const int sleepDuration = 10; // in milliseconds

				while (!stop_capture)
				{
				
					Sleep(sleepDuration);
				}
			}
			catch (Spinnaker::Exception &e)
			{
				cout << "Error: " << e.what() << endl;
				result = -1;
			}

			return result;
		}

		int VideoSource_FLIR::PrintDeviceInfo(INodeMap & nodeMap)
		{
			int result = 0;

			cout << endl << "*** DEVICE INFORMATION ***" << endl << endl;

			try
			{
				FeatureList_t features;
				CCategoryPtr category = nodeMap.GetNode("DeviceInformation");
				if (IsAvailable(category) && IsReadable(category))
				{
					category->GetFeatures(features);

					FeatureList_t::const_iterator it;
					for (it = features.begin(); it != features.end(); ++it)
					{
						CNodePtr pfeatureNode = *it;
						cout << pfeatureNode->GetName() << " : ";
						CValuePtr pValue = (CValuePtr)pfeatureNode;
						cout << (IsReadable(pValue) ? pValue->ToString() : "Node not readable");
						cout << endl;
					}
				}
				else
				{
					cout << "Device control information not available." << endl;
				}
			}
			catch (Spinnaker::Exception &e)
			{
				cout << "Error: " << e.what() << endl;
				result = -1;
			}

			return result;
		}

		int VideoSource_FLIR::setHardTrigger(INodeMap & nodeMap, bool onOff) {
			cout << "Setting hardware trigger.." << endl;
			CEnumerationPtr ptrTriggerMode = nodeMap.GetNode("TriggerMode");
			if (!IsAvailable(ptrTriggerMode) || !IsReadable(ptrTriggerMode))
			{
				cout << "Unable to disable trigger mode (node retrieval). Aborting..." << endl;
				return -1;
			}

			CEnumEntryPtr ptrTriggerModeOff = ptrTriggerMode->GetEntryByName("Off");
			if (!IsAvailable(ptrTriggerModeOff) || !IsReadable(ptrTriggerModeOff))
			{
				cout << "Unable to disable trigger mode (enum entry retrieval). Aborting..." << endl;
				return -1;
			}

			ptrTriggerMode->SetIntValue(ptrTriggerModeOff->GetValue());

			cout << "Trigger mode disabled..." << endl;
			
			if (onOff)
			{
				CEnumerationPtr ptrTriggerSource = nodeMap.GetNode("TriggerSource");
				// Set trigger mode to hardware ('Line0')
				CEnumEntryPtr ptrTriggerSourceHardware = ptrTriggerSource->GetEntryByName("Line0");
				if (!IsAvailable(ptrTriggerSourceHardware) || !IsReadable(ptrTriggerSourceHardware))
				{
					cout << "Unable to set trigger mode (enum entry retrieval). Aborting..." << endl;
					return -1;
				}
				ptrTriggerSource->SetIntValue(ptrTriggerSourceHardware->GetValue());
				cout << "Trigger source set to Line0..." << endl;
				//// Test
				//CEnumerationPtr ptrTriggerOverlap = nodeMap.GetNode("TriggerOverlap");
				//CEnumEntryPtr ptrTriggerOverlapReadOut = ptrTriggerOverlap->GetEntryByName("ReadOut");
				//ptrTriggerOverlap->SetIntValue(ptrTriggerOverlapReadOut->GetValue());
				//cout << "Trigger Overlap set to Read Out..." << endl;
			}
			
			// Line Selector
			CEnumerationPtr ptrLineSelector = nodeMap.GetNode("LineSelector");
			if (!IsAvailable(ptrLineSelector) || !IsWritable(ptrLineSelector))
			{
				cout << "Unable to select Line (node retrieval). Aborting..." << endl;
				return -1;
			}
			CEnumEntryPtr ptrLineSelect = ptrLineSelector->GetEntryByName("Line2");
			if (!IsAvailable(ptrLineSelect) || !IsReadable(ptrLineSelect))
			{
				cout << "Unable to set Line Selector mode (enum entry retrieval). Aborting..." << endl;
				return -1;
			}
			ptrLineSelector->SetIntValue(ptrLineSelect->GetValue());
			cout << "Line Selector set to Line 2..." << endl;

			// Line Mode
			CEnumerationPtr ptrLineMode = nodeMap.GetNode("LineMode");
			if (!IsAvailable(ptrLineMode) || !IsWritable(ptrLineMode))
			{
				cout << "Unable to select Line Mode (node retrieval). Aborting..." << endl;
				return -1;
			}
			CEnumEntryPtr ptrModeSelect = ptrLineMode->GetEntryByName("Input");
			if (!IsAvailable(ptrModeSelect) || !IsReadable(ptrModeSelect))
			{
				cout << "Unable to set line mode (enum entry retrieval). Aborting..." << endl;
				return -1;
			}
			ptrLineMode->SetIntValue(ptrModeSelect->GetValue());
			cout << "Line Mode set to Input..." << endl;

			// 3.3V Enable
			CBooleanPtr ptr33VEnable = nodeMap.GetNode("V3_3Enable");
			if (!IsAvailable(ptr33VEnable) || !IsWritable(ptr33VEnable))
			{
				cout << "Unable to select V3.3 Enable (node retrieval). Aborting..." << endl;
				return -1;
			}
			if (onOff)
			{
				ptr33VEnable->SetValue(true);
				cout << "3.3V Enabled..." << endl;
			}
			else
			{
				ptr33VEnable->SetValue(false);
				cout << "3.3V Disable..." << endl;
			}

			// Setting for Line 1
			ptrLineSelector = nodeMap.GetNode("LineSelector");
			if (!IsAvailable(ptrLineSelector) || !IsWritable(ptrLineSelector))
			{
				cout << "Unable to select Line (node retrieval). Aborting..." << endl;
				return -1;
			}
			ptrLineSelect = ptrLineSelector->GetEntryByName("Line1");
			if (!IsAvailable(ptrLineSelect) || !IsReadable(ptrLineSelect))
			{
				cout << "Unable to set Line Selector mode (enum entry retrieval). Aborting..." << endl;
				return -1;
			}
			ptrLineSelector->SetIntValue(ptrLineSelect->GetValue());
			cout << "Line Selector set to Line 1..." << endl;
			ptrLineMode = nodeMap.GetNode("LineMode");
			if (!IsAvailable(ptrLineMode) || !IsWritable(ptrLineMode))
			{
				cout << "Unable to select Line Mode (node retrieval). Aborting..." << endl;
				return -1;
			}
			ptrModeSelect = ptrLineMode->GetEntryByName("Output");
			if (!IsAvailable(ptrModeSelect) || !IsReadable(ptrModeSelect))
			{
				cout << "Unable to set line mode (enum entry retrieval). Aborting..." << endl;
				return -1;
			}
			ptrLineMode->SetIntValue(ptrModeSelect->GetValue());
			cout << "Line Mode set to Output..." << endl;

			// Setting for Line 3
			ptrLineSelector = nodeMap.GetNode("LineSelector");
			if (!IsAvailable(ptrLineSelector) || !IsWritable(ptrLineSelector))
			{
				cout << "Unable to select Line (node retrieval). Aborting..." << endl;
				return -1;
			}
			ptrLineSelect = ptrLineSelector->GetEntryByName("Line3");
			if (!IsAvailable(ptrLineSelect) || !IsReadable(ptrLineSelect))
			{
				cout << "Unable to set Line Selector mode (enum entry retrieval). Aborting..." << endl;
				return -1;
			}
			ptrLineSelector->SetIntValue(ptrLineSelect->GetValue());
			cout << "Line Selector set to Line 3..." << endl;
			ptrLineMode = nodeMap.GetNode("LineMode");
			if (!IsAvailable(ptrLineMode) || !IsWritable(ptrLineMode))
			{
				cout << "Unable to select Line Mode (node retrieval). Aborting..." << endl;
				return -1;
			}
			ptrModeSelect = ptrLineMode->GetEntryByName("Input");
			if (!IsAvailable(ptrModeSelect) || !IsReadable(ptrModeSelect))
			{
				cout << "Unable to set line mode (enum entry retrieval). Aborting..." << endl;
				return -1;
			}
			ptrLineMode->SetIntValue(ptrModeSelect->GetValue());
			cout << "Line Mode set to Input..." << endl;

			m_lastSwitchStatus = false; m_currentSwitchStatus = false; //init status
			if (onOff)
			{
				CEnumerationPtr triggerSelector = nodeMap.GetNode("TriggerSelector");
				triggerSelector->SetIntValue(triggerSelector->GetEntryByName("FrameStart")->GetValue());

				CEnumerationPtr triggerActivation = nodeMap.GetNode("TriggerActivation");
				triggerActivation->SetIntValue(triggerActivation->GetEntryByName("FallingEdge")->GetValue());

				cout << "Trigger source set to hardware..." << endl;

				CEnumEntryPtr ptrTriggerModeOn = ptrTriggerMode->GetEntryByName("On");
				if (!IsAvailable(ptrTriggerModeOn) || !IsReadable(ptrTriggerModeOn))
				{
					cout << "Unable to enable trigger mode (enum entry retrieval). Aborting..." << endl;
					return -1;
				}

				ptrTriggerMode->SetIntValue(ptrTriggerModeOn->GetValue());

				cout << "Trigger mode turned back on..." << endl << endl;
			}
			return 0;
		}

		int VideoSource_FLIR::setGamma(INodeMap &nodeMap, bool enableGamma) {
			int result = 0;
			cout << endl << endl << "*** CONFIGURING Auto Gamma ***" << endl << endl;

			try
			{
				CFloatPtr ptrGamma = nodeMap.GetNode("Gamma");
				if (!IsAvailable(ptrGamma) || !IsWritable(ptrGamma))
				{
					cout << "Unable to set gamma. nAborting..." << endl << endl;
					return -1;
				}
				if (enableGamma) // exposure calibration is based on Gamma 1.0
				{
					ptrGamma->SetValue(1.0);
					cout << "Gamma set 1.0!" << endl;
				}
				else
				{
					ptrGamma->SetValue(0);
					cout << "Gamma disable!" << endl;
				}
			}
			catch (Spinnaker::Exception &e)
			{
				cout << "Error: " << e.what() << endl;
				result = -1;
			}
			return result;
		}

		int VideoSource_FLIR::setAutoGain(INodeMap & nodeMap, bool autoGain) {

			int result = 0;
			cout << endl << endl << "*** CONFIGURING Auto Gain ***" << endl << endl;

			try
			{
				CEnumerationPtr ptrGainAuto = nodeMap.GetNode("GainAuto");
				if (!IsAvailable(ptrGainAuto) || !IsWritable(ptrGainAuto))
				{
					cout << "Unable to set auto gain. Aborting..." << endl << endl;
					return -1;
				}
				if (autoGain)
				{
					CEnumEntryPtr ptrSetGainAutoOn = ptrGainAuto->GetEntryByName("Continuous");
					if (!IsAvailable(ptrSetGainAutoOn) || !IsReadable(ptrSetGainAutoOn))
					{
						cout << "Unable to enable automatic gain (enum entry retrieval). Aborting..." << endl << endl;
						return -1;
					}

					ptrGainAuto->SetIntValue(ptrSetGainAutoOn->GetValue());
					cout << "Auto Gain Enabled!" << endl;
				}
				else
				{
					CEnumEntryPtr ptrSetGainAutoOff = ptrGainAuto->GetEntryByName("Off");
					if (!IsAvailable(ptrSetGainAutoOff) || !IsReadable(ptrSetGainAutoOff))
					{
						cout << "Unable to disable automatic gain (enum entry retrieval). Aborting..." << endl << endl;
						return -1;
					}

					ptrGainAuto->SetIntValue(ptrSetGainAutoOff->GetValue());
					cout << "Auto Gain Disabled!" << endl;
					CFloatPtr ptrGain = nodeMap.GetNode("Gain");
					ptrGain->SetValue(0.0);
					cout << "Gain Set to 0.0!" << endl;
				}
			}
			catch (Spinnaker::Exception &e)
			{
				cout << "Error: " << e.what() << endl;
				result = -1;
			}
			return result;
		}

		int VideoSource_FLIR::setGain(INodeMap & nodeMap) {

			int result = 0;
			cout << endl << endl << "*** Setting Gain ***" << endl << endl;

			try
			{
				CFloatPtr ptrGain = nodeMap.GetNode("Gain");
				if (!IsAvailable(ptrGain) || !IsWritable(ptrGain))
				{
					cout << "Unable to set gain. Aborting..." << endl << endl;
					return -1;
				}
				ptrGain->SetValue(camGain);
				cout << "Gain is set to: " << camGain << endl;
			}
			catch (Spinnaker::Exception &e)
			{
				cout << "Error: " << e.what() << endl;
				result = -1;
			}
			return result;
		}
		////////////
		int VideoSource_FLIR::setAutoExposure(INodeMap & nodeMap, bool autoExp)
		{
			int result = 0;

			try
			{
				if (autoExp) // Turn automatic exposure back on
				{
					CEnumerationPtr ptrExposureAuto = nodeMap.GetNode("ExposureAuto");
					if (!IsAvailable(ptrExposureAuto) || !IsWritable(ptrExposureAuto))
					{
						cout << "Unable to enable automatic exposure (node retrieval). Non-fatal error..." << endl << endl;
						return -1;
					}

					CEnumEntryPtr ptrExposureAutoContinuous = ptrExposureAuto->GetEntryByName("Continuous");
					if (!IsAvailable(ptrExposureAutoContinuous) || !IsReadable(ptrExposureAutoContinuous))
					{
						cout << "Unable to enable automatic exposure (enum entry retrieval). Non-fatal error..." << endl << endl;
						return -1;
					}

					ptrExposureAuto->SetIntValue(ptrExposureAutoContinuous->GetValue());

					// Set Upper and Lower Exposure Limit
					CFloatPtr ptrExposureLower = nodeMap.GetNode("AutoExposureExposureTimeLowerLimit");
					if (!IsAvailable(ptrExposureLower) || !IsReadable(ptrExposureLower))
					{
						cout << "Unable to enable AutoExposureExposureTimeLowerLimit (enum entry retrieval). Non-fatal error..." << endl << endl;
						return -1;
					}

					ptrExposureLower->SetValue(EXP_LOWER_LIMIT);
					cout << "Exposure Time Lower Limit is set to: " << EXP_LOWER_LIMIT << endl;

					CFloatPtr ptrExposureUpper = nodeMap.GetNode("AutoExposureExposureTimeUpperLimit");
					if (!IsAvailable(ptrExposureUpper) || !IsReadable(ptrExposureUpper))
					{
						cout << "Unable to enable AutoExposureExposureTimeUpperLimit (enum entry retrieval). Non-fatal error..." << endl << endl;
						return -1;
					}

					ptrExposureUpper->SetValue(EXP_UPPER_LIMIT);
					cout << "Exposure Time Upper Limit is set to: " << EXP_UPPER_LIMIT << endl;
					cout << "Automatic exposure enabled..." << endl << endl;
				}
				else // Turn off automatic exposure mode
				{
					CEnumerationPtr ptrExposureAuto = nodeMap.GetNode("ExposureAuto");
					if (!IsAvailable(ptrExposureAuto) || !IsWritable(ptrExposureAuto))
					{
						cout << "Unable to disable automatic exposure (node retrieval). Aborting..." << endl << endl;
						return -1;
					}

					CEnumEntryPtr ptrExposureAutoOff = ptrExposureAuto->GetEntryByName("Off");
					if (!IsAvailable(ptrExposureAutoOff) || !IsReadable(ptrExposureAutoOff))
					{
						cout << "Unable to disable automatic exposure (enum entry retrieval). Aborting..." << endl << endl;
						return -1;
					}

					ptrExposureAuto->SetIntValue(ptrExposureAutoOff->GetValue());
					cout << "Automatic exposure disabled..."<< endl;
				}
			}
			catch (Spinnaker::Exception &e)
			{
				cout << "Error: " << e.what() << endl;
				result = -1;
			}
			return result;
		}

		void VideoSource_FLIR::checkSwitchStatus(Spinnaker::Camera* cam)
		{
			// Retrieve GenICam nodemap
			INodeMap & nodeMap = cam->GetNodeMap();
			// Setting for Line 3
			CEnumerationPtr ptrLineSelector = nodeMap.GetNode("LineSelector");
			if (!IsAvailable(ptrLineSelector) || !IsWritable(ptrLineSelector))
				cout << "Unable to select Line (node retrieval). " << endl;

			CEnumEntryPtr ptrLineSelect = ptrLineSelector->GetEntryByName("Line3");
			if (!IsAvailable(ptrLineSelect) || !IsReadable(ptrLineSelect))
				cout << "Unable to set Line Selector mode (enum entry retrieval). " << endl;

			ptrLineSelector->SetIntValue(ptrLineSelect->GetValue());
			CBooleanPtr ptrLineStatus = nodeMap.GetNode("LineStatus");
			if (!IsAvailable(ptrLineStatus))
				cout << "Unable to read LineStatus (node retrieval). " << endl;

			m_currentSwitchStatus = !ptrLineStatus->GetValue();
			if (m_currentSwitchStatus && !m_lastSwitchStatus) // first frame trigger detected
			{
				CCommandPtr ptrResetTimeStamp = nodeMap.GetNode("TimestampReset");
				if (!IsAvailable(ptrResetTimeStamp) || !IsWritable(ptrResetTimeStamp))
				{
					cout << "Unable to reset Timestamp." << endl << endl;
				}
				ptrResetTimeStamp->Execute();
				cout << "Timestamp reset..." << endl << endl;
				m_lastSwitchStatus = m_currentSwitchStatus;
			}
			else if (!m_currentSwitchStatus) // switch not pressed
				m_lastSwitchStatus = m_currentSwitchStatus;
		}

		int VideoSource_FLIR::setFrameRate(INodeMap & nodeMap, float fps)
		{
			// Retrieve GenICam nodemap
			CBooleanPtr ptrFrameRateEnable = nodeMap.GetNode("AcquisitionFrameRateEnable");
			if (!IsAvailable(ptrFrameRateEnable) || !IsWritable(ptrFrameRateEnable))
			{
				cout << "Unable to select Frame Rate Enable. " << endl;
				return false;
			}
			else
			{
				ptrFrameRateEnable->SetValue(true);
				cout << "AcquisitionFrameRateEnable set to True" << endl;
			}
			// Set AcquisitionFrameRate to FPS
			CFloatPtr AcquisitionFrameRateNode = nodeMap.GetNode("AcquisitionFrameRate");
			if (!IsAvailable(AcquisitionFrameRateNode) || !IsWritable(AcquisitionFrameRateNode))
			{
				cout << "Unable to set AcquisitionFrameRate to Max. Aborting..." << endl << endl;
				return false;
			}
			else
			{
				AcquisitionFrameRateNode->SetValue(fps);
				cout << "Frame Rate set to " << fps << endl;
			}
			return true;
		}

		bool VideoSource_FLIR::getSkipFrame()
		{
			return skipFrame;
		}
		void VideoSource_FLIR::setSkipFrame(bool isSkipFrame)
		{
			skipFrame = isSkipFrame;
		}
		void VideoSource_FLIR::setNumSkipFrame(int numFrame)
		{
			numSkipFrame = numFrame;
		}
		int VideoSource_FLIR::getNumSkipFrame()
		{
			return numSkipFrame;
		}

		int VideoSource_FLIR::setChunkEnable(INodeMap & nodeMap, bool onOff)
		{
			CBooleanPtr ptrChunkModeActive = nodeMap.GetNode("ChunkModeActive"); 
			if (!IsAvailable(ptrChunkModeActive) || !IsWritable(ptrChunkModeActive)) 
			{ 
				cout << "Unable to activate chunk mode. Aborting..." << endl << endl;
			}
			ptrChunkModeActive->SetValue(onOff);
			cout << endl << "Chunk Mode Active Set!" << endl;
			// Retrieve the selector node
			NodeList_t entries;
			CEnumerationPtr ptrChunkSelector = nodeMap.GetNode("ChunkSelector");

			if (!IsAvailable(ptrChunkSelector) || !IsReadable(ptrChunkSelector))
			{
				cout << "Unable to retrieve chunk selector. Aborting..." << endl << endl;
				return -1;
			}
			ptrChunkSelector->GetEntries(entries);
			CEnumEntryPtr ptrChunkSelectorEntry = entries.at(7); // ExposureTime
			ptrChunkSelector->SetIntValue(ptrChunkSelectorEntry->GetValue());

			CBooleanPtr ptrChunkEnable = nodeMap.GetNode("ChunkEnable");
			if (!IsAvailable(ptrChunkEnable) || !IsWritable(ptrChunkEnable))
			{
				cout << "Unable to enable Exposure Time chunk mode. Aborting..." << endl << endl;
			}
			ptrChunkEnable->SetValue(onOff);
			cout << "Exposure Time Chunk Mode Enable Set!" << endl;
			ptrChunkSelectorEntry = entries.at(11); // Timestamp
			ptrChunkSelector->SetIntValue(ptrChunkSelectorEntry->GetValue());

			ptrChunkEnable = nodeMap.GetNode("ChunkEnable");
			if (!IsAvailable(ptrChunkEnable) || !IsWritable(ptrChunkEnable))
			{
				cout << "Unable to enable Timestamp chunk mode. Aborting..." << endl << endl;
			}
			ptrChunkEnable->SetValue(onOff);
			cout << "Timestamp Chunk Mode Enable Set!" << endl;
			ptrChunkSelectorEntry = entries.at(2); // ChunkID
			ptrChunkSelector->SetIntValue(ptrChunkSelectorEntry->GetValue());

			ptrChunkEnable = nodeMap.GetNode("ChunkEnable");
			if (!IsAvailable(ptrChunkEnable) || !IsWritable(ptrChunkEnable))
			{
				cout << "Unable to enable ChunkID chunk mode. Aborting..." << endl << endl;
			}
			ptrChunkEnable->SetValue(onOff);
			cout << "ChunkID Chunk Mode Enable Set!" << endl;
			return 0;
		}

		int VideoSource_FLIR::setGIGEPacketSize(INodeMap & nodeMap)
		{
			int PacketSizeToSet = 1400;
			int PacketDelayToSet = 10000;
			int DeviceThroughputLimit = 115000000;
			//int DeviceThroughputLimit = 31250000; // for 640 x 512
			try {
				cout << "Setting the Packet Size to: " << PacketSizeToSet << endl << endl;
				CIntegerPtr PacketSizeNode = nodeMap.GetNode("GevSCPSPacketSize");
				if (!IsAvailable(PacketSizeNode) || !IsWritable(PacketSizeNode)) {
					cout << "Unable to set Packet Size to: " << PacketSizeToSet << ". Aborting..." << endl << endl;
				}
				PacketSizeNode->SetValue(PacketSizeToSet);
				cout << "Setting the Packet Delay to: " << PacketDelayToSet << endl << endl;
				CIntegerPtr PacketDelayNode = nodeMap.GetNode("GevSCPD");
				if (!IsAvailable(PacketSizeNode) || !IsWritable(PacketSizeNode))
				{
					cout << "Unable to set Packet Delay to: " << PacketDelayToSet << ". Aborting..." << endl << endl;
				}
				PacketDelayNode->SetValue(PacketDelayToSet);
				cout << "Setting the Device Link Throughput Limit to: " << DeviceThroughputLimit << endl << endl;
				CIntegerPtr DeviceThroughputLimitNode = nodeMap.GetNode("DeviceLinkThroughputLimit");
				if (!IsAvailable(DeviceThroughputLimitNode) || !IsWritable(DeviceThroughputLimitNode))
				{
					cout << "Unable to set  Device Link Throughput Limit to: " << DeviceThroughputLimit << ". Aborting..." << endl << endl;
				}
				DeviceThroughputLimitNode->SetValue(DeviceThroughputLimit);
			}
			catch (Spinnaker::Exception &e) 
			{ 
				cout << "Exception setting packet size to " << PacketSizeToSet << ". Exception: " << e.what() << endl; 
			}
			return 1;
		}

		int VideoSource_FLIR::setBuffer(INodeMap & nodeMap)
		{
			try {
				cout << "Setting Stream Buffer Count Mode to: Auto... ";
				CEnumerationPtr ptrStreamBufferCountMode = nodeMap.GetNode("StreamBufferCountMode");
				NodeList_t entries;
				ptrStreamBufferCountMode->GetEntries(entries);
				CEnumEntryPtr ptrBufferMode = entries.at(1); // Auto
				if (!IsAvailable(ptrStreamBufferCountMode) || !IsWritable(ptrStreamBufferCountMode))
				{
					cout << "Unable to set Stream Buffer Mode to: Auto. Aborting..." << endl << endl;
				}
				ptrStreamBufferCountMode->SetIntValue(ptrBufferMode->GetValue());
				cout << "Done! " << endl << endl;
				cout << "Setting Stream BufferHandling Mode to: Oldest First... ";
				CEnumerationPtr ptrStreamBufferHandlingMode = nodeMap.GetNode("StreamBufferHandlingMode");
				if (!IsAvailable(ptrStreamBufferHandlingMode) || !IsReadable(ptrStreamBufferHandlingMode))
				{
					cout << "Unable to set Stream BufferHandling Mode to: Oldest First. Aborting..." << endl << endl;
				}
				ptrStreamBufferHandlingMode->GetEntries(entries);
				CEnumEntryPtr ptrHandlingMode = entries.at(0); // Oldest First

				ptrStreamBufferHandlingMode->SetIntValue(ptrHandlingMode->GetValue());
				cout << "Done! " << endl << endl;

				CBooleanPtr ptrGevPacketResendMode = nodeMap.GetNode("GevPacketResendMode");
				if (!IsAvailable(ptrGevPacketResendMode) || !IsWritable(ptrGevPacketResendMode))
				{
					cout << "Unable to deactivate GevPacketResendMode. Aborting..." << endl << endl;
				}
				ptrGevPacketResendMode->SetValue(false);
				cout << endl << "GevPacketResendMode deactivated!" << endl;
			}
			catch (Spinnaker::Exception &e)
			{
				cout << "Exception setting Buffer Mode. Exception: " << e.what() << endl;
			}
			return 1;
		}

		void VideoSource_FLIR::setLastFrameID(int frameID)
		{
			lastFrameID = frameID;
		}

		int VideoSource_FLIR::getLastFrameID()
		{
			return lastFrameID;
		}

		int VideoSource_FLIR::getCameraCableStatus()
		{
			return CameraCableStatus;
		}

		void VideoSource_FLIR::setCameraCableStatus(int cable_status)
		{
			// cable_status = 0 - cable is connected, cable_status = 1 - cable is disconnected, cable_status = 2 - cable is reconnected
			CameraCableStatus = cable_status;
		}

		int VideoSource_FLIR::setImageBinning(INodeMap & nodeMap, bool doBinning)
		{
			int binningFactor = 2; // image resolution reduced by half
			try {
				// Retrieve the selector node
				NodeList_t entries;
				CEnumerationPtr ptrBinningHorizontalMode = nodeMap.GetNode("BinningHorizontalMode");
				if (!IsAvailable(ptrBinningHorizontalMode) || !IsReadable(ptrBinningHorizontalMode))
				{
					cout << "Unable to retrieve Binning Horizontal Mode. Aborting..." << endl << endl;
					return -1;
				}
				ptrBinningHorizontalMode->GetEntries(entries);
				CEnumEntryPtr ptrModeSelectorEntry = entries.at(1); // Average Mode
				ptrBinningHorizontalMode->SetIntValue(ptrModeSelectorEntry->GetValue());
				CEnumerationPtr ptrBinningVerticalMode = nodeMap.GetNode("BinningVerticalMode");
				if (!IsAvailable(ptrBinningVerticalMode) || !IsReadable(ptrBinningVerticalMode))
				{
					cout << "Unable to retrieve Binning Vertical Mode. Aborting..." << endl << endl;
					return -1;
				}
				ptrBinningVerticalMode->GetEntries(entries);
				ptrModeSelectorEntry = entries.at(1); // Average Mode
				ptrBinningVerticalMode->SetIntValue(ptrModeSelectorEntry->GetValue());

				CIntegerPtr BinningHorizontalNode = nodeMap.GetNode("BinningHorizontal");
				CIntegerPtr BinningVerticalNode = nodeMap.GetNode("BinningVertical");
				if (doBinning) // reduce resolution by half
				{
					BinningHorizontalNode->SetValue(binningFactor);
					BinningVerticalNode->SetValue(binningFactor);
				}
				else
				{
					BinningHorizontalNode->SetValue(1);
					BinningVerticalNode->SetValue(1);
					CIntegerPtr ImageWidthNode = nodeMap.GetNode("Width");
					CIntegerPtr ImageHeightNode = nodeMap.GetNode("Height");
					ImageWidthNode->SetValue(1280);
					ImageHeightNode->SetValue(1024);
				}

			}
			catch (Spinnaker::Exception &e)
			{
				cout << "Exception setting image binning to " << binningFactor << ". Exception: " << e.what() << endl;
				cin.ignore();
				return -1;
			}
			cout << "Binning set done" << endl;
			//Sleep(10000);
			return 1;
		}
	}
}
