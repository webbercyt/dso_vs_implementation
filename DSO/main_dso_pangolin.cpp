/**
* This file is part of DSO.
*
* Copyright 2016 Technical University of Munich and Intel.
* Developed by Jakob Engel <engelj at in dot tum dot de>,
* for more information see <http://vision.in.tum.de/dso>.
* If you use this code, please cite the respective publications as
* listed on the above website.
*
* DSO is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* DSO is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with DSO. If not, see <http://www.gnu.org/licenses/>.
*/


#include <thread>
#include <locale.h>
#include <signal.h>
#include <stdlib.h>
#include <stdio.h>

//windows mod:Yang
#if defined WIN32 || _WIN32
#define NOMINMAX
#define usleep(TIME) std::this_thread::sleep_for(std::chrono::microseconds(TIME))
#define _timeval dso::timeval
#else
#define _timeval timeval
#endif

/////////////////////
#include "IOWrapper/Output3DWrapper.h"
#include "IOWrapper/ImageDisplay.h"


#include <boost/thread.hpp>
#include "util/settings.h"
#include "util/globalFuncs.h"
#include "util/DatasetReader.h"
#include "util/globalCalib.h"

#include "util/NumType.h"
#include "FullSystem/FullSystem.h"
#include "OptimizationBackend/MatrixAccumulators.h"
#include "FullSystem/PixelSelector2.h"
#include "INIReader/INIReader.h"



#include "IOWrapper/OutputWrapper/SocketWrapper.h"
#include "IOWrapper/Pangolin/PangolinDSOViewer.h"
#include "IOWrapper/OutputWrapper/SampleOutputWrapper.h"

int channel = 0;

std::string path = "";
std::string imageFile = "";
std::string timeFile = "";
std::string vignetteFile = "";
std::string gammaCalibFile = "";
std::string calibFile = "";
std::string camType = "";
std::string camSerial = "";


std::string settingFilePath = "C:/cinetec/Saturn_2.5/data/cinetec/ctcam2/";
std::string commonSettingFile = "common.ini";
std::string readerSettingFile = "";

double rescale = 1;
bool dso_reverse = false;
bool disableROS = false;
int dso_start = 0;
int dso_end = 100000;
bool prefetch = false;
float playbackSpeed = 0;	// 0 for linearize (play as fast as possible, while sequentializing tracking & mapping). otherwise, factor on timestamps.
bool preload = false;
bool useSampleOutput = false;
bool useCamera = false;
bool skipFrame = true;
bool hardTrigger = false;
float camFPS = 25;
float camGain = 0.0;

int cache_length = 25;
int loop = 1;
bool is_new_loop = true;

int mode = 0;

bool firstRosSpin = false;
ImageFolderReader* reader = NULL;
FullSystem* fullSystem = NULL;
IOWrap::PangolinDSOViewer* viewer = NULL;
std::vector<FrameHessian*> frameHessians;
std::mutex hessianMutex;
bool can_exit = false;
bool exit_system = false;
bool exit_load_images = false;
bool load_current_image = false;

using namespace dso;


void my_exit_handler(int s)
{
	//windows mod:Yang
#if defined WIN32||_WIN32
	printf("got win sigal %d\n", s);
	can_exit = true;
	while (!exit_system && !exit_load_images) _sleep(10);

	reader->close();
	if(viewer) viewer->close();
	if(fullSystem) delete fullSystem;
	if(reader) delete reader;
	if(viewer) delete viewer;

	exit(1);
#else
	can_exit = true;
	printf("Caught signal %d\n", s);
	while (!exit_system && !exit_load_images) sleep(10);
	exit(1);
#endif
}

void exitThread()
{
	//windows mod:Yang
#if defined WIN32||_WIN32
	signal(SIGINT, my_exit_handler);
#else
	struct sigaction sigIntHandler;
	sigIntHandler.sa_handler = my_exit_handler;
	sigemptyset(&sigIntHandler.sa_mask);
	sigIntHandler.sa_flags = 0;
	sigaction(SIGINT, &sigIntHandler, NULL);

	firstRosSpin = true;
	while (true) pause();
#endif
}



void settingsDefault(int preset)
{
	printf("\n=============== PRESET Settings: ===============\n");
	if (preset == 0 || preset == 1)
	{
		printf("DEFAULT settings:\n"
			"- %s real-time enforcing\n"
			"- 2000 active points\n"
			"- 5-7 active frames\n"
			"- 1-6 LM iteration each KF\n"
			"- original image resolution\n", preset == 0 ? "no " : "1x");

		playbackSpeed = (preset == 0 ? 0 : 1);
		preload = preset == 1;
		setting_desiredImmatureDensity = 800;
		setting_desiredPointDensity =1000;
		setting_minFrames = 5;
		setting_maxFrames = 7;
		setting_maxOptIterations = 6;
		setting_minOptIterations = 1;

		setting_logStuff = false;
	}

	if (preset == 2 || preset == 3)
	{
		printf("FAST settings:\n"
			"- %s real-time enforcing\n"
			"- 800 active points\n"
			"- 4-6 active frames\n"
			"- 1-4 LM iteration each KF\n"
			"- 424 x 320 image resolution\n", preset == 0 ? "no " : "5x");

		playbackSpeed = (preset == 2 ? 0 : 5);
		preload = preset == 3;
		setting_desiredImmatureDensity = 600;
		setting_desiredPointDensity = 800;
		setting_minFrames = 4;
		setting_maxFrames = 6;
		setting_maxOptIterations = 4;
		setting_minOptIterations = 1;

		benchmarkSetting_width = 424;
		benchmarkSetting_height = 320;

		setting_logStuff = false;
	}

	printf("==============================================\n");
}






void parseArgument(char* arg)
{
	char buf[1000];

	if (1 == sscanf(arg, "settingPath=%s", buf))
	{
		settingFilePath = buf;
		printf("settingPath %s!\n", settingFilePath.c_str());
		return;
	}

	if (1 == sscanf(arg, "commonSetting=%s", buf))
	{
		commonSettingFile = buf;
		printf("commonFile %s!\n", commonSettingFile.c_str());
		return;
	}

	if (1 == sscanf(arg, "readerSetting=%s", buf))
	{
		readerSettingFile = buf;
		printf("settingFile %s!\n", readerSettingFile.c_str());
		return;
	}
	
	printf("could not parse argument \"%s\"!!!!\n", arg);
}

void parseArgument()
{
	string section = "";
	int optINT;
	bool optBOOL;
	float optFLT;
	string optSTR;

	INIReader* commonSettings = new INIReader(settingFilePath + commonSettingFile);
	if (commonSettings->ParseError() < 0)
	{
		printf("Error: failed to read common setting file. \n");
		return;
	}

	if (commonSettings->getBool(section, "sampleoutput"))
	{
		useSampleOutput = true;
		printf("USING SAMPLE OUTPUT WRAPPER!\n");
	}

	if (commonSettings->getBool(section, "quiet"))
	{
		setting_debugout_runquiet[0] = true;
		printf("QUIET MODE, I'll shut up!\n");
	}

	if (commonSettings->getBool(section, "nolog"))
	{
		setting_logStuff = false;
		printf("DISABLE LOGGING!\n");
	}

	if ((optFLT = commonSettings->getFloat(section, "speed")) != INI_DEFAULT_FLT)
	{
		playbackSpeed = optFLT;
		printf("PLAYBACK SPEED %f!\n", playbackSpeed);
	}

	if ((optINT = commonSettings->getInt(section, "preset")) != INI_DEFAULT_INT)
		settingsDefault(optINT);

	if (commonSettings->getBool(section, "useProfile"))
	{
		setting_useProfile[0] = true;
		printf("Using profile!\n");
	}

	if (commonSettings->getInt(section, "skipFrame") == 0)
	{
		skipFrame = false;
		printf("Skip frame!\n");
	}

	if ((optINT = commonSettings->getInt(section, "cacheLength")) > 0)
	{
		cache_length = optINT;
		printf("Set Cache Length!\n");
	}

	if (commonSettings->getInt(section, "rec") == 0)
	{
		disableReconfigure = true;
		printf("DISABLE RECONFIGURE!\n");
	}

	if (commonSettings->getBool(section, "noros"))
	{
		disableROS = true;
		disableReconfigure = true;
		printf("DISABLE ROS (AND RECONFIGURE)!\n");
	}

	if (commonSettings->getBool(section, "reverse"))
	{
		dso_reverse = true;
		printf("REVERSE!\n");
	}

	if (commonSettings->getBool(section, "nogui"))
	{
		disableAllDisplay = true;
		printf("NO GUI!\n");
	}

	if (commonSettings->getBool(section, "nomt"))
	{
		multiThreading = false;
		printf("NO MultiThreading!\n");
	}

	if (commonSettings->getBool(section, "prefetch"))
	{
		prefetch = true;
		printf("PREFETCH!\n");
	}

	if ((optINT = commonSettings->getInt(section, "start")) != INI_DEFAULT_INT)
	{
		dso_start = optINT;
		printf("START AT %d!\n", dso_start);
	}

	if ((optINT = commonSettings->getInt(section, "end")) != INI_DEFAULT_INT)
	{
		dso_end = optINT;
		printf("END AT %d!\n", dso_end);
	}

	if ((optINT = commonSettings->getInt(section, "rescale")) != INI_DEFAULT_INT)
	{
		rescale = optINT;
		printf("RESCALE %f!\n", rescale);
	}

	if (commonSettings->getBool(section, "save"))
	{
		debugSaveImages = true;
		if (42 == system("rm -rf images_out")) printf("system call returned 42 - what are the odds?. This is only here to shut up the compiler.\n");
		if (42 == system("mkdir images_out")) printf("system call returned 42 - what are the odds?. This is only here to shut up the compiler.\n");
		printf("SAVE IMAGES!\n");
	}

	if ((optINT = commonSettings->getInt(section, "mode")) != INI_DEFAULT_INT)
	{
		mode = optINT;
		if (mode == 0)
		{
			printf("PHOTOMETRIC MODE WITH CALIBRATION!\n");
		}
		if (mode == 1)
		{
			printf("PHOTOMETRIC MODE WITHOUT CALIBRATION!\n");
			setting_photometricCalibration = 0;
			setting_affineOptModeA = 0; //-1: fix. >=0: optimize (with prior, if > 0).
			setting_affineOptModeB = 0; //-1: fix. >=0: optimize (with prior, if > 0).
		}
		if (mode == 2)
		{
			printf("PHOTOMETRIC MODE WITH PERFECT IMAGES!\n");
			setting_photometricCalibration = 0;
			setting_affineOptModeA = -1; //-1: fix. >=0: optimize (with prior, if > 0).
			setting_affineOptModeB = -1; //-1: fix. >=0: optimize (with prior, if > 0).
			setting_minGradHistAdd = 3;
		}
	}

	if ((optINT = commonSettings->getInt(section, "gc_ON")) != INI_DEFAULT_INT)
	{
		setting_GC_ON = optINT;
		setting_GC_ON ? printf("turn on ground calibrator\n") : printf("turn off ground calibrator\n");
	}

	if ((optINT = commonSettings->getInt(section, "gc_boardType")) != INI_DEFAULT_INT)
	{
		setting_GCBoardType = optINT;
		printf("use board type = %d\n", setting_GCBoardType);
	}

	if ((optFLT = commonSettings->getFloat(section, "gc_charuco_3x3_squareLength")) > 0)
	{
		setting_GCChArUco_3X3_SquareLength = optFLT;
		printf("board width = %f\n", setting_GCChArUco_3X3_SquareLength);
	}

	if ((optFLT = commonSettings->getFloat(section, "gc_markerLength")) != INI_DEFAULT_FLT)
	{
		setting_GCMarkerLength = optFLT;
		printf("marker length = %f\n", setting_GCMarkerLength);
	}

	if ((optFLT = commonSettings->getFloat(section, "gc_markerWidth")) > 0)
	{
		setting_GCMarkerSize[0] = optFLT;
		printf("marker width = %f\n", setting_GCMarkerSize[0]);
	}

	if ((optFLT = commonSettings->getFloat(section, "gc_markerHeight")) > 0)
	{
		setting_GCMarkerSize[1] = optFLT;
		printf("marker height = %f\n", setting_GCMarkerSize[1]);
	}

	if ((optFLT = commonSettings->getFloat(section, "gc_smallMarkerLength")) > 0)
	{
		setting_GCSmallMarkerLength = optFLT;
		printf("small marker length = %f\n", setting_GCSmallMarkerLength);
	}

	if ((optINT = commonSettings->getInt(section, "gc_maxFrames")) > 0)
	{
		setting_GCMaxFrames = optINT;
		printf("max ground calibration frames = %d\n", setting_GCMaxFrames);
	}

	if ((optINT = commonSettings->getInt(section, "gc_minCornerCount")) > 0)
	{
		setting_GCMinCornerCount = optINT;
		printf("single marker corner needed = %d\n", setting_GCMinCornerCount);
	}

	if ((optFLT = commonSettings->getFloat(section, "gc_errorTH")) != INI_DEFAULT_FLT)
	{
		setting_GCErrorTH = optFLT;
		printf("error threshold of PC calibration= %f\n", setting_GCErrorTH);
	}

	if ((optFLT = commonSettings->getFloat(section, "corner3DPtOptTH")) > 0)
	{
		setting_GCIntPtOptTH = optFLT;
		printf("board corner optimization threshold = %f\n", setting_GCIntPtOptTH);
	}

	if ((optINT = commonSettings->getInt(section, "lostTracker_ON")) != INI_DEFAULT_INT)
	{
		setting_lostTrackerON = optINT;
		setting_lostTrackerON ? printf("turn on lost tracker\n") : printf("turn off lost tracker\n");
	}

	if ((optINT = commonSettings->getInt(section, "lostTracker_backupFrames")) != INI_DEFAULT_INT)
	{
		setting_lostTrackerBackupFrames = optINT;
		printf("max lost track frame: %d\n", setting_lostTrackerBackupFrames);
	}

	delete commonSettings;



	INIReader* readerSettings = new INIReader(settingFilePath + readerSettingFile);
	if (readerSettings->ParseError() < 0)
	{
		printf("Error: failed to read image reader setting file. \n");
		return;
	}

	if (optBOOL = readerSettings->getBool(section, "useCamera"))
	{
		useCamera = optBOOL;
		printf("Using Camera!\n");
	}

	if ((optSTR = readerSettings->getStr(section, "path")) != INI_DEFAULT_STR)
	{
		path = settingFilePath + optSTR;
		printf("data path %s!\n", path.c_str());
	}

	if ((optINT = readerSettings->getInt(section, "loop")) != INI_DEFAULT_INT)
	{
		loop = optINT;
		printf("Loop %d times!\n", loop);
	}

	if (readerSettings->getBool(section, "hardTrigger"))
	{
		hardTrigger = true;
		printf("Use Hardware Trigger!\n");
	}

	if ((optSTR = readerSettings->getStr(section, "imageFile")) != INI_DEFAULT_STR)
	{
		imageFile = optSTR;
		printf("loading images from %s!\n", imageFile.c_str());
	}

	if ((optSTR = readerSettings->getStr(section, "timeFile")) != INI_DEFAULT_STR)
	{
		timeFile = optSTR;
		printf("loading times from %s!\n", timeFile.c_str());
	}

	if ((optSTR = readerSettings->getStr(section, "calibFile")) != INI_DEFAULT_STR)
	{
		calibFile = optSTR;
		printf("loading calibration from %s!\n", calibFile.c_str());
	}

	if ((optSTR = readerSettings->getStr(section, "vignetteFile")) != INI_DEFAULT_STR)
	{
		vignetteFile = optSTR;
		printf("loading vignette from %s!\n", vignetteFile.c_str());
	}

	if ((optSTR = readerSettings->getStr(section, "gammaCalibFile")) != INI_DEFAULT_STR)
	{
		gammaCalibFile = optSTR;
		printf("loading gamma calibration from %s!\n", gammaCalibFile.c_str());
	}

	if ((optSTR = readerSettings->getStr(section, "camSerial")) != INI_DEFAULT_STR)
	{
		camSerial = optSTR;
		printf("Use camera serial number: %s!\n", camSerial.c_str());
	}

	if ((optSTR = readerSettings->getStr(section, "camType")) != INI_DEFAULT_STR)
	{
		camType = optSTR;
		printf("loading camera type %s!\n", camType.c_str());
	}

	if ((optSTR = readerSettings->getStr(section, "camType")) != INI_DEFAULT_STR)
	{
		camType = optSTR;
		printf("loading camera type %s!\n", camType.c_str());
	}

	if ((optFLT = readerSettings->getFloat(section, "camFPS")) != INI_DEFAULT_FLT)
	{
		camFPS = optFLT;
		printf("loading camera FPS: %f!\n", optFLT);
	}

	delete readerSettings;
}

void resetFullSystem()
{
	hessianMutex.lock();

	load_current_image = true;

	std::vector<IOWrap::Output3DWrapper*> wraps = fullSystem->outputWrapper;
	delete fullSystem;

	for (IOWrap::Output3DWrapper* ow : wraps)
		ow->reset();

	fullSystem = new FullSystem(channel);
	fullSystem->setGammaFunction(reader->getPhotometricGamma());
	fullSystem->linearizeOperation = (playbackSpeed == 0);

	fullSystem->outputWrapper = wraps;
	if (camType == "flir")
		reader->videoS->setNumSkipFrame(0);


	setting_groundRotate[channel] = Sophus::Matrix4f::Identity();
	setting_groundOffset[channel] = Sophus::Matrix4f::Zero();
	setting_groundScale[channel] = Sophus::Matrix4f::Identity();

	hessianMutex.unlock();
}

void runCamera()
{
	reader = new ImageFolderReader(channel, path, calibFile, gammaCalibFile, vignetteFile, skipFrame, camType, hardTrigger,camSerial, camFPS, camGain);
	reader->setGlobalCalibration();
	
	if (setting_photometricCalibration > 0 && reader->getPhotometricGamma() == 0)
	{
		printf("ERROR: dont't have photometric calibation. Need to use commandline options mode=1 or mode=2 ");
		exit(1);
	}

	fullSystem = new FullSystem(channel);
	fullSystem->setGammaFunction(reader->getPhotometricGamma());
	fullSystem->linearizeOperation = (playbackSpeed == 0);


	// viewer wrapper
	if (!disableAllDisplay)
	{
		viewer = new IOWrap::PangolinDSOViewer(0, globalCalibs[channel].wG[0], globalCalibs[channel].hG[0], false);
		fullSystem->outputWrapper.push_back(viewer);
	}
	// sample wrapper
	if (useSampleOutput)
		fullSystem->outputWrapper.push_back(new IOWrap::SampleOutputWrapper());
	// UDP socket wrapper
	IOWrap::SocketWrapper* socket_wraper = new IOWrap::SocketWrapper(channel);
	socket_wraper->initSocket();
	fullSystem->outputWrapper.push_back(socket_wraper);

	std::thread runthreadLoadImages = std::thread([&]() {
		int id = 0;
		while (!can_exit)
		{
			//if (fullSystem->initialized && !fullSystem->isLost && !setting_fullResetRequested[channel])
			{
				if (load_current_image)
				{
					reader->videoS->mtx.lock();
					id = reader->videoS->images.rbegin()->first;
					reader->videoS->mtx.unlock();
					load_current_image = false;
				}
				ImageAndExposure* img = reader->getImage_external(id, 0);
				if (camType == "flir")
				{
					if (reader->videoS->getSkipFrame())
					{
						socket_wraper->setSkipFrame(true);
						socket_wraper->setNumSkipFrame(reader->videoS->getNumSkipFrame());
					}
					else
						socket_wraper->setSkipFrame(false);
				}

				if (img == NULL) continue;

				hessianMutex.lock();
				FrameHessian* fh = fullSystem->makeImages(img);
				hessianMutex.unlock();
				delete img;
				if (fh == NULL) continue;

				hessianMutex.lock();
				if (frameHessians.size() > cache_length)
				{
					int remove_id = rand() % frameHessians.size();
					delete frameHessians[remove_id];
					frameHessians.erase(frameHessians.begin() + remove_id);
					printf("frameHessians.size() > cache_length, frameHessians.size(): %zd\n", frameHessians.size());
				}
				frameHessians.push_back(fh);
				hessianMutex.unlock();

				if (!setting_debugout_runquiet[channel]) printf("runthreadLoadImages(): frame id=%d\n", id);
				id++;
			}
		}
	});

	
	// to make MacOS happy: run this in dedicated thread -- and use this one to run the GUI.
	std::thread runthread([&]() {

		struct _timeval tv_start;
		gettimeofday(&tv_start, NULL);
		clock_t started = clock();

		int id = 0;
		while (!can_exit)
		{
			clock_t start = clock();

			hessianMutex.lock();
			int fsize = frameHessians.size();
			hessianMutex.unlock();
			if (fsize > 0)
			{
				if (setting_useProfile[channel]) printf("main():for start!\n");

				clock_t start = clock();

				hessianMutex.lock();
				fullSystem->addActiveFrame(frameHessians.front(), id);
				frameHessians.erase(frameHessians.begin());
				hessianMutex.unlock();

				if (setting_useProfile[channel]) {
					clock_t end = clock();
					double duration = (double)(end - start) / CLOCKS_PER_SEC;
					printf("main():addActiveFrame(): id=%d  %f ms\n", id, duration * 1000.0f);
				}
			}

			clock_t start3 = clock();
			//socket_wraper->run(fullSystem->initialized ? 0 : 2);
			if (fullSystem->initialized && fullSystem->GroundInitialized)
				socket_wraper->run(0);
			else if (fullSystem->initialized && !fullSystem->GroundInitialized)
				socket_wraper->run(1);
			else
				socket_wraper->run(2);


			if (fullSystem->initFailed || setting_fullResetRequested[channel]/* || fullSystem->needLostReset()*/)
			{
				//if (id < 250 || setting_fullResetRequested[channel] || fullSystem->needLostReset())
				{
					printf("RESETTING!\n");
					resetFullSystem();
					setting_fullResetRequested[channel] = false;
					id = 0;
				}
			}
			if (setting_useProfile[channel]) {
				clock_t end3 = clock();
				double duration3 = (double)(end3 - start3) / CLOCKS_PER_SEC;
				printf("main():RESETTING: id=%d  %f ms\n", id, duration3 * 1000.0f);
			}

			if (setting_useProfile[channel]) {
				clock_t end = clock();
				double duration = (double)(end - start) / CLOCKS_PER_SEC;
				printf("main(): id=%d  %f ms\n\n\n", id, duration * 1000.0f);
			}

			id++;
		}

		fullSystem->blockUntilMappingIsFinished();
		clock_t ended = clock();
		struct _timeval tv_end;
		gettimeofday(&tv_end, NULL);


		fullSystem->printResult("result.txt");


		int numFramesProcessed = id;
		double numSecondsProcessed = fabs(reader->getTimestamp(0) - reader->getTimestamp(id - 1));
		double MilliSecondsTakenSingle = 1000.0f*(ended - started) / (float)(CLOCKS_PER_SEC);
		double MilliSecondsTakenMT = ((tv_end.tv_sec - tv_start.tv_sec)*1000.0f + (tv_end.tv_usec - tv_start.tv_usec) / 1000.0f);
		printf("\n======================"
			"\nTotal Run Time:%f min"
			"\n%d Frames (%.2f fps)"
			"\n%.2fms/%.2ffps per frame (single core); "
			"\n%.2fms/%.2ffps per frame (multi core); "
			"\n%.3fx (single core); "
			"\n%.3fx (multi core); "
			"\n======================\n\n",
			(tv_end.tv_sec - tv_start.tv_sec)/60.0,
			numFramesProcessed, numFramesProcessed / numSecondsProcessed,
			MilliSecondsTakenSingle / numFramesProcessed, 1000 * numFramesProcessed / MilliSecondsTakenSingle,
			MilliSecondsTakenMT / numFramesProcessed, 1000 * numFramesProcessed / MilliSecondsTakenMT,
			1000 / (MilliSecondsTakenSingle / numSecondsProcessed),
			1000 / (MilliSecondsTakenMT / numSecondsProcessed));
		//fullSystem->printFrameLifetimes();
		if (setting_logStuff)
		{
			std::ofstream tmlog;
			tmlog.open("logs/time.txt", std::ios::trunc | std::ios::out);
			tmlog << 1000.0f*(ended - started) / (float)(CLOCKS_PER_SEC*reader->getNumImages()) << " "
				<< ((tv_end.tv_sec - tv_start.tv_sec)*1000.0f + (tv_end.tv_usec - tv_start.tv_usec) / 1000.0f) / (float)reader->getNumImages() << "\n";
			tmlog.flush();
			tmlog.close();
		}

		exit_system = true;

		// don't change calling order
		if (useCamera && reader) reader->close();
		//if(viewer) viewer->close();
	});
	

	// don't change calling order
	if (viewer) viewer->run();

	runthreadLoadImages.join();
	runthread.join();
	
	if (!skipFrame && reader) reader->run();

}

void runFile()
{
	reader = new ImageFolderReader(channel, path, imageFile, timeFile, calibFile, gammaCalibFile, vignetteFile);
	reader->setGlobalCalibration();

	if (setting_photometricCalibration > 0 && reader->getPhotometricGamma() == 0)
	{
		printf("ERROR: dont't have photometric calibation. Need to use commandline options mode=1 or mode=2 ");
		exit(1);
	}



	int lstart = dso_start;
	int lend = dso_end;
	int linc = 1;
	if (dso_reverse)
	{
		printf("REVERSE!!!!");
		lstart = dso_end - 1;
		if (lstart >= reader->getNumImages())
			lstart = reader->getNumImages() - 1;
		lend = dso_start;
		linc = -1;
	}



	fullSystem = new FullSystem(channel);
	fullSystem->setGammaFunction(reader->getPhotometricGamma());
	fullSystem->linearizeOperation = (playbackSpeed == 0);


	// viewer wrapper
	if (!disableAllDisplay)
	{
		viewer = new IOWrap::PangolinDSOViewer(0, globalCalibs[channel].wG[0], globalCalibs[channel].hG[0], false);
		fullSystem->outputWrapper.push_back(viewer);
	}
	// sample wrapper
	if (useSampleOutput)
		fullSystem->outputWrapper.push_back(new IOWrap::SampleOutputWrapper());
	// UDP socket wrapper
	IOWrap::SocketWrapper* socket_wraper = new IOWrap::SocketWrapper(channel);
	socket_wraper->initSocket();
	fullSystem->outputWrapper.push_back(socket_wraper);


	std::vector<int> idsToPlay;
	std::vector<double> timesToPlayAt;
	for (int i = lstart; i >= 0 && i< reader->getNumImages() && linc*i < linc*lend; i += linc)
	{
		idsToPlay.push_back(i);
		if (timesToPlayAt.size() == 0)
		{
			timesToPlayAt.push_back((double)0);
		}
		else
		{
			double tsThis = reader->getTimestamp(idsToPlay[idsToPlay.size() - 1]);
			double tsPrev = reader->getTimestamp(idsToPlay[idsToPlay.size() - 2]);
			timesToPlayAt.push_back(timesToPlayAt.back() + fabs(tsThis - tsPrev) / (playbackSpeed == 0 ? 1 : playbackSpeed));
		}
	}

	std::vector<ImageAndExposure*> preloadedImages;
	std::thread runthreadLoadImages;
	if (preload)
	{
		printf("LOADING ALL IMAGES!\n");
		for (int ii = 0; ii<(int)idsToPlay.size(); ii++)
		{
			int id = idsToPlay[ii];
			preloadedImages.push_back(reader->getImage_internal(id, 0));
		}
	}
	else
	{
		runthreadLoadImages = std::thread([&]() {
			exit_load_images = false;
			while (!can_exit)
			{
				hessianMutex.lock();
				if (!is_new_loop)
				{
					hessianMutex.unlock();
					continue;
				}
				hessianMutex.unlock();

				int ii = 0;
				while (ii<(int)idsToPlay.size())
				{
					hessianMutex.lock();
					int fsize=frameHessians.size();
					hessianMutex.unlock();
					if (fsize> cache_length) continue;
					
					int id = idsToPlay[ii];
					ImageAndExposure* img = reader->getImage_internal(id, 0);
					if (img == NULL) continue;

					hessianMutex.lock();
					FrameHessian* fh = fullSystem->makeImages(img);
					hessianMutex.unlock();
					delete img;
					if (fh == NULL) continue;

					hessianMutex.lock();
					frameHessians.push_back(fh);
					hessianMutex.unlock();

					ii++;
				}
			}

			exit_load_images = true;
		});
	}

	// to make MacOS happy: run this in dedicated thread -- and use this one to run the GUI.
	std::thread runthread([&]() {

		for (int i = 0; i < loop && !can_exit; i++)
		{
			if (loop > 1) printf("\n\nloop %d:", i);
			hessianMutex.lock();			
			is_new_loop = true;
			hessianMutex.unlock();

			struct _timeval tv_start;
			gettimeofday(&tv_start, NULL);
			clock_t started = clock();
			double sInitializerOffset = 0;

			int id = 0;
			while (!can_exit && id < idsToPlay.size())
			{

				if (preload && id >= preloadedImages.size()) continue;
				hessianMutex.lock();
				int fsize = frameHessians.size();
				hessianMutex.unlock();
				if (!preload && fsize == 0) continue;
				if (setting_useProfile[channel]) printf("main():for start!\n");
				clock_t start = clock();

				if (!fullSystem->initialized && id < timesToPlayAt.size())	// if not initialized: reset start time.
				{
					gettimeofday(&tv_start, NULL);
					started = clock();
					sInitializerOffset = timesToPlayAt[id];
				}

				clock_t start1 = clock();

				skipFrame = false;
				if (playbackSpeed != 0)
				{
					struct _timeval tv_now; gettimeofday(&tv_now, NULL);
					double sSinceStart = sInitializerOffset + ((tv_now.tv_sec - tv_start.tv_sec) + (tv_now.tv_usec - tv_start.tv_usec) / (1000.0f*1000.0f));

					if (sSinceStart < timesToPlayAt[id])
						usleep((int)((timesToPlayAt[id] - sSinceStart) * 1000 * 1000));
					else if (sSinceStart > timesToPlayAt[id] + 0.5 + 0.1*(id % 2))
					{
						printf("SKIPFRAME %d (play at %f, now it is %f)!\n", id, timesToPlayAt[id], sSinceStart);
						skipFrame = true;
					}
				}

				if (setting_useProfile[channel]) {
					clock_t end1 = clock();
					double duration1 = (double)(end1 - start1) / CLOCKS_PER_SEC;
					printf("main():skipFrame: id=%d  %f ms\n", id, duration1 * 1000.0f);
				}

				if (preload)
				{
					clock_t start2 = clock();

					ImageAndExposure* img = preloadedImages[id];
					if (!skipFrame) fullSystem->addActiveFrame(img, idsToPlay[id]);
					delete img;

					if (setting_useProfile[channel]) {
						clock_t end2 = clock();
						double duration2 = (double)(end2 - start2) / CLOCKS_PER_SEC;
						printf("main():addActiveFrame(): id=%d  %f ms\n", id, duration2 * 1000.0f);
					}

				}
				else
				{
					clock_t start2 = clock();

					hessianMutex.lock();
					if (!skipFrame) fullSystem->addActiveFrame(frameHessians.front(), id);
					else delete frameHessians[0];
					frameHessians.erase(frameHessians.begin());
					hessianMutex.unlock();

					if (setting_useProfile[channel]) {
						clock_t end2 = clock();
						double duration2 = (double)(end2 - start2) / CLOCKS_PER_SEC;
						printf("main():addActiveFrame(): id=%d  %f ms\n", id, duration2 * 1000.0f);
					}
				}

				clock_t start3 = clock();
				socket_wraper->run(fullSystem->initialized ? 0 : 2);
				if (fullSystem->initFailed || setting_fullResetRequested[channel]/* || fullSystem->needLostReset()*/)
				{
					//if (id < 250 || setting_fullResetRequested[channel]/* || fullSystem->needLostReset()*/)
					{
						printf("RESETTING!\n");
						resetFullSystem();
						setting_fullResetRequested[channel] = false;
					}
				}
				if (setting_useProfile[channel]) {
					clock_t end3 = clock();
					double duration3 = (double)(end3 - start3) / CLOCKS_PER_SEC;
					printf("main():RESETTING: id=%d  %f ms\n", id, duration3 * 1000.0f);
				}

				socket_wraper->setTrackingState(fullSystem->initialized ? 0 : 2);

				if (setting_useProfile[channel]) {
					clock_t end = clock();
					double duration = (double)(end - start) / CLOCKS_PER_SEC;
					printf("main(): id=%d  %f ms\n\n\n", id, duration * 1000.0f);
				}

				id++;

				hessianMutex.lock();
				is_new_loop = false;
				hessianMutex.unlock();
			}

			fullSystem->blockUntilMappingIsFinished();
			clock_t ended = clock();
			struct _timeval tv_end;
			gettimeofday(&tv_end, NULL);


			fullSystem->printResult("result.txt");


			int numFramesProcessed = abs(idsToPlay[0] - idsToPlay.back()) + 1;
			double numSecondsProcessed = fabs(reader->getTimestamp(idsToPlay[0]) - reader->getTimestamp(idsToPlay.back()));
			double MilliSecondsTakenSingle = 1000.0f*(ended - started) / (float)(CLOCKS_PER_SEC);
			double MilliSecondsTakenMT = sInitializerOffset + ((tv_end.tv_sec - tv_start.tv_sec)*1000.0f + (tv_end.tv_usec - tv_start.tv_usec) / 1000.0f);
			printf("\n======================"
				"\nTotal Run Time:%f min"
				"\n%d Frames (%.2f fps)"
				"\n%.2fms/%.2ffps per frame (single core); "
				"\n%.2fms/%.2ffps per frame (multi core); "
				"\n%.3fx (single core); "
				"\n%.3fx (multi core); "
				"\n======================\n\n",
				(tv_end.tv_sec - tv_start.tv_sec)/60.0,
				numFramesProcessed, numFramesProcessed / numSecondsProcessed,
				MilliSecondsTakenSingle / numFramesProcessed, 1000 * numFramesProcessed / MilliSecondsTakenSingle,
				MilliSecondsTakenMT / numFramesProcessed, 1000 * numFramesProcessed / MilliSecondsTakenMT,
				1000 / (MilliSecondsTakenSingle / numSecondsProcessed),
				1000 / (MilliSecondsTakenMT / numSecondsProcessed));
			//fullSystem->printFrameLifetimes();
			if (setting_logStuff)
			{
				std::ofstream tmlog;
				tmlog.open("logs/time.txt", std::ios::trunc | std::ios::out);
				tmlog << 1000.0f*(ended - started) / (float)(CLOCKS_PER_SEC*reader->getNumImages()) << " "
					<< ((tv_end.tv_sec - tv_start.tv_sec)*1000.0f + (tv_end.tv_usec - tv_start.tv_usec) / 1000.0f) / (float)reader->getNumImages() << "\n";
				tmlog.flush();
				tmlog.close();
			}

			if (i < loop - 1)
			{
				resetFullSystem();

				setting_fullResetRequested[channel] = false;
			}
		}

		exit_system = true;

		// don't change calling order
		if (useCamera && reader) reader->close();
		//if(viewer) viewer->close();
	});


	// don't change calling order
	if (viewer) viewer->run();

	if (!preload) runthreadLoadImages.join();
	runthread.join();
}

int main(int argc, char** argv)
{
	printf("main(): called!\n");
	//setlocale(LC_ALL, "");

	for (int i = 1; i<argc; i++)
		parseArgument(argv[i]);

	parseArgument();

	// hook crtl+C.
	boost::thread exThread = boost::thread(exitThread);

	if (useCamera) runCamera();
	else runFile();

	for (IOWrap::Output3DWrapper* ow : fullSystem->outputWrapper)
	{
		ow->join();
		delete ow;
	}

	printf("DELETE FULLSYSTEM!\n");
	if(fullSystem) delete fullSystem;

	printf("DELETE READER!\n");
	if(reader) delete reader;

	printf("DELETE VIEWER!\n");
	if (viewer) delete viewer;

	printf("EXIT NOW!\n");
	return 0;
}
