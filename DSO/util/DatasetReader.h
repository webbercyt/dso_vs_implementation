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


#pragma once
#include "util/settings.h"
#include "util/globalFuncs.h"
#include "util/globalCalib.h"

#include <sstream>
#include <fstream>
#include <dirent.h>
#include <algorithm>

#include "util/Undistort.h"
#include "IOWrapper/ImageRW.h"
#include "cuda/dso.h"

#if HAS_ZIPLIB
	#include "zip.h"
#endif

#include <boost/thread.hpp>

#include "VideoSource.h"
//#include "VideoSource_PointGrey.h"
//#include "VideoSource_ids.h"
//#include "VideoSource_Basler.h"
#include "VideoSource_FLIR.h"

using namespace dso;
using namespace VideoSourceNS;

inline int getdir (std::string dir, std::vector<std::string> &files)
{
    DIR *dp;
    struct dirent *dirp;
    if((dp  = opendir(dir.c_str())) == NULL)
    {
        return -1;
    }

    while ((dirp = readdir(dp)) != NULL) {
    	std::string name = std::string(dirp->d_name);

    	if(name != "." && name != "..")
    		files.push_back(name);
    }
    closedir(dp);


    std::sort(files.begin(), files.end());

    if(dir.at( dir.length() - 1 ) != '/') dir = dir+"/";
	for(unsigned int i=0;i<files.size();i++)
	{
		if(files[i].at(0) != '/')
			files[i] = dir + files[i];
	}

    return files.size();
}

class ImageFolderReader
{
public:
	ImageFolderReader(int channel, std::string path, std::string imageFile, std::string timeFile, std::string calibFile, std::string gammaFile, std::string vignetteFile)
	{
		videoS = NULL;
		undistort = NULL;

		this->channel = channel;
		this->path = path;
		this->imageFile = imageFile;
		this->timeFile = timeFile;
		this->calibFile = calibFile;

#if HAS_ZIPLIB
		ziparchive=0;
		databuffer=0;
#endif

		isZipped = (imageFile.length()>4 && (imageFile.substr(imageFile.length()-4) == ".zip" || imageFile.substr(imageFile.length()-7) == ".images"));

		if(isZipped)
		{
#if HAS_ZIPLIB
			int ziperror=0;
			ziparchive = zip_open((path + imageFile).c_str(),  ZIP_RDONLY, &ziperror);
			if(ziperror!=0)
			{
				printf("ERROR %d reading archive %s!\n", ziperror, (path + imageFile).c_str());
				exit(1);
			}

			files.clear();
			int numEntries = zip_get_num_entries(ziparchive, 0);
			for(int k=0;k<numEntries;k++)
			{
				const char* name = zip_get_name(ziparchive, k,  ZIP_FL_ENC_STRICT);
				std::string nstr = std::string(name);
				if(nstr == "." || nstr == "..") continue;
				files.push_back(name);
			}

			printf("got %d entries and %d files!\n", numEntries, (int)files.size());
			std::sort(files.begin(), files.end());

#else
			printf("ERROR: cannot read .zip archive, as compile without ziplib!\n");
			exit(1);
#endif
		}
		else
			getdir (path + imageFile, files);


		undistort = Undistort::getUndistorterForFile(path + calibFile, path + gammaFile, path + vignetteFile);


		widthOrg = undistort->getOriginalSize()[0];
		heightOrg = undistort->getOriginalSize()[1];
		width=undistort->getSize()[0];
		height=undistort->getSize()[1];


		// load timestamps if possible.
		loadTimestamps();
		printf("ImageFolderReader: got %d files in %s!\n", (int)files.size(), (path + imageFile).c_str());

	}

	ImageFolderReader(int channel, std::string path, std::string calibFile, std::string gammaFile, std::string vignetteFile, bool skipFrame, std::string camType, bool hard_trigger, std::string camSerial, float camFPS, float camGain)
	{
		videoS = NULL;
		undistort = NULL;
#if HAS_ZIPLIB
		ziparchive = 0;
		databuffer = 0;
#endif

		if (camType == "flir") {
			videoS = new FLIR::VideoSource_FLIR(channel, skipFrame, hard_trigger, camSerial, camFPS, camGain);
		}
		else {
			printf("No video source assigned!\n");
			videoS = NULL;
			return;
		}

		if (videoS && videoS->numCameras == 0)
		{
			if (videoS)
			{
				delete videoS;
				videoS = NULL;
			}
			return;
		}

		this->channel = channel;
		this->path = path;
		this->imageFile = "";
		this->calibFile = calibFile;
		idProcesed = -1;

		undistort = Undistort::getUndistorterForFile(path + calibFile, path + gammaFile, path + vignetteFile);

		widthOrg = undistort->getOriginalSize()[0];
		heightOrg = undistort->getOriginalSize()[1];
		width = undistort->getSize()[0];
		height = undistort->getSize()[1];
		printf("widthOrg=%d  heightOrg=%d  width=%d  height=%d\n", widthOrg, heightOrg, width, height);
	}

	~ImageFolderReader()
	{
#if HAS_ZIPLIB
		if(ziparchive) zip_close(ziparchive);
		if(databuffer) delete databuffer;
#endif

		if (videoS) delete videoS;
		if(undistort) delete undistort;
	};

	Eigen::VectorXf getOriginalCalib()
	{
		return undistort->getOriginalParameter().cast<float>();
	}
	Eigen::Vector2i getOriginalDimensions()
	{
		return  undistort->getOriginalSize();
	}

	void getCalibMono(Eigen::Matrix3f &K, int &w, int &h)
	{
		K = undistort->getK().cast<float>();
		w = undistort->getSize()[0];
		h = undistort->getSize()[1];
	}

	void setGlobalCalibration()
	{
		int w_out, h_out;
		Eigen::Matrix3f K;
		getCalibMono(K, w_out, h_out);
		globalCalibs[channel].setGlobalCalib(w_out, h_out, K);
	}

	int getNumImages()
	{
		return files.size();
	}

	double getTimestamp(int id)
	{
		if(timestamps.size()==0) return id*0.1f;
		if(id >= (int)timestamps.size()) id = (int)timestamps.size() - 1;
		if(id < 0) id = 0;
		return timestamps[id];
	}

	inline float* getPhotometricGamma()
	{
		if(undistort==0 || undistort->photometricUndist==0) return 0;
		return undistort->photometricUndist->getG();
	}

	MinimalImageB* getImageRaw_internal(int id, int unused)
	{
		if(!isZipped)
		{
			// CHANGE FOR ZIP FILE
			clock_t start = clock();
			MinimalImageB* ret = IOWrap::readImageBW_8U(files[id]);
			clock_t end = clock();
			double duration = (double)(end - start) / CLOCKS_PER_SEC;
			if(setting_useProfile[channel]) printf("\t\tgetImageRaw_internal(): readImageBW_8U: %f ms\n", duration * 1000.0f);
			return ret;
		}
		else
		{
#if HAS_ZIPLIB
			clock_t start = clock();
			if(databuffer==0) databuffer = new char[widthOrg*heightOrg*6+10000];
			zip_file_t* fle = zip_fopen(ziparchive, files[id].c_str(), 0);
			long readbytes = zip_fread(fle, databuffer, (long)widthOrg*heightOrg*6+10000);
			zip_fclose(fle);

			if(readbytes > (long)widthOrg*heightOrg*6)
			{
				printf("read %ld/%ld bytes for file %s. increase buffer!!\n", readbytes,(long)widthOrg*heightOrg*6+10000, files[id].c_str());
				delete[] databuffer;
				databuffer = new char[(long)widthOrg*heightOrg*30];
				fle = zip_fopen(ziparchive, files[id].c_str(), 0);
				readbytes = zip_fread(fle, databuffer, (long)widthOrg*heightOrg*30+10000);
				zip_fclose(fle);

				if(readbytes > (long)widthOrg*heightOrg*30)
				{
					printf("buffer still to small (read %ld/%ld). abort.\n", readbytes,(long)widthOrg*heightOrg*30+10000);
					exit(1);
				}
			}

			clock_t end = clock();
			double duration = (double)(end - start) / CLOCKS_PER_SEC;
			if (setting_useProfile[channel]) printf("\t\tgetImageRaw_internal(): read zip: id=%d  readbytes=%d %f ms\n", id, readbytes, duration * 1000.0f);

			start = clock();
			MinimalImageB* ret = NULL;
			if (files[id].length() > 4 && files[id].substr(files[id].length() - 4) == ".raw")
			{
				ret = new MinimalImageB(widthOrg, heightOrg);
				memcpy(ret->data, databuffer, readbytes);
			}
			else
				ret = IOWrap::readStreamBW_8U(databuffer, readbytes);
			end = clock();
			duration = (double)(end - start) / CLOCKS_PER_SEC;
			if (setting_useProfile[channel]) printf("\t\tgetImageRaw_internal(): readStreamBW_8U: %f ms\n", duration * 1000.0f);

			return ret;
#else
			printf("ERROR: cannot read .zip archive, as compile without ziplib!\n");
			exit(1);
#endif
		}
	}

	TrackingImage* getImageRaw_external(int& id, int unused)
	{
		TrackingImage* img = videoS->GetAndFillFrameBW(id);
		if(img && setting_useProfile[channel]) printf("getImageRaw_external(): width=%d  height=%d  data=%p\n", img->w(), img->h(), img->getImage());
		return img;
	}

	TrackingImage* getNewestImageRaw_external(int unused)
	{
		TrackingImage* img = videoS->getNewestImage();
		if (img && setting_useProfile[channel]) printf("getNewestImageRaw_external(): width=%d  height=%d  data=%p\n", img->w(), img->h(), img->getImage());
		return img;
	}


	ImageAndExposure* getImage_internal(int id, int unused)
	{
		MinimalImageB* minimg = getImageRaw_internal(id, 0);
		if(minimg == NULL) return NULL;
		ImageAndExposure* ret2 = undistort->undistort<unsigned char>(
				minimg,
				(exposures.size() == 0 ? 1.0f : exposures[id]),
				(timestamps.size() == 0 ? 0.0 : timestamps[id]),
				1.0f);
		ret2->camFrameID = id;
		ret2->setOrgImage(minimg);
		delete minimg;
		return ret2;
	}

	ImageAndExposure* getImage_external(int& id, int unused)
	{
		TrackingImage* minimg = getImageRaw_external(id, 0);
		if (minimg == NULL || minimg->getImage() == NULL) return NULL;
		if (id == idProcesed) {
			id++;
			return NULL;
		}
		idProcesed = id;

		ImageAndExposure* ret2 = undistort->undistort<unsigned char>(
			minimg->getImage(),
			minimg->getExposure(),
			minimg->getTimeStamp(),
			1.0f);
		ret2->camFrameID = minimg->getFrameID();
		ret2->setOrgImage(minimg->getImage());
		timestamps.push_back(minimg->getTimeStamp());
		exposures.push_back(minimg->getExposure());
		delete minimg;
		return ret2;
	}

	ImageAndExposure* getNewestImage_external(int unused)
	{
		TrackingImage* minimg = getNewestImageRaw_external(0);
		if (minimg == NULL || minimg->getImage() == NULL) return NULL;

		ImageAndExposure* ret2 = undistort->undistort<unsigned char>(
			minimg->getImage(),
			minimg->getExposure(),
			minimg->getTimeStamp(),
			1.0f);
		ret2->camFrameID = minimg->getFrameID();
		ret2->setOrgImage(minimg->getImage());
		timestamps.push_back(minimg->getTimeStamp());
		exposures.push_back(minimg->getExposure());
		delete minimg;
		return ret2;
	}

	inline void loadTimestamps()
	{
		std::ifstream tr;
		tr.open((path + timeFile).c_str());
		while(!tr.eof() && tr.good())
		{
			std::string line;
			char buf[1000];
			tr.getline(buf, 1000);

			int id;
			double stamp;
			float exposure = 0;

			if(3 == sscanf(buf, "%d %lf %f", &id, &stamp, &exposure))
			{
				timestamps.push_back(stamp);
				exposures.push_back(exposure);
			}

			else if(2 == sscanf(buf, "%d %lf", &id, &stamp))
			{
				timestamps.push_back(stamp);
				exposures.push_back(exposure);
			}
		}
		tr.close();

		// check if exposures are correct, (possibly skip)
		bool exposuresGood = ((int)exposures.size()==(int)getNumImages()) ;
		for(int i=0;i<(int)exposures.size();i++)
		{
			if(exposures[i] == 0)
			{
				// fix!
				float sum=0,num=0;
				if(i>0 && exposures[i-1] > 0) {sum += exposures[i-1]; num++;}
				if(i+1<(int)exposures.size() && exposures[i+1] > 0) {sum += exposures[i+1]; num++;}

				if(num>0)
					exposures[i] = sum/num;
			}

			if(exposures[i] == 0) exposuresGood=false;
		}


		if((int)getNumImages() != (int)timestamps.size())
		{
			printf("set timestamps and exposures to zero! %i and tempstamps:%zi\n", getNumImages(), timestamps.size());
			exposures.clear();
			timestamps.clear();
		}

		if((int)getNumImages() != (int)exposures.size() || !exposuresGood)
		{
			printf("set EXPOSURES to zero!\n");
			exposures.clear();
		}

		printf("got %d images and %d timestamps and %d exposures.!\n", (int)getNumImages(), (int)timestamps.size(), (int)exposures.size());
	}

	void run()
	{
	}
	
	void close()
	{
		if (videoS) videoS->close();
	}

	void setDsoTrackingStatus(char status)
	{
		dsoTrackingStatus = status;
	}

	char getDsoTrackingStatus()
	{
		return dsoTrackingStatus;
	}

	// undistorter. [0] always exists, [1-2] only when MT is enabled.
	Undistort* undistort;
	int idProcesed;


	std::vector<ImageAndExposure*> preloadedImages;
	std::vector<std::string> files;
	std::vector<double> timestamps;
	std::vector<float> exposures;

	int width, height;
	int widthOrg, heightOrg;

	std::string path;
	std::string imageFile;
	std::string timeFile;
	std::string calibFile;

	bool isZipped;

#if HAS_ZIPLIB
	zip_t* ziparchive;
	char* databuffer;
#endif

	VideoSourceNS::VideoSource* videoS;

	int channel;
	char dsoTrackingStatus = ' ';
};

