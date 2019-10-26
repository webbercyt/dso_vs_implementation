#ifndef __DATASET_READER_CUH__
#define __DATASET_READER_CUH__

#include <sstream>
#include <fstream>
#include <dirent.h>
#include <algorithm>
#include <vector>

#if HAS_ZIPLIB
	#include "zip.h"
#endif

#include "cuda/ImageAndExposure.cuh"
#include "cuda/Undistort.cuh"

extern "C"
int cudaGetDir(std::string dir, std::vector<std::string> &files)
{
	DIR *dp;
	struct dirent *dirp;
	if ((dp = opendir(dir.c_str())) == NULL)
	{
		return -1;
	}

	while ((dirp = readdir(dp)) != NULL) {
		std::string name = std::string(dirp->d_name);

		if (name != "." && name != "..")
			files.push_back(name);
	}
	closedir(dp);


	std::sort(files.begin(), files.end());

	if (dir.at(dir.length() - 1) != '/') dir = dir + "/";
	for (unsigned int i = 0; i<files.size(); i++)
	{
		if (files[i].at(0) != '/')
			files[i] = dir + files[i];
	}

	return files.size();
}

struct CudaPrepImageItem
{
	int id;
	bool isQueud;
	CudaImageAndExposure* pt;
};

extern "C"
void cudaSetPrepImageItem(CudaPrepImageItem& pii, int _id)
{
	pii.id = _id;
	pii.isQueud = false;
	pii.pt = 0;
}

extern "C"
void cudaReleasePrepImageItem(CudaPrepImageItem& pii)
{
	if (pii.pt != 0) delete pii.pt;
	pii.pt = 0;
}

struct CudaImageFolderReader
{
	//Undistort* undistort;

	std::vector<CudaImageAndExposure*> preloadedImages;
	std::vector<std::string> files;
	std::vector<double> timestamps;
	std::vector<float> exposures;

	int width, height;
	int widthOrg, heightOrg;

	std::string path;
	std::string calibfile;

	bool isZipped;

#if HAS_ZIPLIB
	zip_t* ziparchive;
	char* databuffer;
#endif
};

extern "C"
void cudaSetImageFolderReader(CudaImageFolderReader& ifr, std::string path, std::string calibFile, std::string gammaFile, std::string vignetteFile)
{
	ifr.path = path;
	ifr.calibfile = calibFile;

#if HAS_ZIPLIB
	ifr.ziparchive = 0;
	ifr.databuffer = 0;
#endif

	ifr.isZipped = (path.length()>4 && path.substr(path.length() - 4) == ".zip");

	if (ifr.isZipped)
	{
#if HAS_ZIPLIB
		int ziperror = 0;
		ifr.ziparchive = zip_open(path.c_str(), ZIP_RDONLY, &ziperror);
		if (ziperror != 0)
		{
			printf("ERROR %d reading archive %s!\n", ziperror, path.c_str());
			exit(1);
		}

		ifr.files.clear();
		int numEntries = zip_get_num_entries(ifr.ziparchive, 0);
		for (int k = 0; k<numEntries; k++)
		{
			const char* name = zip_get_name(ifr.ziparchive, k, ZIP_FL_ENC_STRICT);
			std::string nstr = std::string(name);
			if (nstr == "." || nstr == "..") continue;
			ifr.files.push_back(name);
		}

		printf("got %d entries and %d files!\n", numEntries, (int)ifr.files.size());
		std::sort(ifr.files.begin(), ifr.files.end());
#else
		printf("ERROR: cannot read .zip archive, as compile without ziplib!\n");
		exit(1);
#endif
	}
	else
		cudaGetDir(path, ifr.files);


	ifr.undistort = NULL;// getUndistorterForFile(calibFile, gammaFile, vignetteFile);


	//ifr.widthOrg = ifr.undistort->getOriginalSize()[0];
	//ifr.heightOrg = ifr.undistort->getOriginalSize()[1];
	//ifr.width = ifr.undistort->getSize()[0];
	//ifr.height = ifr.undistort->getSize()[1];


	// load timestamps if possible.
	cudaLoadTimestamps(ifr);
	printf("ImageFolderReader: got %d files in %s!\n", (int)ifr.files.size(), path.c_str());

}

extern "C"
void cudaFreeImageFolderReader(CudaImageFolderReader& ifr)
{
#if HAS_ZIPLIB
	if (ifr.ziparchive != 0) zip_close(ifr.ziparchive);
	if (ifr.databuffer != 0) delete ifr.databuffer;
#endif

	delete ifr.undistort;
};



extern "C"
int cudaGetNumImages(CudaImageFolderReader& ifr)
{
	return ifr.files.size();
}

extern "C"
double cudaGetTimestamp(CudaImageFolderReader& ifr, int id)
{
	if (ifr.timestamps.size() == 0) return id*0.1f;
	if (id >= (int)ifr.timestamps.size()) return 0;
	if (id < 0) return 0;
	return ifr.timestamps[id];
}

extern "C"
void cudaPrepImage(CudaImageFolderReader& ifr, int id, bool as8U = false)
{

}



extern "C"
void cudaLoadTimestamps(CudaImageFolderReader& ifr)
{
	std::ifstream tr;
	std::string timesFile = ifr.path.substr(0, ifr.path.find_last_of('/')) + "/times.txt";
	tr.open(timesFile.c_str());
	while (!tr.eof() && tr.good())
	{
		std::string line;
		char buf[1000];
		tr.getline(buf, 1000);

		int id;
		double stamp;
		float exposure = 0;

		if (3 == sscanf(buf, "%d %lf %f", &id, &stamp, &exposure))
		{
			ifr.timestamps.push_back(stamp);
			ifr.exposures.push_back(exposure);
		}

		else if (2 == sscanf(buf, "%d %lf", &id, &stamp))
		{
			ifr.timestamps.push_back(stamp);
			ifr.exposures.push_back(exposure);
		}
	}
	tr.close();

	// check if exposures are correct, (possibly skip)
	bool exposuresGood = ((int)ifr.exposures.size() == (int)cudaGetNumImages(ifr));
	for (int i = 0; i<(int)ifr.exposures.size(); i++)
	{
		if (ifr.exposures[i] == 0)
		{
			// fix!
			float sum = 0, num = 0;
			if (i>0 && ifr.exposures[i - 1] > 0) { sum += ifr.exposures[i - 1]; num++; }
			if (i + 1<(int)ifr.exposures.size() && ifr.exposures[i + 1] > 0) { sum += ifr.exposures[i + 1]; num++; }

			if (num>0)
				ifr.exposures[i] = sum / num;
		}

		if (ifr.exposures[i] == 0) exposuresGood = false;
	}


	if ((int)cudaGetNumImages(ifr) != (int)ifr.timestamps.size())
	{
		printf("set timestamps and exposures to zero! %i and tempstamps:%i\n", cudaGetNumImages(ifr), ifr.timestamps.size());
		ifr.exposures.clear();
		ifr.timestamps.clear();
	}

	if ((int)cudaGetNumImages(ifr) != (int)ifr.exposures.size() || !exposuresGood)
	{
		printf("set EXPOSURES to zero!\n");
		ifr.exposures.clear();
	}

	printf("got %d images and %d timestamps and %d exposures.!\n", (int)cudaGetNumImages(ifr), (int)ifr.timestamps.size(), (int)ifr.exposures.size());
}

/*
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
		setGlobalCalib(w_out, h_out, K);
	}



	MinimalImageB* getImageRaw(int id)
	{
			return getImageRaw_internal(id,0);
	}

	ImageAndExposure* getImage(int id, bool forceLoadDirectly=false)
	{
		return getImage_internal(id, 0);
	}

	ImageAndExposure* getImage_cuda(int id, bool forceLoadDirectly = false)
	{
		return getImage_internal_cuda(id, 0);
	}

	inline float* getPhotometricGamma()
	{
		if(undistort==0 || undistort->photometricUndist==0) return 0;
		return undistort->photometricUndist->getG();
	}


	// undistorter. [0] always exists, [1-2] only when MT is enabled.
	Undistort* undistort;
private:


	MinimalImageB* getImageRaw_internal(int id, int unused)
	{
		if(!isZipped)
		{
			// CHANGE FOR ZIP FILE
			return IOWrap::readImageBW_8U(files[id]);
		}
		else
		{
#if HAS_ZIPLIB
			if(databuffer==0) databuffer = new char[widthOrg*heightOrg*6+10000];
			zip_file_t* fle = zip_fopen(ziparchive, files[id].c_str(), 0);
			long readbytes = zip_fread(fle, databuffer, (long)widthOrg*heightOrg*6+10000);

			if(readbytes > (long)widthOrg*heightOrg*6)
			{
				printf("read %ld/%ld bytes for file %s. increase buffer!!\n", readbytes,(long)widthOrg*heightOrg*6+10000, files[id].c_str());
				delete[] databuffer;
				databuffer = new char[(long)widthOrg*heightOrg*30];
				fle = zip_fopen(ziparchive, files[id].c_str(), 0);
				readbytes = zip_fread(fle, databuffer, (long)widthOrg*heightOrg*30+10000);

				if(readbytes > (long)widthOrg*heightOrg*30)
				{
					printf("buffer still to small (read %ld/%ld). abort.\n", readbytes,(long)widthOrg*heightOrg*30+10000);
					exit(1);
				}
			}

			return IOWrap::readStreamBW_8U(databuffer, readbytes);
#else
			printf("ERROR: cannot read .zip archive, as compile without ziplib!\n");
			exit(1);
#endif
		}
	}


	ImageAndExposure* getImage_internal(int id, int unused)
	{
		MinimalImageB* minimg = getImageRaw_internal(id, 0);
		ImageAndExposure* ret2 = undistort->undistort<unsigned char>(
				minimg,
				(exposures.size() == 0 ? 1.0f : exposures[id]),
				(timestamps.size() == 0 ? 0.0 : timestamps[id]));
		delete minimg;
		return ret2;
	}

	ImageAndExposure* getImage_internal_cuda(int id, int unused)
	{
		clock_t start = clock();
		MinimalImageB* minimg = getImageRaw_internal(id, 0);
		clock_t end = clock();
		double duration = (double)(end - start) / CLOCKS_PER_SEC;
		printf("	getImage_internal_cuda(): getImageRaw_internal: %f ms\n", duration * 1000.0f);
		start = clock();
		ImageAndExposure* ret2 = undistort->undistort_cuda<unsigned char>(
			minimg,
			(exposures.size() == 0 ? 1.0f : exposures[id]),
			(timestamps.size() == 0 ? 0.0 : timestamps[id]));
		delete minimg;
		end = clock();
		duration = (double)(end - start) / CLOCKS_PER_SEC;
		printf("	getImage_internal_cuda(): undistort_cuda: %f ms\n", duration * 1000.0f);
		return ret2;
	}

*/
#endif //__DATASET_READER_CUH__