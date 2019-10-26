#ifndef __UNDISTORT_CUH__
#define __UNDISTORT_CUH__

#include <sstream>
#include <fstream>
#include <dirent.h>
#include <algorithm>
#include <iterator>
#include <vector>

#include "cuda/ImageAndExposure.cuh"
#include "cuda/MinimalImage.cuh"

struct CudaPhotometricUndistorter
{
	CudaImageAndExposure* output;
	float *G;
	int GDepth;
	float* vignetteMap;
	float* vignetteMapInv;
	int w, h;
	bool valid;
};

extern "C"
void cudaSetPhotometricUndistorter(CudaPhotometricUndistorter& pu, std::string file, std::string noiseImage, std::string vignetteImage, int w_, int h_)
{
	pu.valid = false;
	pu.vignetteMap = 0;
	pu.vignetteMapInv = 0;
	pu.w = w_;
	pu.h = h_;
	pu.output = new CudaImageAndExposure;
	pu.output->w = w_;
	pu.output->h = h_;
	if (file == "" || vignetteImage == "")
	{
		printf("NO PHOTOMETRIC Calibration!\n");
	}

	// read G.
	checkCudaErrors(cudaMalloc((void **)&pu.G, pu.w * pu.h * sizeof(float)));
	std::ifstream f(file.c_str());
	printf("Reading Photometric Calibration from file %s\n", file.c_str());
	if (!f.good())
	{
		printf("PhotometricUndistorter: Could not open file!\n");
		return;
	}

	{
		std::string line;
		std::getline(f, line);
		std::istringstream l1i(line);
		std::vector<float> Gvec = std::vector<float>(std::istream_iterator<float>(l1i), std::istream_iterator<float>());

		pu.GDepth = Gvec.size();

		if (pu.GDepth < 256)
		{
			printf("PhotometricUndistorter: invalid format! got %d entries in first line, expected at least 256!\n", (int)Gvec.size());
			return;
		}


		for (int i = 0; i<pu.GDepth; i++) pu.G[i] = Gvec[i];

		for (int i = 0; i<pu.GDepth - 1; i++)
		{
			if (pu.G[i + 1] <= pu.G[i])
			{
				printf("PhotometricUndistorter: G invalid! it has to be strictly increasing, but it isnt!\n");
				return;
			}
		}

		float min = pu.G[0];
		float max = pu.G[pu.GDepth - 1];
		for (int i = 0; i<pu.GDepth; i++) pu.G[i] = 255.0 * (pu.G[i] - min) / (max - min);			// make it to 0..255 => 0..255.
	}

	if (setting_photometricCalibration == 0)
	{
		for (int i = 0; i<pu.GDepth; i++) pu.G[i] = 255.0f*i / (float)(pu.GDepth - 1);
	}

	printf("Reading Vignette Image from %s\n", vignetteImage.c_str());
	CudaMinimalImageUInt* vm16 = IOWrap::readImageBW_16U(vignetteImage.c_str());
	CudaMinimalImageUChar* vm8 = IOWrap::readImageBW_8U(vignetteImage.c_str());
	checkCudaErrors(cudaMalloc((void **)&pu.vignetteMap, pu.w * pu.h * sizeof(float)));
	checkCudaErrors(cudaMalloc((void **)&pu.vignetteMapInv, pu.w * pu.h * sizeof(float)));

	if (vm16 != 0)
	{
		if (vm16->w != pu.w || vm16->h != pu.h)
		{
			printf("PhotometricUndistorter: Invalid vignette image size! got %d x %d, expected %d x %d\n",
				vm16->w, vm16->h, pu.w, pu.h);
			if (vm16 != 0) delete vm16;
			if (vm8 != 0) delete vm8;
			return;
		}

		float maxV = 0;
		for (int i = 0; i < pu.w*pu.h; i++)
			if (cudaAt(vm16, i) > maxV) maxV = cudaAt(vm16, i);

		for (int i = 0; i < pu.w*pu.h; i++)
			pu.vignetteMap[i] = cudaAt(vm16, i) / maxV;
	}
	else if (vm8 != 0)
	{
		if (vm8->w != pu.w || vm8->h != pu.h)
		{
			printf("PhotometricUndistorter: Invalid vignette image size! got %d x %d, expected %d x %d\n",
				vm8->w, vm8->h, pu.w, pu.h);
			if (vm16 != 0) delete vm16;
			if (vm8 != 0) delete vm8;
			return;
		}

		float maxV = 0;
		for (int i = 0; i < pu.w*pu.h; i++)
			if (cudaAt(vm8, i) > maxV) maxV = cudaAt(vm8, i);

		for (int i = 0; i < pu.w*pu.h; i++)
			pu.vignetteMap[i] = cudaAt(vm8, i) / maxV;
	}
	else
	{
		printf("PhotometricUndistorter: Invalid vignette image\n");
		if (vm16 != 0) delete vm16;
		if (vm8 != 0) delete vm8;
		return;
	}

	if (vm16 != 0) delete vm16;
	if (vm8 != 0) delete vm8;


	for (int i = 0; i < pu.w*pu.h; i++)
		pu.vignetteMapInv[i] = 1.0f / pu.vignetteMap[i];


	printf("Successfully read photometric calibration!\n");
	pu.valid = true;
}

extern "C"
void cudaFreePhotometricUndistorter(CudaPhotometricUndistorter& pu)
{
	checkCudaErrors(cudaFree((void *)pu.G));
	checkCudaErrors(cudaFree((void *)pu.vignetteMap));
	checkCudaErrors(cudaFree((void *)pu.vignetteMapInv));
}

// removes readout noise, and converts to irradiance.
// affine normalizes values to 0 <= I < 256.
// raw irradiance = a*I + b.
// output will be written in [output].
extern "C"
void cudaProcessFrame(CudaPhotometricUndistorter& pu, const char* image_in, float exposure_time, float factor = 1)
{
}

extern "C"
void cudaUnMapFloatImage(CudaPhotometricUndistorter& pu, float* image)
{
}

extern "C"
float* getG(CudaPhotometricUndistorter& pu)
{
	if(!pu.valid) return 0; else return pu.G;
}

/*
class Undistort
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
	virtual ~Undistort();

	virtual void distortCoordinates(float* in_x, float* in_y, float* out_x, float* out_y, int n) const = 0;

	
	inline const Mat33 getK() const {return K;};
	inline const Eigen::Vector2i getSize() const {return Eigen::Vector2i(w,h);};
	inline const VecX getOriginalParameter() const {return parsOrg;};
	inline const Eigen::Vector2i getOriginalSize() {return Eigen::Vector2i(wOrg,hOrg);};
	inline bool isValid() {return valid;};

	template<typename T>
	ImageAndExposure* undistort(const MinimalImage<T>* image_raw, float exposure=0, double timestamp=0, float factor=1) const;
	template<typename T>
	ImageAndExposure* undistort_cuda(const MinimalImage<T>* image_raw, float exposure = 0, double timestamp = 0, float factor = 1) const;
	static Undistort* getUndistorterForFile(std::string configFilename, std::string gammaFilename, std::string vignetteFilename);

	void loadPhotometricCalibration(std::string file, std::string noiseImage, std::string vignetteImage);

	PhotometricUndistorter* photometricUndist;

protected:
    int w, h, wOrg, hOrg, wUp, hUp;
    int upsampleUndistFactor;
	Mat33 K;
	VecX parsOrg;
	bool valid;
	bool passthrough;

	float* remapX;
	float* remapY;

	void applyBlurNoise(float* img) const;

	void makeOptimalK_crop();
	void makeOptimalK_full();

	void readFromFile(const char* configFileName, int nPars, std::string prefix = "");
};

class UndistortFOV : public Undistort
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    UndistortFOV(const char* configFileName, bool noprefix);
	~UndistortFOV();
	void distortCoordinates(float* in_x, float* in_y, float* out_x, float* out_y, int n) const;

};

class UndistortRadTan : public Undistort
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    UndistortRadTan(const char* configFileName, bool noprefix);
    ~UndistortRadTan();
    void distortCoordinates(float* in_x, float* in_y, float* out_x, float* out_y, int n) const;

};

class UndistortEquidistant : public Undistort
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    UndistortEquidistant(const char* configFileName, bool noprefix);
    ~UndistortEquidistant();
    void distortCoordinates(float* in_x, float* in_y, float* out_x, float* out_y, int n) const;

};

class UndistortPinhole : public Undistort
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    UndistortPinhole(const char* configFileName, bool noprefix);
	~UndistortPinhole();
	void distortCoordinates(float* in_x, float* in_y, float* out_x, float* out_y, int n) const;

private:
	float inputCalibration[8];
};

class UndistortKB : public Undistort
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    UndistortKB(const char* configFileName, bool noprefix);
	~UndistortKB();
	void distortCoordinates(float* in_x, float* in_y, float* out_x, float* out_y, int n) const;

};

}
*/
#endif // !__UNDISTORT_CUH__
