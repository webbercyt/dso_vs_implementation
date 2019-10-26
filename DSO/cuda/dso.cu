/*
* Copyright 1993-2014 NVIDIA Corporation.  All rights reserved.
*
* Please refer to the NVIDIA end user license agreement (EULA) associated
* with this source code for terms and conditions that govern your use of
* this software. Any use, reproduction, disclosure, or distribution of
* this software and related documentation outside the terms of the EULA
* is strictly prohibited.
*
*/

/*
* This sample demonstrates how use textures fetches in CUDA
*
* This sample takes an input PGM image (image_filename) and generates
* an output PGM image (image_filename_out).  This CUDA kernel performs
* a simple 2D transform (rotation) on the textures coordinates (u,v).
*/

// Includes, system
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>


#ifdef _WIN32
#  define WINDOWS_LEAN_AND_MEAN
#  define NOMINMAX
#  include <windows.h>
#endif

// Includes CUDA
#include <cuda_runtime.h>

// Utilities and timing functions
#include <helper_math.h>
#include <helper_functions.h> 

// CUDA helper functions
#include <helper_cuda.h>

typedef unsigned int uint;
typedef unsigned char uchar;

//#include <cuda.h>
#include <curand.h>

//#include <memory>
//#include <iostream>
//#include <cassert>

//#include <cuda/HessianBlocks.cuh>
#include <cuda/Residuals.cuh>

#define max(a,b) ((a)>(b)?(a):(b))
#define min(a,b) ((a)<(b)?(a):(b))

#define MAX_NUM_FRAMES 11
#define PYR_LEVELS 6

curandGenerator_t gen;
float *cuda_noiseXY = NULL;

float* cuda_remapX = NULL;
float* cuda_remapY = NULL;
float* cuda_noiseMapX = NULL;
float* cuda_noiseMapY = NULL;
float* cuda_in_data = NULL;
float* cuda_out_data = NULL;
uchar3 *cuda_image = NULL;

float* cuda_color = NULL;
float* cuda_dIp[PYR_LEVELS] = { NULL, NULL,NULL,NULL,NULL,NULL };
float* cuda_absSquaredGrad[PYR_LEVELS] = { NULL, NULL,NULL,NULL,NULL,NULL };
//float* cuda_B = NULL;

float* cuda_idepth = NULL;

CalibHessian* HCalib = NULL;
FrameHessian* firstFrame = NULL;
FrameHessian* frameHessians_cuda[MAX_NUM_FRAMES];


int num_pfr = 0;
cuda_PointFrameResidual* pfr = NULL;
int* targetIdxs = NULL;


#define CURAND_CALL(x) do { if((x)!=CURAND_STATUS_SUCCESS) { \
	printf("Error at %s:%d\n",__FILE__,__LINE__);\
	return EXIT_FAILURE;}} while(0) 

__global__
void applyNoiseKernel(int numnoise, float benchmark_varNoise, float* noiseXY, float* noiseMapX, float* noiseMapY)
{
	// calculate normalized textures coordinates
	unsigned int idx = blockIdx.x*blockDim.x + threadIdx.x;
	if (idx >= numnoise) return;

	noiseMapX[idx] = 2 * benchmark_varNoise * (noiseXY[2 * idx + 0] - 0.5f);
	noiseMapY[idx] = 2 * benchmark_varNoise * (noiseXY[2 * idx + 1] - 0.5f);
}

__device__
float getInterpolatedElement11Cub(const float* const p, const float x)	// for x=0, this returns p[1].
{
	return p[1] + 0.5f * x*(p[2] - p[0] + x*(2.0f*p[0] - 5.0f*p[1] + 4.0f*p[2] - p[3] + x*(3.0f*(p[1] - p[2]) + p[3] - p[0])));
}

__device__
float getInterpolatedElement11BiCub(const float* const mat, const float x, const float y, const int width)
{
	int ix = (int)x;
	int iy = (int)y;
	float dx = x - ix;
	const float* bp = mat + ix + iy*width;

	float val[4];
	val[0] = getInterpolatedElement11Cub(bp - width - 1, dx);
	val[1] = getInterpolatedElement11Cub(bp - 1, dx);
	val[2] = getInterpolatedElement11Cub(bp + width - 1, dx);
	val[3] = getInterpolatedElement11Cub(bp + 2 * width - 1, dx);

	float dy = y - iy;
	return getInterpolatedElement11Cub(val, dy);
}

__device__
float getInterpolatedElement(const float* const mat, const float x, const float y, const int width)
{
	int ix = (int)x;
	int iy = (int)y;
	float dx = x - ix;
	float dy = y - iy;
	float dxdy = dx*dy;
	const float* bp = mat + ix + iy*width;

	float res = dxdy * bp[1 + width]
		+ (dy - dxdy) * bp[width]
		+ (dx - dxdy) * bp[1]
		+ (1 - dx - dy + dxdy) * bp[0];

	return res;
}

__global__
void unditortKernel(float* remapX, float* remapY, float* noiseMapX, float* noiseMapY, float benchmark_varNoise, int benchmark_noiseGridsize,
	int wOrg, int hOrg, int w, int h, float* in_data, float* out_data)
{
	// calculate normalized textures coordinates
	unsigned int idx = blockIdx.x*blockDim.x + threadIdx.x;
	if (idx >= w * h) return;

	// get interp. values
	float xx = remapX[idx];
	float yy = remapY[idx];

	if (benchmark_varNoise>0)
	{
		float deltax = getInterpolatedElement11BiCub(noiseMapX, 4 + (xx / (float)wOrg)*benchmark_noiseGridsize, 4 + (yy / (float)hOrg)*benchmark_noiseGridsize, benchmark_noiseGridsize + 8);
		float deltay = getInterpolatedElement11BiCub(noiseMapY, 4 + (xx / (float)wOrg)*benchmark_noiseGridsize, 4 + (yy / (float)hOrg)*benchmark_noiseGridsize, benchmark_noiseGridsize + 8);
		float x = idx % w + deltax;
		float y = idx / w + deltay;
		if (x < 0.01) x = 0.01;
		if (y < 0.01) y = 0.01;
		if (x > w - 1.01) x = w - 1.01;
		if (y > h - 1.01) y = h - 1.01;

		xx = getInterpolatedElement(remapX, x, y, w);
		yy = getInterpolatedElement(remapY, x, y, w);
	}

	if (xx<0)
		out_data[idx] = 0;
	else
	{
		// get integer and rational parts
		int xxi = xx;
		int yyi = yy;
		xx -= xxi;
		yy -= yyi;
		float xxyy = xx*yy;

		// get array base pointer
		const float* src = in_data + xxi + yyi * wOrg;

		// interpolate (bilinear)
		out_data[idx] = xxyy * src[1 + wOrg]
			+ (yy - xxyy) * src[wOrg]
			+ (xx - xxyy) * src[1]
			+ (1 - xx - yy + xxyy) * src[0];
	}
}

extern "C"
int cudaSetRemapData(float* remapX, float* remapY, int w, int h, bool profile)
{
	StopWatchInterface *timer = NULL;
	if (profile)
	{
		sdkCreateTimer(&timer);
		sdkStartTimer(&timer);
	}

	size_t out_data_size = w*h*sizeof(float);
	checkCudaErrors(cudaHostRegister(remapX, out_data_size, cudaHostRegisterDefault));
	if (cuda_remapX == NULL) checkCudaErrors(cudaMalloc((void **)&cuda_remapX, out_data_size));
	checkCudaErrors(cudaMemcpyAsync((void *)cuda_remapX, (const void *)remapX, out_data_size, cudaMemcpyHostToDevice, 0));
	checkCudaErrors(cudaDeviceSynchronize());
	checkCudaErrors(cudaHostUnregister(remapX));

	if (profile)
	{
		sdkStopTimer(&timer);
		printf("\t\tcudaSetRemapData(): transfer in cuda_remapX: w=%d  h=%d  %f\n", w, h, sdkGetTimerValue(&timer));
		sdkResetTimer(&timer);
		sdkStartTimer(&timer);
	}

	checkCudaErrors(cudaHostRegister(remapY, out_data_size, cudaHostRegisterDefault));
	if (cuda_remapY == NULL) checkCudaErrors(cudaMalloc((void **)&cuda_remapY, out_data_size));
	checkCudaErrors(cudaMemcpyAsync((void *)cuda_remapY, (const void *)remapY, out_data_size, cudaMemcpyHostToDevice, 0));
	checkCudaErrors(cudaDeviceSynchronize());
	checkCudaErrors(cudaHostUnregister(remapY));

	if (profile)
	{
		sdkStopTimer(&timer);
		printf("\t\tcudaSetRemapData(): transfer in cuda_remapY: w=%d  h=%d  %f\n", w, h, sdkGetTimerValue(&timer));
		sdkResetTimer(&timer);
		sdkDeleteTimer(&timer);
	}

	// malloc first frame
	checkCudaErrors(cudaMallocHost((void **)&(firstFrame), sizeof(FrameHessian)));
	// malloc frameHessians_cuda
	for (int index = 0; index < MAX_NUM_FRAMES; index++)
		checkCudaErrors(cudaMallocHost((void **)&(frameHessians_cuda[index]), sizeof(FrameHessian)));

	return 1;
}

extern "C"
int cudaUndistort(float benchmark_varNoise, int benchmark_noiseGridsize,
	int wOrg, int hOrg, int w, int h, float* in_data, float* out_data, bool profile)
{
	StopWatchInterface *timer = NULL;
	if (profile) sdkCreateTimer(&timer);

	if (benchmark_varNoise > 0)
	{
		if(profile) sdkStartTimer(&timer);

		int num_noise = (benchmark_noiseGridsize + 8)*(benchmark_noiseGridsize + 8);
		size_t noise_size = num_noise*sizeof(float);

		if (cuda_noiseXY == NULL) checkCudaErrors(cudaMalloc((void **)&cuda_noiseXY, 2 * noise_size));
		CURAND_CALL(curandCreateGenerator(&gen, CURAND_RNG_PSEUDO_DEFAULT));
		CURAND_CALL(curandGenerateUniform(gen, cuda_noiseXY, 2 * num_noise));
		CURAND_CALL(curandDestroyGenerator(gen));

		if (cuda_noiseMapX == NULL) checkCudaErrors(cudaMalloc((void **)&cuda_noiseMapX, noise_size));
		//checkCudaErrors(cudaMemcpy((void *)cuda_noiseMapX, (const void *)noiseMapX, noise_size, cudaMemcpyHostToDevice));
		checkCudaErrors(cudaMemset((void*)cuda_noiseMapX, 0, noise_size));

		if (cuda_noiseMapY == NULL) checkCudaErrors(cudaMalloc((void **)&cuda_noiseMapY, noise_size));
		//checkCudaErrors(cudaMemcpy((void *)cuda_noiseMapY, (const void *)noiseMapY, noise_size, cudaMemcpyHostToDevice));
		checkCudaErrors(cudaMemset((void*)cuda_noiseMapX, 0, noise_size));

		int threadsPerBlock = 256;
		int blocksPerGrid = (num_noise + threadsPerBlock - 1) / threadsPerBlock;
		applyNoiseKernel << <blocksPerGrid, threadsPerBlock >> >(num_noise, benchmark_varNoise, cuda_noiseXY, cuda_noiseMapX, cuda_noiseMapY);
		checkCudaErrors(cudaDeviceSynchronize());

		if (profile)
		{
			sdkStopTimer(&timer);
			printf("\t\t\tcudaUndistort(): transfer in data: wOrg=%d  hOrg=%d  %f\n", wOrg, hOrg, sdkGetTimerValue(&timer));
			sdkResetTimer(&timer);
		}
	}

	if (profile) sdkStartTimer(&timer);

	size_t in_data_size = wOrg*hOrg*sizeof(float);
	size_t out_data_size = w*h*sizeof(float);
	checkCudaErrors(cudaHostRegister(in_data, in_data_size, cudaHostRegisterDefault));
	if (cuda_in_data == NULL) checkCudaErrors(cudaMalloc((void **)&cuda_in_data, in_data_size));
	checkCudaErrors(cudaMemcpyAsync((void *)cuda_in_data, (const void *)in_data, in_data_size, cudaMemcpyHostToDevice, 0));
	checkCudaErrors(cudaDeviceSynchronize());
	checkCudaErrors(cudaHostUnregister(in_data));

	if (profile)
	{
		sdkStopTimer(&timer);
		printf("\t\t\tcudaUndistort(): transfer in data: wOrg=%d  hOrg=%d  %f\n", wOrg, hOrg, sdkGetTimerValue(&timer));
		sdkResetTimer(&timer);
		sdkStartTimer(&timer);
	}

	if (cuda_out_data == NULL) checkCudaErrors(cudaMalloc((void **)&cuda_out_data, out_data_size));
	checkCudaErrors(cudaDeviceSynchronize());

	if (profile)
	{
		sdkStopTimer(&timer);
		printf("\t\t\tcudaUndistort(): cudaMalloc out data: w=%d  h=%d  %f\n", w, h, sdkGetTimerValue(&timer));
		sdkResetTimer(&timer);
		sdkStartTimer(&timer);
	}

	int threadsPerBlock = 256;
	int blocksPerGrid = (w * h + threadsPerBlock - 1) / threadsPerBlock;
	unditortKernel << <blocksPerGrid, threadsPerBlock >> >(cuda_remapX, cuda_remapY, cuda_noiseMapX, cuda_noiseMapY, benchmark_varNoise, benchmark_noiseGridsize,
		wOrg, hOrg, w, h, cuda_in_data, cuda_out_data);
	checkCudaErrors(cudaDeviceSynchronize());

	if (profile)
	{
		sdkStopTimer(&timer);
		printf("\t\t\tcudaUndistort(): unditortKernel: %f\n", sdkGetTimerValue(&timer));
		sdkResetTimer(&timer);
		//sdkStartTimer(&timer);
	}

	//checkCudaErrors(cudaHostRegister(out_data, out_data_size, cudaHostRegisterDefault));
	//checkCudaErrors(cudaMemcpyAsync((void *)out_data, (const void *)cuda_out_data, out_data_size, cudaMemcpyDeviceToHost, 0));
	//checkCudaErrors(cudaDeviceSynchronize());
	//checkCudaErrors(cudaHostUnregister(out_data));

	if (profile)
	{
		//sdkStopTimer(&timer);
		//printf("\t\t\tcudaUndistort(): transfer out data: w=%d  h=%d  %f\n", w, h, sdkGetTimerValue(&timer));
		//sdkResetTimer(&timer);
		sdkDeleteTimer(&timer);
	}

	return 1;
}


// Hessian
__device__
float getBGradOnly(float color, float* B)
{
	int c = color + 0.5f;
	if (c<5) c = 5;
	if (c>250) c = 250;
	return B[c + 1] - B[c];
}

__global__
void assignColorKernel(int w, int h, float* color, float* dI)
{
	// calculate normalized textures coordinates
	unsigned int idx = blockIdx.x*blockDim.x + threadIdx.x;
	if (idx < w * h) dI[3 * idx] = color[idx];
}

__global__
void makeImage1Kernel(int wl, int hl, int wlm1, float* dI_l, float* dI_lm)
{
	// calculate normalized textures coordinates
	unsigned int idx = blockIdx.x*blockDim.x + threadIdx.x;
	if (idx >= wl * hl) return;

	int x = idx % wl;
	int y = idx / wl;

	dI_l[idx * 3] = 0.25f * (dI_lm[(2 * x + 2 * y*wlm1) * 3] +
		dI_lm[(2 * x + 1 + 2 * y*wlm1) * 3] +
		dI_lm[(2 * x + 2 * y*wlm1 + wlm1) * 3] +
		dI_lm[(2 * x + 1 + 2 * y*wlm1 + wlm1) * 3]);
}

__global__
void makeImage2Kernel(int wl, int hl, float* dI_l, float* dabs_l, float* B)
{
	// calculate normalized textures coordinates
	unsigned int idx = blockIdx.x*blockDim.x + threadIdx.x;
	if (idx < wl || idx >= wl * (hl - 1)) return;

	float dx = 0.5f*(dI_l[3 * (idx + 1)] - dI_l[3 * (idx - 1)]);
	float dy = 0.5f*(dI_l[3 * (idx + wl)] - dI_l[3 * (idx - wl)]);

	if (!isfinite(dx)) dx = 0;
	if (!isfinite(dy)) dy = 0;

	dI_l[3 * idx + 1] = dx;
	dI_l[3 * idx + 2] = dy;

	dabs_l[idx] = dx*dx + dy*dy;

	if (setting_gammaWeightsPixelSelect == 1 && B != NULL)
	{
		float gw = getBGradOnly((float)dI_l[3 * idx], B);
		dabs_l[idx] *= gw*gw;	// convert to gradient of original color space (before removing response).
	}
}

extern "C"
void cudaSetCalibHessian(void* data, bool profile)
{
	StopWatchInterface *timer = NULL;
	if (profile)
	{
		sdkCreateTimer(&timer);
		sdkStartTimer(&timer);
	}

	if (HCalib == NULL) checkCudaErrors(cudaMalloc((void **)&HCalib, sizeof(CalibHessian)));
	checkCudaErrors(cudaMemcpy((void *)HCalib, (const void *)data, sizeof(CalibHessian), cudaMemcpyHostToDevice));

	//CalibHessian* cal = NULL;
	//checkCudaErrors(cudaMallocHost((void **)&cal, sizeof(CalibHessian)));
	//checkCudaErrors(cudaMemcpy((void *)cal, (const void *)data, sizeof(CalibHessian), cudaMemcpyHostToHost));

	if (profile)
	{
		sdkStopTimer(&timer);
		printf("\t\t\t\t\t\t\tcudaSetCalibHessian(): transfer in CalibHessian: %f\n", sdkGetTimerValue(&timer));

		//printf("\t\t\t\t\t\t\tcudaSetCalibHessian(): transfer in CalibHessian: fxl=%f, fyl=%f, cxl=%f, cyl=%f\n", cal->fxl, cal->fyl, cal->cxl, cal->cyl);

		sdkResetTimer(&timer);
		sdkDeleteTimer(&timer);
	}
	//checkCudaErrors(cudaFreeHost((void *)cal));
}

extern "C"
void cudaMakeImages(float** dIp, float** absSquaredGrad, bool profile)
{
	//size_t color_size = wG[0] * hG[0] * sizeof(float);

	//checkCudaErrors(cudaHostRegister((void *)color, color_size, cudaHostRegisterDefault));
	//if (cuda_color == NULL) checkCudaErrors(cudaMalloc((void **)&cuda_color, color_size));
	//checkCudaErrors(cudaMemcpyAsync((void *)cuda_color, (const void *)color, color_size, cudaMemcpyHostToDevice, 0));
	//checkCudaErrors(cudaDeviceSynchronize());
	//checkCudaErrors(cudaHostUnregister((void *)color));

	StopWatchInterface *timer = NULL;
	if (profile)
	{
		sdkCreateTimer(&timer);
		sdkStartTimer(&timer);
	}

	for (int lvl = 0; lvl < pyrLevelsUsed; lvl++)
	{
		size_t size = wG[lvl] * hG[lvl] * sizeof(float);
		if (cuda_dIp[lvl] == NULL) checkCudaErrors(cudaMalloc((void **)&cuda_dIp[lvl], 3 * size));
		//checkCudaErrors(cudaMemset((void *)cuda_dIp[lvl], 0, 3 * size));
		if (cuda_absSquaredGrad[lvl] == NULL) checkCudaErrors(cudaMalloc((void **)&cuda_absSquaredGrad[lvl], size));
		//checkCudaErrors(cudaMemset((void *)cuda_absSquaredGrad[lvl], 0, size));
	}

	if (profile)
	{
		sdkStopTimer(&timer);
		printf("\t\tcudaMakeImages(): cudaMalloc cuda_dIp cuda_absSquaredGrad: %f\n", sdkGetTimerValue(&timer));
		sdkResetTimer(&timer);
		sdkStartTimer(&timer);
	}

	int threadsPerBlock = 256;
	int blocksPerGrid = (wG[0] * hG[0] + threadsPerBlock - 1) / threadsPerBlock;
	assignColorKernel << <blocksPerGrid, threadsPerBlock >> >(wG[0], hG[0], cuda_out_data, (float*)(cuda_dIp[0]));
	checkCudaErrors(cudaDeviceSynchronize());

	if (profile)
	{
		sdkStopTimer(&timer);
		printf("\t\tcudaMakeImages(): assignColorKernel: %f\n", sdkGetTimerValue(&timer));
		sdkResetTimer(&timer);
	}

	for (int lvl = 0; lvl < pyrLevelsUsed; lvl++)
	{
		int wl = wG[lvl];
		int hl = hG[lvl];
		float* dI_l = cuda_dIp[lvl];

		int threadsPerBlock = 256;
		int blocksPerGrid = (wl * hl + threadsPerBlock - 1) / threadsPerBlock;

		if (lvl > 0)
		{
			int lvlm1 = lvl - 1;
			int wlm1 = wG[lvlm1];
			float* dI_lm = cuda_dIp[lvlm1];

			if (profile) sdkStartTimer(&timer);

			makeImage1Kernel << <blocksPerGrid, threadsPerBlock >> >(wl, hl, wlm1, dI_l, dI_lm);
			checkCudaErrors(cudaDeviceSynchronize());

			if (profile)
			{
				sdkStopTimer(&timer);
				printf("\t\tcudaMakeImages(): makeImage1Kernel lvl=%d: %f\n", lvl, sdkGetTimerValue(&timer));
				sdkResetTimer(&timer);
			}
		}

		if (profile) sdkStartTimer(&timer);

		float* dabs_l = cuda_absSquaredGrad[lvl];
		makeImage2Kernel << <blocksPerGrid, threadsPerBlock >> >(wl, hl, dI_l, dabs_l, HCalib->B);
		checkCudaErrors(cudaDeviceSynchronize());

		if (profile)
		{
			sdkStopTimer(&timer);
			printf("\t\tcudaMakeImages(): makeImage2Kernel lvl=%d: %f\n", lvl, sdkGetTimerValue(&timer));
			sdkResetTimer(&timer);
		}
	}

	for (int lvl = 0; lvl < pyrLevelsUsed; lvl++)
	{
		if (profile) sdkStartTimer(&timer);

		size_t size = wG[lvl] * hG[lvl] * sizeof(float);
		checkCudaErrors(cudaHostRegister((void *)dIp[lvl], 3 * size, cudaHostRegisterDefault));
		checkCudaErrors(cudaMemcpyAsync((void *)dIp[lvl], (const void *)cuda_dIp[lvl], 3 * size, cudaMemcpyDeviceToHost, 0));
		checkCudaErrors(cudaDeviceSynchronize());
		checkCudaErrors(cudaHostUnregister((void *)dIp[lvl]));

		if (profile)
		{
			sdkStopTimer(&timer);
			printf("\t\tcudaMakeImages(): transfer out dIp  lvl=%d  wG[%d]=%d  hG[%d]=%d: %f\n", lvl, lvl, wG[lvl], lvl, hG[lvl], sdkGetTimerValue(&timer));
			sdkResetTimer(&timer);
		}

		if (lvl <= 2)
		{
			if (profile) sdkStartTimer(&timer);

			checkCudaErrors(cudaHostRegister((void *)absSquaredGrad[lvl], size, cudaHostRegisterDefault));
			checkCudaErrors(cudaMemcpyAsync((void *)absSquaredGrad[lvl], (const void *)cuda_absSquaredGrad[lvl], size, cudaMemcpyDeviceToHost, 0));
			checkCudaErrors(cudaDeviceSynchronize());
			checkCudaErrors(cudaHostUnregister((void *)absSquaredGrad[lvl]));

			if (profile)
			{
				sdkStopTimer(&timer);
				printf("\t\tcudaMakeImages(): transfer out absSquaredGrad  lvl=%d  wG[%d]=%d  hG[%d]=%d: %f\n", lvl, lvl, wG[lvl], lvl, hG[lvl], sdkGetTimerValue(&timer));
				sdkResetTimer(&timer);
			}
		}
	}

	if (profile) sdkDeleteTimer(&timer);
}

__global__
void getImageKernel(int wl, int hl, float* dI, float factor, uchar3* data)
{
	// calculate normalized textures coordinates
	unsigned int idx = blockIdx.x*blockDim.x + threadIdx.x;
	if (idx >= wl * hl) return;
	unsigned char c = (unsigned char)clamp(dI[3 * idx] * factor, 0.0f, 255.0f);
	data[idx] = make_uchar3(c, c, c);
}

__device__
uchar3 makeJet3B(float id)
{
	if (id <= 0) return make_uchar3(128, 0, 0);
	if (id >= 1) return make_uchar3(0, 0, 128);

	int icP = (id * 8);
	float ifP = (id * 8) - icP;

	if (icP == 0) return make_uchar3(255 * (0.5 + 0.5*ifP), 0, 0);
	if (icP == 1) return make_uchar3(255, 255 * (0.5*ifP), 0);
	if (icP == 2) return make_uchar3(255, 255 * (0.5 + 0.5*ifP), 0);
	if (icP == 3) return make_uchar3(255 * (1 - 0.5*ifP), 255, 255 * (0.5*ifP));
	if (icP == 4) return make_uchar3(255 * (0.5 - 0.5*ifP), 255, 255 * (0.5 + 0.5*ifP));
	if (icP == 5) return make_uchar3(0, 255 * (1 - 0.5*ifP), 255);
	if (icP == 6) return make_uchar3(0, 255 * (0.5 - 0.5*ifP), 255);
	if (icP == 7) return make_uchar3(0, 0, 255 * (1 - 0.5*ifP));
	return make_uchar3(255, 255, 255);
}

__device__
void setPixelCirc(int wl, int hl, uchar3* img, const int &u, const int &v, uchar3 val)
{
	for (int i = -3; i <= 3; i++)
	{
		img[u + 3 + (v + i) * wl] = val;
		img[u - 3 + (v + i) * wl] = val;
		img[u + 2 + (v + i) * wl] = val;
		img[u - 2 + (v + i) * wl] = val;

		img[u + i + (v - 3) * wl] = val;
		img[u + i + (v + 3) * wl] = val;
		img[u + i + (v - 2) * wl] = val;
		img[u + i + (v + 2) * wl] = val;
	}
}

__global__
void makeDepthImageKernel(int wl, int hl, float* dI, float factor, float* idepth, float minID, float maxID, uchar3* out_data)
{
	// calculate normalized textures coordinates
	unsigned int idx = blockIdx.x*blockDim.x + threadIdx.x;
	if (idx >= wl * hl) return;

	unsigned char c = (unsigned char)clamp(dI[3 * idx] * factor, 0.0f, 255.0f);
	out_data[idx] = make_uchar3(c, c, c);

	int x = idx % wl;
	if (x < 3 || x > wl - 3) return;

	int y = idx / wl;
	if (y < 3 || y > hl - 3) return;

	float sid = 0, nid = 0;
	float* bp = idepth + idx;

	if (bp[0] > 0) { sid += bp[0]; nid++; }
	if (bp[1] > 0) { sid += bp[1]; nid++; }
	if (bp[-1] > 0) { sid += bp[-1]; nid++; }
	if (bp[wl] > 0) { sid += bp[wl]; nid++; }
	if (bp[-wl] > 0) { sid += bp[-wl]; nid++; }

	if (bp[0] > 0 || nid >= 3)
	{
		float id = ((sid / nid) - minID) / ((maxID - minID));
		setPixelCirc(wl, hl, out_data, x, y, makeJet3B(id));
	}
}

extern "C"
void cudaGetImage(int lvl, int wl, int hl, float factor, unsigned char* out_data, bool profile)
{
	StopWatchInterface *timer = NULL;
	if (profile)
	{
		sdkCreateTimer(&timer);
		sdkStartTimer(&timer);
	}

	size_t out_data_size = wl * hl * sizeof(uchar3);
	if(cuda_image == NULL) checkCudaErrors(cudaMalloc((void **)&cuda_image, out_data_size));
	int threadsPerBlock = 256;
	int blocksPerGrid = (wl * hl + threadsPerBlock - 1) / threadsPerBlock;
	getImageKernel << <blocksPerGrid, threadsPerBlock >> >(wl, hl, (float*)(cuda_dIp[lvl]), factor, cuda_image);
	checkCudaErrors(cudaDeviceSynchronize());

	if (profile)
	{
		sdkStopTimer(&timer);
		printf("\t\tcudaGetImage(): getImageKernel: %f\n", sdkGetTimerValue(&timer));
		sdkResetTimer(&timer);
		sdkStartTimer(&timer);
	}

	checkCudaErrors(cudaHostRegister(out_data, out_data_size, cudaHostRegisterDefault));
	checkCudaErrors(cudaMemcpyAsync((void *)out_data, (const void *)cuda_image, out_data_size, cudaMemcpyDeviceToHost, 0));
	checkCudaErrors(cudaDeviceSynchronize());
	checkCudaErrors(cudaHostUnregister(out_data));

	if (profile)
	{
		sdkStopTimer(&timer);
		printf("\t\tcudaGetImage(): transfer out Image  lvl=%d  wl=%d  hl=%d: %f\n", lvl, wl, hl, sdkGetTimerValue(&timer));
		sdkResetTimer(&timer);
		sdkDeleteTimer(&timer);
	}

	/*
	Eigen::Vector3f* colorRef = (Eigen::Vector3f*)firstFrame->dIp[lvl];
	for (int i = 0; i<wl*hl; i++)
		iRImg.at(i) = Vec3b(colorRef[i][0], colorRef[i][0], colorRef[i][0]);


	int npts = numPoints[lvl];

	float nid = 0, sid = 0;
	for (int i = 0; i<npts; i++)
	{
		Pnt* point = points[lvl] + i;
		if (point->isGood)
		{
			nid++;
			sid += point->iR;
		}
	}
	float fac = nid / sid;



	for (int i = 0; i<npts; i++)
	{
		Pnt* point = points[lvl] + i;

		if (!point->isGood)
			iRImg.setPixel9(point->u + 0.5f, point->v + 0.5f, Vec3b(0, 0, 0));

		else
			iRImg.setPixel9(point->u + 0.5f, point->v + 0.5f, makeRainbow3B(point->iR*fac));
	}
*/
}

extern "C"
void cudaGetFirstFrameImage(int lvl, float factor, unsigned char* out_data, bool profile)
{
	StopWatchInterface *timer = NULL;
	if (profile)
	{
		sdkCreateTimer(&timer);
		sdkStartTimer(&timer);
	}

	size_t out_data_size = wG[lvl] * hG[lvl] * sizeof(uchar3);
	if (cuda_image == NULL) checkCudaErrors(cudaMalloc((void **)&cuda_image, out_data_size));
	int threadsPerBlock = 256;
	int blocksPerGrid = (wG[lvl] * hG[lvl] + threadsPerBlock - 1) / threadsPerBlock;
	getImageKernel << <blocksPerGrid, threadsPerBlock >> >(wG[lvl], hG[lvl], (float*)(firstFrame->dIp[lvl]), factor, cuda_image);
	checkCudaErrors(cudaDeviceSynchronize());

	if (profile)
	{
		sdkStopTimer(&timer);
		printf("\t\tcudaGetFirstFrameImage(): getImageKernel: %f\n", sdkGetTimerValue(&timer));
		sdkResetTimer(&timer);
		sdkStartTimer(&timer);
	}

	checkCudaErrors(cudaHostRegister(out_data, out_data_size, cudaHostRegisterDefault));
	checkCudaErrors(cudaMemcpyAsync((void *)out_data, (const void *)cuda_image, out_data_size, cudaMemcpyDeviceToHost, 0));
	checkCudaErrors(cudaDeviceSynchronize());
	checkCudaErrors(cudaHostUnregister(out_data));

	if (profile)
	{
		sdkStopTimer(&timer);
		printf("\t\tcudaGetFirstFrameImage(): transfer out Image  lvl=%d  wl=%d  hl=%d: %f\n", lvl, wG[lvl], hG[lvl], sdkGetTimerValue(&timer));
		sdkResetTimer(&timer);
		sdkDeleteTimer(&timer);
	}
}

extern "C"
void cudaGetDepthImage(int index, int lvl, float factor, float* idepth, float minID, float maxID, unsigned char* out_data, bool profile)
{
	StopWatchInterface *timer = NULL;
	if (profile)
	{
		sdkCreateTimer(&timer);
		sdkStartTimer(&timer);
	}

	size_t idepth_size = wG[lvl] * hG[lvl] * sizeof(float);
	checkCudaErrors(cudaHostRegister(idepth, idepth_size, cudaHostRegisterDefault));
	if (cuda_idepth == NULL) checkCudaErrors(cudaMalloc((void **)&cuda_idepth, idepth_size));
	checkCudaErrors(cudaMemcpyAsync((void *)cuda_idepth, (const void *)idepth, idepth_size, cudaMemcpyHostToDevice, 0));
	checkCudaErrors(cudaDeviceSynchronize());
	checkCudaErrors(cudaHostUnregister(idepth));

	if (profile)
	{
		sdkStopTimer(&timer);
		printf("\t\t\t\t\tcudaGetDepthImage2: transfer in idepth: %f\n", sdkGetTimerValue(&timer));
		sdkResetTimer(&timer);
		sdkStartTimer(&timer);
	}

	size_t out_data_size = wG[lvl] * hG[lvl] * sizeof(uchar3);
	if (cuda_image == NULL) checkCudaErrors(cudaMalloc((void **)&cuda_image, out_data_size));
	int threadsPerBlock = 256;
	int blocksPerGrid = (wG[lvl] * hG[lvl] + threadsPerBlock - 1) / threadsPerBlock;
	makeDepthImageKernel << <blocksPerGrid, threadsPerBlock >> >(wG[lvl], hG[lvl], (float*)(frameHessians_cuda[index]->dIp[lvl]), factor, cuda_idepth, minID, maxID, cuda_image);

	if (profile)
	{
		sdkStopTimer(&timer);
		printf("\t\t\t\t\tcudaGetDepthImage2:makeDepthImageKernel: %f\n", sdkGetTimerValue(&timer));
		sdkResetTimer(&timer);
		sdkStartTimer(&timer);
	}

	checkCudaErrors(cudaHostRegister(out_data, out_data_size, cudaHostRegisterDefault));
	checkCudaErrors(cudaMemcpyAsync((void *)out_data, (const void *)cuda_image, out_data_size, cudaMemcpyDeviceToHost, 0));
	checkCudaErrors(cudaDeviceSynchronize());
	checkCudaErrors(cudaHostUnregister(out_data));


	if (profile)
	{
		sdkStopTimer(&timer);
		printf("\t\t\t\t\tcudaGetDepthImage2(): transfer out Image: %f\n", sdkGetTimerValue(&timer));
		sdkResetTimer(&timer);
		sdkDeleteTimer(&timer);
	}
}

extern "C"
void cudaAddFirstFrame(bool profile)
{
	StopWatchInterface *timer = NULL;
	if (profile)
	{
		sdkCreateTimer(&timer);
		sdkStartTimer(&timer);
	}

	for (int lvl = 0; lvl < pyrLevelsUsed; lvl++)
	{
		size_t size = wG[lvl] * hG[lvl] * sizeof(Vector3f);
		if(firstFrame->dIp[lvl] == NULL) checkCudaErrors(cudaMalloc((void **)&(firstFrame->dIp[lvl]), size));
		checkCudaErrors(cudaMemcpy((void *)(firstFrame->dIp[lvl]), (const void *)cuda_dIp[lvl], size, cudaMemcpyDeviceToDevice));

		size = wG[lvl] * hG[lvl] * sizeof(float);
		if (firstFrame->absSquaredGrad[lvl] == NULL) checkCudaErrors(cudaMalloc((void **)&(firstFrame->absSquaredGrad[lvl]), size));
		checkCudaErrors(cudaMemcpy((void *)(firstFrame->absSquaredGrad[lvl]), (const void *)cuda_absSquaredGrad[lvl], size, cudaMemcpyDeviceToDevice));
	}

	if (profile)
	{
		sdkStopTimer(&timer);
		printf("\t\t\t\t\tcudaAddFirstFrame(): %f\n", sdkGetTimerValue(&timer));
		sdkResetTimer(&timer);
		sdkDeleteTimer(&timer);
	}
}

extern "C"
void cudaAddKeyFrame(int index, bool profile)
{
	if (index >= MAX_NUM_FRAMES) return;

	StopWatchInterface *timer = NULL;
	if (profile)
	{
		sdkCreateTimer(&timer);
		sdkStartTimer(&timer);
	}

	for (int lvl = 0; lvl < pyrLevelsUsed; lvl++)
	{
		size_t size = wG[lvl] * hG[lvl] * sizeof(Vector3f);
		if (frameHessians_cuda[index]->dIp[lvl] == NULL) checkCudaErrors(cudaMalloc((void **)&(frameHessians_cuda[index]->dIp[lvl]), size));
		checkCudaErrors(cudaMemcpy((void *)(frameHessians_cuda[index]->dIp[lvl]), (const void *)cuda_dIp[lvl], size, cudaMemcpyDeviceToDevice));

		size = wG[lvl] * hG[lvl] * sizeof(float);
		if (frameHessians_cuda[index]->absSquaredGrad[lvl] == NULL) checkCudaErrors(cudaMalloc((void **)&(frameHessians_cuda[index]->absSquaredGrad[lvl]), size));
		checkCudaErrors(cudaMemcpy((void *)(frameHessians_cuda[index]->absSquaredGrad[lvl]), (const void *)cuda_absSquaredGrad[lvl], size, cudaMemcpyDeviceToDevice));
	}

	if (profile)
	{
		sdkStopTimer(&timer);
		printf("\t\t\t\t\tcudaAddKeyFrame(): index=%d  %f\n", index, sdkGetTimerValue(&timer));
		sdkResetTimer(&timer);
		sdkDeleteTimer(&timer);
	}
}

extern "C" 
void cudaMarginalizeFrame(int index, bool profile)
{
	if (index >= MAX_NUM_FRAMES) return;

	StopWatchInterface *timer = NULL;
	if (profile)
	{
		sdkCreateTimer(&timer);
		sdkStartTimer(&timer);
	}

	FrameHessian* tmp = frameHessians_cuda[index];
	for (int i = index + 1; i < MAX_NUM_FRAMES; i++)
	{
		frameHessians_cuda[i - 1] = frameHessians_cuda[i];
		frameHessians_cuda[i - 1]->idx = i - 1;
	}
	frameHessians_cuda[MAX_NUM_FRAMES - 1] = tmp;

	if (profile)
	{
		sdkStopTimer(&timer);
		printf("\t\t\t\t\tcudaMarginalizeFrame(): index=%d  %f\n", index, sdkGetTimerValue(&timer));
		sdkResetTimer(&timer);
		sdkDeleteTimer(&timer);
	}
}


extern "C"
void cudaSetPointFrameResidual(void* r, void* idx, int num, bool profile)
{
	if (num == 0) return;
	num_pfr = num;

	StopWatchInterface *timer = NULL;
	if (profile)
	{
		printf("\t\t\t\t\t\t\tcudaSetPointFrameResidual(): num=%d\n", num* sizeof(cuda_PointFrameResidual));
		sdkCreateTimer(&timer);
		sdkStartTimer(&timer);
	}

	checkCudaErrors(cudaHostRegister((void *)r, num * sizeof(cuda_PointFrameResidual), cudaHostRegisterDefault));
	if (pfr != NULL) checkCudaErrors(cudaFree((void*)pfr));
	checkCudaErrors(cudaMalloc((void **)&pfr, num * sizeof(cuda_PointFrameResidual)));
	checkCudaErrors(cudaMemcpyAsync((void *)pfr, (const void *)r, num * sizeof(cuda_PointFrameResidual), cudaMemcpyHostToDevice, 0));
	checkCudaErrors(cudaDeviceSynchronize());
	checkCudaErrors(cudaHostUnregister(r));

	if (targetIdxs != NULL) checkCudaErrors(cudaFreeHost((void*)targetIdxs));
	checkCudaErrors(cudaMallocHost((void **)&targetIdxs, num * sizeof(int)));
	checkCudaErrors(cudaMemcpy((void *)targetIdxs, (const void *)idx, num * sizeof(int), cudaMemcpyHostToHost));
	checkCudaErrors(cudaDeviceSynchronize());

	//cuda_PointFrameResidual * p = NULL;
	//checkCudaErrors(cudaMallocHost((void **)&p, num * sizeof(cuda_PointFrameResidual)));
	//checkCudaErrors(cudaMemcpy((void *)p, (const void *)r, num * sizeof(cuda_PointFrameResidual), cudaMemcpyHostToHost));
	//checkCudaErrors(cudaDeviceSynchronize());
	//for (int i = 0; i < num; i++)
	//	printf("i=%d p->u=%f  ", i, (p + i)->u);
	printf("RawResidualJacobian size=%d\n", sizeof(RawResidualJacobian));
	printf("cuda_PointFrameResidual size=%d\n", sizeof(cuda_PointFrameResidual));

	if (profile)
	{
		sdkStopTimer(&timer);
		printf("\t\t\t\t\t\t\tcudaSetPointFrameResidual(): transfer in pointFrameResidual: %f\n", sdkGetTimerValue(&timer));
		sdkResetTimer(&timer);
		sdkDeleteTimer(&timer);
	}
}

__global__
void linearizeKernel(bool fixLinearization, int num, cuda_PointFrameResidual* pfr, Vector3f* ptr[], CalibHessian* HCalib, int w, float wM3G, float hM3G)
{
	// calculate normalized textures coordinates
	unsigned int idx = blockIdx.x*blockDim.x + threadIdx.x;
	if (idx >= num) return;

	cuda_PointFrameResidual* p = (cuda_PointFrameResidual*)pfr + idx;
	p->stats0 = linearize(p, ptr[idx], HCalib, w, wM3G, hM3G);

	p->isGoodResidual = false;
	p->toRemove = false;
	if (fixLinearization)
	{
		applyRes(p, true);

		if (p->efResidual.isActiveAndIsGoodNEW)
		{
			if (p->isNew)
			{
				Vec3f ptp_inf = mat33f_mul_vec3f(p->PRE_KRKiTll, make_Vec3f(p->u, p->v, 1));	// projected point assuming infinite depth.
				Vec3f ptp = Vec3f_add(ptp_inf, Vec3f_mul_float(p->PRE_KtTll, p->idepth_scaled));	// projected point with real depth.
				//PointHessian* p = r->point;
				//Vec3f ptp_inf = r->host->targetPrecalc[r->target->idx].PRE_KRKiTll * Vec3f(p->u, p->v, 1);	// projected point assuming infinite depth.
				//Vec3f ptp = ptp_inf + r->host->targetPrecalc[r->target->idx].PRE_KtTll*p->idepth_scaled;	// projected point with real depth.

				Vec2f delta = Vec2f_sub(make_Vec2f(ptp_inf.x / ptp_inf.z, ptp_inf.y / ptp_inf.z), make_Vec2f(ptp.x / ptp.z, ptp.y / ptp.z));
				float relBS = 0.01*(sqrtf(delta.x * delta.x + delta.y * delta.y));	// 0.01 = one pixel.
				//float relBS = 0.01*((ptp_inf.head<2>() / ptp_inf[2]) - (ptp.head<2>() / ptp[2])).norm();	// 0.01 = one pixel.


				if (relBS > p->maxRelBaseline)
					p->maxRelBaseline = relBS;

				p->isGoodResidual = true;
			}
		}
		else
		{
			p->toRemove = true;
		}
	}

}


extern "C"
void cudaLinearize(bool fixLinearization, void* ar, bool profile)
{
	StopWatchInterface *timer = NULL;
	if (profile)
	{
		sdkCreateTimer(&timer);
		sdkStartTimer(&timer);
	}

	Vector3f** ptr = NULL;
	checkCudaErrors(cudaMallocHost((void **)&ptr, num_pfr * sizeof(Vector3f*)));
	for (int i = 0; i < num_pfr; i++)
		ptr[i] = frameHessians_cuda[targetIdxs[i]]->dIp[0];
	Vector3f** cuda_ptr = NULL;
	checkCudaErrors(cudaMalloc((void **)&cuda_ptr, num_pfr * sizeof(Vector3f*)));
	checkCudaErrors(cudaMemcpy(cuda_ptr, ptr, num_pfr * sizeof(Vector3f*), cudaMemcpyHostToDevice));
	checkCudaErrors(cudaFreeHost(ptr));

	int threadsPerBlock = 256;
	int blocksPerGrid = (num_pfr + threadsPerBlock - 1) / threadsPerBlock;
	linearizeKernel << <blocksPerGrid, threadsPerBlock >> >(fixLinearization, num_pfr, pfr, cuda_ptr, HCalib, wG[0], wM3G, hM3G);
	checkCudaErrors(cudaDeviceSynchronize());
	checkCudaErrors(cudaFree(cuda_ptr));

	checkCudaErrors(cudaHostRegister((void *)ar, num_pfr * sizeof(cuda_PointFrameResidual), cudaHostRegisterDefault));
	checkCudaErrors(cudaMemcpyAsync((void *)ar, (const void *)pfr, num_pfr * sizeof(cuda_PointFrameResidual), cudaMemcpyDeviceToHost, 0));
	checkCudaErrors(cudaDeviceSynchronize());
	checkCudaErrors(cudaHostUnregister(ar));


	if (profile)
	{
		sdkStopTimer(&timer);
		printf("\t\t\t\t\t\t\tcudaLinearize(): %f\n", sdkGetTimerValue(&timer));
		sdkResetTimer(&timer);
		sdkDeleteTimer(&timer);
	}

}
