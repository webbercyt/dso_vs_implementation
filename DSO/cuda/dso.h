#ifndef __DSO_H__
#define __DSO_H__

extern "C" void cudaSetGlobalCalib(int w, int h, float* K);

extern "C" int cudaSetRemapData(float* remapX, float* remapY, int w, int h, bool profile);
extern "C" int cudaUndistort(float benchmark_varNoise, int benchmark_noiseGridsize, int wOrg, int hOrg, int w, int h, float* in_data, float* out_data, bool profile);


extern "C" void cudaMakeImages(float** dIp, float** absSquaredGrad, bool profile);
extern "C" void cudaGetImage(int lvl, int wl, int hl, float factor, unsigned char* out_data, bool profile);
extern "C" void cudaGetFirstFrameImage(int lvl, float factor, unsigned char* out_data, bool profile);
extern "C" void cudaGetDepthImage(int index, int lvl, float factor, float* idepth, float minID, float maxID, unsigned char* out_data, bool profile);

extern "C" void cudaAddFirstFrame(bool profile);
extern "C" void cudaAddKeyFrame(int index, bool profile);
extern "C" void cudaMarginalizeFrame(int index, bool profile);

extern "C" void cudaSetCalibHessian(void* data, bool profile);
//extern "C" void cudaSetFrameFramePrecalc(float* PRE_RTll, float* PRE_KRKiTll, float* PRE_RKiTll, float* PRE_RTll_0, float* PRE_aff_mode, float PRE_b0_mode, float* PRE_tTll, float* PRE_KtTll, float* PRE_tTll_0, float distanceLL, bool profile);
//extern "C" void cudaSetFrameFramePrecalc2(void* data, bool profile);
extern "C" void cudaSetPointFrameResidual(void* r, void* idx, int num, bool profile);
extern "C" void cudaLinearize(bool fixLinearization, void* ar, bool profile);

#endif	//__DSO_H__
