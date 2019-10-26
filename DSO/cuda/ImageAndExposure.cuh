#ifndef __IMAGE_AND_EXPOSURE_CUH__
#define __IMAGE_AND_EXPOSURE_CUH__

struct CudaImageAndExposure
{
	float* image;			// irradiance. between 0 and 256
	int w, h;				// width and height;
	double timestamp;
	float exposure_time;	// exposure time in ms.
};

extern "C"
void cudaSetImageAndExposure(CudaImageAndExposure &cuda_img, int w_, int h_, double timestamp_ = 0)
{
	if (cuda_img.image != NULL && (cuda_img.w != w_ || cuda_img.h != h_))
		checkCudaErrors(cudaFree((void *)cuda_img.image));
	if (cuda_img.image == NULL)
		checkCudaErrors(cudaMalloc((void **)&cuda_img.image, w_ * h_ * sizeof(float)));

	cuda_img.w = w_;
	cuda_img.h = h_;
	cuda_img.timestamp = timestamp_;
	cuda_img.exposure_time = 1;
}

extern "C"
void cudaFreeImageAndExposure(CudaImageAndExposure &cuda_img)
{
	if (cuda_img.image != NULL)
		checkCudaErrors(cudaFree((void *)cuda_img.image));
}

extern "C"
void cudaCopyMetaTo(CudaImageAndExposure &cuda_img, CudaImageAndExposure &other)
{
	other.exposure_time = cuda_img.exposure_time;
}

#endif //__IMAGE_AND_EXPOSURE_CUH__