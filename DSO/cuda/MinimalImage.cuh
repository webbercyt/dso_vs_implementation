#ifndef __MINIMALL_IMAGE_CUH__
#define __MINIMALL_IMAGE_CUH__

#include "algorithm"

struct CudaMinimalImageUInt
{
	int w;
	int h;
	unsigned short* data;
	bool ownData;
};

struct CudaMinimalImageUChar
{
	int w;
	int h;
	unsigned char* data;
	bool ownData;
};


/*
 * creates minimal image with own memory
 */
extern "C"
void cudaSetMinimalImageUInt(CudaMinimalImageUInt& mi, int w_, int h_)
{
	mi.w = w_;
	mi.h = h_;
	checkCudaErrors(cudaMalloc((void **)&mi.data, mi.w * mi.h * sizeof(unsigned short)));
	mi.ownData = true;
}

/*
 * creates minimal image wrapping around existing memory
 */
extern "C"
void cudaSetMinimalImageUInt(CudaMinimalImageUInt& mi, int w_, int h_, unsigned short* data_)
{
	mi.w = w_;
	mi.h = h_;
	mi.data = data_;
	mi.ownData = false;
}

extern "C"
void cudaFreeMinimalImageUInt(CudaMinimalImageUInt& mi)
{
	if(mi.ownData) checkCudaErrors(cudaFree((void *)mi.data));
}

extern "C"
CudaMinimalImageUInt* cudaGetClone(CudaMinimalImageUInt& mi)
{
	CudaMinimalImageUInt* clone = new CudaMinimalImageUInt();
	clone->w = mi.w;
	clone->h = mi.h;
	checkCudaErrors(cudaMemcpy((void *)clone->data, (void *)mi.data, mi.w * mi.h * sizeof(unsigned short), cudaMemcpyDeviceToDevice));
	return clone;
}

extern "C"
unsigned short& cudaAt(CudaMinimalImageUInt& mi, int x, int y)
{
	return mi.data[(int)x+((int)y)*mi.w];
}

extern "C"
unsigned short& cudaAt(CudaMinimalImageUInt& mi, int i)
{
	return mi.data[i];
}

extern "C"
unsigned short& cudaAt(CudaMinimalImageUInt* mi, int x, int y)
{
	return mi->data[(int)x + ((int)y)*mi->w];
}

extern "C"
unsigned short& cudaAt(CudaMinimalImageUInt* mi, int i)
{
	return mi->data[i];
}

extern "C"
void cudaSetBlack(CudaMinimalImageUInt& mi)
{
	checkCudaErrors(cudaMemset((void *)mi.data, 0, mi.w * mi.h * sizeof(unsigned short)));
}

extern "C"
void cudaSetConst(CudaMinimalImageUInt& mi, unsigned short val)
{
	for (int i = 0; i < mi.w * mi.h; i++) mi.data[i] = val;
}

extern "C"
void cudaSetPixel1(CudaMinimalImageUInt& mi, const float &u, const float &v, unsigned short val)
{
	cudaAt(mi, u + 0.5f, v + 0.5f) = val;
}

extern "C"
void cudaSetPixel4(CudaMinimalImageUInt& mi, const float &u, const float &v, unsigned short val)
{
	cudaAt(mi, u + 1.0f, v + 1.0f) = val;
	cudaAt(mi, u + 1.0f, v) = val;
	cudaAt(mi, u, v + 1.0f) = val;
	cudaAt(mi, u, v) = val;
}

extern "C"
void cudaSetPixel9(CudaMinimalImageUInt& mi, const int &u, const int &v, unsigned short val)
{
	cudaAt(mi, u + 1, v - 1) = val;
	cudaAt(mi, u + 1, v) = val;
	cudaAt(mi, u + 1, v + 1) = val;
	cudaAt(mi, u, v - 1) = val;
	cudaAt(mi, u, v) = val;
	cudaAt(mi, u, v + 1) = val;
	cudaAt(mi, u - 1, v - 1) = val;
	cudaAt(mi, u - 1, v) = val;
	cudaAt(mi, u - 1, v + 1) = val;
}

extern "C"
void cudaSetPixelCirc(CudaMinimalImageUInt& mi, const int &u, const int &v, unsigned short val)
{
	for (int i = -3; i <= 3; i++)
	{
		cudaAt(mi, u + 3, v + i) = val;
		cudaAt(mi, u - 3, v + i) = val;
		cudaAt(mi, u + 2, v + i) = val;
		cudaAt(mi, u - 2, v + i) = val;

		cudaAt(mi, u + i, v - 3) = val;
		cudaAt(mi, u + i, v + 3) = val;
		cudaAt(mi, u + i, v - 2) = val;
		cudaAt(mi, u + i, v + 2) = val;
	}
}



/*
* creates minimal image with own memory
*/
extern "C"
void cudaSetMinimalImageUChar(CudaMinimalImageUChar& mi, int w_, int h_)
{
	mi.w = w_;
	mi.h = h_;
	checkCudaErrors(cudaMalloc((void **)&mi.data, mi.w * mi.h * sizeof(unsigned char)));
	mi.ownData = true;
}

/*
* creates minimal image wrapping around existing memory
*/
extern "C"
void cudaSetMinimalImageUChar(CudaMinimalImageUChar& mi, int w_, int h_, unsigned char* data_)
{
	mi.w = w_;
	mi.h = h_;
	mi.data = data_;
	mi.ownData = false;
}

extern "C"
void cudaFreeMinimalImageUChar(CudaMinimalImageUChar& mi)
{
	if (mi.ownData) checkCudaErrors(cudaFree((void *)mi.data));
}

extern "C"
CudaMinimalImageUChar* cudaGetClone(CudaMinimalImageUChar& mi)
{
	CudaMinimalImageUChar* clone = new CudaMinimalImageUChar();
	clone->w = mi.w;
	clone->h = mi.h;
	checkCudaErrors(cudaMemcpy((void *)clone->data, (void *)mi.data, mi.w * mi.h * sizeof(unsigned char), cudaMemcpyDeviceToDevice));
	return clone;
}

extern "C"
unsigned char& cudaAt(CudaMinimalImageUChar& mi, int x, int y)
{
	return mi.data[(int)x + ((int)y)*mi.w];
}

extern "C"
unsigned char& cudaAt(CudaMinimalImageUChar& mi, int i)
{
	return mi.data[i];
}

extern "C"
unsigned char& cudaAt(CudaMinimalImageUChar* mi, int x, int y)
{
	return mi->data[(int)x + ((int)y)*mi->w];
}

extern "C"
unsigned char& cudaAt(CudaMinimalImageUChar* mi, int i)
{
	return mi->data[i];
}

extern "C"
void cudaSetBlack(CudaMinimalImageUChar& mi)
{
	checkCudaErrors(cudaMemset((void *)mi.data, 0, mi.w * mi.h * sizeof(unsigned char)));
}

extern "C"
void cudaSetConst(CudaMinimalImageUChar& mi, unsigned char val)
{
	for (int i = 0; i < mi.w * mi.h; i++) mi.data[i] = val;
}

extern "C"
void cudaSetPixel1(CudaMinimalImageUChar& mi, const float &u, const float &v, unsigned char val)
{
	cudaAt(mi, u + 0.5f, v + 0.5f) = val;
}

extern "C"
void cudaSetPixel4(CudaMinimalImageUChar& mi, const float &u, const float &v, unsigned char val)
{
	cudaAt(mi, u + 1.0f, v + 1.0f) = val;
	cudaAt(mi, u + 1.0f, v) = val;
	cudaAt(mi, u, v + 1.0f) = val;
	cudaAt(mi, u, v) = val;
}

extern "C"
void cudaSetPixel9(CudaMinimalImageUChar& mi, const int &u, const int &v, unsigned char val)
{
	cudaAt(mi, u + 1, v - 1) = val;
	cudaAt(mi, u + 1, v) = val;
	cudaAt(mi, u + 1, v + 1) = val;
	cudaAt(mi, u, v - 1) = val;
	cudaAt(mi, u, v) = val;
	cudaAt(mi, u, v + 1) = val;
	cudaAt(mi, u - 1, v - 1) = val;
	cudaAt(mi, u - 1, v) = val;
	cudaAt(mi, u - 1, v + 1) = val;
}

extern "C"
void cudaSetPixelCirc(CudaMinimalImageUChar& mi, const int &u, const int &v, unsigned char val)
{
	for (int i = -3; i <= 3; i++)
	{
		cudaAt(mi, u + 3, v + i) = val;
		cudaAt(mi, u - 3, v + i) = val;
		cudaAt(mi, u + 2, v + i) = val;
		cudaAt(mi, u - 2, v + i) = val;

		cudaAt(mi, u + i, v - 3) = val;
		cudaAt(mi, u + i, v + 3) = val;
		cudaAt(mi, u + i, v - 2) = val;
		cudaAt(mi, u + i, v + 2) = val;
	}
}


//typedef Eigen::Matrix<unsigned char,3,1> Vec3b;
//typedef MinimalImageUInt<float> MinimalImageUIntF;
//typedef MinimalImageUInt<Vec3f> MinimalImageUIntF3;
//typedef MinimalImageUInt<unsigned char> MinimalImageUIntB;
//typedef MinimalImageUInt<Vec3b> MinimalImageUIntB3;
//typedef MinimalImageUInt<unsigned short> MinimalImageUIntB16;

#endif // !__MINIMALL_IMAGE_CUH__
