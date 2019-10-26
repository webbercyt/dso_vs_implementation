#ifndef __GLOBAL_CALIB_CUH__
#define __GLOBAL_CALIB_CUH__


#include <stdlib.h>
#include <stdio.h>

#include "cuda/settings.cuh"
#include "cuda/NumType.cuh"


int wG[PYR_LEVELS], hG[PYR_LEVELS];
float fxG[PYR_LEVELS], fyG[PYR_LEVELS],
		cxG[PYR_LEVELS], cyG[PYR_LEVELS];

float fxiG[PYR_LEVELS], fyiG[PYR_LEVELS],
		cxiG[PYR_LEVELS], cyiG[PYR_LEVELS];

Matrix3f KG[PYR_LEVELS],KiG[PYR_LEVELS];

float wM3G;
float hM3G;


extern "C" 
void cudaSetGlobalCalib(int w, int h, float* fK)
{
	int wlvl = w;
	int hlvl = h;
	pyrLevelsUsed = 1;
	while (wlvl % 2 == 0 && hlvl % 2 == 0 && wlvl*hlvl > 5000 && pyrLevelsUsed < PYR_LEVELS)
	{
		wlvl /= 2;
		hlvl /= 2;
		pyrLevelsUsed++;
	}
	printf("using pyramid levels 0 to %d. coarsest resolution: %d x %d!\n",
		pyrLevelsUsed - 1, wlvl, hlvl);
	if (wlvl>100 && hlvl > 100)
	{
		printf("\n\n===============WARNING!===================\n "
			"using not enough pyramid levels.\n"
			"Consider scaling to a resolution that is a multiple of a power of 2.\n");
	}
	if (pyrLevelsUsed < 3)
	{
		printf("\n\n===============WARNING!===================\n "
			"I need higher resolution.\n"
			"I will probably segfault.\n");
	}

	wM3G = w - 3;
	hM3G = h - 3;

	Mat33f K;
	cudaMemcpy(&K, fK, sizeof(Mat33f), cudaMemcpyHostToHost);
	
	wG[0] = w;
	hG[0] = h;
	KG[0] = K;
	fxG[0] = K.d00;
	fyG[0] = K.d11;
	cxG[0] = K.d02;
	cyG[0] = K.d12;
	KiG[0] = inverse_Matrix3f(KG[0]);
	fxiG[0] = KiG[0].d00;
	fyiG[0] = KiG[0].d01;
	cxiG[0] = KiG[0].d02;
	cyiG[0] = KiG[0].d12;
	//fxG[0] = K(0, 0);
	//fyG[0] = K(1, 1);
	//cxG[0] = K(0, 2);
	//cyG[0] = K(1, 2);
	//KiG[0] = KG[0].inverse();
	//fxiG[0] = KiG[0](0, 0);
	//fyiG[0] = KiG[0](1, 1);
	//cxiG[0] = KiG[0](0, 2);
	//cyiG[0] = KiG[0](1, 2);

	for (int level = 1; level < pyrLevelsUsed; ++level)
	{
		wG[level] = w >> level;
		hG[level] = h >> level;

		fxG[level] = fxG[level - 1] * 0.5;
		fyG[level] = fyG[level - 1] * 0.5;
		cxG[level] = (cxG[0] + 0.5) / ((int)1 << level) - 0.5;
		cyG[level] = (cyG[0] + 0.5) / ((int)1 << level) - 0.5;

		KG[level] = make_Matrix3f(fxG[level], 0.0, cxG[level], 0.0, fyG[level], cyG[level], 0.0, 0.0, 1.0);	// synthetic
		KiG[level] = inverse_Matrix3f(KG[level]);
		//KG[level] << fxG[level], 0.0, cxG[level], 0.0, fyG[level], cyG[level], 0.0, 0.0, 1.0;	// synthetic
		//KiG[level] = KG[level].inverse();

		fxiG[level] = KiG[level].d00;
		fyiG[level] = KiG[level].d11;
		cxiG[level] = KiG[level].d02;
		cyiG[level] = KiG[level].d12;
		//fxiG[level] = KiG[level](0, 0);
		//fyiG[level] = KiG[level](1, 1);
		//cxiG[level] = KiG[level](0, 2);
		//cyiG[level] = KiG[level](1, 2);
	}
}

#endif // !__GLOBAL_CALIB_CUH__
