#ifndef __RAW_RESIDUAL_JACOBIAN_CUH__
#define __RAW_RESIDUAL_JACOBIAN_CUH__

#include "cuda/NumType.cuh"

struct RawResidualJacobian
{
	// ================== new structure: save independently =============.
	VecNRf resF;

	// the two rows of d[x,y]/d[xi].
	Vec6f Jpdxi0;			// 2x6
	Vec6f Jpdxi1;
	// the two rows of d[x,y]/d[C].
	VecCf Jpdc0;			// 2x4
	VecCf Jpdc1;
	// the two rows of d[x,y]/d[idepth].
	Vec2f Jpdd;				// 2x1

	// the two columns of d[r]/d[x,y].
	VecNRf JIdx0;			// 9x2
	VecNRf JIdx1;
	// = the two columns of d[r] / d[ab]
	VecNRf JabF0;			// 9x2
	VecNRf JabF1;

	// = JIdx^T * JIdx (inner product). Only as a shorthand.
	Mat22f JIdx2;				// 2x2
	// = Jab^T * JIdx (inner product). Only as a shorthand.
	Mat22f JabJIdx;			// 2x2
	// = Jab^T * Jab (inner product). Only as a shorthand.
	Mat22f Jab2;			// 2x2

};

#endif // ! __RAW_RESIDUAL_JACOBIAN_CUH__
