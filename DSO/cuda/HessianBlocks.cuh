#ifndef __HESSIAN_BLOCKS_CUH__
#define __HESSIAN_BLOCKS_CUH__

#define MAX_ACTIVE_FRAMES 100


//#include "util/globalCalib.h"
//#include "vector"
//
//#include <iostream>
//#include <fstream>
//#include "util/NumType.h"
//#include "FullSystem/Residuals.h"
//#include "util/ImageAndExposure.h"

#include "cuda/globalCalib.cuh"
#include "cuda/NumType.cuh"
#include "cuda/Residuals.cuh"

__VECTOR_FUNCTIONS_DECL__
Vec2 affFromTo(Vec2 from, Vec2 to)	// contains affine parameters as XtoWorld.
{
	return make_Vec2(from.x / to.x, (from.y - to.y) / to.x);
	//return Vec2(from[0] / to[0], (from[1] - to[1]) / to[0]);
}


struct FrameHessian;
struct PointHessian;

struct ImmaturePoint;
struct FrameShell;

struct EFFrame;
struct EFPoint;

#define SCALE_IDEPTH 1.0f		// scales internal value to idepth.
#define SCALE_XI_ROT 1.0f
#define SCALE_XI_TRANS 0.5f
#define SCALE_F 50.0f
#define SCALE_C 50.0f
#define SCALE_W 1.0f
#define SCALE_A 10.0f
#define SCALE_B 1000.0f

#define SCALE_IDEPTH_INVERSE (1.0f / SCALE_IDEPTH)
#define SCALE_XI_ROT_INVERSE (1.0f / SCALE_XI_ROT)
#define SCALE_XI_TRANS_INVERSE (1.0f / SCALE_XI_TRANS)
#define SCALE_F_INVERSE (1.0f / SCALE_F)
#define SCALE_C_INVERSE (1.0f / SCALE_C)
#define SCALE_W_INVERSE (1.0f / SCALE_W)
#define SCALE_A_INVERSE (1.0f / SCALE_A)
#define SCALE_B_INVERSE (1.0f / SCALE_B)



struct FrameFramePrecalc
{
	// static values
	static int instanceCounter;
	FrameHessian* host;	// defines row
	FrameHessian* target;	// defines column

							// precalc values
	Mat33f PRE_RTll;
	Mat33f PRE_KRKiTll;
	Mat33f PRE_RKiTll;
	Mat33f PRE_RTll_0;

	Vec2f PRE_aff_mode;
	float PRE_b0_mode;

	Vec3f PRE_tTll;
	Vec3f PRE_KtTll;
	Vec3f PRE_tTll_0;

	float distanceLL;
};

struct FrameHessian
{
	EFFrame* efFrame;

	// constant info & pre-calculated values
	//DepthImageWrap* frame;
	FrameShell* shell;

	Vector3f* dI;				 // trace, fine tracking. Used for direction select (not for gradient histograms etc.)
	Vector3f* dIp[PYR_LEVELS];	 // coarse tracking / coarse initializer. NAN in [0] only.
	float* absSquaredGrad[PYR_LEVELS];  // only used for pixel select (histograms etc.). no NAN.


	int frameID;						// incremental ID for keyframes only!
	static int instanceCounter;
	int idx;

	// Photometric Calibration Stuff
	float frameEnergyTH;	// set dynamically depending on tracking residual
	float ab_exposure;
	float timestamp;

	bool flaggedForMarginalization;

	//std::vector<PointHessian*> pointHessians;				// contains all ACTIVE points.
	//std::vector<PointHessian*> pointHessiansMarginalized;	// contains all MARGINALIZED points (= fully marginalized, usually because point went OOB.)
	//std::vector<PointHessian*> pointHessiansOut;		// contains all OUTLIER points (= discarded.).
	//std::vector<ImmaturePoint*> immaturePoints;		// contains all OUTLIER points (= discarded.).


	Mat66 nullspaces_pose;
	Mat42 nullspaces_affine;
	Vec6 nullspaces_scale;

	// variable info.
	//SE3 worldToCam_evalPT;
	Vec10 state_zero;
	Vec10 state_scaled;
	Vec10 state;	// [0-5: worldToCam-leftEps. 6-7: a,b]
	Vec10 step;
	Vec10 step_backup;
	Vec10 state_backup;


	// precalc values
	//SE3 PRE_worldToCam;
	//SE3 PRE_camToWorld;
	//std::vector<FrameFramePrecalc, Eigen::aligned_allocator<FrameFramePrecalc>> targetPrecalc;
	FrameFramePrecalc* targetPrecalc;
	//MinimalImageB3* debugImage;

};

struct CalibHessian
{
	static int instanceCounter;

	VecC value_zero;
	VecC value_scaled;
	float fxl, fyl, cxl, cyl;
	float fxli, fyli, cxli, cyli;
	VecC value;
	VecC step;
	VecC step_backup;
	VecC value_backup;
	VecC value_minus_value_zero;

	float Binv[256];
	float B[256];

};


// hessian component associated with one point.
struct PointHessian
{
	static int instanceCounter;
	EFPoint* efPoint;

	// static values
	float color[MAX_RES_PER_POINT];			// colors in host frame
	float weights[MAX_RES_PER_POINT];		// host-weights for respective residuals.



	float u, v;
	int idx;
	float energyTH;
	FrameHessian* host;
	bool hasDepthPrior;

	float my_type;

	float idepth_scaled;
	float idepth_zero_scaled;
	float idepth_zero;
	float idepth;
	float step;
	float step_backup;
	float idepth_backup;

	float nullspaces_scale;
	float idepth_hessian;
	float maxRelBaseline;
	int numGoodResiduals;

	enum PtStatus { ACTIVE = 0, INACTIVE, OUTLIER, OOB, MARGINALIZED };
	PtStatus status;

	//std::vector<PointFrameResidual*> residuals;					// only contains good residuals (not OOB and not OUTLIER). Arbitrary order.
	//std::pair<PointFrameResidual*, ResState> lastResiduals[2]; 	// contains information about residuals to the last two (!) frames. ([0] = latest, [1] = the one before).

};


#endif // ! __HESSIAN_BLOCKS_CUH__

