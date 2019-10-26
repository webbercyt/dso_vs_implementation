#ifndef __SETTINGS_CUH__
#define __SETTINGS_CUH__

#define SOLVER_SVD (int)1
#define SOLVER_ORTHOGONALIZE_SYSTEM (int)2
#define SOLVER_ORTHOGONALIZE_POINTMARG (int)4
#define SOLVER_ORTHOGONALIZE_FULL (int)8
#define SOLVER_SVD_CUT7 (int)16
#define SOLVER_REMOVE_POSEPRIOR (int)32
#define SOLVER_USE_GN (int)64
#define SOLVER_FIX_LAMBDA (int)128
#define SOLVER_ORTHOGONALIZE_X (int)256
#define SOLVER_MOMENTUM (int)512
#define SOLVER_STEPMOMENTUM (int)1024
#define SOLVER_ORTHOGONALIZE_X_LATER (int)2048


// ============== PARAMETERS TO BE DECIDED ON COMPILE TIME =================
#define PYR_LEVELS 6
//
#define patternNum 8
#define patternP staticPattern[8]
#define patternPadding 2

int pyrLevelsUsed = PYR_LEVELS;



/* Parameters controlling when KF's are taken */
float setting_keyframesPerSecond = 0;   // if !=0, takes a fixed number of KF per second.
bool setting_realTimeMaxKF = false;   // if true, takes as many KF's as possible (will break the system if the camera stays stationary)
float setting_maxShiftWeightT= 0.04f * (640+480);
float setting_maxShiftWeightR= 0.0f * (640+480);
float setting_maxShiftWeightRT= 0.02f * (640+480);
float setting_kfGlobalWeight = 1;   // general weight on threshold, the larger the more KF's are taken (e.g., 2 = double the amount of KF's).
float setting_maxAffineWeight= 2;


/* initial hessian values to fix unobservable dimensions / priors on affine lighting parameters.
 */
float setting_idepthFixPrior = 50*50;
float setting_idepthFixPriorMargFac = 600*600;
float setting_initialRotPrior = 1e11;
float setting_initialTransPrior = 1e10;
float setting_initialAffBPrior = 1e14;
float setting_initialAffAPrior = 1e14;
float setting_initialCalibHessian = 5e9;

/* some modes for solving the resulting linear system (e.g. orthogonalize wrt. unobservable dimensions) */
int setting_solverMode = SOLVER_FIX_LAMBDA | SOLVER_ORTHOGONALIZE_X_LATER;
double setting_solverModeDelta = 0.00001;
bool setting_forceAceptStep = true;

/* some thresholds on when to activate / marginalize points */
float setting_minIdepthH_act = 100;
float setting_minIdepthH_marg = 50;

float setting_desiredImmatureDensity = 800; // immature points per frame
float setting_desiredPointDensity = 1000; // aimed total points in the active window.
float setting_minPointsRemaining = 0.05;  // marg a frame if less than X% points remain.
float setting_maxLogAffFacInWindow = 0.7; // marg a frame if factor between intensities to current frame is larger than 1/X or X.

int   setting_minFrames = 5; // min frames in window.
int   setting_maxFrames = 7; // max frames in window.
int   setting_minFrameAge = 1;
int   setting_maxOptIterations=6; // max GN iterations.
int   setting_minOptIterations=1; // min GN iterations.
float setting_thOptIterations=1.2; // factor on break threshold for GN iteration (larger = break earlier)

/* Outlier Threshold on photometric energy */
float setting_outlierTH = 12*12;					// higher -> less strict
__constant__ float setting_outlierTHSumComponent = 50*50; 		// higher -> less strong gradient-based reweighting .

int setting_pattern = 8;						// point pattern used. DISABLED.
float setting_margWeightFac = 0.5*0.5;          // factor on hessian when marginalizing, to account for inaccurate linearization points.

/* when to re-track a frame */
float setting_reTrackThreshold = 1.5; // (larger = re-track more often)

/* require some minimum number of residuals for a point to become valid */
int   setting_minGoodActiveResForMarg=3;
int   setting_minGoodResForMarg=4;

// 0 = nothing.
// 1 = apply inv. response.
// 2 = apply inv. response & remove V.
int setting_photometricCalibration = 2;
bool setting_useExposure = true;
__constant__ float setting_affineOptModeA = 1e12; //-1: fix. >=0: optimize (with prior, if > 0).
__constant__ float setting_affineOptModeB = 1e8; //-1: fix. >=0: optimize (with prior, if > 0).

__constant__ int setting_gammaWeightsPixelSelect = 1; // 1 = use original intensity for pixel selection; 0 = use gamma-corrected intensity.

__constant__ float setting_huberTH = 9; // Huber Threshold

// parameters controlling adaptive energy threshold computation.
float setting_frameEnergyTHConstWeight = 0.5;
float setting_frameEnergyTHN = 0.7f;
float setting_frameEnergyTHFacMedian = 1.5;
float setting_overallEnergyTHWeight = 1;
float setting_coarseCutoffTH = 20;

// parameters controlling pixel selection
float setting_minGradHistCut = 0.5;
float setting_minGradHistAdd = 7;
float setting_gradDownweightPerLevel = 0.75;
bool  setting_selectDirectionDistribution = true;

/* settings controling initial immature point tracking */
float setting_maxPixSearch = 0.027; // max length of the ep. line segment searched during immature point tracking. relative to image resolution.
float setting_minTraceQuality = 3;
int setting_minTraceTestRadius = 2;
int setting_GNItsOnPointActivation = 3;
float setting_trace_stepsize = 1.0;				// stepsize for initial discrete search.
int setting_trace_GNIterations = 3;				// max # GN iterations
float setting_trace_GNThreshold = 0.1;				// GN stop after this stepsize.
float setting_trace_extraSlackOnTH = 1.2;			// for energy-based outlier check, be slightly more relaxed by this factor.
float setting_trace_slackInterval = 1.5;			// if pixel-interval is smaller than this, leave it be.
float setting_trace_minImprovementFactor = 2;		// if pixel-interval is smaller than this, leave it be.

// for benchmarking different undistortion settings
float benchmarkSetting_fxfyfac = 0;
int benchmarkSetting_width = 0;
int benchmarkSetting_height = 0;
float benchmark_varNoise = 0;
float benchmark_varBlurNoise = 0;
float benchmark_initializerSlackFactor = 1;
int benchmark_noiseGridsize = 3;

float freeDebugParam1 = 1;
float freeDebugParam2 = 1;
float freeDebugParam3 = 1;
float freeDebugParam4 = 1;
float freeDebugParam5 = 1;

bool disableReconfigure=false;
bool debugSaveImages = false;
bool multiThreading = true;
bool disableAllDisplay = false;
bool setting_onlyLogKFPoses = true;
bool setting_logStuff = true;

bool goStepByStep = false;

bool setting_render_displayCoarseTrackingFull=false;
bool setting_render_renderWindowFrames = false;
bool setting_render_plotTrackingFull = false;
bool setting_render_display3D = true;
bool setting_render_displayResidual = true;
bool setting_render_displayVideo = true;
bool setting_render_displayDepth = true;

bool setting_fullResetRequested = false;

bool setting_debugout_runquiet = false;

int sparsityFactor = 5;	// not actually a setting, only some legacy stuff for coarse initializer.

void handleKey(char k)
{
	char kkk = k;
	switch(kkk)
	{
	case 'd': case 'D':
		freeDebugParam5 = ((int)(freeDebugParam5+1))%10;
		printf("new freeDebugParam5: %f!\n", freeDebugParam5);
		break;
	case 's': case 'S':
		freeDebugParam5 = ((int)(freeDebugParam5-1+10))%10;
		printf("new freeDebugParam5: %f!\n", freeDebugParam5);
		break;
	}

}

__constant__
int staticPattern[10][40][2] = {
		{{0,0}, 	  {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100},	// .
		 {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100},
		 {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100},
		 {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}},

		{{0,-1},	  {-1,0},	   {0,0},	    {1,0},	     {0,1}, 	  {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100},	// +
		 {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100},
		 {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100},
		 {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}},

		{{-1,-1},	  {1,1},	   {0,0},	    {-1,1},	     {1,-1}, 	  {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100},	// x
		 {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100},
		 {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100},
		 {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}},

		{{-1,-1},	  {-1,0},	   {-1,1},		{-1,0},		 {0,0},		  {0,1},	   {1,-1},		{1,0},		 {1,1},       {-100,-100},	// full-tight
		 {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100},
		 {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100},
		 {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}},

		{{0,-2},	  {-1,-1},	   {1,-1},		{-2,0},		 {0,0},		  {2,0},	   {-1,1},		{1,1},		 {0,2},       {-100,-100},	// full-spread-9
		 {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100},
		 {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100},
		 {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}},

		{{0,-2},	  {-1,-1},	   {1,-1},		{-2,0},		 {0,0},		  {2,0},	   {-1,1},		{1,1},		 {0,2},       {-2,-2},   // full-spread-13
		 {-2,2},      {2,-2},      {2,2},       {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100},
		 {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100},
		 {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}},

		{{-2,-2},     {-2,-1}, {-2,-0}, {-2,1}, {-2,2}, {-1,-2}, {-1,-1}, {-1,-0}, {-1,1}, {-1,2}, 										// full-25
		 {-0,-2},     {-0,-1}, {-0,-0}, {-0,1}, {-0,2}, {+1,-2}, {+1,-1}, {+1,-0}, {+1,1}, {+1,2},
		 {+2,-2}, 	  {+2,-1}, {+2,-0}, {+2,1}, {+2,2}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100},
		 {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}},

		{{0,-2},	  {-1,-1},	   {1,-1},		{-2,0},		 {0,0},		  {2,0},	   {-1,1},		{1,1},		 {0,2},       {-2,-2},   // full-spread-21
		 {-2,2},      {2,-2},      {2,2},       {-3,-1},     {-3,1},      {3,-1}, 	   {3,1},       {1,-3},      {-1,-3},     {1,3},
		 {-1,3},      {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100},
		 {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}},

		{{0,-2},	  {-1,-1},	   {1,-1},		{-2,0},		 {0,0},		  {2,0},	   {-1,1},		{0,2},		 {-100,-100}, {-100,-100},	// 8 for SSE efficiency
		 {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100},
		 {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100},
		 {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}},

		{{-4,-4},     {-4,-2}, {-4,-0}, {-4,2}, {-4,4}, {-2,-4}, {-2,-2}, {-2,-0}, {-2,2}, {-2,4}, 										// full-45-SPREAD
		 {-0,-4},     {-0,-2}, {-0,-0}, {-0,2}, {-0,4}, {+2,-4}, {+2,-2}, {+2,-0}, {+2,2}, {+2,4},
		 {+4,-4}, 	  {+4,-2}, {+4,-0}, {+4,2}, {+4,4}, {-200,-200}, {-200,-200}, {-200,-200}, {-200,-200}, {-200,-200},
		 {-200,-200}, {-200,-200}, {-200,-200}, {-200,-200}, {-200,-200}, {-200,-200}, {-200,-200}, {-200,-200}, {-200,-200}, {-200,-200}},
};

__constant__
int staticPatternNum[10] = {
		1,
		5,
		5,
		9,
		9,
		13,
		25,
		21,
		8,
		25
};

__constant__
int staticPatternPadding[10] = {
		1,
		1,
		1,
		1,
		2,
		2,
		2,
		3,
		2,
		4
};


#endif // !__SETTINGS_CUH__
