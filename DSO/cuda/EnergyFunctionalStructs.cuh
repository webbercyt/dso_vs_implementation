#ifndef  __ENERGY_FUNCTION_STRUCTS_CUH__
#define  __ENERGY_FUNCTION_STRUCTS_CUH__

//#include "util/NumType.h"
//#include "vector"
//#include <math.h>
#include "cuda/NumType.cuh"
#include "cuda/RawResidualJacobian.cuh"

struct PointFrameResidual;
struct CalibHessian;
struct FrameHessian;
struct PointHessian;

struct EFResidual;
struct EFPoint;
struct EFFrame;
struct EnergyFunctional;






struct EFResidual
{
	// structural pointers
	PointFrameResidual* data;
	int hostIDX, targetIDX;
	EFPoint* point;
	EFFrame* host;
	EFFrame* target;
	int idxInAll;

	RawResidualJacobian* J;

	VecNRf res_toZeroF;
	Vec8f JpJdF;

	// status.
	bool isLinearized;

	// if residual is not OOB & not OUTLIER & should be used during accumulations
	bool isActiveAndIsGoodNEW;
};


struct cuda_EFResidual
{
	RawResidualJacobian J;

	Vec8f JpJdF;

	// if residual is not OOB & not OUTLIER & should be used during accumulations
	bool isActiveAndIsGoodNEW;
};

enum EFPointStatus {PS_GOOD=0, PS_MARGINALIZE, PS_DROP};

struct EFPoint
{
	PointHessian* data;



	float priorF;
	float deltaF;


	// constant info (never changes in-between).
	int idxInPoints;
	EFFrame* host;

	// contains all residuals.
	//std::vector<EFResidual*> residualsAll;
	EFResidual** residualsAll;

	float bdSumF;
	float HdiF;
	float Hdd_accLF;
	VecCf Hcd_accLF;
	float bd_accLF;
	float Hdd_accAF;
	VecCf Hcd_accAF;
	float bd_accAF;


	EFPointStatus stateFlag;
};



struct EFFrame
{
	Vec8 prior;				// prior hessian (diagonal)
	Vec8 delta_prior;		// = state-state_prior (E_prior = (delta_prior)' * diag(prior) * (delta_prior)
	Vec8 delta;				// state - state_zero.



	//std::vector<EFPoint*> points;
	EFPoint** points;
	FrameHessian* data;
	int idx;	// idx in frames.

	int frameID;
};

#endif // ! __ENERGY_FUNCTION_STRUCTS_CUH__
