/**
* This file is part of DSO.
* 
* Copyright 2016 Technical University of Munich and Intel.
* Developed by Jakob Engel <engelj at in dot tum dot de>,
* for more information see <http://vision.in.tum.de/dso>.
* If you use this code, please cite the respective publications as
* listed on the above website.
*
* DSO is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* DSO is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with DSO. If not, see <http://www.gnu.org/licenses/>.
*/


#pragma once

 
#include "util/globalCalib.h"
#include "vector"
 
#include "util/NumType.h"
#include <iostream>
#include <fstream>
#include "util/globalFuncs.h"
#include "OptimizationBackend/RawResidualJacobian.h"

namespace dso
{
struct PointHessian;
struct FrameHessian;
struct CalibHessian;

class EFResidual;


enum ResLocation {
	ResLocation_ACTIVE = 0,
	ResLocation_LINEARIZED,
	ResLocation_MARGINALIZED,
	ResLocation_NONE,
};
enum ResState {
	ResState_IN = 0,
	ResState_OOB,
	ResState_OUTLIER,
};

struct FullJacRowT
{
	Eigen::Vector2f projectedTo[MAX_RES_PER_POINT];
};

class PointFrameResidual
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	EFResidual* efResidual;

	static int instanceCounter;


	ResState state_state;
	double state_energy;
	ResState state_NewState;
	double state_NewEnergy;
	double state_NewEnergyWithOutlier;


	void setState(ResState s) {state_state = s;}


	PointHessian* point;
	FrameHessian* host;
	FrameHessian* target;
	RawResidualJacobian* J;


	bool isNew;


	Eigen::Vector2f projectedTo[MAX_RES_PER_POINT];
	Vec3f centerProjectedTo;

	int channel;

	~PointFrameResidual();
	PointFrameResidual(int channel);
	PointFrameResidual(int channel, PointHessian* point_, FrameHessian* host_, FrameHessian* target_);
	double linearize(CalibHessian* HCalib);


	void resetOOB()
	{
		state_NewEnergy = state_energy = 0;
		state_NewState = ResState::ResState_OUTLIER;

		setState(ResState::ResState_IN);
	};
	void applyRes( bool copyJacobians);

	void debugPlot();

	void printRows(std::vector<VecX> &v, VecX &r, int nFrames, int nPoints, int M, int res);
};

struct cuda_EFResidual
{
	struct V8f {
		float d0, d1, d2, d3, d4, d5, d6, d7;
	};

	cuda_RawResidualJacobian J;

	V8f JpJdF;

	// if residual is not OOB & not OUTLIER & should be used during accumulations
	bool isActiveAndIsGoodNEW;
};

struct cuda_PointFrameResidual
{
	struct V2f{
		float x, y;
	};
	struct V3f{
		float x, y, z;
	};
	struct M33f{
		float d00, d10, d20, d01, d11, d21, d02, d12, d22;
	};

	cuda_EFResidual efResidual;

	ResState state_state;
	double state_energy;
	ResState state_NewState;
	double state_NewEnergy;
	double state_NewEnergyWithOutlier;
	cuda_RawResidualJacobian J;
	bool isNew;
	V2f projectedTo[MAX_RES_PER_POINT];
	V3f centerProjectedTo;
	double stats0;
	bool toRemove;

	// PointHessian values
	float color[MAX_RES_PER_POINT];			// colors in host frame
	float weights[MAX_RES_PER_POINT];		// host-weights for respective residuals.
	float u, v;
	float idepth_scaled;
	float idepth_zero_scaled;
	float maxRelBaseline;
	bool isGoodResidual;

	// precalc values
	M33f PRE_KRKiTll;
	M33f  PRE_RTll_0;
	V2f PRE_aff_mode;
	float PRE_b0_mode;
	V3f PRE_KtTll;
	V3f PRE_tTll_0;

	//int host_idx;
	float host_frameEnergyTH;	// set dynamically depending on tracking residual

	//int target_idx;
	float target_frameEnergyTH;	// set dynamically depending on tracking residual
};
}

