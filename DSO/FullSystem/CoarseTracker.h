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

 
#include "util/NumType.h"
#include "vector"
#include <math.h>
#include "util/settings.h"
#include "util/IndexThreadReduce.h"
#include "OptimizationBackend/MatrixAccumulators.h"
#include "IOWrapper/Output3DWrapper.h"

//windows mod:Yang
#include <Eigen/Cholesky>
/////////////////
//#include "cuda/dso.h"


namespace dso
{
struct CalibHessian;
struct FrameHessian;
struct PointFrameResidual;

class CoarseTracker {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

	CoarseTracker(int channel, int w, int h);
	~CoarseTracker();

	bool trackNewestCoarse(
			const FrameHessian* const newFrameHessian,
			SE3 &lastToNew_out, AffLight &aff_g2l_out,
			int coarsestLvl, Vec6 minResForAbort,	//mod by Arthur: 5-->6
			IOWrap::Output3DWrapper* wrap=0);

	void setCoarseTrackingRef(
			std::vector<FrameHessian*> frameHessians);

	void makeK(
			CalibHessian* HCalib);

	bool debugPrint, debugPlot;

	Mat33f K[PYR_LEVELS];
	Mat33f Ki[PYR_LEVELS];
	float fx[PYR_LEVELS];
	float fy[PYR_LEVELS];
	float fxi[PYR_LEVELS];
	float fyi[PYR_LEVELS];
	float cx[PYR_LEVELS];
	float cy[PYR_LEVELS];
	float cxi[PYR_LEVELS];
	float cyi[PYR_LEVELS];
	int w[PYR_LEVELS];
	int h[PYR_LEVELS];

    void debugPlotIDepthMap(float* minID, float* maxID, std::vector<IOWrap::Output3DWrapper*> &wraps);
    void debugPlotIDepthMapFloat(std::vector<IOWrap::Output3DWrapper*> &wraps);

	void clone(CoarseTracker* ct);

	FrameHessian* lastRef;
	AffLight lastRef_aff_g2l;
	FrameHessian* newFrame;
	int refFrameID;

	// act as pure ouptut
	Vec6 lastResiduals;
	Vec3 lastFlowIndicators;
	double firstCoarseRMSE;
private:
	int channel;

	void makeCoarseDepthL0(std::vector<FrameHessian*> frameHessians);
	float* idepth[PYR_LEVELS];
	float* weightSums[PYR_LEVELS];
	float* weightSums_bak[PYR_LEVELS];


	Vec6 calcResAndGS(int lvl, Mat88 &H_out, Vec8 &b_out, const SE3 &refToNew, AffLight aff_g2l, float cutoffTH);
	Vec6 calcRes(int lvl, const SE3 &refToNew, AffLight aff_g2l, float cutoffTH);
	void calcGSSSE(int lvl, Mat88 &H_out, Vec8 &b_out, const SE3 &refToNew, AffLight aff_g2l);
	void calcGS(int lvl, Mat88 &H_out, Vec8 &b_out, const SE3 &refToNew, AffLight aff_g2l);

	// pc buffers
	float* pc_u[PYR_LEVELS];
	float* pc_v[PYR_LEVELS];
	float* pc_idepth[PYR_LEVELS];
	float* pc_color[PYR_LEVELS];
	int pc_n[PYR_LEVELS];

	// warped buffers
	float* buf_warped_idepth;
	float* buf_warped_u;
	float* buf_warped_v;
	float* buf_warped_dx;
	float* buf_warped_dy;
	float* buf_warped_residual;
	float* buf_warped_weight;
	float* buf_warped_refColor;
	int buf_warped_n;


    std::vector<float*> ptrToDelete;


	Accumulator9 acc;

	IndexThreadReduce<Vec10> depthReduce;
	void depth_Reductor(int lvl, MinimalImageB3 *mf, float minID, float maxID, int min, int max, Vec10* stats, int tid);

	IndexThreadReduce<Vec10> makeCoarseDepthL0Reduce;
	void makeCoarseDepthL0_Reductor0(FrameHessian* fh, int min, int max, Vec10* stats, int tid);
	void makeCoarseDepthL0_Reductor1(int lvl, int min, int max, Vec10* stats, int tid);
	void makeCoarseDepthL0_Reductor2(int lvl, float* weightSumsl_bak, int min, int max, Vec10* stats, int tid);
	void makeCoarseDepthL0_Reductor3(int lvl, float* weightSumsl_bak, int min, int max, Vec10* stats, int tid);
	struct makeCoarseDepthL0Result
	{
		std::vector<float> lpc_u[NUM_THREADS];
		std::vector<float> lpc_v[NUM_THREADS];
		std::vector<float> lpc_idepth[NUM_THREADS];
		std::vector<float> lpc_color[NUM_THREADS];
		int lpc_n[NUM_THREADS];
	};
	makeCoarseDepthL0Result result;
	void makeCoarseDepthL0_Reductor4(int lvl, makeCoarseDepthL0Result *result, int min, int max, Vec10* stats, int tid);

};


class CoarseDistanceMap {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

	CoarseDistanceMap(int channel, int w, int h);
	~CoarseDistanceMap();

	void makeDistanceMap(
			std::vector<FrameHessian*> frameHessians,
			FrameHessian* frame);

	void makeInlierVotes(
			std::vector<FrameHessian*> frameHessians);

	void makeK( CalibHessian* HCalib);


	float* fwdWarpedIDDistFinal;

	Mat33f K[PYR_LEVELS];
	Mat33f Ki[PYR_LEVELS];
	float fx[PYR_LEVELS];
	float fy[PYR_LEVELS];
	float fxi[PYR_LEVELS];
	float fyi[PYR_LEVELS];
	float cx[PYR_LEVELS];
	float cy[PYR_LEVELS];
	float cxi[PYR_LEVELS];
	float cyi[PYR_LEVELS];
	int w[PYR_LEVELS];
	int h[PYR_LEVELS];

	void addIntoDistFinal(int u, int v);


private:
	int channel;

	PointFrameResidual** coarseProjectionGrid;
	int* coarseProjectionGridNum;
	Eigen::Vector2i* bfsList1;
	Eigen::Vector2i* bfsList2;

	void growDistBFS(int bfsNum);
};

}

