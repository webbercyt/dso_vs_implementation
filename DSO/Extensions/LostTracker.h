/*
* This file is part of DSO
*
* LostTracker.h
*	Created on: Nov 7, 2017
*	by: Mumbear
*
* dso lost tracker: 
*	recover lost dso from backup
*/

#pragma once

#include <vector>
#include <set>
#include <unordered_map>

#include "FullSystem/ImmaturePoint.h"
#include "FullSystem/Residuals.h"
#include "util/IndexThreadReduce.h"

#include "opencv2/core/mat.hpp"
#include "opencv2/core/utility.hpp"

using namespace std;
using namespace cv;

namespace dso
{	
	struct PointHessian;
	struct FrameHessian;
	class CoarseTracker;
	class FrameShell;

	namespace PointPoseID
	{
		// e.g. [u, v] = [125, 544], id = 125 * 10000 + 544 = 125 0544
		static int inline encodeID(float u, float v)
		{
			return (int)u * 10000 + (int)v;
		}

		static void inline decodeID(int id, float& u, float& v)
		{
			u = (float)floor((float)id / 10000.0);
			v = (float)(id - u * 10000);
		}
	}
	

	//--------------------------------------------------
	//		      Lightweight ImmaturePoint
	//--------------------------------------------------
	struct ImmaturePointData
	{
		float quality;
		float idepth_min;
		float idepth_max;
		ImmaturePointStatus lastTraceStatus;

		inline ImmaturePointData(ImmaturePoint* pt)
		{
			quality = pt->quality;
			idepth_min = pt->idepth_min;
			idepth_max = pt->idepth_max;
			lastTraceStatus = pt->lastTraceStatus;
		}

		void setupImmaturePoint(ImmaturePoint* pt);
	};


	//--------------------------------------------------
	//		    Lightweight PointFrameResidual
	//--------------------------------------------------
	struct PointFrameResidualData
	{
		ResState state_state;
		ResState state_NewState;
		double state_energy;
		double state_NewEnergy;
		double state_NewEnergyWithOutlier;

		FrameHessian* target;
		RawResidualJacobian* J;

		Vec3f centerProjectedTo;

		inline PointFrameResidualData(PointFrameResidual* residual)
		{
			state_state = residual->state_state;
			state_energy = residual->state_energy;
			state_NewState = residual->state_NewState;
			state_NewEnergy = residual->state_NewEnergy;
			state_NewEnergyWithOutlier = residual->state_NewEnergyWithOutlier;

			target = residual->target;

			J = new RawResidualJacobian();
			cloneJacobian(residual->J, J);
			centerProjectedTo = residual->centerProjectedTo;
		}

		inline ~PointFrameResidualData(){ delete J; }

		static inline void cloneJacobian(RawResidualJacobian* src, RawResidualJacobian* dst)
		{
			src->resF = dst->resF;
			src->Jpdd = dst->Jpdd;
			src->JIdx2 = dst->JIdx2;
			src->JabJIdx = dst->JabJIdx;
			src->Jab2 = dst->Jab2;

			for (int i = 0; i < 2; i++)
			{
				src->Jpdxi[i] = dst->Jpdxi[i];
				src->Jpdc[i] = dst->Jpdc[i];
				src->JIdx[i] = dst->JIdx[i];
				src->JabF[i] = dst->JabF[i];
			}
		}

		void setupPointFrameResidual(PointFrameResidual* residual);
	};


	//--------------------------------------------------
	//			   Lightweight PointHesssian
	//--------------------------------------------------
	struct PointHessianData
	{
		float idepth_zero;
		float idepth;
		float step;
		float step_backup;
		float idepth_backup;

		float idepth_hessian;
		float maxRelBaseline;
		int numGoodResiduals;

		vector<PointFrameResidualData*> residualData;
		pair<int, ResState> lastResidualData[2];	// int: index in vector residualData 

		PointHessianData(PointHessian* ph);
		~PointHessianData();

		void setupPointHessian(PointHessian* ph);
	};


	//--------------------------------------------------
	//					  FrameMatcher
	//--------------------------------------------------
	struct FrameMatcher
	{
		int w, h;
		bool marginalized;
		FrameHessian* frame;

		// params for matching, check empty() before use since no data in initialization
		Mat histMat;
		Mat descriptors;
		vector<KeyPoint> keyPoints;

		set<int> refFrameIds;

		FrameMatcher(int channel, FrameHessian* fh);
		~FrameMatcher();
	};


	//--------------------------------------------------
	//			  Lightweight  FrameHessian
	//  [contains all immuaturePoints & pointHessians]
	//--------------------------------------------------
	typedef vector<pair<int, ImmaturePointData*>> ImmaturePointDataVec;
	typedef vector<pair<int, PointHessianData*>>  PointHessianDataVec;
	struct FrameHessianData
	{
		Vec10 state;
		Vec10 state_backup;
		Vec10 step;
		Vec10 step_backup;

		ImmaturePointDataVec immaturePointVec;	// int : id = encode(u, v)
		PointHessianDataVec  pointHessianVec;	// int : id = encode(u, v)	

		FrameHessianData(FrameHessian* fh);
		~FrameHessianData();
		void setupFrame(FrameHessian* fh);
		void deletePointHessianData();
	};


	//------------------------------------------------------
	//			    Lightweight FullSystem
	// [ the last frame in frameHessian is also kept here] 
	//------------------------------------------------------
	typedef vector<pair<int, FrameHessianData*>> FrameHessianDataVec;
	struct FullSystemData : FrameMatcher
	{
		// shell[0]: the shel of this key frame
		// shell[1]: the shell before current frame used in "trackNewCoarseFromBackup(...)" of FullSystem
		pair<int, FrameShell*> sprelast;	// int: frameHistoryID

		// PixelSelector param.
		int psPotential;

		// Energy param.
		MatXX HM;
		VecX bM;

		// prevFrameValid = true: all prev. frames are kept in LostTracker::fullSystemByCamID  
		bool prevFrameValid;
		FrameHessianDataVec prevFrameDataVec;

		FullSystemData(int channel, FrameShell* _preShell, int _currentPotential,
			FrameHessian* _frame, MatXX _HM, VecX _bM);
		~FullSystemData();
		void setupLastFrame(FrameHessian* fh);

	private:
		// the state of current frame when being saved to LostTracker 
		Vec10 state;
		Vec10 state_backup;
		Vec10 step;
		Vec10 step_backup;
	};


	//--------------------------------------------------------------------------
	//						   LostTrackerStatus 
	//	Sleeping	|	wait for change to Collecting		
	//	Collecting	|	collect keyframe and states related to keyframe
	//	Matching	|	track is lost and searching matched keyframe to recover DSO
	//  Matched		|	found matched keyframe and try to recover DSO
	//  Failed		|	fail to recover DSO, DSO will be reset
	//--------------------------------------------------------------------------
	enum LostTrackerStatus { Sleeping = 0, Collecting, Matching, Matched, Failed };

	class LostTracker {
	public:
		LostTracker(int _maxHistoryframe);
		~LostTracker();

		void reset();
		void startCollecting();
		void startCollecting(int channel, vector<FrameHessian*> frameHessians);
		void startMatching();
		void deliverSystemData(int channel, FrameShell* _preShell, int _currentPotential,
			vector<FrameHessian*> frameHessians, MatXX _HM, VecX _bM);

		bool setFrameMarginalized(FrameHessian* frame);
		bool matchImage(ImageAndExposure* fh);
		FullSystemData* getMatchedSystem();

		unordered_map<int, FullSystemData*> fullSystemByCamID;
		unordered_map<int, FrameMatcher*> initFrameByCamID;	// handle the frames used to initialize DSO
		
		LostTrackerStatus status;
		int matched_camFrameID;
		int lastID;
		int lastFrameHistoryID;

	private:
		
		// flags to check FullSystemData valid
		int maxHistoryframe;
		bool allShellValid;

		Mat prevHist;

		bool findHomographyFromFeature(Mat next, FrameMatcher* fMatcher);
		bool deleteInvalidShellData();
		void marginalizeFrame();
		void insertFrameRef(int insertID, FrameHessianDataVec prevFHData);
		void dropFrameRef(int dropID, FrameHessianDataVec prevFHData);
		void createPointHessianData(vector<PointHessian*> pointHessians, 
			PointHessianDataVec *pointHessianData);
	};

}