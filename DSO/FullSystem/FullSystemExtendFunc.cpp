/*
* This file is part of DSO
*
* FullSystemExtendFunc.cpp
*	Created on: Nov 7, 2017
*	by: Mumbear
*
* extended functions of dso: 
*	point cloud calibrator & dso lost tracker
*/


#include "FullSystem.h"
#include "ImmaturePoint.h"
#include "CoarseTracker.h"
#include "OptimizationBackend/EnergyFunctionalStructs.h"

#include <set>

#include "Extensions/GroundCalibrator.h"
#include "Extensions/LostTracker.h"

namespace dso
{
	void FullSystem::createGroundCalibrator()
	{
		if (!gCalibrator && setting_GC_ON)
		{
			gCalibrator = new GroundCalibrator(channel);

			Mat33f K = Mat33f::Zero();
			K(0, 0) = Hcalib->fxl();
			K(1, 1) = Hcalib->fyl();
			K(0, 2) = Hcalib->cxl();
			K(1, 2) = Hcalib->cyl();
			K(2, 2) = 1;

			gCalibrator->setCamMatrix(K);
		}
	}

	void FullSystem::updateGroundCalibrator(bool needToMakeKF, FrameHessian* fh)
	{
		if (gCalibrator->state != GroundCalibState::FINISHED && GroundInitialized)
			GroundInitialized = false;

		switch (gCalibrator->state)
		{
		case GroundCalibState::COLLECT_CORNERS:
		{
			if (needToMakeKF) gCalibrator->pushNewFrame(fh);
			break;
		}
		case GroundCalibState::WAIT:
		{
			if (needToMakeKF) gCalibrator->pushNewFrame(fh);
			break;
		}
		case GroundCalibState::FINISHED:
		{
			if (!GroundInitialized)
			{
				for (IOWrap::Output3DWrapper* ow : outputWrapper)
					ow->lockModel3DMutex();

				setting_groundRotate[channel] = gCalibrator->getRotateMatrix();
				setting_groundOffset[channel] = gCalibrator->getOffsetMatrix();

				float scale = gCalibrator->getScale();
				setting_groundScale[channel](0, 0) = scale;
				setting_groundScale[channel](1, 1) = scale;
				setting_groundScale[channel](2, 2) = scale;

				for (IOWrap::Output3DWrapper* ow : outputWrapper)
					ow->publishGroundCalibParams();

				GroundInitialized = true;

				gCalibrator->stopCalibration();
				delete gCalibrator;
				gCalibrator = NULL;
			}
			break;
		}
		default:
			break;
		}
	}

	void FullSystem::updateLostTracker(bool needToMakeKF, FrameHessian* fh)
	{
		if (lostTracker->status == LostTrackerStatus::Matched)
		{
			lostTracker->reset();
			lostTracker->startCollecting();
		}

		if (lostTracker->status == LostTrackerStatus::Collecting && needToMakeKF)
		{
			lostTracker->deliverSystemData(channel, allFrameHistory[allFrameHistory.size() - 2], 
				pixelSelector->currentPotential, frameHessians, ef->HM, ef->bM);
		}
	}

	bool FullSystem::handleLostTrack()
	{
		assert(lostTracker);

		printf("\nRecover from Lost Backup ...\n");

		recoverFromLostBackup();
		if (frameHessians.size() < 2)
		{
			lostTracker->status = LostTrackerStatus::Failed;
			printf("No enough frame backups\n");
			isLost = true;
			return false;
		}

		isLost = false;
		printf("Done.\n\n");

		return true;
	}

	void FullSystem::recoverFromLostBackup()
	{
		assert(lostTracker);

		//--------------------------------------------------------------
		//					   Clear FrameHessians
		//--------------------------------------------------------------
		{
			for (int i = 0; i < (int)frameHessians.size(); i++)
				frameHessians[i]->flaggedForMarginalization = true;

			flagPointsForRemoval();
			ef->dropPointsF();
			ef->marginalizePointsF();

			for (int i = 0; i < (int)frameHessians.size(); i++)
				if (frameHessians[i]->flaggedForMarginalization)
				{
					bool deletable = !(lostTracker->setFrameMarginalized(frameHessians[i]));
					marginalizeFrame(frameHessians[i], deletable);
					i--;
				}
		}
		// -------------------------------------------------------------


		int matched_camFrameID = lostTracker->matched_camFrameID;
		FullSystemData* fsData = lostTracker->getMatchedSystem();
		FrameHessianDataVec prevFHData = fsData->prevFrameDataVec;

		//--------------------------------------------------------------
		//              Add Needed Previous FrameHessians 
		//--------------------------------------------------------------
		for (auto it = prevFHData.begin(); it != prevFHData.end(); it++)
		{
			FrameHessian* fh;
			if (lostTracker->fullSystemByCamID.find(it->first) != lostTracker->fullSystemByCamID.end())
			{
				// push_back to frameHessians
				fh = lostTracker->fullSystemByCamID[it->first]->frame;
				lostTracker->fullSystemByCamID[it->first]->marginalized = false;
			}
			else if (!lostTracker->initFrameByCamID.empty() &&
				lostTracker->initFrameByCamID.find(it->first) != lostTracker->initFrameByCamID.end())
			{
				fh = lostTracker->initFrameByCamID[it->first]->frame;
				lostTracker->initFrameByCamID[it->first]->marginalized = false;
			}
			else
				continue;

			fh->flaggedForMarginalization = false;
			fh->idx = frameHessians.size();
			frameHessians.push_back(fh);
			ef->insertFrame(fh, Hcalib);
		}



		// ----------------------------------------------------------------
		//						 Add Mathced Frame
		// ----------------------------------------------------------------
		FrameHessian* lastFH = fsData->frame;
		fsData->marginalized = false;
		lastFH->flaggedForMarginalization = false;

		lastFH->idx = frameHessians.size();
		frameHessians.push_back(lastFH);
		ef->insertFrame(lastFH, Hcalib);

		fsData->setupLastFrame(lastFH);

		// recover all ImmaturePoint from immaturePoints
		for (int i = 0; i < lastFH->immaturePoints.size(); i++)
		{
			ImmaturePoint* pt = lastFH->immaturePoints[i];
			ImmaturePoint* impt = new ImmaturePoint(channel, pt->u, pt->v, lastFH, pt->my_type, Hcalib);
			delete pt;

			if (!std::isfinite(impt->energyTH))
			{
				delete impt;
				lastFH->immaturePoints[i] = lastFH->immaturePoints.back();
				lastFH->immaturePoints.pop_back();
				i--;
			}
			else
				lastFH->immaturePoints[i] = impt;
		}

		// recover all ImmaturePoint from pointHessians
		for (PointHessian* ph : lastFH->pointHessians)
		{
			ImmaturePoint* pt = new ImmaturePoint(channel, ph->u, ph->v, lastFH, ph->my_type, Hcalib);
			if (!std::isfinite(pt->energyTH)) delete pt;
			else lastFH->immaturePoints.push_back(pt);
			delete ph;
		}
		lastFH->pointHessians.clear();
		
		// recover all ImmaturePoint from pointHessiansMarginalized
		for (PointHessian* ph : lastFH->pointHessiansMarginalized)
		{
			ImmaturePoint* pt = new ImmaturePoint(channel, ph->u, ph->v, lastFH, ph->my_type, Hcalib);
			if (!std::isfinite(pt->energyTH)) delete pt;
			else lastFH->immaturePoints.push_back(pt);
			delete ph;
		}
		lastFH->pointHessiansMarginalized.clear();
		
		// recover all ImmaturePoint from pointHessiansOut
		for (PointHessian* ph : lastFH->pointHessiansOut)
		{
			ImmaturePoint* pt = new ImmaturePoint(channel, ph->u, ph->v, lastFH, ph->my_type, Hcalib);
			if (!std::isfinite(pt->energyTH)) delete pt;
			else lastFH->immaturePoints.push_back(pt);
			delete ph;
		}
		lastFH->pointHessiansOut.clear();
		



		//----------------------------------------------------------------------------------
		// recover points & residuals (insertResidual has to be called after insertFrame) 
		//----------------------------------------------------------------------------------
		int loop = (int)frameHessians.size() - 1;
		for (int i = 0; i < loop; i++)
		{
			FrameHessian* fh = frameHessians[i];
			FrameHessianData* fhFData;
			for (int j = i; j < prevFHData.size(); j++)
				if (fh->camFrameID == prevFHData[j].first)
					fhFData = prevFHData[j].second;

			fhFData->setupFrame(fh);

			
			std::vector<PointHessian*> addToPointHessiansOut;

			// gather all frames pointer for fast searching
			std::set<FrameHessian*> onlineFrames;
			for (FrameHessian* onlineFrame : frameHessians)
				onlineFrames.emplace(onlineFrame);

			std::unordered_map<int, ImmaturePointData*> immaturePointMap;
			for (auto ipData : fhFData->immaturePointVec)
				immaturePointMap.emplace(ipData);
			std::unordered_map<int, ImmaturePointData*>::iterator itPt;

			std::unordered_map<int, PointHessianData*> pointHessianMap;
			for (auto phData : fhFData->pointHessianVec)
				pointHessianMap.emplace(phData);
			std::unordered_map<int, PointHessianData*>::iterator itPh;

			// recover PointHessians
			for (int i = 0; i < fh->pointHessians.size(); i++)
			{
				PointHessian* ph = fh->pointHessians[i];
				itPh = pointHessianMap.find(PointPoseID::encodeID(ph->u, ph->v));

				if (itPh != pointHessianMap.end())
				{
					itPh->second->setupPointHessian(ph);
					ef->insertPoint(ph);

					// recover point residuals
					ph->release();
					
					for (int i = 0; i < itPh->second->residualData.size(); i++)
					{
						PointFrameResidualData* rData = itPh->second->residualData[i];
						if (onlineFrames.find(rData->target) == onlineFrames.end())
							continue;

						PointFrameResidual* r = new PointFrameResidual(channel, ph, fh, rData->target);
						rData->setupPointFrameResidual(r);
						ph->residuals.push_back(r);
						ef->insertResidual(r);

						if (r->state_NewState == ResState::ResState_IN)
						{
							r->efResidual->isActiveAndIsGoodNEW = true;
							PointFrameResidualData::cloneJacobian(r->J, r->efResidual->J);
							r->efResidual->takeDataF();
						}

						if (itPh->second->lastResidualData[0].first == i)
							ph->lastResiduals[0].first = r;
						else if (itPh->second->lastResidualData[1].first == i)
							ph->lastResiduals[1].first = r;
					}
				}
				else
				{
					addToPointHessiansOut.push_back(ph);
					fh->pointHessians[i] = fh->pointHessians.back();
					fh->pointHessians.pop_back();
					i--;
				}
			}



			// recover pointHessiansMarginalized
			for (int i = 0; i < fh->pointHessiansMarginalized.size(); i++)
			{
				PointHessian* ph = fh->pointHessiansMarginalized[i];
				itPh = pointHessianMap.find(PointPoseID::encodeID(ph->u, ph->v));

				if (itPh != pointHessianMap.end())
				{
					itPh->second->setupPointHessian(ph);
					fh->pointHessians.push_back(ph);
					ef->insertPoint(ph);

					// recover point residuals
					ph->release();

					for (int i = 0; i < itPh->second->residualData.size(); i++)
					{
						PointFrameResidualData* rData = itPh->second->residualData[i];
						if (onlineFrames.find(rData->target) == onlineFrames.end())
							continue;

						PointFrameResidual* r = new PointFrameResidual(channel, ph, fh, rData->target);
						rData->setupPointFrameResidual(r);
						ph->residuals.push_back(r);
						ef->insertResidual(r);

						if (r->state_NewState == ResState::ResState_IN)
						{
							r->efResidual->isActiveAndIsGoodNEW = true;
							PointFrameResidualData::cloneJacobian(r->J, r->efResidual->J);
							r->efResidual->takeDataF();
						}

						if (itPh->second->lastResidualData[0].first == i)
							ph->lastResiduals[0].first = r;
						else if (itPh->second->lastResidualData[1].first == i)
							ph->lastResiduals[1].first = r;
					}
				
					fh->pointHessiansMarginalized[i] = fh->pointHessiansMarginalized.back();
					fh->pointHessiansMarginalized.pop_back();
					i--;
				}
			}



			// recover pointHessiansOut
			for (int i = 0; i < fh->pointHessiansOut.size(); i++)
			{
				PointHessian* ph = fh->pointHessiansOut[i];
				itPh = pointHessianMap.find(PointPoseID::encodeID(ph->u, ph->v));

				if (itPh != pointHessianMap.end())
				{
					itPh->second->setupPointHessian(ph);
					fh->pointHessians.push_back(ph);
					ef->insertPoint(ph);

					// recover point residuals
					ph->release();

					for (int i = 0; i < itPh->second->residualData.size(); i++)
					{
						PointFrameResidualData* rData = itPh->second->residualData[i];
						if (onlineFrames.find(rData->target) == onlineFrames.end())
							continue;

						PointFrameResidual* r = new PointFrameResidual(channel, ph, fh, rData->target);
						rData->setupPointFrameResidual(r);
						ph->residuals.push_back(r);
						ef->insertResidual(r);

						if (r->state_NewState == ResState::ResState_IN)
						{
							r->efResidual->isActiveAndIsGoodNEW = true;
							PointFrameResidualData::cloneJacobian(r->J, r->efResidual->J);
							r->efResidual->takeDataF();
						}

						if (itPh->second->lastResidualData[0].first == i)
							ph->lastResiduals[0].first = r;
						else if (itPh->second->lastResidualData[1].first == i)
							ph->lastResiduals[1].first = r;
					}

					fh->pointHessiansOut[i] = fh->pointHessiansOut.back();
					fh->pointHessiansOut.pop_back();
					i--;
				}
			}


			// recover ImmaturePoints
			for (ImmaturePoint* pt : fh->immaturePoints)
			{
				itPt = immaturePointMap.find(PointPoseID::encodeID(pt->u, pt->v));

				if (itPt != immaturePointMap.end())
					itPt->second->setupImmaturePoint(pt);
			}

			for (PointHessian* ph : addToPointHessiansOut)
				fh->pointHessiansOut.push_back(ph);

			printf("id = %d	", fh->camFrameID);
			printf("pointHessians: %zd\n", fh->pointHessians.size());
		}

		printf("id = %d	", lastFH->camFrameID);
		printf("ImmaturePoints: %zd\n", lastFH->immaturePoints.size());

		printf("Set frame size = %zd\n", frameHessians.size());

		// ----------------------------------------------------------------



		//-----------------------------------------------------------------
		//					  setup energy from backend  
		//-----------------------------------------------------------------
		ef->HM = fsData->HM;
		ef->bM = fsData->bM;
		ef->setAdjointsF(Hcalib);
		setPrecalcValues();
		ef->makeIDX();
		optimize(setting_maxOptIterations);
		// ----------------------------------------------------------------



		pixelSelector->currentPotential = fsData->psPotential;

		// setup coarseTracker's
		boost::unique_lock<boost::mutex> crlock(coarseTrackerSwapMutex);
		coarseTracker->setCoarseTrackingRef(frameHessians);
		coarseTracker_forNewKF->setCoarseTrackingRef(frameHessians);
		for (int i = 0; i < NUM_THREADS; i++)
			result.coarseTrackers[i]->clone(coarseTracker_forNewKF);
	}
}