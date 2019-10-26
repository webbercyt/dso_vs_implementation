/*
* This file is part of DSO
*
* LostTracker.cpp
*	Created on: Nov 7, 2017
*	by: Mumbear
*
* dso lost tracker: 
*	recover lost dso from backup
*/

#include "LostTracker.h"
#include "FullSystem/CoarseTracker.h"
#include "FullSystem/HessianBlocks.h"
#include "util/FrameShell.h"
#include "OptimizationBackend/EnergyFunctionalStructs.h"

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

#define HIST_MATCH_THRESHOLD		0.20
#define PREV_HIST_MATCH_THRESHOLD	0.05

#define NN_MATCH_RATIO				0.8			// Nearest neighbor matching ratio
#define HOMOGRAPHY_ROTATE_TH		0.35
#define HOMOGRAPHY_TRANS_TH			50

namespace dso
{
	Mat getImage(int w, int h, float* image)
	{
		Mat buf = Mat(h, w, CV_32FC1, image);
		Mat ret;
		buf.convertTo(ret, CV_8UC1);

		return ret;
	}

	Mat getHist(Mat image)
	{
		// Set histogram bins count
		int bins = 256;
		int histSize[] = { bins };
		// Set ranges for histogram bins
		float lranges[] = { 0, 256 };
		const float* ranges[] = { lranges };
		// create matrix for histogram
		Mat hist;
		int channels[] = { 0 };

		calcHist(&image, 1, channels, Mat(), hist, 1, histSize, ranges, true, false);

		return hist;
	}

	void ImmaturePointData::setupImmaturePoint(ImmaturePoint* pt)
	{
		pt->quality = quality;
		pt->idepth_min = idepth_min;
		pt->idepth_max = idepth_max;
		pt->lastTraceStatus = lastTraceStatus;
	}

	void PointFrameResidualData::setupPointFrameResidual(PointFrameResidual* residual)
	{
		residual->state_state = state_state;
		residual->state_energy = state_energy;
		residual->state_NewState = state_NewState;
		residual->state_NewEnergy = state_NewEnergy;
		residual->state_NewEnergyWithOutlier = state_NewEnergyWithOutlier;

		residual->target = residual->target;

		cloneJacobian(J, residual->J);
		residual->centerProjectedTo = centerProjectedTo;
	}

	PointHessianData::PointHessianData(PointHessian* ph)
	{
		idepth_zero = ph->idepth_zero;
		idepth = ph->idepth;
		step = ph->step;
		step_backup = ph->step_backup;
		idepth_backup = ph->idepth_backup;

		idepth_hessian = ph->idepth_hessian;
		maxRelBaseline = ph->maxRelBaseline;
		numGoodResiduals = ph->numGoodResiduals;

		lastResidualData[0].first = -1;
		lastResidualData[0].second = ph->lastResiduals[0].second;
		lastResidualData[1].first = -1;
		lastResidualData[1].second = ph->lastResiduals[1].second;

		for (PointFrameResidual* r : ph->residuals)
		{
			residualData.push_back(new PointFrameResidualData(r));
			if (ph->lastResiduals[0].first == r)
				lastResidualData[0].first = (int)(residualData.size() - 1);
			else if (ph->lastResiduals[1].first == r)
				lastResidualData[1].first = (int)(residualData.size() - 1);
		}
	}

	PointHessianData::~PointHessianData()
	{
		for (PointFrameResidualData* rData : residualData)
			delete rData;
		residualData.swap(vector<PointFrameResidualData*>());
	}

	void PointHessianData::setupPointHessian(PointHessian* ph)
	{
		ph->setIdepthZero(idepth_zero);
		ph->setIdepth(idepth);
		ph->step = step;
		ph->step_backup = step_backup;
		ph->idepth_backup = idepth_backup;

		ph->idepth_hessian = idepth_hessian;
		ph->maxRelBaseline = maxRelBaseline;
		ph->numGoodResiduals = numGoodResiduals;

		ph->setPointStatus(PointHessian::ACTIVE);

		ph->lastResiduals[0].first = 0;
		ph->lastResiduals[0].second = lastResidualData[0].second;
		ph->lastResiduals[1].first = 0;
		ph->lastResiduals[1].second = lastResidualData[1].second;
	}

	FrameHessianData::FrameHessianData(FrameHessian* fh)
	{
		state = fh->state;
		state_backup = fh->state_backup;
		step = fh->step;
		step_backup = fh->step_backup;
	}

	FrameHessianData::~FrameHessianData()
	{
		for (auto it = immaturePointVec.begin(); it != immaturePointVec.end(); it++)
			delete it->second;
		immaturePointVec.clear();

		if (multiThreading)
			deletePointHessianData();
		else
		{
			for (auto it = pointHessianVec.begin(); it != pointHessianVec.end(); it++)
				delete it->second;
		}
		pointHessianVec.clear();
	}

	void FrameHessianData::setupFrame(FrameHessian* fh)
	{
		fh->setState(state);
		fh->state_backup = state_backup;
		fh->step = step;
		fh->step_backup = step_backup;
	}

	void FrameHessianData::deletePointHessianData()
	{
		parallel_for_(Range(0, pointHessianVec.size()), [&](const Range& range) {
			for (int r = range.start; r < range.end; r++)
				delete pointHessianVec[r].second;
		});
	}

	FullSystemData::FullSystemData(int channel, FrameShell* _preShell, int _currentPotential,
		FrameHessian* _frame, MatXX _HM, VecX _bM) : FrameMatcher(channel, _frame)
	{
		psPotential = _currentPotential;

		state = frame->state;
		state_backup = frame->state_backup;
		step = frame->step;
		step_backup = frame->step_backup;

		sprelast = make_pair(_preShell->id, _preShell);

		HM = _HM;
		bM = _bM;

		prevFrameValid = true;
	}

	FullSystemData::~FullSystemData()
	{
		for (auto it = prevFrameDataVec.begin(); it != prevFrameDataVec.end(); it++)
			delete it->second;
		prevFrameDataVec.clear();
	}

	void FullSystemData::setupLastFrame(FrameHessian* fh)
	{
		fh->setState(state);
		fh->state_backup = state_backup;
		fh->step = step;
		fh->step_backup = step_backup;
	}

	FrameMatcher::FrameMatcher(int channel, FrameHessian* fh)
	{
		w = globalCalibs[channel].wG[0];
		h = globalCalibs[channel].hG[0];

		frame = fh;
		marginalized = fh->flaggedForMarginalization;

		histMat.data = NULL;
		descriptors.data = NULL;
	}

	FrameMatcher::~FrameMatcher()
	{
		if(marginalized)
			delete frame;

		keyPoints.clear();
		keyPoints.shrink_to_fit();
	
		refFrameIds.clear();
	}

	LostTracker::LostTracker(int _maxHistoryframe) : maxHistoryframe(_maxHistoryframe)
	{
		status = Sleeping;

		allShellValid = false;
		prevHist.data = NULL;

		matched_camFrameID = -1;
		lastID = -1;
		lastFrameHistoryID = -1;

		setNumThreads(NUM_THREADS);
	}

	LostTracker::~LostTracker()
	{
		for (auto it = fullSystemByCamID.begin(); it != fullSystemByCamID.end(); it++)
			delete it->second;
		fullSystemByCamID.clear();

		for (auto it = initFrameByCamID.begin(); it != initFrameByCamID.end(); it++)
			delete it->second;
		initFrameByCamID.clear();
	}

	void LostTracker::reset()
	{
		for (auto it = fullSystemByCamID.begin(); it != fullSystemByCamID.end();)
		{
			if (it->first > matched_camFrameID)
			{
				dropFrameRef(it->first, it->second->prevFrameDataVec);
				delete it->second;
				it = fullSystemByCamID.erase(it);
			}
			else
				it++;
		}

		allShellValid = false;
		prevHist.data = NULL;

		matched_camFrameID = -1;
		lastID = -1;
		lastFrameHistoryID = -1;
	}

	void LostTracker::startCollecting()
	{
		status = LostTrackerStatus::Collecting;
	}

	void LostTracker::startCollecting(int channel, vector<FrameHessian*> frameHessians)
	{
		status = LostTrackerStatus::Collecting;

		for (FrameHessian* fh : frameHessians)
			initFrameByCamID.emplace(fh->camFrameID, new FrameMatcher(channel, fh));
	}

	void LostTracker::startMatching()
	{
		status = LostTrackerStatus::Matching;

		// check there are needed frames in fullSystemByCamID for each element
		for (auto itData = fullSystemByCamID.begin(); itData != fullSystemByCamID.end(); itData++)
		{
			for (auto itPrevData = itData->second->prevFrameDataVec.begin(); 
				itPrevData != itData->second->prevFrameDataVec.end(); itPrevData++)
			{
				if (!initFrameByCamID.empty() && (initFrameByCamID.find(itPrevData->first) != initFrameByCamID.end()))
					continue;

				if (fullSystemByCamID.find(itPrevData->first) == fullSystemByCamID.end())
				{
					itData->second->prevFrameValid = false;
					break;
				}
			}
		}
	}

	void LostTracker::deliverSystemData(int channel, FrameShell* _preShell, int _currentPotential,
		vector<FrameHessian*> frameHessians, MatXX _HM, VecX _bM)
	{
		FullSystemData* fsData =
			new FullSystemData(channel, _preShell, _currentPotential, frameHessians.back(), _HM, _bM);

		for (int i = 0; i < (int)frameHessians.size() - 1 /* '-1' means ignoring the last frame*/; i++)
		{
			FrameHessianData* fhData = new FrameHessianData(frameHessians[i]);

			fhData->immaturePointVec.reserve(frameHessians[i]->immaturePoints.size());
			for (ImmaturePoint* pt : frameHessians[i]->immaturePoints)
				fhData->immaturePointVec.push_back(make_pair(PointPoseID::encodeID(pt->u, pt->v), new ImmaturePointData(pt)));

			fhData->pointHessianVec.reserve(frameHessians[i]->pointHessians.size());
			if (multiThreading)
			{
				PointHessianDataVec phData[NUM_THREADS];
				size_t vecSize = frameHessians[i]->pointHessians.size() / NUM_THREADS;
				for (int j = 0; j < NUM_THREADS; j++)
					phData[j].reserve(vecSize);

				createPointHessianData(frameHessians[i]->pointHessians, phData);

				for (int j = 0; j < NUM_THREADS; j++)
					fhData->pointHessianVec.insert(fhData->pointHessianVec.end(), phData[j].begin(), phData[j].end());
			}
			else
			{
				for (PointHessian* ph : frameHessians[i]->pointHessians)
					fhData->pointHessianVec.push_back(make_pair(PointPoseID::encodeID(ph->u, ph->v), new PointHessianData(ph)));
			}

			fsData->prevFrameDataVec.push_back(make_pair(frameHessians[i]->camFrameID, fhData));
		}

		fullSystemByCamID.emplace(frameHessians.back()->camFrameID, fsData);
		insertFrameRef(frameHessians.back()->camFrameID, fsData->prevFrameDataVec);
		marginalizeFrame();

		/*printf("fullSystemByCamID.size = %zd\n", fullSystemByCamID.size());*/
	}

	bool LostTracker::setFrameMarginalized(FrameHessian* _frame)
	{
		if (!initFrameByCamID.empty())
		{
			auto itF = initFrameByCamID.find(_frame->camFrameID);
			if (itF != initFrameByCamID.end())
			{
				itF->second->marginalized = true;
				return true;
			}
		}
		
		auto itKF = fullSystemByCamID.find(_frame->camFrameID);
		if (itKF != fullSystemByCamID.end())
		{
			itKF->second->marginalized = true;
			return true;
		}
		else
			return false;
	}

	bool LostTracker::matchImage(ImageAndExposure* image)
	{
		if (image->camFrameID < lastID) return false;
		if ((fullSystemByCamID.empty() && initFrameByCamID.empty())
			|| lastID < fullSystemByCamID.begin()->first)
		{
			status = LostTrackerStatus::Failed;
			return false;
		}

		// ignore the image who is similar to previous one, and its prev. one is unable to be matched
		Mat origImage = getImage(image->w, image->h, image->image);
		Mat hist = getHist(origImage);

		if (!prevHist.empty() &&
			compareHist(hist, prevHist, CV_COMP_BHATTACHARYYA) <= PREV_HIST_MATCH_THRESHOLD)
			return false;

		prevHist = hist;

		//	// create matrix for histogram visualization
		//	//int bins = 256;
		//	//int const hist_height = 256;
		//	//Mat3b hist_image = Mat3b::zeros(hist_height, bins);

		//	//double max_val = 0;
		//	//minMaxLoc(hist, 0, &max_val);

		//	//// visualize each bin
		//	//for (int b = 0; b < bins; b++) {
		//	//	float const binVal = hist.at<float>(b);
		//	//	int   const height = cvRound(binVal*hist_height / max_val);
		//	//	line
		//	//	(hist_image
		//	//		, Point(b, hist_height - height), Point(b, hist_height)
		//	//		, Scalar::all(255)
		//	//	);
		//	//}
		//	//imshow("Hist", hist_image);

		// we want to use valid key frame data
		if (!allShellValid)
			deleteInvalidShellData();

		bool hasGood = false;
		bool hasMatched = false;
		for (auto it = fullSystemByCamID.begin(); it != fullSystemByCamID.end(); it++)
		{
			FullSystemData* tKFData = it->second;
			if (it->first > lastID) break;

			if (tKFData->histMat.empty())
				tKFData->histMat = getHist(tKFData->frame->getImage());

			if (compareHist(hist, tKFData->histMat, CV_COMP_BHATTACHARYYA) <= HIST_MATCH_THRESHOLD)
			{
				if (findHomographyFromFeature(origImage, tKFData))
				{
					hasMatched = true;
					if (!it->second->prevFrameValid) continue;

					hasGood = true;
					matched_camFrameID = it->first;
					break;
				}
			}
		}

		if (!initFrameByCamID.empty())
		{
			for (auto it = initFrameByCamID.begin(); it != initFrameByCamID.end(); it++)
			{
				if (it->second->histMat.empty())
					it->second->histMat = getHist(it->second->frame->getImage());
				if (compareHist(hist, it->second->histMat, CV_COMP_BHATTACHARYYA) <= HIST_MATCH_THRESHOLD &&
					findHomographyFromFeature(origImage, it->second))
				{
					hasMatched = true;
					break;
				}
			}
		}

		if (hasMatched && !hasGood)
		{
			printf("has Matched but has no Good\n");
			status = LostTrackerStatus::Failed;
			return false;
		}

		if (!hasGood) return false;

		// Setup tracker status
		status = LostTrackerStatus::Matched;
		printf("Lost Tracker: Matching Succeseed\n");
		return true;
	}

	FullSystemData* LostTracker::getMatchedSystem()
	{
		if (matched_camFrameID != -1)
			return fullSystemByCamID[matched_camFrameID];
		else
		{
			printf("Invalid index\n");
			return 0;
		}
		
	}

	bool LostTracker::findHomographyFromFeature(Mat next, FrameMatcher* fMatcher)
	{
		// Initiate ORB detector
		Ptr<FeatureDetector> detector = ORB::create();
		Ptr<DescriptorExtractor> extractor = ORB::create();

		if (fMatcher->descriptors.empty())
		{
			cv::Mat prev = fMatcher->frame->getImage();
			detector->detect(prev, fMatcher->keyPoints);
			extractor->compute(prev, fMatcher->keyPoints, fMatcher->descriptors);
		}

		vector<KeyPoint> keypoints_next;
		Mat descriptors_next;

		// find the keypoints and descriptors with ORB
		detector->detect(next, keypoints_next);

		if (fMatcher->keyPoints.size() < 1 || keypoints_next.size() < 1)
			return false;

		extractor->compute(next, keypoints_next, descriptors_next);

		BFMatcher matcher(NORM_HAMMING);
		vector< vector<DMatch> > nn_matches;
		matcher.knnMatch(fMatcher->descriptors, descriptors_next, nn_matches, 2);

		vector<Point2f> prev_matched_kp, next_matched_kp;

		// Find the Homography Matrix
		if (nn_matches.size() < 1)
			return false;

		vector<DMatch> good_matches;
		for (int i = 0; i < nn_matches.size(); i++)
		{
			DMatch first = nn_matches[i][0];
			float dist1 = nn_matches[i][0].distance;
			float dist2 = nn_matches[i][1].distance;

			if (dist1 < NN_MATCH_RATIO * dist2) {
				int new_i = static_cast<int>(prev_matched_kp.size());

				prev_matched_kp.push_back(fMatcher->keyPoints[first.queryIdx].pt);
				next_matched_kp.push_back(keypoints_next[first.trainIdx].pt);

				good_matches.push_back(DMatch(new_i, new_i, 0));
			}
		}

		if (prev_matched_kp.size() < 1 || next_matched_kp.size() < 1)
			return false;
		Mat H = findHomography(next_matched_kp, prev_matched_kp, CV_RANSAC/*, 2.5f*/);
		if (H.empty())
			return false;

		Mat diff = Mat::zeros(3, 3, CV_64FC1);
		diff.ptr<double>(0)[0] = 1;
		diff.ptr<double>(1)[1] = 1;
		diff.ptr<double>(2)[2] = 1;

		diff = abs(H - diff);
		if (diff.ptr<double>(0)[0] < HOMOGRAPHY_ROTATE_TH && diff.ptr<double>(0)[1] < HOMOGRAPHY_ROTATE_TH &&
			diff.ptr<double>(1)[0] < HOMOGRAPHY_ROTATE_TH && diff.ptr<double>(1)[1] < HOMOGRAPHY_ROTATE_TH &&
			diff.ptr<double>(0)[2] < HOMOGRAPHY_TRANS_TH && diff.ptr<double>(1)[2] < HOMOGRAPHY_TRANS_TH)
			return true;
		else
			return false;
	}

	bool LostTracker::deleteInvalidShellData()
	{
		if (lastFrameHistoryID < 1)
			return false;

		if (lastFrameHistoryID > maxHistoryframe)
		{
			for (auto it = fullSystemByCamID.begin(); it != fullSystemByCamID.end(); it++)
			{
				if (lastFrameHistoryID - it->second->sprelast.first >= maxHistoryframe)
					fullSystemByCamID.erase(it->first);
			}
		}

		allShellValid = true;
		return true;
	}

	void LostTracker::marginalizeFrame()
	{
		// keep the saved frame amount = MAX_TRACK_KEYFRAME
		// delete the frame: last refered frame is earlist; fewest frames refer to this

		if (fullSystemByCamID.size() > setting_lostTrackerBackupFrames && !fullSystemByCamID.empty())
		{
			auto it = fullSystemByCamID.begin();
			while (!it->second->marginalized)
			{
				if (it == fullSystemByCamID.end())
				{
					printf("There may be an error: no frame can be marginalized !\n");
					return;
				}
				it++;
			}
			int dropID = it->first;

			if (fullSystemByCamID.size() > 1 && !it->second->refFrameIds.empty())
			{
				// prior - first: id, second: size
				pair<int, size_t> prior = make_pair(*it->second->refFrameIds.rbegin(), it->second->refFrameIds.size());
				it++;

				for (; it != fullSystemByCamID.end(); it++)
				{
					if (!it->second->marginalized) continue;
					if (it->second->refFrameIds.empty())
					{
						dropID = it->first;
						break;
					}

					int newLastID = *it->second->refFrameIds.rbegin();
					size_t newSize = it->second->refFrameIds.size();

					if (newLastID < prior.first)
					{
						dropID = it->first;
						prior = make_pair(newLastID, newSize);
					}
					else if (newLastID == prior.first)
					{
						if (newSize < prior.second)
						{
							dropID = it->first;
							prior.second = newSize;
						}
					}
				}

				/*printf("after id = %d\n", prior.first);*/
			}

			auto itFS = fullSystemByCamID[dropID];
			dropFrameRef(dropID, itFS->prevFrameDataVec);
			delete itFS;
			fullSystemByCamID.erase(dropID);

			marginalizeFrame();
		}
	}

	void LostTracker::insertFrameRef(int insertID, FrameHessianDataVec prevFHData)
	{
		// add frame to refFrame
		for (auto it = prevFHData.begin(); it != prevFHData.end(); it++)
		{
			if (!initFrameByCamID.empty())
			{
				auto itInitF = initFrameByCamID.find(it->first);
				if (itInitF != initFrameByCamID.end())
					itInitF->second->refFrameIds.emplace(insertID);
			}

			auto itFS = fullSystemByCamID.find(it->first);
			if (itFS != fullSystemByCamID.end())
				itFS->second->refFrameIds.emplace(insertID);
		}
	}

	void LostTracker::dropFrameRef(int dropID, FrameHessianDataVec prevFHData)
	{
		for (auto it = prevFHData.begin(); it != prevFHData.end(); it++)
		{
			if (!initFrameByCamID.empty())
			{
				auto itInitF = initFrameByCamID.find(it->first);
				if (itInitF != initFrameByCamID.end())
				{
					itInitF->second->refFrameIds.erase(dropID);
					if (itInitF->second->refFrameIds.empty())
					{
						delete itInitF->second;
						itInitF = initFrameByCamID.erase(itInitF);
					}
				}
			}

			auto itFS = fullSystemByCamID.find(it->first);
			if (itFS != fullSystemByCamID.end())
				itFS->second->refFrameIds.erase(dropID);
		}
	}

	void LostTracker::createPointHessianData(vector<PointHessian*> pointHessians, 
		PointHessianDataVec *pointHessianData)
	{
		assert(multiThreading);
		parallel_for_(Range(0, pointHessians.size()), [&](const Range& range) {
			for (int r = range.start; r < range.end; r++)
				pointHessianData[getThreadNum()].push_back(make_pair(
					PointPoseID::encodeID(pointHessians[r]->u, pointHessians[r]->v), new PointHessianData(pointHessians[r])));
		});
	}
}