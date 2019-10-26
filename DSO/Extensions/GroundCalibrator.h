/*
* This file is part of DSO
*
* PointCloudCalibrator.h
*	Created on: Aug 30, 2017
*	by: Mumbear
*
* dso point cloud calibrator: 
*	calibrate point cloud rotation and scale
*/

#pragma once

#include "MathLib.h"
#include "util/FrameShell.h"
#include "util/IndexThreadReduce.h"
#include "FullSystem/HessianBlocks.h"

#include <map>
#include <vector>
#include <time.h>
#include <thread>
#include <set>

//#include <opencv2/aruco.hpp>
#include <opencv2/aruco/charuco.hpp>

#define BOARD_CORNER_NUM	4
#define BOARD_SIDE_NUM		4
#define ONLINE_POINT_NUM	222/*13*/

typedef std::pair<dso::Vec3, dso::Vec3> lineVertexPair;


namespace dso
{ 
	//----------------------------------------------------
	//					Smart Board in 2D
	//----------------------------------------------------
	//----------------------------------------------------
	//					     Marker
	/*
				y
				^
				|	[C_0]-----S_0----->[C_1]
				|	  ^	  __	_____	 |
				|	  |	 |__|__|__   |	 |
				|	  |		|__|__|  |	 |
				|	 S_3	   |   __|	S_1
				|	  |		   |  |		 |
				|	  |		   |__|		 |
				|	  |					 v
				|	[C_3]<----S_2------[C_2]
				|
			  --+---------------------------> x
				|
	*/
	//----------------------------------------------------
	struct SmartBoard2D
	{
		SmartBoard2D(float _boardWidth, float _boardHeight);
		void getInterpolatedSidePt(std::vector<cv::Point2f> imageCorner, unsigned int sidePtNum, 
			std::vector<std::vector<cv::Point2f>>& sidePoints);
	private:
		std::vector<cv::Point2f> corner;
		std::vector<cv::Point2f> sideNormalized;

		cv::Point2f dirtVecH;
		cv::Point2f dirtVecV;

		float boardWidth;
		float boardHeight;
	};



	//----------------------------------------------------
	//						Plane3D
	// [a 3D plane is represented as ax + by + cz + d = 0]
	//----------------------------------------------------
	struct Plane3D
	{
		double d;
		Vec3 pt;	// a point in the plane
		Vec3 normal;

		Plane3D(Vec3 p0, Vec3 p1, Vec3 p2);
		Plane3D(Vec3 _pt, Vec3 _normal);
		Plane3D(Vec4 pVec);
		Vec3 pointToPlane(Vec3 pt);
		double distToPlane(Vec3 pt);
	};



	//----------------------------------------------------
	//					  Side in 3D
	//----------------------------------------------------
	enum SideType {WITDH = 0, HEIGHT};
	struct Side3D
	{
		Vec3 sideVec;
		double length;
		double ratio;

		Side3D(Vec3 _vertex1, Vec3 _vertex2, SideType _type);
		void updateTo(Vec3 _vertex1, Vec3 _vertex2, SideType _type);
	};



	//----------------------------------------------------
	//					 Corner in 3D
	//----------------------------------------------------
	struct Corner3D
	{
		Vec3 position;

		Corner3D* neighbor1;
		Corner3D* neighbor2;

		Side3D* side1;
		Side3D* side2;

		double angle;	// always non-negative range: [0, PI)
		double privateWeight;
		double publicWeight;
	
		Corner3D(Vec3 _position, Side3D* _side1, Side3D* _side2);
		void setNeighbor(Corner3D* _neighbor1, Corner3D* _neighbor2);
		void updateTo(Vec3 _position, Side3D* _side1, Side3D* _side2);
	};



	//----------------------------------------------------
	//					Board in 3D
	//----------------------------------------------------
	
	// two corners with the highest weights will be used to compute result
	// corner weight w = w3 * (w1 + w2) / 2
	// a: angle of two sides belonging to a corner
	// l1 = side 1 length; l2 = side 2 length; l' = (l1 + l2) / 2
	// p1: neighbor corner 1; p2: neighbor corner 2
	// w1 = (90 - (a)) / 90
	// w2 = (l' - (l1 - l2)) / l'
	// w3 = (p1's (w1 + w2) + p2's (w1 + w2)) / 2
	
	struct Board3D
	{
		Side3D*	  side[BOARD_CORNER_NUM];	// S_0, S_1, S_2, S_3
		Corner3D* corner[BOARD_SIDE_NUM];	// C_0, C_1, C_2, C_3
		
		Board3D(Vec3 p0, Vec3 p1, Vec3 p2, Vec3 p3, double eps = 0.00001);
		~Board3D();

		void optimizePoints();
		void update(TMat4 tMat);
		TMat4 getRotateDiffToGround();

		int bestCornerIndex() { return weight_vec[0].first; };
		double getSideError() { return sideError; };
		double getAngleError() { return angleError; };
		double getMediumHeight() { return medianHeight; };

		bool isValid;
		bool isGood;
		
		std::vector<std::pair<int, double>> weight_vec; // sort corners from highest weight to lowest

	private:
		void evaluate();
		void updateWight();
		void updateTo(Vec3 p0, Vec3 p1, Vec3 p2, Vec3 p3);

		double sideError;	// the variance of length of all sides
		double angleError;	// the variance of all corner angles to 90 degree
		double medianHeight;

		Vec3 optimizedCorner[BOARD_CORNER_NUM];
	};



	//----------------------------------------------------
	//				    GroundCalibrator
	//----------------------------------------------------
	// 1. good resulr process:							(WAIT ->) COLLECT_CORNERS -> VERIFY -> COMPUTE -> FINISHED
	// 2. reach max frame but fail to calibrate:		(WAIT ->) COLLECT_CORNERS -> (WAIT)
	// 3. get error result then retry until success:	(WAIT ->) COLLECT_CORNERS -> COMPUTE -> (WAIT)
	enum GroundCalibState
	{
		WAIT = 0,
		FINISHED,
		COLLECT_CORNERS,
		COMPUTE
	};

	enum GroundCalibBoardType
	{
		SingleMarker = 0,
		ChArUcoBoard_3X3
	};

	class GroundCalibrator
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
		GroundCalibrator(int channel);
		~GroundCalibrator();

		// general func.
		void setCamMatrix(Eigen::Matrix3f mat);
		void startCalibration();
		void stopCalibration();
		bool pushNewFrame(FrameHessian* fh);
		bool pushNewFrame(cv::Mat image, TMat4 transform);

		// get func.
		float getScale() { return (float)groundScale; };
		Sophus::Matrix4f getRotateMatrix() { return groundRotateMatrix.cast<float>(); };
		Sophus::Matrix4f getOffsetMatrix() { return groundOffsetMatrix.cast<float>(); };



	public:
		GroundCalibState state;



	private:
		void resetCalibration();
		RMat3 transRotateVec2Mat(cv::Vec3d rotateVector);
		TMat4 getTMatFromRVec(cv::Vec3d rotateVector);
		lineVertexPair createCorner3DLine(cv::Point2f pt, TMat4 transform);
		double dist3DFromPtToLine(Vec3 pt, lineVertexPair vertexPair);

		bool intersectLines(Eigen::MatrixXd &PA, Eigen::MatrixXd &PB, Vec3& corner);
		bool optimizePoint(std::map<int, lineVertexPair>& lineVertex, Vec3& cornerPt, double prevDist = DBL_MAX);
		void optimizePlane(std::map<int, Vec3>& planePts, Vec4& planeVec, double prevDist = DBL_MAX);
		bool detectCorners(cv::Mat image, std::vector<cv::Point2f>& markerCorners);
		void recordData(TMat4 transform, std::vector<cv::Point2f>& boardCorners);
		bool verifyResult();
		void retry();

		// state func.
		bool collectCorners(cv::Mat image, TMat4 transform);
		bool waitToStart(cv::Mat image, TMat4 transform);
		void computeCalibResult();



	private:
		int channel;
		int frameCount_WAIT;		// for state WAIT
		int frameCount_COLLECT;		// for state COLLECT_CORNERS
		int frameCount_COMPUTE;
		int retryCount;

		clock_t opTime;
		RMat3 cameraMatrix;
		cv::Ptr<cv::aruco::Dictionary> markerDictionary;
		cv::Ptr<cv::aruco::DetectorParameters> detectorParams;

		// calibration results
		double distError;
		double groundScale;
		TMat4 groundRotateMatrix;
		TMat4 groundOffsetMatrix;

		std::vector<std::vector<std::map<int, lineVertexPair>>> planePtLineVertex;
		std::unique_ptr<SmartBoard2D> board2D;
		std::thread threadComputeResult;

		IndexThreadReduce<Vec10> ptOptimizeReduce;
		void ptOptimize_Reductor(int index, std::map<int, Vec3>* map_buf, int min, int max, Vec10* stats, int tid);

		GroundCalibBoardType boardType;
		cv::Ptr<cv::aruco::CharucoBoard> charucoBoard;
		int target_marker_id;
	};
}