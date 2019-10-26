/*
* This file is part of DSO
*
* PointCloudCalibrator.cpp
*	Created on: Aug 30, 2017
*	by: Mumbear
*
* dso point cloud calibrator: 
*	calibrate point cloud rotation and scale
*/

#include "GroundCalibrator.h"
#include <math.h>
#include <stdio.h>
#include "util/settings.h"

#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#define BOARD_CORNER_ID_LT				0	// corner id: left top
#define BOARD_CORNER_ID_RT				1	// corner id: right top
#define BOARD_CORNER_ID_RB				2	// corner id: right bottom
#define BOARD_CORNER_ID_LB				3	// corner id: left bottom 

#define WAIT_TO_START_FRAME				10
#define MAX_RETRY_COUNT					0/*3*/
#define MIN_RESULT_VERIFY_FRAMES		7
#define MAX_RESULT_VERIFY_FRAMES		10
#define VERIFY_GOOD_FRAME_RATE			0.8f

#define PI_05							CV_PI / 2.0f
#define RAD_2_DEG						180.0f / CV_PI
#define DEG_2_RAD						CV_PI / 180.0f
#define VERIFY_BOARD_ANGLE_DIFF			1.0f * DEG_2_RAD/*2.0f / 180.0f * CV_PI*/
#define VERIFY_BOARD_SIDE_PERCENTAGE	0.01f
#define GOOD_RESULT_PERCENTAGE			0.95f

namespace dso
{
	const double dbl_max = std::numeric_limits<double>::max();
	const double int_max = std::numeric_limits<int>::max();

	double board_w_by_h_ratio = 1.0;
	double board_w = 1.0;
	double board_h = 1.0;

	//----------------------------------------------------
	//					 SmartBoard2D
	//----------------------------------------------------
	SmartBoard2D::SmartBoard2D(float _boardWidth, float _boardHeight) : boardWidth(_boardWidth), boardHeight(_boardHeight)
	{
		corner.reserve(BOARD_CORNER_NUM);
		corner.push_back(cv::Point2f(0, boardHeight));
		corner.push_back(cv::Point2f(boardWidth, boardHeight));
		corner.push_back(cv::Point2f(boardWidth, 0));
		corner.push_back(cv::Point2f(0, 0));

		dirtVecH = (corner[1] - corner[0]) / cv::norm(corner[1] - corner[0]);
		dirtVecV = (corner[2] - corner[1]) / cv::norm(corner[2] - corner[1]);
	}

	void SmartBoard2D::getInterpolatedSidePt(std::vector<cv::Point2f> imageCorner, unsigned int sidePtNum,
		std::vector<std::vector<cv::Point2f>>& sidePoints)
	{
		if (imageCorner.size() != BOARD_CORNER_NUM)
			return;

		if (sidePtNum < 2)
			return;
		
		cv::Mat m = cv::findHomography(corner, imageCorner, CV_RANSAC);
		//cv::Mat m = cv::getPerspectiveTransform(corner, imageCorner);

		float unit_v = boardHeight / (float)(sidePtNum - 1);
		float unit_h = boardWidth / (float)(sidePtNum - 1);

		std::vector<cv::Point2f> src_points;

		for (int i = 0; i < sidePtNum; i++)
		{
			cv::Point2f v_step = unit_v * dirtVecV * i;
			for (int j = 0; j < sidePtNum; j++)
				src_points.push_back(corner[0] + v_step + unit_h * dirtVecH * j);
		}

		std::vector<cv::Point2f> dst_points;
		cv::perspectiveTransform(src_points, dst_points, m);
		
		sidePoints.clear();
		for (int i = 0; i < sidePtNum; i++)
		{
			sidePoints.push_back(std::vector<cv::Point2f>());
			for (int j = 0; j < sidePtNum; j++)
				sidePoints[i].push_back(dst_points[i * sidePtNum + j]);
		}
	}

	//----------------------------------------------------
	//						Plane3D
	//----------------------------------------------------
	Plane3D::Plane3D(Vec3 p0, Vec3 p1, Vec3 p2)
	{
		// calculate the ground plane from 3 board corners
		Vec3 v1 = p1 - p0;
		Vec3 v2 = p0 - p2;
		normal = v1.cross(v2);
		normal.normalize();

		d = (normal.dot(p0));
		pt = p0;
	}

	Plane3D::Plane3D(Vec3 _pt, Vec3 _normal) : pt(_pt), normal(_normal)
	{
		if (normal.norm() != 1)
			normal.normalize();

		d = (normal.x() * pt.x() + normal.y() * pt.y() + normal.z() * pt.z());
	}

	Plane3D::Plane3D(Vec4 pVec)
	{
		normal = Vec3(pVec[0], pVec[1], pVec[2]);
		
		if (normal.norm() != 1)
			normal.normalize();

		pt = normal * pVec[3];
		d = (normal.x() * pt.x() + normal.y() * pt.y() + normal.z() * pt.z());
	}

	Vec3 Plane3D::pointToPlane(Vec3 _pt)
	{
		return MathLib::pointToPlane(normal, pt, _pt);
	}

	double Plane3D::distToPlane(Vec3 _pt)
	{
		return MathLib::distToPlane(normal, d, _pt);
	}

	//----------------------------------------------------
	//					  Side in 3D
	//----------------------------------------------------
	Side3D::Side3D(Vec3 _vertex1, Vec3 _vertex2, SideType _type)
	{
		updateTo(_vertex1, _vertex2, _type);
	}

	void Side3D::updateTo(Vec3 _vertex1, Vec3 _vertex2, SideType _type)
	{
		sideVec = _vertex2 - _vertex1;
		length = sideVec.norm();

		ratio = _type == SideType::HEIGHT ? 1 : (double)(board_w_by_h_ratio);
	}

	//----------------------------------------------------
	//					 Corner in 3D
	//----------------------------------------------------
	Corner3D::Corner3D(Vec3 _position, Side3D* _side1, Side3D* _side2)
	{
		updateTo(_position, _side1, _side2);
	}

	void Corner3D::setNeighbor(Corner3D* _neighbor1, Corner3D* _neighbor2)
	{
		neighbor1 = _neighbor1;
		neighbor2 = _neighbor2;

		publicWeight = (neighbor1->privateWeight + neighbor2->privateWeight) / 2;
	}

	void Corner3D::updateTo(Vec3 _position, Side3D* _side1, Side3D* _side2)
	{
		position = _position;
		side1 = _side1;
		side2 = _side2;
		angle = abs(MathLib::getAngle(side1->sideVec, side2->sideVec, side1->sideVec.cross(side2->sideVec)));

		// l' = (side1 + side2) / 2
		float unit_length = (side1->length + side2->length) / (1.0 + (double)(board_w_by_h_ratio));

		// w1 = 1 - (90 - (a)) / 90
		// w2 = (l' - (l1 - l2)) / l'
		double w1 = 1 - abs(PI_05 - angle) / PI_05;
		double unit_diff = abs(side1->length / side1->ratio - side2->length / side2->ratio);
		double w2 = unit_length > unit_diff ? (unit_length - unit_diff) / unit_length : 0;

		privateWeight = (w1 + w2) / 2;
		publicWeight = 0;

		neighbor1 = NULL;
		neighbor2 = NULL;
	}

	//----------------------------------------------------
	//					  Board in 3D
	//----------------------------------------------------
	Board3D::Board3D(Vec3 p0, Vec3 p1, Vec3 p2, Vec3 p3, double eps)
	{
		isValid = true;

		// determine overlapping points
		if ((p0 - p1).norm() <= eps) isValid = false;
		if (isValid) if ((p0 - p2).norm() <= eps) isValid = false;
		if (isValid) if ((p0 - p3).norm() <= eps) isValid = false;
		if (isValid) if ((p1 - p2).norm() <= eps) isValid = false;
		if (isValid) if ((p1 - p3).norm() <= eps) isValid = false;
		if (isValid) if ((p2 - p3).norm() <= eps) isValid = false;

		side[0] = new Side3D(p0, p1, SideType::WITDH);
		side[1] = new Side3D(p1, p2, SideType::HEIGHT);
		side[2] = new Side3D(p2, p3, SideType::WITDH);
		side[3] = new Side3D(p3, p0, SideType::HEIGHT);

		corner[0] = new Corner3D(p0, side[3], side[0]);
		corner[1] = new Corner3D(p1, side[0], side[1]);
		corner[2] = new Corner3D(p2, side[1], side[2]);
		corner[3] = new Corner3D(p3, side[2], side[3]);

		corner[0]->setNeighbor(corner[3], corner[1]);
		corner[1]->setNeighbor(corner[0], corner[2]);
		corner[2]->setNeighbor(corner[1], corner[3]);
		corner[3]->setNeighbor(corner[2], corner[0]);

		evaluate();
	}

	Board3D::~Board3D()
	{
		for (int i = 0; i < BOARD_CORNER_NUM; i++)
			delete corner[i];

		for (int i = 0; i < BOARD_SIDE_NUM; i++)
			delete side[i];
	}


	void Board3D::optimizePoints()
	{
		if (!isGood)
			return;
		
		bool hasDiff = false;
		for (int i = 0; i < BOARD_CORNER_NUM; i++)
		{
			if (optimizedCorner[i] != corner[i]->position)
			{
				hasDiff = true;
				break;
			}
		}

		if (!hasDiff)
			return;

		updateTo(optimizedCorner[0], optimizedCorner[1], optimizedCorner[2], optimizedCorner[3]);
	}

	void Board3D::update(TMat4 tMat)
	{
		Vec4 p[BOARD_CORNER_NUM];
		
		for (int i = 0; i < BOARD_CORNER_NUM; i++)
		{
			p[i] = Vec4(corner[i]->position.x(), corner[i]->position.y(), corner[i]->position.z(), 1);
			p[i] = tMat * p[i];
		}

		updateTo(p[0].block<3, 1>(0, 0), p[1].block<3, 1>(0, 0), p[2].block<3, 1>(0, 0), p[3].block<3, 1>(0, 0));
		evaluate();
	}

	void Board3D::updateWight()
	{
		weight_vec.clear();

		for (int i = 0; i < BOARD_CORNER_NUM; i++)
			weight_vec.push_back(std::make_pair(i, corner[i]->publicWeight));

		std::sort(weight_vec.begin(), weight_vec.end(),
			[](const std::pair<int, double>& p1, const std::pair<int, double>& p2) {
			return p1.second > p2.second; });
	}

	void Board3D::updateTo(Vec3 p0, Vec3 p1, Vec3 p2, Vec3 p3)
	{
		side[0]->updateTo(p0, p1, SideType::WITDH);
		side[1]->updateTo(p1, p2, SideType::HEIGHT);
		side[2]->updateTo(p2, p3, SideType::WITDH);
		side[3]->updateTo(p3, p0, SideType::HEIGHT);

		corner[0]->updateTo(p0, side[3], side[0]);
		corner[1]->updateTo(p1, side[0], side[1]);
		corner[2]->updateTo(p2, side[1], side[2]);
		corner[3]->updateTo(p3, side[2], side[3]);

		corner[0]->setNeighbor(corner[3], corner[1]);
		corner[1]->setNeighbor(corner[0], corner[2]);
		corner[2]->setNeighbor(corner[1], corner[3]);
		corner[3]->setNeighbor(corner[2], corner[0]);

		evaluate();
	}

	TMat4 Board3D::getRotateDiffToGround()
	{
		std::unique_ptr<Plane3D> board_plane = 
			std::make_unique<Plane3D>(corner[0]->position, corner[1]->position, corner[2]->position);
		std::unique_ptr<Plane3D> ground_plane = std::make_unique<Plane3D>(Vec3(0, 0, 1), Vec3(1, 0, 1), Vec3(1, 0, 0));

		double angle_1 = MathLib::getAngle(board_plane->normal, ground_plane->normal, board_plane->normal.cross(ground_plane->normal));
		Vec3 axis_1 = board_plane->normal.cross(ground_plane->normal);
		TMat4 t_1 = MathLib::rotate(axis_1, angle_1);

		TMat4 t_2;
		Vec3 ground_s2_vec = Vec3(1, 0, 0) - Vec3(0, 0, 0);

		Vec4 p[2];
		p[0] = Vec4(corner[BOARD_CORNER_ID_RB]->position.x(), corner[BOARD_CORNER_ID_RB]->position.y(), corner[BOARD_CORNER_ID_RB]->position.z(), 1);
		p[0] = t_1 * p[0];
		p[1] = Vec4(corner[BOARD_CORNER_ID_LB]->position.x(), corner[BOARD_CORNER_ID_LB]->position.y(), corner[BOARD_CORNER_ID_LB]->position.z(), 1);
		p[1] = t_1 * p[1];
		Vec3 board_on_ground_s2_vec = p[0].block<3, 1>(0, 0) - p[1].block<3, 1>(0, 0);
		double angle_2 = MathLib::getAngle(board_on_ground_s2_vec, ground_s2_vec, board_on_ground_s2_vec.cross(ground_s2_vec));
		Vec3 axis_2 = board_on_ground_s2_vec.cross(ground_s2_vec);
		t_2 = MathLib::rotate(axis_2, angle_2);

		return t_2 * t_1;
	}

	void Board3D::evaluate()
	{
		isGood = false;

		updateWight();

		// we alse want the chosen corners are the vectex of the side at medium length  
		std::multimap<double, std::pair<int, int>> dist_map;
		dist_map.emplace(side[0]->length / side[0]->ratio, std::make_pair(0, 1));
		dist_map.emplace(side[1]->length / side[1]->ratio, std::make_pair(1, 2));
		dist_map.emplace(side[2]->length / side[2]->ratio, std::make_pair(2, 3));
		dist_map.emplace(side[3]->length / side[3]->ratio, std::make_pair(0, 3));
		double slope_by_height_ratio =  sqrt(1 + (double)(board_w_by_h_ratio) * (double)(board_w_by_h_ratio));
		dist_map.emplace((corner[0]->position - corner[2]->position).norm() / slope_by_height_ratio, std::make_pair(0, 2));
		dist_map.emplace((corner[1]->position - corner[3]->position).norm() / slope_by_height_ratio, std::make_pair(1, 3));
		

		auto it = dist_map.begin();
		std::advance(it, (int)(dist_map.size() / 2) - 1);
		double medium_height = it->first;


		// have two good points can be regarded as board corners ?
		if ((it->second.first == weight_vec[0].first || it->second.first == weight_vec[1].first) &&
			(it->second.second == weight_vec[0].first || it->second.first == weight_vec[1].first))
			isGood = true;

		std::advance(it, 1);
		if (2 * (medium_height - it->first) / (medium_height + it->first) > VERIFY_BOARD_SIDE_PERCENTAGE)
		{
			isGood = false;
			return;	// bad values: the distant between two medium values is too large
		}
			
		medium_height = (medium_height + it->first) / 2;

		// no good, so further check the next mediam value
		if (!isGood)
		{
			if ((it->second.first == weight_vec[0].first || it->second.first == weight_vec[1].first) &&
				(it->second.second == weight_vec[0].first || it->second.first == weight_vec[1].first))
				isGood = true;
		}

		double side_length;
		if (abs(weight_vec[0].first - weight_vec[1].first) == 2)
		{
			side_length = (corner[weight_vec[0].first]->position - corner[weight_vec[1].first]->position).norm() / slope_by_height_ratio;
		}
		else
		{
			side_length = (corner[weight_vec[0].first]->position - corner[weight_vec[1].first]->position).norm();

			if (weight_vec[0].first == 0 && weight_vec[1].first == 1 ||
				weight_vec[0].first == 1 && weight_vec[1].first == 0 ||
				weight_vec[0].first == 2 && weight_vec[1].first == 3 || 
				weight_vec[0].first == 3 && weight_vec[1].first == 2)
				side_length /= (double)(board_w_by_h_ratio);
		}

		// alse set to good as side length is close enough to the mediam
		if (!isGood)
			if (side_length >= medium_height * (1 - VERIFY_BOARD_SIDE_PERCENTAGE) &&
				side_length <= medium_height * (1 + VERIFY_BOARD_SIDE_PERCENTAGE))
				isGood = true;	

		// check whether a 3rd point can be regarded as the 3rd corner of the board
		if (isGood)
		{
			double side_length_0;
			if (abs(weight_vec[2].first - weight_vec[0].first) == 2)
			{
				side_length_0 = (corner[weight_vec[2].first]->position - corner[weight_vec[0].first]->position).norm() / slope_by_height_ratio;
			}
			else
			{
				side_length_0 = (corner[weight_vec[2].first]->position - corner[weight_vec[0].first]->position).norm();

				if (weight_vec[0].first == 0 && weight_vec[2].first == 1 ||
					weight_vec[0].first == 1 && weight_vec[2].first == 0 ||
					weight_vec[0].first == 2 && weight_vec[2].first == 3 ||
					weight_vec[0].first == 3 && weight_vec[2].first == 2)
					side_length_0 /= (double)(board_w_by_h_ratio);
			}

			double side_length_1;
			if (abs(weight_vec[2].first - weight_vec[0].first) == 2)
			{
				side_length_1 = (corner[weight_vec[2].first]->position - corner[weight_vec[0].first]->position).norm() / slope_by_height_ratio;
			}
			else
			{
				side_length_1 = (corner[weight_vec[2].first]->position - corner[weight_vec[0].first]->position).norm();

				if (weight_vec[0].first == 0 && weight_vec[2].first == 1 ||
					weight_vec[0].first == 1 && weight_vec[2].first == 0 ||
					weight_vec[0].first == 2 && weight_vec[2].first == 3 ||
					weight_vec[0].first == 3 && weight_vec[2].first == 2)
					side_length_1 /= (double)(board_w_by_h_ratio);
			}

			if (side_length_0 < medium_height * (1 - VERIFY_BOARD_SIDE_PERCENTAGE) ||
				side_length_0 > medium_height * (1 + VERIFY_BOARD_SIDE_PERCENTAGE) ||
				side_length_1 < medium_height * (1 - VERIFY_BOARD_SIDE_PERCENTAGE) ||
				side_length_1 > medium_height * (1 + VERIFY_BOARD_SIDE_PERCENTAGE))
				isGood = false;
		}

		if (isGood)
		{
			medianHeight = medium_height;

			double side_height_mean = 0;
			for (int i = 0; i < BOARD_SIDE_NUM; i++)
				side_height_mean += side[i]->length / side[i]->ratio;
			side_height_mean /= BOARD_SIDE_NUM;

			double side_height_variance = 0;
			for (int i = 0; i < BOARD_SIDE_NUM; i++)
				side_height_variance += pow((side[i]->length / side[i]->ratio - side_height_mean), 2);

			sideError = sqrt(side_height_variance / BOARD_SIDE_NUM);

			double corner_angle_mean = 0;
			for (int i = 0; i < BOARD_CORNER_NUM; i++)
				corner_angle_mean += corner[i]->angle;
			corner_angle_mean /= BOARD_CORNER_NUM;

			double corner_angle_variance = 0;
			for (int i = 0; i < BOARD_CORNER_NUM; i++)
				corner_angle_variance += pow((corner[i]->angle - PI_05), 2);

			angleError = sqrt(corner_angle_variance / BOARD_CORNER_NUM) * RAD_2_DEG;

			if (medianHeight == 0 ||
				sideError / medianHeight > VERIFY_BOARD_SIDE_PERCENTAGE ||
				angleError * DEG_2_RAD > VERIFY_BOARD_ANGLE_DIFF)
				isGood = false;
		}

		// compose corner points for optimization
		if (isGood)
		{
			int min_index = std::min(weight_vec[0].first, std::min(weight_vec[1].first, weight_vec[2].first));
			int max_index = std::max(weight_vec[0].first, std::max(weight_vec[1].first, weight_vec[2].first));
			int mid_index = (weight_vec[0].first + weight_vec[1].first + weight_vec[2].first) - min_index - max_index;


			std::unique_ptr<Plane3D> board_plane = std::make_unique<Plane3D>(
				corner[min_index]->position,
				corner[mid_index]->position,
				corner[max_index]->position);

			int index_diff = abs(weight_vec[0].first - weight_vec[1].first);

			if (index_diff == 2)
			{
				optimizedCorner[weight_vec[0].first] = corner[weight_vec[0].first]->position;
				Vec3 diff_vec = corner[weight_vec[1].first]->position - corner[weight_vec[0].first]->position;
				if (diff_vec.norm() != medianHeight * slope_by_height_ratio)
				{
					Vec3 s_normalized = diff_vec.normalized();
					/*s_normalized /= s_normalized.norm();*/
					optimizedCorner[weight_vec[1].first] = optimizedCorner[weight_vec[0].first] + s_normalized * medianHeight * slope_by_height_ratio;
				}
				else
					optimizedCorner[weight_vec[1].first] = corner[weight_vec[1].first]->position;

				Vec3 center_point = (optimizedCorner[weight_vec[1].first] + optimizedCorner[weight_vec[0].first]) / 2;
				double half_slope_length = (optimizedCorner[weight_vec[1].first] - optimizedCorner[weight_vec[0].first]).norm() / 2;

				int max_index = std::max(weight_vec[0].first, weight_vec[1].first);
				int min_index = std::min(weight_vec[0].first, weight_vec[1].first);

				double half_side_length = (max_index == 2 ? medianHeight : medianHeight * (double)(board_w_by_h_ratio)) / 2;
				double a =  2 * asin(half_side_length / half_slope_length);
				Vec3 v = corner[max_index]->position - center_point;

				Vec3 vec_normal = cos(a) * v + sin(a) * (board_plane->normal.cross(v));
				vec_normal.normalize();
				
				if (max_index == BOARD_CORNER_ID_RB)	// "\"
				{
					optimizedCorner[BOARD_CORNER_ID_RT] = center_point - vec_normal * half_slope_length;
					optimizedCorner[BOARD_CORNER_ID_LB] = center_point + vec_normal * half_slope_length;

					if ((optimizedCorner[BOARD_CORNER_ID_RT] - corner[BOARD_CORNER_ID_RT]->position).norm() > 
						(optimizedCorner[BOARD_CORNER_ID_RT] - corner[BOARD_CORNER_ID_LB]->position).norm() &&
						(optimizedCorner[BOARD_CORNER_ID_LB] - corner[BOARD_CORNER_ID_LB]->position).norm() >
						(optimizedCorner[BOARD_CORNER_ID_LB] - corner[BOARD_CORNER_ID_RT]->position).norm())
					{
						Vec3 temp = optimizedCorner[BOARD_CORNER_ID_RT];
						optimizedCorner[BOARD_CORNER_ID_RT] = optimizedCorner[BOARD_CORNER_ID_LB];
						optimizedCorner[BOARD_CORNER_ID_LB] = temp;
					}
				}
				else   // "/"
				{
					optimizedCorner[BOARD_CORNER_ID_LT] = center_point - vec_normal * half_slope_length;
					optimizedCorner[BOARD_CORNER_ID_RB] = center_point + vec_normal * half_slope_length;
					

					if ((optimizedCorner[BOARD_CORNER_ID_LT] - corner[BOARD_CORNER_ID_LT]->position).norm() > 
						(optimizedCorner[BOARD_CORNER_ID_LT] - corner[BOARD_CORNER_ID_RB]->position).norm() &&
						(optimizedCorner[BOARD_CORNER_ID_RB] - corner[BOARD_CORNER_ID_RB]->position).norm() >
						(optimizedCorner[BOARD_CORNER_ID_RB] - corner[BOARD_CORNER_ID_LT]->position).norm())
					{
						Vec3 temp = optimizedCorner[BOARD_CORNER_ID_LT];
						optimizedCorner[BOARD_CORNER_ID_LT] = optimizedCorner[BOARD_CORNER_ID_RB];
						optimizedCorner[BOARD_CORNER_ID_RB] = temp;
					}
				}

				/*for (int i = 0; i < BOARD_CORNER_NUM; i++)
				{
					printf("Old: [%f, %f, %f]\n", corner[i]->position.x(), corner[i]->position.y(), corner[i]->position.z());
					printf("New: [%f, %f, %f]\n", optimizedCorner[i].x(), optimizedCorner[i].y(), optimizedCorner[i].z());
				}*/
			}
			else if (index_diff != 2)
			{
				optimizedCorner[weight_vec[0].first] = corner[weight_vec[0].first]->position;
				/*printf("Old: [%f, %f, %f]\n", corner[weight_vec[0].first]->position.x(), corner[weight_vec[0].first]->position.y(), corner[weight_vec[0].first]->position.z());
				printf("New: [%f, %f, %f]\n", optimizedCorner[weight_vec[0].first].x(), optimizedCorner[weight_vec[0].first].y(), optimizedCorner[weight_vec[0].first].z());*/

				double ratio = 1;
				if (weight_vec[0].first == 0 && weight_vec[1].first == 1 ||
					weight_vec[0].first == 1 && weight_vec[1].first == 0 ||
					weight_vec[0].first == 2 && weight_vec[1].first == 3 ||
					weight_vec[0].first == 3 && weight_vec[1].first == 2)
					ratio = (double)(board_w_by_h_ratio);

				if ((corner[weight_vec[0].first]->position - corner[weight_vec[1].first]->position).norm() != medianHeight * ratio)
				{
					Vec3 s_normalized = (corner[weight_vec[1].first]->position - corner[weight_vec[0].first]->position).normalized();
					optimizedCorner[weight_vec[1].first] = corner[weight_vec[0].first]->position + s_normalized * medianHeight * ratio;
				}
				else
					optimizedCorner[weight_vec[1].first] = corner[weight_vec[1].first]->position;
				/*printf("Old: [%f, %f, %f]\n", corner[weight_vec[1].first]->position.x(), corner[weight_vec[1].first]->position.y(), corner[weight_vec[1].first]->position.z());
				printf("New: [%f, %f, %f]\n", optimizedCorner[weight_vec[1].first].x(), optimizedCorner[weight_vec[1].first].y(), optimizedCorner[weight_vec[1].first].z());*/


				int l_index = std::min(weight_vec[0].first, weight_vec[1].first);
				int h_index = std::max(weight_vec[0].first, weight_vec[1].first);

				if (h_index == BOARD_CORNER_NUM - 1)
				{
					h_index = l_index;
					l_index = BOARD_CORNER_NUM - 1;
				}

				Vec3 ref_vec = optimizedCorner[h_index] - optimizedCorner[l_index];
				Vec3 vec_normal = (board_plane->normal.cross(ref_vec)).normalized() * (double)(board_w_by_h_ratio) / ratio ;

				int index;
				index = h_index + 1;
				if (index > BOARD_CORNER_NUM - 1) index = 0;
				optimizedCorner[index] = optimizedCorner[h_index] - vec_normal * medianHeight/* * ratio*/;

				/*printf("Old: [%f, %f, %f]\n", corner[index]->position.x(), corner[index]->position.y(), corner[index]->position.z());
				printf("New: [%f, %f, %f]\n", optimizedCorner[index].x(), optimizedCorner[index].y(), optimizedCorner[index].z());*/

				index = l_index - 1;
				if (index < 0) index = BOARD_CORNER_NUM - 1;
				optimizedCorner[index] = optimizedCorner[l_index] - vec_normal * medianHeight/* * ratio*/;

				/*printf("Old: [%f, %f, %f]\n", corner[index]->position.x(), corner[index]->position.y(), corner[index]->position.z());
				printf("New: [%f, %f, %f]\n", optimizedCorner[index].x(), optimizedCorner[index].y(), optimizedCorner[index].z());*/
			}

			//std::cout << "Index Diff: " << index_diff << std::endl;
		}
	}

	//----------------------------------------------------
	//				    GroundCalibrator
	//----------------------------------------------------
	GroundCalibrator::GroundCalibrator(int channel) :
		channel(channel)
	{
		cameraMatrix = RMat3::Zero();

		boardType = static_cast<GroundCalibBoardType>(setting_GCBoardType);
		switch (boardType)
		{
		case SingleMarker:
		{
			target_marker_id = 0;

			markerDictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);

			detectorParams = cv::aruco::DetectorParameters::create();
			//detectorParams->adaptiveThreshWinSizeMin = 51;
			detectorParams->adaptiveThreshWinSizeMax = 131;
			detectorParams->adaptiveThreshWinSizeStep = 10;
			detectorParams->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
			detectorParams->cornerRefinementMaxIterations = 100;
			detectorParams->cornerRefinementMinAccuracy = 0.001;

			board_w = (double)(setting_GCMarkerSize[0]);
			board_h = (double)(setting_GCMarkerSize[1]);

			break;
		}

		case ChArUcoBoard_3X3:
		{
			markerDictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_50);
			float default_square_length = 5;
			charucoBoard = cv::aruco::CharucoBoard::create(3, 3, default_square_length, default_square_length - 1, markerDictionary);

			detectorParams = cv::aruco::DetectorParameters::create();
			detectorParams->adaptiveThreshWinSizeMax = 131;
			detectorParams->adaptiveThreshWinSizeStep = 10;

			board_w = (double)(setting_GCChArUco_3X3_SquareLength);
			board_h = (double)(setting_GCChArUco_3X3_SquareLength);

			break;
		}

		default:
			break;
		}

		board2D = std::make_unique<SmartBoard2D>((float)board_w, (float)board_h);
		board_w_by_h_ratio = board_w / board_h;

		groundRotateMatrix.setIdentity();
		groundOffsetMatrix.setZero();

		//resetCalibration();
		planePtLineVertex.reserve(ONLINE_POINT_NUM);
		for (int i = 0; i < ONLINE_POINT_NUM; i++)
		{
			planePtLineVertex.push_back(std::vector<std::map<int, lineVertexPair>>());
			planePtLineVertex[i].reserve(ONLINE_POINT_NUM);

			for (int j = 0; j < ONLINE_POINT_NUM; j++)
				planePtLineVertex[i].push_back(std::map<int, lineVertexPair>());
		}

		state = GroundCalibState::WAIT;
	}

	GroundCalibrator::~GroundCalibrator()
	{
		if (threadComputeResult.joinable()) threadComputeResult.join();
	}

	void GroundCalibrator::setCamMatrix(Eigen::Matrix3f mat)
	{
		cameraMatrix.setZero();
		cameraMatrix(0, 0) = (double)(mat(0, 0));
		cameraMatrix(1, 1) = (double)(mat(1, 1));
		cameraMatrix(0, 2) = (double)(mat(0, 2));
		cameraMatrix(1, 2) = (double)(mat(1, 2));
		cameraMatrix(2, 2) = 1;
	}

	void GroundCalibrator::startCalibration()
	{
		groundScale = 1;
		groundRotateMatrix.setIdentity();
		groundOffsetMatrix.setZero();
		opTime = clock();
		resetCalibration();
		state = GroundCalibState::COLLECT_CORNERS;
	}

	void GroundCalibrator::stopCalibration()
	{
		if (threadComputeResult.joinable())
			threadComputeResult.join();

		resetCalibration();
	}

	bool GroundCalibrator::pushNewFrame(FrameHessian* fh)
	{
		return(pushNewFrame(fh->getImage(), fh->shell->camToWorld.matrix()));
	}

	bool GroundCalibrator::pushNewFrame(cv::Mat image, TMat4 transform)
	{
		bool ret = false;

		switch (state)
		{
		case GroundCalibState::COLLECT_CORNERS:
		{
			if (collectCorners(image, transform))
				ret = true;
			break;
		}
		case GroundCalibState::WAIT:
		{
			if (waitToStart(image, transform))
				printf("\nCalibrating Point Cloud ...\n");
			break;
		}
		default:
			break;
		}

		return ret;
	}

	void GroundCalibrator::resetCalibration()
	{
		frameCount_WAIT = 0;
		frameCount_COLLECT = 0;
		frameCount_COMPUTE = setting_GCMinCornerCount;
		retryCount = 0;
		distError = 0;

		for (int i = 0; i < planePtLineVertex.size(); i++)
			for (int j = 0; j < planePtLineVertex[i].size(); j++)
				planePtLineVertex[i][j].clear();
	}

	RMat3 GroundCalibrator::transRotateVec2Mat(cv::Vec3d rotateVector)
	{
		cv::Mat rotate_matrix;
		cv::Rodrigues(rotateVector, rotate_matrix);

		// x-axis rotates at extra 90 degree to match true ground
		cv::Mat extra_rotate_matrix = cv::Mat::zeros(3, 3, CV_64F);
		/*cv::Rodrigues(cv::Vec3d(-CV_PI / 2, 0, 0), extra_rotate_matrix);*/

		cv::Rodrigues(cv::Vec3d(-PI_05, 0, 0), extra_rotate_matrix);

		rotate_matrix *= extra_rotate_matrix;

		RMat3 ret;
		ret.setIdentity();

		for (int i = 0; i < rotate_matrix.rows; i++)
			for (int j = 0; j < rotate_matrix.cols; j++)
				ret(i, j) = rotate_matrix.at<double>(i, j);

		return ret;
	}

	TMat4 GroundCalibrator::getTMatFromRVec(cv::Vec3d rotateVector)
	{
		TMat4 ret = TMat4::Identity();

		if (rotateVector[0] == 0 && rotateVector[1] == 0 && rotateVector[2] == 0)
			return ret;

		cv::Mat extra_rotate_matrix;;
		cv::Rodrigues(rotateVector, extra_rotate_matrix);
			
		for (int i = 0; i < extra_rotate_matrix.rows; i++)
			for (int j = 0; j < extra_rotate_matrix.cols; j++)
				ret(i, j) = extra_rotate_matrix.at<double>(i, j);

		return ret;
	}

	lineVertexPair GroundCalibrator::createCorner3DLine(cv::Point2f pt, TMat4 transform)
	{
		Vec3 uvPoint = Vec3(pt.x, pt.y, 1);

		RMat3 rotationMatrix = transform.block<3, 3>(0, 0);
		Vec3 tvec = transform.block<3, 1>(0, 3);

		Vec3 vec1 = rotationMatrix.inverse() * cameraMatrix.inverse() * uvPoint;
		Vec3 vec2 = rotationMatrix.inverse() * tvec;

		double s1 = 50 + vec2(2, 0); //50 represents the height Zconst
		s1 /= vec1(2, 0);
		Vec3 vex1 = rotationMatrix.inverse() * (s1 * cameraMatrix.inverse() * uvPoint - tvec);

		double s2 = 1 + vec2(2, 0); //1 represents the height Zconst
		s2 /= vec1(2, 0);
		Vec3 vex2 = rotationMatrix.inverse() * (s2 * cameraMatrix.inverse() * uvPoint - tvec);

		std::pair<Vec3, Vec3> ret = std::pair<Vec3, Vec3>(vex1, vex2);

		return ret;
	}

	double GroundCalibrator::dist3DFromPtToLine(Vec3 pt, lineVertexPair vertexPair)
	{
		Vec3 vec = vertexPair.second - vertexPair.first;

		return (vec.cross(pt - vertexPair.second)).norm()
			/ vec.norm();
	}

	void GroundCalibrator::computeCalibResult()
	{
		if (threadComputeResult.joinable())
			threadComputeResult.join();

		threadComputeResult = std::thread([&]() {
		
			//----------------------------------------------------
			//				   compute plane points
			//----------------------------------------------------
			//Vec3 corners[BOARD_CORNER_NUM] = { Vec3::Zero(), Vec3::Zero(), Vec3::Zero(), Vec3::Zero() };
			std::map<int, Vec3> ptIndexMap;
			if (multiThreading)
			{
				std::map<int, Vec3> map_buf[NUM_THREADS];
				for (int i = 0; i < ONLINE_POINT_NUM; i++)
					ptOptimizeReduce.reduce(boost::bind(&GroundCalibrator::ptOptimize_Reductor, this, i, map_buf, _1, _2, _3, _4), 0, ONLINE_POINT_NUM, 0);

				for (int i = 0; i < NUM_THREADS; i++)
					ptIndexMap.insert(map_buf[i].begin(), map_buf[i].end());
			}
			else
			{
				for (int i = 0; i < ONLINE_POINT_NUM; i++)
				{
					for (int j = 0; j < ONLINE_POINT_NUM; j++)
					{
						Vec3 pt;
						std::map<int, lineVertexPair> planePtVertex = planePtLineVertex[i][j];

						if (!optimizePoint(planePtVertex, pt))
							continue;

						int index = i * ONLINE_POINT_NUM + j;
						ptIndexMap.emplace(index, pt);
					}
				}
			}

			//----------------------------------------------------
			//				compute an optimized plane
			//----------------------------------------------------
			Vec4 plane_vec;
			optimizePlane(ptIndexMap, plane_vec);
			std::unique_ptr<Plane3D> plane = std::make_unique<Plane3D>(plane_vec);
			//std::cout << "Left Points: " << (float)(ptIndexMap.size()) / (float)(ONLINE_POINT_NUM * ONLINE_POINT_NUM) * 100 << "%" << std::endl;

			//----------------------------------------------------
			//  recover the board from left good board points
			//----------------------------------------------------
			// set up board 2D-3D pose map
			Vec3 pose_default = Vec3(dbl_max, dbl_max, dbl_max);
			std::vector<std::vector<Vec3>> ptIndexVec;
			ptIndexVec.reserve(ONLINE_POINT_NUM);
			for (int i = 0; i < ONLINE_POINT_NUM; i++)
			{
				ptIndexVec.push_back(std::vector<Vec3>());
				ptIndexVec[i].reserve(ONLINE_POINT_NUM);

				for (int j = 0; j < ONLINE_POINT_NUM; j++)
				{
					int index = i * ONLINE_POINT_NUM + j;
					auto it = ptIndexMap.find(index);
					if (it != ptIndexMap.end())
						ptIndexVec[i].push_back(/*it->second*/plane->pointToPlane(it->second));
					else
						ptIndexVec[i].push_back(pose_default);
				}
			}

			// compute unit height, board vertical direction & horizontal direction
			int unit_height_count = 0;
			double unit_height = 0;

			int h_vec_count = 0;
			Vec3 h_vec_mean = Vec3::Zero();

			int v_vec_count = 0;
			Vec3 v_vec_mean = Vec3::Zero();

			std::vector<Vec3> v_vec;
			std::vector<Vec3> h_vec;
			std::vector<std::pair<int, int>> indeh_vec;

			for (int i = 0; i < ONLINE_POINT_NUM; i++)
			{
				for (int j = 0; j < ONLINE_POINT_NUM; j++)
				{
					if (ptIndexVec[i][j] == pose_default)
						continue;

					indeh_vec.push_back(std::make_pair(i, j));

					// bottom
					if (i != ONLINE_POINT_NUM - 1)
					{
						if (ptIndexVec[i + 1][j] != pose_default)
						{
							unit_height_count++;
							unit_height += (ptIndexVec[i][j] - ptIndexVec[i + 1][j]).norm();

							v_vec_count++;
							Vec3 v_vec_buf = (ptIndexVec[i + 1][j] - ptIndexVec[i][j]).normalized();
							v_vec.push_back(v_vec_buf);
							v_vec_mean += v_vec_buf;
						}
					}

					// right
					if (j != ONLINE_POINT_NUM - 1)
					{
						if (ptIndexVec[i][j + 1] != pose_default)
						{
							unit_height_count++;
							unit_height += (ptIndexVec[i][j + 1] - ptIndexVec[i][j]).norm() / (double)(board_w_by_h_ratio);

							h_vec_count++;
							Vec3 h_vec_buf = (ptIndexVec[i][j + 1] - ptIndexVec[i][j]).normalized();
							h_vec.push_back(h_vec_buf);
							h_vec_mean += h_vec_buf;
						}
					}
				}
			}

			if (unit_height_count != 0)
				unit_height /= (double)(unit_height_count);

			if (h_vec_count != 0)
				h_vec_mean /= (double)(h_vec_count);
			h_vec_mean.normalize();

			if (v_vec_count != 0)
				v_vec_mean /= (double)(v_vec_count);
			v_vec_mean.normalize();


			double angle = abs(MathLib::getAngle(h_vec_mean, v_vec_mean, plane->normal));
			if (abs(angle - double(PI_05)) > 2.0 * (double)(DEG_2_RAD))
				return retry();


			// find an ideal combination of perpendicular directions
			if (angle >= (double)(PI_05) + MathLib::eps || angle <= (double)(PI_05) - MathLib::eps)
			{
				Vec3 h_vec_buf = h_vec_mean.normalized();
				//Vec3 v_vec_buf = (cos(PI_05) * h_vec_buf + sin(PI_05) * (plane->normal.cross(h_vec_buf))).normalized();
				Vec3 v_vec_buf = (plane->normal.cross(h_vec_buf)).normalized();
				if ((v_vec_buf - v_vec_mean).norm() > v_vec_mean.norm())
					v_vec_buf = -v_vec_buf;

				double shift_angle = 0;
				for (int j = 0; j < h_vec.size(); j++)
					shift_angle += MathLib::getAngle(h_vec_buf, h_vec[j], plane->normal);
				for (int j = 0; j < v_vec.size(); j++)
					shift_angle += MathLib::getAngle(v_vec_buf, v_vec[j], plane->normal);
				shift_angle /= (int)(h_vec.size() + v_vec.size());

				h_vec_mean = (cos(shift_angle) * h_vec_buf + sin(shift_angle) * (plane->normal.cross(h_vec_buf))).normalized();
				v_vec_mean = (cos(shift_angle) * v_vec_buf + sin(shift_angle) * (plane->normal.cross(v_vec_buf))).normalized();
			}

			Vec3 h_vec_unit = h_vec_mean * unit_height * (double)(board_w_by_h_ratio);
			Vec3 v_vec_unit = v_vec_mean * unit_height;

			Vec3 pt_offset = Vec3::Zero();
			for (int i = 0; i < indeh_vec.size(); i++)
				pt_offset += (ptIndexVec[indeh_vec[i].first][indeh_vec[i].second] +
				(double)(ONLINE_POINT_NUM - 1 - indeh_vec[i].first) * v_vec_unit - (double)(indeh_vec[i].second) * h_vec_unit);
			pt_offset /= (double)(indeh_vec.size());

			Vec3 new_corners[BOARD_CORNER_NUM];
			new_corners[BOARD_CORNER_ID_LT] = pt_offset - (double)(ONLINE_POINT_NUM - 1) * v_vec_unit;
			new_corners[BOARD_CORNER_ID_RT] = pt_offset - (double)(ONLINE_POINT_NUM - 1) * v_vec_unit + (double)(ONLINE_POINT_NUM - 1) * h_vec_unit;
			new_corners[BOARD_CORNER_ID_RB] = pt_offset + (double)(ONLINE_POINT_NUM - 1) * h_vec_unit;
			new_corners[BOARD_CORNER_ID_LB] = pt_offset;

			std::unique_ptr<Board3D> board3D = std::make_unique<Board3D>(
				new_corners[BOARD_CORNER_ID_LT], new_corners[BOARD_CORNER_ID_RT],
				new_corners[BOARD_CORNER_ID_RB], new_corners[BOARD_CORNER_ID_LB]);

			if (!board3D->isGood)
				return retry();

			//// further optimize board
			//board3D->optimizePoints();

			groundScale = board_h / board3D->getMediumHeight();
			groundOffsetMatrix.setZero();
			groundOffsetMatrix.col(3).segment<3>(0) = board3D->corner[BOARD_CORNER_ID_LB]->position;

			// compute rotate
			TMat4 tt = board3D->getRotateDiffToGround();
			groundRotateMatrix = tt;

			//----------------------------------------------------
			//                   verify result
			//----------------------------------------------------
			if (!verifyResult())
				return retry();
				
			printf("Computed Scale = %f\n", groundScale);
			printf("Error: = %.2f mm\n", distError * 1000);
			opTime = clock() - opTime;
			double duration = (double)opTime / CLOCKS_PER_SEC;
			printf("Use Time: %f s\n", duration);
			printf("Point Cloud Calibration Succesed\n");

			resetCalibration();
			state = GroundCalibState::FINISHED;
		});
	}

	bool GroundCalibrator::intersectLines(Eigen::MatrixXd &PA, Eigen::MatrixXd &PB, Vec3& corner)
	{
		int row = PA.rows();
		Eigen::MatrixXd Si = PA - PB;

		// Normalize vectors
		Eigen::MatrixXd Si_cwiseProdocut = Si.cwiseProduct(Si);
		Eigen::MatrixXd ni_R(row, 3);
		for (int i = 0; i < row; i++)
			ni_R(i, 0) = ni_R(i, 1) = ni_R(i, 2) = 1 / sqrt(Si_cwiseProdocut.row(i).sum());
		Eigen::MatrixXd ni(row, 3);
		ni = Si.block(0, 0, row, 3).cwiseProduct(ni_R.block(0, 0, row, 3));

		Eigen::MatrixXd nx = ni.col(0);
		Eigen::MatrixXd ny = ni.col(1);
		Eigen::MatrixXd nz = ni.col(2);

		Eigen::MatrixXd ones(row, 1);
		ones.setOnes();
		Eigen::MatrixXd nxSquare = nx.cwiseProduct(nx) - ones;
		Eigen::MatrixXd nySquare = ny.cwiseProduct(ny) - ones;
		Eigen::MatrixXd nzSquare = nz.cwiseProduct(nz) - ones;
		Eigen::MatrixXd nxny = nx.cwiseProduct(ny);
		Eigen::MatrixXd nxnz = nx.cwiseProduct(nz);
		Eigen::MatrixXd nynz = ny.cwiseProduct(nz);

		double SXX = nxSquare.sum();
		double SYY = nySquare.sum();
		double SZZ = nzSquare.sum();
		double SXY = nxny.sum();
		double SXZ = nxnz.sum();
		double SYZ = nynz.sum();

		Eigen::Matrix3d S;
		S(0, 0) = SXX;
		S(1, 0) = S(0, 1) = SXY;
		S(0, 2) = S(2, 0) = SXZ;
		S(1, 1) = SYY;
		S(2, 1) = S(1, 2) = SYZ;
		S(2, 2) = SZZ;

		Eigen::MatrixXd PAx = PA.col(0);
		Eigen::MatrixXd PAy = PA.col(1);
		Eigen::MatrixXd PAz = PA.col(2);
		double CX = (PAx.cwiseProduct(nxSquare) + PAy.cwiseProduct(nxny) + PAz.cwiseProduct(nxnz)).sum();
		double CY = (PAx.cwiseProduct(nxny) + PAy.cwiseProduct(nySquare) + PAz.cwiseProduct(nynz)).sum();
		double CZ = (PAx.cwiseProduct(nxnz) + PAy.cwiseProduct(nynz) + PAz.cwiseProduct(nzSquare)).sum();

		Eigen::Matrix<double, 3, 1> C;
		C(0, 0) = CX;
		C(1, 0) = CY;
		C(2, 0) = CZ;

		try
		{
			corner = S.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV).solve(C);
			/*corner = S.colPivHouseholderQr().solve(C);*/
		}
		catch (cv::Exception& e)
		{
			return false;
		}

		return true;
	}

	bool GroundCalibrator::optimizePoint(std::map<int, lineVertexPair>& lineVertex, Vec3& cornerPt, double prevDist)
	{
		// compute intersection point using least square
		int size = (int)lineVertex.size();
		Eigen::MatrixXd PA(size, 3), PB(size, 3);

		int j = 0;
		for (auto it = lineVertex.begin(); it != lineVertex.end(); it++)
		{
			PA.row(j) = it->second.first;
			PB.row(j) = it->second.second;
			j++;
		}

		//cv::Mat corner;
		Vec3 corner;

		if (!intersectLines(PA, PB, corner))
			return false;
		//Vec3 pt = corner;




		// compute distance from intersection point to lines in 3D
		std::vector<std::pair<int, double>> distVector;
		float avgDist = 0;

		for (auto it = lineVertex.begin(); it != lineVertex.end(); it++)
		{
			double dist = dist3DFromPtToLine(/*pt*/corner, it->second);
			avgDist += dist;
			distVector.push_back(std::pair<int, double>(it->first, dist));
		}
		avgDist /= (float)size;
		
		float var_dist = 0;
		for (auto it = distVector.begin(); it != distVector.end(); it++)
			var_dist += pow(it->second - avgDist, 2);

		var_dist = sqrt(var_dist / (float)size);


		// optimize: make avgDist closest to threshold(corner3DPtOptTH)
		int min_single_corner = (int)setting_GCMinCornerCount * 0.1;
		if (/*avgDist*/var_dist < prevDist && 
			/*avgDist*/var_dist > setting_GCIntPtOptTH && 
			lineVertex.size() > (2 > min_single_corner ? 2 : min_single_corner))
		{
			std::sort(distVector.begin(), distVector.end(),
				[](const std::pair<int, double>& p1, const std::pair<int, double>& p2) {
				return p1.second < p2.second; });

			// erase max 10% lines for fast converge
			int erase_lines = (int)std::max(double(1), (1 - pow((setting_GCIntPtOptTH / var_dist/*avgDist*/), 2)) * 0.10 * size);
			for (int i = 1; i <= erase_lines; i++)
				lineVertex.erase(distVector[size - i].first);

			if (optimizePoint(lineVertex, cornerPt, var_dist/*avgDist*/))
				return true;
			else
				return false;
		}
		else
		{
			cornerPt = corner/*pt*/;
			return true;
		}
	}

	void GroundCalibrator::optimizePlane(std::map<int, Vec3>& planePts, Vec4& planeVec, double prevDist)
	{
		// compute a plane using least square
		int size = (int)planePts.size();

		Vec4 plane_vec = MathLib::fitPlane(planePts);
		std::unique_ptr<Plane3D> plane = std::make_unique<Plane3D>(plane_vec);

		// compute distance from intersection point to lines in 3D
		std::vector<std::pair<int, double>> distVector;
		float avgDist = 0;

		for (auto it = planePts.begin(); it != planePts.end(); it++)
		{
			double dist = plane->distToPlane(it->second);
			avgDist += dist;
			distVector.push_back(std::pair<int, double>(it->first, dist));
		}
		avgDist /= (float)size;

		float var_dist = 0;
		for (auto it = distVector.begin(); it != distVector.end(); it++)
			var_dist += pow(it->second - avgDist, 2);

		var_dist = sqrt(var_dist / (float)size);

		// optimize: make avgDist closest to threshold(corner3DPtOptTH)
		if (var_dist < prevDist &&
			prevDist - var_dist >= setting_GCPlane3DPtOptTH * 0.01 &&
			var_dist > setting_GCPlane3DPtOptTH &&
			planePts.size() > 3)
		{
			std::sort(distVector.begin(), distVector.end(),
				[](const std::pair<int, double>& p1, const std::pair<int, double>& p2) {
				return p1.second < p2.second; });

			// erase max 10% lines for fast converge
			int erase_lines = (int)std::max(double(1), (1 - pow((setting_GCPlane3DPtOptTH / var_dist), 2)) * 0.10 * size);
			for (int i = 1; i <= erase_lines; i++)
				planePts.erase(distVector[size - i].first);

			optimizePlane(planePts, planeVec, var_dist);
		}
		else
			planeVec = plane_vec;
	}

	bool GroundCalibrator::detectCorners(cv::Mat image, std::vector<cv::Point2f>& markerCorners)
	{
		std::vector<int> markerIds;
		std::vector<std::vector<cv::Point2f>> markersCorners;
		// marker detection
		try
		{
			cv::aruco::detectMarkers(image, markerDictionary, markersCorners, markerIds, detectorParams);
		}
		catch (cv::Exception& e)
		{
			const char* err_msg = e.what();
			std::cout << "cv::aruco::detectMarkers() exception caught: " << err_msg << std::endl;
			return false;
		}

		switch (boardType)
		{
		case SingleMarker:
		{
			for (int i = 0; i < markerIds.size(); i++)
			{
				if (markerIds[i] == target_marker_id)
				{
					markerCorners = markersCorners[i];
					return true;
				}
			}

			//// draw marker border
			//{
			//	cv::Mat color;
			//	cv::cvtColor(image, color, cv::COLOR_GRAY2BGR);

			//	std::vector<std::vector<cv::Point2f>> markerCorners;

			//	markerCorners.push_back(marker_corners);
			//	//markerCorners.push_back(vv);
			//	cv::aruco::drawDetectedMarkers(color, markerCorners);

			//	std::vector<std::vector<cv::Point2f>> side_pts;
			//	board2D->getInterpolatedSidePt(marker_corners, ONLINE_POINT_NUM, side_pts);

			//	/*for (int i = 0; i < side_pts.size(); i += 37)
			//		for (int j = 0; j < side_pts[i].size(); j += 37)
			//			cv::circle(color, side_pts[i][j], 2, (0, 0, 255), -1);*/

			//	cv::imshow(std::to_string(frameCount_COLLECT), color);
			//	cv::waitKey(1);
			//}

			return false;
			break;
		}
		case ChArUcoBoard_3X3:
		{
			if (markerIds.size() < 4) return false;

			//cv::Ptr<cv::aruco::CharucoBoard> board = cv::aruco::CharucoBoard::create(3, 3, 0.04, 0.03, markerDictionary);
			std::vector<cv::Point2f> charucoCorners;
			std::vector<int> charucoIds;

			cv::aruco::interpolateCornersCharuco(markersCorners, markerIds, image, charucoBoard, charucoCorners, charucoIds);
			if (charucoIds.size() != BOARD_CORNER_NUM)
				return false;
			
			//// test only: draw chess board
			//{
			//	cv::Mat color;
			//	cv::cvtColor(image, color, cv::COLOR_GRAY2BGR);
			//	cv::aruco::drawDetectedCornersCharuco(color, charucoCorners, charucoIds, cv::Scalar(255, 0, 0));
			//	cv::imshow(std::to_string(frameCount_COLLECT), color);
			//	cv::waitKey(1);
			//}

			for (int i = 0; i < 4; i++)
				markerCorners.push_back(cv::Point2f(0, 0));

			for (int i = 0; i < 4; i++)
			{
				if (charucoIds[i] == 0)
					markerCorners[3] = charucoCorners[i];
				else if (charucoIds[i] == 1)
					markerCorners[2] = charucoCorners[i];
				else if (charucoIds[i] == 2)
					markerCorners[0] = charucoCorners[i];
				else if (charucoIds[i] == 3)
					markerCorners[1] = charucoCorners[i];
			}

			return true;
			break;
		}
		default:
			return false;
			break;
		}
	}

	void GroundCalibrator::recordData(TMat4 transform, std::vector<cv::Point2f>& boardCorners)
	{
		std::vector<std::vector<cv::Point2f>> side_pts;
		board2D->getInterpolatedSidePt(boardCorners, ONLINE_POINT_NUM, side_pts);

		for (int i = 0; i < ONLINE_POINT_NUM; i++)
			for (int j = 0; j < ONLINE_POINT_NUM; j++)
				planePtLineVertex[i][j].emplace((int)planePtLineVertex[i][j].size(),
					createCorner3DLine(side_pts[i][j], transform.inverse()));
	}

	void GroundCalibrator::retry()
	{
		if (retryCount >= MAX_RETRY_COUNT)
		{
			printf("Stop: bad result.\n");
			printf("Point Cloud Calibration Failed,\nLet's Try Again or Lower the Error Threshold Setting !\n");
			resetCalibration();
			state = GroundCalibState::WAIT;
		}
		else
		{
			//printf("Bad corner collection, collect more ...\n");
			retryCount++;
			frameCount_COMPUTE += (int)(setting_GCMinCornerCount * 0.5f);
			state = GroundCalibState::COLLECT_CORNERS;
		}
	}

	bool GroundCalibrator::collectCorners(cv::Mat image, TMat4 transform)
	{
		frameCount_COLLECT++;

		// parameter settings
		std::vector<cv::Point2f> board_corners;

		if (!detectCorners(image, board_corners))
		{
			if (frameCount_COLLECT >= setting_GCMaxFrames)
			{
				printf("Stop: no enough corners when reach max frame.\n");
				printf("Point Cloud Calibration Failed\n");
				stopCalibration();
				state = GroundCalibState::WAIT;
			}
		
			return false;
		}

		recordData(transform, board_corners);

		// compute result
		if (frameCount_COLLECT >= frameCount_COMPUTE)
		{
			state = GroundCalibState::COMPUTE;
			computeCalibResult();
		}

		return true;
	}

	bool GroundCalibrator::verifyResult()
	{
		double unit_w = board_w / (double)(ONLINE_POINT_NUM - 1);
		double unit_h = board_h / (double)(ONLINE_POINT_NUM - 1);

		TMat4 scale_matrix = TMat4::Identity();
		scale_matrix(0, 0) = groundScale;
		scale_matrix(1, 1) = groundScale;
		scale_matrix(2, 2) = groundScale;
		TMat4 m = scale_matrix * groundRotateMatrix/* * (ret - groundOffsetMatrix)*/;

		double avg_error = 0;
		double total_result_count = 0;
		double good_result_count = 0;

		std::unique_ptr<Plane3D> groundPlane = std::make_unique<Plane3D>(Vec3(0, 0, 0),
			Vec3(0, 0, board_h), Vec3(board_w, 0, 0));

		for (int i = 0; i < ONLINE_POINT_NUM; i++)
		{
			for (int j = 0; j < ONLINE_POINT_NUM; j++)
			{
				Vec3 pt_ref = Vec3((double)(j)* unit_w, 0, (double)(ONLINE_POINT_NUM - 1 - i) * unit_h);

				for (auto it = planePtLineVertex[i][j].begin(); it != planePtLineVertex[i][j].end(); it++)
				{
					Vec4 p[2] = { 
						Vec4(it->second.first.x(),  it->second.first.y(),  it->second.first.z(),  1.0) ,
						Vec4(it->second.second.x(), it->second.second.y(), it->second.second.z(), 1.0) };

					p[0] = m * (p[0] - groundOffsetMatrix.col(3));
					p[1] = m * (p[1] - groundOffsetMatrix.col(3));

					Vec3 dp = p[1].block<3, 1>(0, 0) - p[0].block<3, 1>(0, 0);
					double t = -(groundPlane->normal.dot(p[0].block<3, 1>(0, 0)) + groundPlane->d) / groundPlane->normal.dot(dp);
					Vec3 pt_int = p[0].block<3, 1>(0, 0) + dp * t;
					
					double error = (pt_int - pt_ref).norm();
					if (error <= setting_GCErrorTH)
					{
						avg_error += error;
						good_result_count++;
					}
					total_result_count++;
				}
			}
		}

		if (float(good_result_count) / (float)(total_result_count) > GOOD_RESULT_PERCENTAGE)
		{
			distError = avg_error / (double)(good_result_count);
			return true;
		}

		//std::cout << "Low Percent: " << float(good_result_count) / (float)(total_result_count) * 100 << std::endl;
			
		return false;
	}

	bool GroundCalibrator::waitToStart(cv::Mat image, TMat4 transform)
	{
		bool ret = false;

		frameCount_WAIT++;
		if (frameCount_WAIT % WAIT_TO_START_FRAME == 0)
		{
			// parameter settings
			std::vector<cv::Point2f> board_corners;

			if (detectCorners(image, board_corners))
			{
				startCalibration();
				collectCorners(image, transform);
				ret = true;
			}
		}

		return ret;
	}

	void GroundCalibrator::ptOptimize_Reductor(int row_index, std::map<int, Vec3>* map_buf, int min, int max, Vec10* stats, int tid)
	{
		for (int k = min; k < max; k++)
		{
			Vec3 pt;
			std::map<int, lineVertexPair> planePtVertex = planePtLineVertex[row_index][k];
			if (!optimizePoint(planePtVertex, pt))
				continue;

			int index = row_index * ONLINE_POINT_NUM + k;
			map_buf[tid].emplace(index, pt);

			/*if (index == 0)
				corners[BOARD_CORNER_ID_LT] = pt;
			else if (index == ONLINE_POINT_NUM - 1)
				corners[BOARD_CORNER_ID_RT] = pt;
			else if (index == ONLINE_POINT_NUM * ONLINE_POINT_NUM - 1)
				corners[BOARD_CORNER_ID_RB] = pt;
			else if (index == (ONLINE_POINT_NUM - 1) * ONLINE_POINT_NUM)
				corners[BOARD_CORNER_ID_LB] = pt;*/
		}
	}
}