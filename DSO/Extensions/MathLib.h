/*
* This file is part of DSO
*
* MathLib.h
*	Created on: May 17, 2018
*	by: Mumbear
*
* geometric math library
*/

#pragma once

#include "util/NumType.h"
#include <map>
#include <opencv2/core.hpp>

typedef Sophus::Matrix4d TMat4;	// 4 * 4 transformation matrix
typedef Sophus::Matrix3d RMat3;	// 3 * 3 rotation matrix

namespace dso
{
	namespace MathLib
	{
		static const Vec3 xAxisVec = Vec3(1, 0, 0);
		static const Vec3 yAxisVec = Vec3(0, 1, 0);
		static const Vec3 zAxisVec = Vec3(0, 0, 1);

		static const double eps = 1e-9;

		// find the plane with the least square distance to a point set
		static void fitPlane(const CvMat* points, double* plane)
		{
			// Estimate geometric centroid.
			int nrows = points->rows;
			int ncols = points->cols;
			int type = points->type;
			CvMat* centroid = cvCreateMat(1, ncols, type);
			cvSet(centroid, cvScalar(0));
			for (int c = 0; c<ncols; c++)
			{
				for (int r = 0; r < nrows; r++)
				{
					centroid->data.db[c] += points->data.db[ncols*r + c];
				}
				centroid->data.db[c] /= nrows;
			}

			// Subtract geometric centroid from each point.
			CvMat* points2 = cvCreateMat(nrows, ncols, type);
			for (int r = 0; r<nrows; r++)
				for (int c = 0; c<ncols; c++)
					points2->data.db[ncols*r + c] = points->data.db[ncols*r + c] - centroid->data.db[c];

			// Evaluate SVD of covariance matrix.
			CvMat* A = cvCreateMat(ncols, ncols, type);
			CvMat* W = cvCreateMat(ncols, ncols, type);
			CvMat* V = cvCreateMat(ncols, ncols, type);
			cvGEMM(points2, points, 1, NULL, 0, A, CV_GEMM_A_T);
			cvSVD(A, W, NULL, V, CV_SVD_V_T);

			// Assign plane coefficients by singular vector corresponding to smallest singular value.
			plane[ncols] = 0;
			for (int c = 0; c<ncols; c++) {
				plane[c] = V->data.db[ncols*(ncols - 1) + c];
				plane[ncols] += plane[c] * centroid->data.db[c];
			}

			// Release allocated resources.
			cvReleaseMat(&centroid);
			cvReleaseMat(&points2);
			cvReleaseMat(&A);
			cvReleaseMat(&W);
			cvReleaseMat(&V);
		}

		// compose data for fitPlane(const CvMat* points, double* plane)
		static Vec4 fitPlane(std::vector<Vec3> positions)
		{
			CvMat*points_mat = cvCreateMat(positions.size(), 3, CV_64FC1);
			for (int i = 0; i < positions.size(); ++i)
			{
				points_mat->data.db[i * 3 + 0] = positions[i].x();
				points_mat->data.db[i * 3 + 1] = positions[i].y();
				points_mat->data.db[i * 3 + 2] = positions[i].z();
			}
			double plane12[4] = { 0 };
			fitPlane(points_mat, plane12);

			return Vec4(plane12[0], plane12[1], plane12[2], plane12[3]);
		}

		// compose data for fitPlane(const CvMat* points, double* plane)
		static Vec4 fitPlane(std::map<int, Vec3> positions)
		{
			CvMat*points_mat = cvCreateMat(positions.size(), 3, CV_64FC1);
			int i = 0;
			for (auto it = positions.begin(); it != positions.end(); it++)
			{
				points_mat->data.db[i * 3 + 0] = it->second.x();
				points_mat->data.db[i * 3 + 1] = it->second.y();
				points_mat->data.db[i * 3 + 2] = it->second.z();
				i++;
			}

			double plane12[4] = { 0 };
			fitPlane(points_mat, plane12);

			return Vec4(plane12[0], plane12[1], plane12[2], plane12[3]);
		}

		// the included angle between two vectors
		static double getAngle(Vec3 v1, Vec3 v2, Vec3 up = Vec3(0, 1, 0)) 
		{
			Vec3 n = v1.cross(v2);
			double a = atan2((up.normalized()).dot(n), v1.dot(v2));
			return a;
		}

		// return the 4 * 4 rotate matrix to an axis and an angle
		static TMat4 rotate(Vec3 axis, double angle)
		{
			TMat4 t = TMat4::Identity();

			double s = sin(angle);
			double c = cos(angle);

			Vec3 v = axis.normalized();

			double xx = v.x() * v.x();
			double yy = v.y() * v.y();
			double zz = v.z() * v.z();
			double xy = v.x() * v.y();
			double yz = v.y() * v.z();
			double zx = v.z() * v.x();
			double xs = v.x() * s;
			double ys = v.y() * s;
			double zs = v.z() * s;

			t(0, 0) = (1.0 - c) * xx + c;
			t(0, 1) = (1.0 - c) * xy - zs;
			t(0, 2) = (1.0 - c) * zx + ys;

			t(1, 0) = (1.0 - c) * xy + zs;
			t(1, 1) = (1.0 - c) * yy + c;
			t(1, 2) = (1.0 - c) * yz - xs;

			t(2, 0) = (1.0 - c) * zx - ys;
			t(2, 1) = (1.0 - c) * yz + xs;
			t(2, 2) = (1.0 - c) * zz + c;

			return t;
		}

		// return the point projected from targetPt to plane
		static Vec3 pointToPlane(Vec3 planeNormal, Vec3 planePt, Vec3 targetPt)
		{
			float t = planeNormal.dot(planePt - targetPt) / (planeNormal.dot(planeNormal));
			return planeNormal * t + targetPt;
		}

		// return the distance from targetPt to plane
		static double distToPlane(Vec3 planeNormal, double planeD, Vec3 targetPt)
		{
			return abs(planeNormal.dot(targetPt) - planeD) / planeNormal.dot(planeNormal);
		}
	}
}
