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

#undef Success
#include <Eigen/Core>
#include "util/NumType.h"
#include <pangolin/pangolin.h>

#include <sstream>
#include <fstream>

namespace dso
{
struct CalibHessian;
struct FrameHessian;
class FrameShell;

namespace IOWrap
{

template<int ppp>
struct InputPointSparse
{
	float u;
	float v;
	float idpeth;
	float idepth_hessian;
	float relObsBaseline;
	int numGoodRes;
	unsigned char color[ppp];
	unsigned char status;
};

struct MyVertex
{
	float point[3];
	unsigned char color[4];
};

// stores a pointcloud associated to a Keyframe.
class KeyFrameDisplay
{

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
	KeyFrameDisplay(int channel);
	~KeyFrameDisplay();

	// copies points from KF over to internal buffer,
	// keeping some additional information so we can render it differently.
	void setFromKF(FrameHessian* fh, CalibHessian* HCalib);

	// copies points from KF over to internal buffer,
	// keeping some additional information so we can render it differently.
	void setFromF(FrameShell* fs, CalibHessian* HCalib);

	// copies & filters internal data to GL buffer for rendering. if nothing to do: does nothing.
	bool refreshPC(bool canRefresh, float scaledTH, float absTH, int mode, float minBS, int sparsity, bool is_last = false, bool settings_pointCloudAlign = false);
	void setNeedRefreshPC(int enable) { needRefresh = enable; }

	// renders cam & pointcloud.
	void drawCam(float lineWidth = 1, float* color = 0, float sizeFactor = 1, bool settings_pointCloudAlign = false);
	void drawPC(float pointSize, bool settings_pointCloudAlign = false);
	//cinetec mod:Yang
	void savePC(std::ofstream& myfile, bool settings_pointCloudAlign = false);
	////////////////

	int id;
	double timestamp;
	float exposure;
	bool active;
	SE3 camToWorld;
	bool is_new_frame;

	inline bool operator < (const KeyFrameDisplay& other) const
	{
		return (id < other.id);
	}

	int getFrameID();
	double getFrameTimestamp() { return timestamp; }
	double getFrameExposure() { return exposure; }
	Sophus::Vector3f getPosition(bool settings_pointCloudAlign = false);
	Sophus::Matrix3f getRotationMatrix(bool settings_pointCloudAlign = false);
	Eigen::Quaternion<float> getRotation(bool settings_pointCloudAlign = false);

	void makeKeyframePC();
	void makeKeyframePC(float maxDisplayDist, bool maturePtOnly = true);
	std::vector<Eigen::Vector3f> getKeyframePC();

	void setDelta(float dx, float dy, float dz, float dsx, float dsy, float dsz) { delta_x = dx; delta_y = dy; delta_z = dz; delta_sx = dsx; delta_sy = dsy;
		delta_sz = dsz;
	}
	float deltaX() { return delta_x; }
	float deltaY() { return delta_y; }
	float deltaZ() { return delta_z; }
	float deltaSX() { return delta_sx; }
	float deltaSY() { return delta_sy; }
	float deltaSZ() { return delta_sz; }
	Sophus::Vector3f deltaPos() { return Sophus::Vector3f(delta_x, delta_y, delta_z); }
	Sophus::Vector3f deltaScale() { return  Sophus::Vector3f(delta_sx, delta_sy, delta_sz); }
	Sophus::Matrix4f getCam2WorldMatrix4f(bool settings_pointCloudAlign = false);
	Sophus::SE3f getCam2WorldSE3f(bool settings_pointCloudAlign = false);

private:
	int channel;

	float fx,fy,cx,cy;
	float fxi,fyi,cxi,cyi;
	int width, height;

	float my_scaledTH, my_absTH, my_scale;
	int my_sparsifyFactor;
	int my_displayMode;
	float my_minRelBS;
	bool needRefresh;


	int numSparsePoints;
	int numSparseBufferSize;
    InputPointSparse<MAX_RES_PER_POINT>* originalInputSparse;


	bool bufferValid;
	int numGLBufferPoints;
	int numGLBufferGoodPoints;
	pangolin::GlBuffer vertexBuffer;
	pangolin::GlBuffer colorBuffer;

	bool is_new_keyframe;
	std::vector<Eigen::Vector3f> keyframePC;


	float delta_x, delta_y, delta_z, delta_sx, delta_sy, delta_sz;
};

}
}

