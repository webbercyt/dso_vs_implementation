#pragma once

#ifndef __SOCKETWRAPPER_H__
#define __SOCKETWRAPPER_H__


#if defined WIN32
	#define NOMINMAX
	#include <winsock2.h>
	#pragma comment(lib,"ws2_32.lib") //Winsock Library
	#define MINISOCK_LAST_ERROR WSAGetLastError()
#define socklen_t int
#else
	#include <sys/socket.h>
	#include <netinet/in.h>
	#include <fcntl.h>
	#define MINISOCK_LAST_ERROR errno
	#define SOCKET_ERROR -1
	#define INVALID_SOCKET -1
#endif

#include "boost/thread.hpp"
#include "IOWrapper/Output3DWrapper.h"
#include "IOWrapper/Pangolin/KeyFrameDisplay.h"
#include "FullSystem/HessianBlocks.h"


#define PORT_CAMERA 8008   //The port on which to listen for incoming data
#define PORT_POINT_CLOUD 8888   //The port on which to listen for incoming data
#define PORT_COMMAND 8088   //The port on which to listen for incoming data
#define PORT_CAMERA2 8009   //The port on which to listen for incoming data
#define PORT_POINT_CLOUD2 8889   //The port on which to listen for incoming data
#define PORT_COMMAND2 8089   //The port on which to listen for incoming data
#define MAX_RECV_LEN 64


namespace dso
{

namespace IOWrap
{
class KeyFrameDisplay;

class SocketWrapper : public Output3DWrapper
{
public:
	SocketWrapper(int channel);
	virtual ~SocketWrapper();
	virtual void publishGraph(const std::map<uint64_t, Eigen::Vector2i> &connectivity);
	virtual void publishKeyframes(std::vector<FrameHessian*> &frames, bool final, CalibHessian* HCalib);
	//virtual void publishCamPose(FrameShell* frame, CalibHessian* HCalib);
	virtual void publishCamPose(FrameShell* frame, CalibHessian* HCalib, char status, int numDropFrame, int camFrameID);
	virtual void pushLiveFrame(FrameHessian* image);
	virtual void pushDepthImage(MinimalImageB3* image);
	virtual bool needPushDepthImage();
	virtual void pushDepthImageFloat(MinimalImageF* image, FrameHessian* KF);
	virtual void reset();
	virtual void pause(bool enable);

	virtual void lockModel3DMutex() override;
	virtual void publishGroundCalibParams() override;
	virtual void setNeedGroundCalibration(bool state) override;
	virtual void setSkipFrame(bool isSkipFrame) override;
	virtual bool getSkipFrame() override;
	virtual void setNumSkipFrame(int numFrame) override;
	virtual int getNumSkipFrame() override;

	void setTrackingState(int state) { tracking_state = state; }
	void isStartStreamingCamera();
	void isStartStreamingPointCloud();
	void checkCommand();
	void sendPC();
	void run(int state);
	void setFullSystemReset(bool state);
	void initSocket();
	void closeSocket();

	// interface
	int getFrameID() { return track_data.frame_id; }
	double getTimeStamp() { return track_data.time_stamp; }
	double getExposure() { return track_data.exposure; }
	float getPositionX() { return track_data.x; }
	float getPositionY() { return track_data.y; }
	float getPositionZ() { return track_data.z; }
	float getRotationX() { return track_data.q0; }
	float getRotationY() { return track_data.q1; }
	float getRotationZ() { return track_data.q2; }
	float getRotationW() { return track_data.q3; }
	void setCallback(void(*CamPoscallback)(void*, void*), void(*LiveImagecallback)(void*, void*), void(*PCcallback)(void*, void*), void(*PCCalicallback)(void*, void*), void* pUserData) {
		this->CamPoscallback = CamPoscallback;
		this->LiveImagecallback = LiveImagecallback;
		this->PCcallback = PCcallback;
		this->PCCalicallback = PCCalicallback;
		this->pUserData = pUserData;
	}
	int getPC(std::vector<float>& pcs) { 
		int size = keyframe_calib_pcs.size();
		pcs.resize(3 * size); 
		memcpy(pcs.data(), keyframe_calib_pcs.data(), 3 * size * sizeof(float));  
		return size;
	}

private:
	int channel;

	//void sendCamera(int frame_id, double time_stamp, Eigen::Vector3f p, Eigen::Quaternion<float> q);
	void getTrackData(int frame_id, double time_stamp, float exposure, Eigen::Vector3f p, Eigen::Quaternion<float> q, char status, int numDropFrame);
	void sendCamera();

	// 3D model rendering
	boost::mutex model3DMutex;
	KeyFrameDisplay* currentCam;
	KeyFrameDisplay* lastCam;

#ifdef WIN32
	SOCKET s_camera,s_point_cloud,s_command;
	WSADATA wsa_camera, wsa_point_cloud, wsa_command;
#else
	int s_camera, s_point_cloud, s_command;
#endif
	struct sockaddr_in server_camera, si_other_camera;
	int slen_camera, send_len_camera, recv_len_camera;
	std::vector<struct sockaddr_in> si_other_cameras;

	struct sockaddr_in server_point_cloud, si_other_point_cloud;
	int slen_point_cloud, send_len_point_cloud, recv_len_point_cloud;
	struct pc_struct
	{
		struct sockaddr_in si;
		int index;
	};
	std::vector<pc_struct> si_other_point_clouds;

	struct sockaddr_in server_command, si_other_command;
	int slen_command, send_len_command, recv_len_command;

	char recvBuf[MAX_RECV_LEN];

	struct TrackData
	{
		int frame_id;				// frame id
		double time_stamp;			// time stamp
		float exposure;				// exposure
		float x,y,z;				// position
		float q0, q1, q2, q3;		// rotation
		char status;				// tracking status
		int numDropFrames;
	} track_data;

	struct ImageData
	{
		int w;
		int h;
		unsigned char* data;

		ImageData()
		{
			w = 640;
			h = 512;
			data = NULL;
		}

		~ImageData()
		{
			if (data) delete[] data;
		}

		void setImage(FrameHessian* image)
		{
			if ((w != image->orgw || h != image->orgh) && data != NULL)
			{
				delete[] data;
				data = NULL;
			}
			w = image->orgw;
			h = image->orgh;
			if(!data) data = new unsigned char[w * h];
			memcpy(data, image->orgImage, w * h);
		}
	}image_data;

	struct PCData
	{
		int num;
		std::vector<float> data;

		PCData()
		{
			num = 0;
		}
	}pc_data;

	struct PCTransform
	{
		float scale;
		float offset[3];
		//float position[3];
		float rotation[4];
	}pc_transform;

	std::vector<Eigen::Vector3f> keyframe_pcs;
	std::vector<Eigen::Vector3f> keyframe_calib_pcs;

	int tracking_state;
	bool skipFrame;
	int numSkipFrame;

	void (*CamPoscallback)(void*, void*);
	void (*LiveImagecallback)(void*, void*);
	void (*PCcallback)(void*, void*);
	void (*PCCalicallback)(void*, void*);
	void *pUserData;
};

}

}
#endif //end of _SOCKETWRAPPER_H_
