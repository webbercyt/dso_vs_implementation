#include "IOWrapper/OutputWrapper/SocketWrapper.h"
#include <opencv2/calib3d.hpp>
#include "util/settings.h"
#ifndef WIN32
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <float.h>
#endif // WIN32

namespace dso
{
namespace IOWrap
{
SocketWrapper::SocketWrapper(int channel) : channel(channel)
{
	printf("OUT: Created SocketWrapper\n");

	currentCam = new KeyFrameDisplay(channel);
	lastCam = new KeyFrameDisplay(channel);

	tracking_state = 0;

	skipFrame = false;
	numSkipFrame = 0;

	CamPoscallback = NULL;
	LiveImagecallback = NULL;
	PCcallback = NULL;
	PCCalicallback = NULL;
	pUserData = NULL;
}

SocketWrapper::~SocketWrapper()
{
	printf("OUT: Destroyed SocketWrapper\n");
	closeSocket();

	delete currentCam;
	delete lastCam;
}

void SocketWrapper::publishGraph(const std::map<uint64_t, Eigen::Vector2i> &connectivity)
{
}



void SocketWrapper::publishKeyframes(std::vector<FrameHessian*> &frames, bool final, CalibHessian* HCalib)
{
	if (lastCam == NULL || final == false) return;

	boost::unique_lock<boost::mutex> lk(model3DMutex);

	FrameHessian* fh = frames[frames.size() - 1];

	// point cloud
	lastCam->setFromKF(fh, HCalib);
	lastCam->makeKeyframePC();
	std::vector<Eigen::Vector3f> pc = lastCam->getKeyframePC();
	pc_data.num = pc.size();
	pc_data.data.clear();
	for (int i = 0; i < pc.size(); i++)
	{
		keyframe_pcs.push_back(pc[i]);

		Sophus::Vector4f buf(pc[i][0], pc[i][1], pc[i][2], 1.0);
		buf = setting_groundScale[channel] * setting_groundRotate[channel] * (buf - setting_groundOffset[channel].col(3));

		Eigen::Vector3f cali_pc = buf.segment<3>(0);
		keyframe_calib_pcs.push_back(cali_pc);

		pc_data.data.push_back(pc[i][0]);
		pc_data.data.push_back(pc[i][1]);
		pc_data.data.push_back(pc[i][2]);
	}

	if (/*!tracking_state && */PCcallback) PCcallback((void*)&pc_data, pUserData);
}

//void SocketWrapper::publishCamPose(FrameShell* frame, CalibHessian* HCalib)
void SocketWrapper::publishCamPose(FrameShell* frame, CalibHessian* HCalib, char status, int numDropFrame, int camFrameID)
{
	currentCam->setFromF(frame, HCalib);

	//int frame_id = currentCam->getFrameID();
	int frame_id = camFrameID;
	if (frame_id != -1)
	{
		double timestamp = currentCam->getFrameTimestamp();
		float exposure = currentCam->getFrameExposure();
		Eigen::Vector3f p = currentCam->getPosition();
		Eigen::Quaternion<float> q = currentCam->getRotation();
		q.normalize();
		getTrackData(frame_id, timestamp, exposure, p, q, status, numDropFrame);
	}

	if (CamPoscallback) CamPoscallback((void*)&track_data, pUserData);

	//if (tracking_state > 0 || si_other_cameras.size() == 0) return;
	if (si_other_cameras.size() == 0) return;
	sendCamera();
}


void SocketWrapper::pushLiveFrame(FrameHessian* image)
{
	// can be used to get the raw image / intensity pyramid.

	image_data.setImage(image);
	delete[] image->orgImage;
	image->orgImage = NULL;

	if (LiveImagecallback) LiveImagecallback((void*)&image_data, pUserData);
}

void SocketWrapper::pushDepthImage(MinimalImageB3* image)
{
	// can be used to get the raw image with depth overlay.
}
bool SocketWrapper::needPushDepthImage()
{
	return false;
}

void SocketWrapper::pushDepthImageFloat(MinimalImageF* image, FrameHessian* KF)
{
}

void SocketWrapper::reset()
{ 
	keyframe_pcs.clear(); 
	keyframe_calib_pcs.clear();
}

void SocketWrapper::pause(bool enable)
{
	setting_pauseRequested[channel] = enable;
	printf("SocketWrapper::pause(): pause command\n");
}

void SocketWrapper::lockModel3DMutex()
{
	model3DMutex.lock();
}

void SocketWrapper::publishGroundCalibParams()
{
	keyframe_calib_pcs.clear();

	for (int i = 0; i < keyframe_pcs.size(); i++)
	{
		Sophus::Vector4f buf(keyframe_pcs[i][0], keyframe_pcs[i][1], keyframe_pcs[i][2], 1.0);
		buf = setting_groundScale[channel] * setting_groundRotate[channel] * (buf - setting_groundOffset[channel].col(3));
		keyframe_calib_pcs.push_back(buf.segment<3>(0));
	}
	if (PCCalicallback)
	{
		pc_transform.scale = setting_groundScale[channel](0, 0);
		Eigen::Quaternion<float> q = Eigen::Quaternion<float>(setting_groundRotate[channel].block<3,3>(0,0));
		pc_transform.rotation[0] = q.x();
		pc_transform.rotation[1] = q.y();
		pc_transform.rotation[2] = q.z();
		pc_transform.rotation[3] = q.w();
		for (int i = 0; i < 3; i++)
		{
			//pc_transform.position[i] = 0.0;
			pc_transform.offset[i] = setting_groundOffset[channel].col(3)[i];
		}

		PCCalicallback((void*)&pc_transform, pUserData);
	}
	model3DMutex.unlock();
}

void SocketWrapper::initSocket()
{
	// ------------- Start of Position Stream UDP Socket Init ------------------
	// Temporary placed here for tracking streaming 
	slen_camera = sizeof(si_other_camera);

	//Initialise winsock
#ifdef WIN32
	std::cout << "Streaming - Initialising Winsock..." << std::endl;
	if (WSAStartup(MAKEWORD(2, 2), &wsa_camera) != 0) {
		printf("Failed. Error Code : %d", MINISOCK_LAST_ERROR);
		exit(EXIT_FAILURE);
	}
	std::cout << "Streaming - Initialised." << std::endl;
#endif
	//Create a socket
	if ((s_camera = socket(AF_INET, SOCK_DGRAM, 0)) == INVALID_SOCKET) {
		printf("Could not create socket : %d", MINISOCK_LAST_ERROR);
	}
	std::cout << "Streaming - Socket created." << std::endl;

	//Prepare the sockaddr_in structure
	memset(&server_camera, 0, sizeof(server_camera));
	server_camera.sin_family = AF_INET;
	server_camera.sin_addr.s_addr = htonl(INADDR_ANY);
	server_camera.sin_port = htons(PORT_CAMERA);
	//server.sin_port = htons(PORT_DEBUG);

	//Bind
	if (bind(s_camera, (struct sockaddr *)&server_camera, sizeof(server_camera)) == SOCKET_ERROR)
	{
		printf("Streaming - server_camera Bind failed with error code : %d trying another port...\n", MINISOCK_LAST_ERROR);
		server_camera.sin_port = htons(PORT_CAMERA2);
		if (bind(s_camera, (struct sockaddr *)&server_camera, sizeof(server_camera)) == SOCKET_ERROR)
		{
			printf("Streaming - Bind failed again with error code : %d exit...\n", MINISOCK_LAST_ERROR);
			exit(EXIT_FAILURE);
		}
		printf("Streaming - server_camera Bind\n");
		
	}
	// If iMode!=0, non-blocking mode is enabled.
	u_long iMode_camera = 1;
#ifdef WIN32
	ioctlsocket(s_camera, FIONBIO, &iMode_camera);
#else
	int ss=fcntl(s_camera,F_GETFL,0);	
	fcntl(s_camera,F_SETFL, ss|O_NONBLOCK);
#endif
	// ------------- End of Position Stream UDP Socket Init ------------------


	// ------------- Start of Image Stream UDP Socket Init ------------------
	// Temporary placed here for tracking streaming 
	slen_point_cloud = sizeof(si_other_point_cloud);

	//Initialise winsock
#ifdef WIN32
	std::cout << "Image - Initialising Winsock..." << std::endl;
	if (WSAStartup(MAKEWORD(2, 2), &wsa_point_cloud) != 0) {
		printf("Failed. Error Code : %d", MINISOCK_LAST_ERROR);
		exit(EXIT_FAILURE);
	}
	std::cout << "Image - Initialised." << std::endl;
#endif
	//Create a socket
	if ((s_point_cloud = socket(AF_INET, SOCK_DGRAM, 0)) == INVALID_SOCKET) {
		printf("Could not create socket : %d", MINISOCK_LAST_ERROR);
	}
	std::cout << "Image - Socket created." << std::endl;

	//Prepare the sockaddr_in structure
	memset(&server_point_cloud, 0, sizeof(server_point_cloud));
	server_point_cloud.sin_family = AF_INET;
	server_point_cloud.sin_addr.s_addr = htonl(INADDR_ANY);
	server_point_cloud.sin_port = htons(PORT_POINT_CLOUD);
	//server_point_cloud.sin_port = htons(PORTIMG_DEBUG);

	//Bind
	if (bind(s_point_cloud, (struct sockaddr *)&server_point_cloud, sizeof(server_point_cloud)) == SOCKET_ERROR)
	{
		printf("Image - server_point_cloud Bind failed with error code : %d try another port...\n", MINISOCK_LAST_ERROR);
		server_point_cloud.sin_port = htons(PORT_POINT_CLOUD2);
		if (bind(s_point_cloud, (struct sockaddr *)&server_point_cloud, sizeof(server_point_cloud)) == SOCKET_ERROR)
		{
			printf("Streaming - Bind failed again with error code : %d exit...\n", MINISOCK_LAST_ERROR);
			exit(EXIT_FAILURE);
		}
	}
	// If iMode!=0, non-blocking mode is enabled.
	u_long iMode_point_cloud = 1;
#ifdef WIN32
	ioctlsocket(s_point_cloud, FIONBIO, &iMode_point_cloud);
#else
	ss=fcntl(s_point_cloud,F_GETFL,0);	
	fcntl(s_point_cloud,F_SETFL, ss|O_NONBLOCK);
#endif
	// ------------- End of Image Stream UDP Socket Init ------------------

	// ------------- Start of Command Stream UDP Socket Init ------------------
	// Temporary placed here for tracking streaming 
	slen_command = sizeof(si_other_command);

	//Initialise winsock
#ifdef WIN32
	std::cout << "Streaming - Initialising Winsock..." << std::endl;
	if (WSAStartup(MAKEWORD(2, 2), &wsa_command) != 0) {
		printf("Failed. Error Code : %d", MINISOCK_LAST_ERROR);
		exit(EXIT_FAILURE);
	}
	std::cout << "Streaming - Initialised." << std::endl;
#endif
	//Create a socket
	if ((s_command = socket(AF_INET, SOCK_DGRAM, 0)) == INVALID_SOCKET) {
		printf("Could not create socket : %d", MINISOCK_LAST_ERROR);
	}
	std::cout << "Streaming - Socket created." << std::endl;

	//Prepare the sockaddr_in structure
	memset(&server_command, 0, sizeof(server_command));
	server_command.sin_family = AF_INET;
	server_command.sin_addr.s_addr = htonl(INADDR_ANY);
	server_command.sin_port = htons(PORT_COMMAND);
	//server.sin_port = htons(PORT_DEBUG);

	//Bind
	if (bind(s_command, (struct sockaddr *)&server_command, sizeof(server_command)) == SOCKET_ERROR)
	{
		printf("Streaming - server_command Bind failed with error code : %d. try another port...\n", MINISOCK_LAST_ERROR);
		server_command.sin_port = htons(PORT_COMMAND2);
		if (bind(s_command, (struct sockaddr *)&server_command, sizeof(server_command)) == SOCKET_ERROR)
		{
			printf("Streaming - Bind failed again with error code : %d exit...\n", MINISOCK_LAST_ERROR);
			exit(EXIT_FAILURE);
		}
	}
	// If iMode!=0, non-blocking mode is enabled.
	u_long iMode_command = 1;
#ifdef WIN32
	ioctlsocket(s_command, FIONBIO, &iMode_command);
#else
	ss = fcntl(s_command, F_GETFL, 0);
	fcntl(s_command, F_SETFL, ss | O_NONBLOCK);
#endif
	// ------------- End of Position Stream UDP Socket Init ------------------
}

void SocketWrapper::closeSocket()
{
#ifdef WIN32
	closesocket(s_camera);
	closesocket(s_point_cloud);
	closesocket(s_command);
	WSACleanup();
#else
	close(s_camera);
	close(s_point_cloud);
	close(s_command);
#endif
}

void SocketWrapper::isStartStreamingCamera()
{
	memset(recvBuf, '\0', MAX_RECV_LEN);
	if ((recv_len_camera = recvfrom(s_camera, recvBuf, MAX_RECV_LEN, 0, (struct sockaddr *) &si_other_camera, (socklen_t*)&slen_camera)) != SOCKET_ERROR)
	{
		// Respond to Engine first
		sendto(s_camera, (char*)recvBuf, 1, 0, (sockaddr *)&si_other_camera, sizeof(struct sockaddr));

		if (recvBuf[0] == '1')
		{
			bool is_found = false;
			for (std::vector<struct sockaddr_in>::iterator it = si_other_cameras.begin(); it != si_other_cameras.end(); it++)
			{
				if (std::string(inet_ntoa(it->sin_addr)) == std::string(inet_ntoa(si_other_camera.sin_addr)) && it->sin_port == si_other_camera.sin_port)
				{
					is_found = true;
					break;
				}
			}

			if (!is_found)
			{
				struct sockaddr_in new_si;
				memcpy(&new_si, &si_other_camera, sizeof(struct sockaddr_in));
				si_other_cameras.push_back(new_si);
				printf("SocketWrapper::isStartStreamingCamera(): start streaming to %s:%d\n", inet_ntoa(new_si.sin_addr), new_si.sin_port);
			}
		}
		else
		{
			for (std::vector<struct sockaddr_in>::iterator it = si_other_cameras.begin(); it != si_other_cameras.end(); it++)
			{
				if (std::string(inet_ntoa(it->sin_addr)) == std::string(inet_ntoa(si_other_camera.sin_addr)) && it->sin_port == si_other_camera.sin_port)
				{
					printf("SocketWrapper::isStartStreamingCamera(): stop streaming to %s:%d\n", inet_ntoa(it->sin_addr), it->sin_port);
					si_other_cameras.erase(it);
					break;
				}
			}
		}
	}
}

//void SocketWrapper::sendCamera(int frame_id, double time_stamp, Eigen::Vector3f p, Eigen::Quaternion<float> q)
void SocketWrapper::getTrackData(int frame_id, double time_stamp, float exposure, Eigen::Vector3f p, Eigen::Quaternion<float> q, char status, int numDropFrame)
{
	track_data.frame_id = frame_id;
	track_data.time_stamp = time_stamp;
	track_data.exposure = exposure;
	track_data.x = p.x();
	track_data.y = p.y();
	track_data.z = p.z();
	track_data.q0 = q.x();
	track_data.q1 = q.y();
	track_data.q2 = q.z();
	track_data.q3 = q.w();
	track_data.status = status;
	track_data.numDropFrames = numDropFrame;
	if (skipFrame)
		track_data.status = 's';
}

void SocketWrapper::sendCamera()
{
	int track_length = sizeof(TrackData);

	// UDP Streaming - Positional Data
	for (std::vector<struct sockaddr_in>::iterator it = si_other_cameras.begin(); it != si_other_cameras.end(); it++)
	{
		send_len_camera = sendto(s_camera, (char*)&track_data, track_length, 0, (sockaddr *)&(*it), slen_camera);
		if (send_len_camera < 0) {
			std::cout << "SocketWrapper::sendCamera(): Error broadcasting to the clients" << std::endl;
			return;
		}
		else if (send_len_camera < track_length) {
			std::cout << "SocketWrapper::sendCamera(): Not all data broadcasted to the clients" << std::endl;
			return;
		}
		else {
			//std::cout << "SocketWrapper::sendCamera(): Broadcasting is done.  frame_id:" << frame_id << std::endl;
		}
	}
}


void SocketWrapper::isStartStreamingPointCloud()
{
	memset(recvBuf, '\0', MAX_RECV_LEN);
	if ((recv_len_point_cloud = recvfrom(s_point_cloud, recvBuf, MAX_RECV_LEN, 0, (sockaddr *)&si_other_point_cloud, (socklen_t*)&slen_point_cloud)) != SOCKET_ERROR)
	{
		// Respond to Engine first
		sendto(s_point_cloud, (char*)recvBuf, 1, 0, (sockaddr *)&si_other_point_cloud, sizeof(sockaddr));

		if (recvBuf[0] == 'p')
		{
			bool is_found = false;
			for (std::vector<pc_struct>::iterator it = si_other_point_clouds.begin(); it != si_other_point_clouds.end(); it++)
			{
				if (std::string(inet_ntoa(it->si.sin_addr)) == std::string(inet_ntoa(si_other_point_cloud.sin_addr)) && it->si.sin_port == si_other_point_cloud.sin_port)
				{
					is_found = true;
					break;
				}
			}

			if(!is_found)
			{
				pc_struct new_pc;
				memcpy(&new_pc.si, &si_other_point_cloud, sizeof(struct sockaddr_in));
				new_pc.index = 0;
				si_other_point_clouds.push_back(new_pc);
				printf("SocketWrapper::isStartStreamingPointCloud(): start streaming point cloud to %s:%d\n", inet_ntoa(new_pc.si.sin_addr), new_pc.si.sin_port);
			}
		}
		else
		{
			for (std::vector<pc_struct>::iterator it = si_other_point_clouds.begin(); it != si_other_point_clouds.end(); it++)
			{
				if (std::string(inet_ntoa(it->si.sin_addr)) == std::string(inet_ntoa(si_other_point_cloud.sin_addr)) && it->si.sin_port == si_other_point_cloud.sin_port)
				{
					printf("SocketWrapper::isStartStreamingPointCloud(): stop streaming point cloud to %s:%d\n", inet_ntoa(it->si.sin_addr), it->si.sin_port);
					si_other_point_clouds.erase(it);
					break;
				}
			}
		}
	}
}

void SocketWrapper::sendPC()
{
	if (si_other_point_clouds.size() == 0) return;
	//printf("SocketWrapper::sendPC() - keyframe_calib_pcs.size(): %d\n", keyframe_calib_pcs.size());
	for (std::vector<pc_struct>::iterator it = si_other_point_clouds.begin(); it != si_other_point_clouds.end(); it++)
	{
		int num_untransfered_pcs = /*keyframe_pcs*/keyframe_calib_pcs.size() - it->index;
		int pc_length = num_untransfered_pcs > NUM_POINT_CLOUDS_PER_FRAME ? NUM_POINT_CLOUDS_PER_FRAME : num_untransfered_pcs;
		if (pc_length == 0) continue;
		send_len_point_cloud = sendto(s_point_cloud, (char*)&pc_length, sizeof(int), 0, (sockaddr *)&it->si, sizeof(sockaddr));
		if (send_len_point_cloud < 0) {
			printf("SocketWrapper::sendPC(): Send point cloud size buffer failed with error code : %d\n", MINISOCK_LAST_ERROR);
			return;
		}
		else if (send_len_point_cloud < sizeof(pc_length)) {
			std::cout << "SocketWrapper::sendPC(): Not all point cloud size buffer broadcasted to the clients" << std::endl;
			return;
		}
		else {
			boost::this_thread::sleep_for(boost::chrono::microseconds(100));
			//	std::cout << "SocketWrapper::sendPC(): Broadcasting point cloud size buffer is done" << std::endl;
		}
		//printf("SocketWrapper::sendPC() - pc_length:%d keyframe_calib_pcs.size(): %d\n", pc_length, keyframe_calib_pcs.size());
		send_len_point_cloud = sendto(s_point_cloud, (char*)/*keyframe_pcs*/keyframe_calib_pcs.data() + it->index * 3 * sizeof(float), pc_length * 3 * sizeof(float), 0, (sockaddr *)&it->si, sizeof(sockaddr));
		it->index += pc_length;
		if (send_len_point_cloud < 0) {
			printf("SocketWrapper::sendPC(): Send point cloud data failed with error code : %d\n", MINISOCK_LAST_ERROR);
			return;
		}
		else if (send_len_point_cloud < pc_length * 3 * sizeof(float)) {
			std::cout << "SocketWrapper::sendPC(): Not all point cloud data broadcasted to the clients" << std::endl;
			return;
		}
		else {
			boost::this_thread::sleep_for(boost::chrono::microseconds(100));
			//	std::cout << "SocketWrapper::sendPC(): Broadcasting point cloud data is done" << std::endl;
		}
	}
}

void SocketWrapper::checkCommand()
{
	memset(recvBuf, '\0', MAX_RECV_LEN);
	if ((recv_len_command = recvfrom(s_command, recvBuf, MAX_RECV_LEN, 0, (sockaddr *)&si_other_command, (socklen_t*)&slen_command)) != SOCKET_ERROR)
	{
		if (recvBuf[0] == 'k') {
			setNeedGroundCalibration(true);
		}
		else if (recvBuf[0] == 'l') {
			setFullSystemReset(true);
		}

		// Respond to Engine
		sendto(s_command, (char*)recvBuf, MAX_RECV_LEN, 0, (sockaddr *)&si_other_command, sizeof(sockaddr));
	}
}

void SocketWrapper::run(int state)
{
	setTrackingState(state);
	isStartStreamingCamera();
	isStartStreamingPointCloud();
	sendPC();
	checkCommand();
}

void SocketWrapper::setFullSystemReset(bool state)
{
	setting_fullResetRequested[channel] = state;
	printf("SocketWrapper::setFullSystemReset(): reset command\n");
}

void SocketWrapper::setNeedGroundCalibration(bool state)
{
	setting_needGroundCalibration[channel] = state;
	printf("SocketWrapper::setNeedGroundCalibration(): targetcalibration started\n");
}

void SocketWrapper::setSkipFrame(bool isSkipFrame)
{
	skipFrame = isSkipFrame;
}

bool SocketWrapper::getSkipFrame()
{
	return skipFrame;
}

void SocketWrapper::setNumSkipFrame(int numFrame)
{
	numSkipFrame = numFrame;
}

int SocketWrapper::getNumSkipFrame()
{
	return numSkipFrame;
}

}

}
