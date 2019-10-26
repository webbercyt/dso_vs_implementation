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



#include "PangolinDSOViewer.h"
#include "KeyFrameDisplay.h"

#include "util/settings.h"
#include "util/globalCalib.h"
#include "FullSystem/HessianBlocks.h"
#include "FullSystem/FullSystem.h"
#include "FullSystem/ImmaturePoint.h"
//#include "cuda/dso.h"

namespace dso
{
	namespace IOWrap
	{
		PangolinDSOViewer::PangolinDSOViewer(int channel, int w, int h, bool startRunThread) :
			channel(channel)
		{
			this->w = w;
			this->h = h;
			running=true;
			settings_showFollowCam = false;

			{
				boost::unique_lock<boost::mutex> lk(openImagesMutex);
				internalVideoImg = new MinimalImageB3(w,h);
				internalKFImg = new MinimalImageB3(w,h);
				internalResImg = new MinimalImageB3(w,h);
				videoImgChanged=kfImgChanged=resImgChanged=true;

				internalVideoImg->setBlack();
				internalKFImg->setBlack();
				internalResImg->setBlack();
			}


			{
				currentCam = new KeyFrameDisplay(channel);
			}

			needReset = false;

			if(startRunThread)
				runThread = boost::thread(&PangolinDSOViewer::run, this);
		}


		PangolinDSOViewer::~PangolinDSOViewer()
		{
			if (internalVideoImg) delete internalVideoImg;
			if (internalKFImg) delete internalKFImg;
			if (internalResImg) delete internalResImg;

			if (currentCam) delete currentCam;

			close();
			runThread.join();
		}


		void PangolinDSOViewer::run()
		{
			printf("START PANGOLIN!\n");

			const int UI_WIDTH = 180;
			pangolin::CreateWindowAndBind("Main", 2 * w + UI_WIDTH, 2 * h);

			glEnable(GL_DEPTH_TEST);

			// 3D visualization
			Visualization3D_camera = pangolin::OpenGlRenderState(
				pangolin::ProjectionMatrix(w, h, 400, 400, w / 2, h / 2, 0.1, 1000),
				pangolin::ModelViewLookAt(-0,-5,-10, 0,0,0, pangolin::AxisNegY)
				);

			pangolin::View& Visualization3D_display = pangolin::CreateDisplay()
				.SetBounds(0.0, 1.0, pangolin::Attach::Pix(UI_WIDTH), 1.0, -w/(float)h)
				.SetHandler(new pangolin::Handler3D(Visualization3D_camera));

			// 3 images
			pangolin::View& d_kfDepth = pangolin::Display("imgKFDepth")
				.SetAspect(w/(float)h);

			pangolin::View& d_video = pangolin::Display("imgVideo")
				.SetAspect(w/(float)h);

			pangolin::View& d_residual = pangolin::Display("imgResidual")
				.SetAspect(w/(float)h);

			pangolin::GlTexture texKFDepth(w,h,GL_RGB,false,0,GL_RGB,GL_UNSIGNED_BYTE);
			pangolin::GlTexture texVideo(w,h,GL_RGB,false,0,GL_RGB,GL_UNSIGNED_BYTE);
			pangolin::GlTexture texResidual(w,h,GL_RGB,false,0,GL_RGB,GL_UNSIGNED_BYTE);


			pangolin::CreateDisplay()
				  .SetBounds(0.0, 0.3, pangolin::Attach::Pix(UI_WIDTH), 1.0)
				  .SetLayout(pangolin::LayoutEqual)
				  .AddDisplay(d_kfDepth)
				  .AddDisplay(d_video)
				  .AddDisplay(d_residual);

			// parameter reconfigure gui
			pangolin::CreatePanel("ui").SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(UI_WIDTH));

			pangolin::Var<int> settings_pointCloudMode("ui.PC_mode",1,0,3,false);
			pangolin::Var<bool> settings_pointCloudAlign("ui.AlignPC", false, true);
			//cinetec mod:Yang save pc
			pangolin::Var<bool> settings_pointCloudSave("ui.savePC", false, false);
			pangolin::Var<int> settings_pointCloudFrames("ui.showPCFrames", 10, 0, 3, false);
			//////////
			pangolin::Var<bool> settings_showFollowCam("ui.FollowCam", false, true);
			pangolin::Var<bool> settings_showCurrentCamera("ui.CurrCam",true,true);

			pangolin::Var<bool> settings_show3D("ui.show3D", true, true);
			pangolin::Var<bool> settings_showLiveDepth("ui.showDepth", true, true);
			pangolin::Var<bool> settings_showLiveVideo("ui.showVideo", true, true);
			#ifdef ADV_PARAM
			pangolin::Var<bool> settings_showKFCameras("ui.KFCam",false,true);
			pangolin::Var<bool> settings_showTrajectory("ui.Trajectory",true,true);
			pangolin::Var<bool> settings_showFullTrajectory("ui.FullTrajectory",false,true);
			pangolin::Var<bool> settings_showActiveConstraints("ui.ActiveConst",true,true);
			pangolin::Var<bool> settings_showAllConstraints("ui.AllConst",false,true);
	
			pangolin::Var<bool> settings_showLiveResidual("ui.showResidual",false,true);

			pangolin::Var<bool> settings_showFramesWindow("ui.showFramesWindow",false,true);
			pangolin::Var<bool> settings_showFullTracking("ui.showFullTracking",false,true);
			pangolin::Var<bool> settings_showCoarseTracking("ui.showCoarseTracking",false,true);

			pangolin::Var<int> settings_sparsity("ui.sparsity",1,1,20,false);
			pangolin::Var<double> settings_scaledVarTH("ui.relVarTH",0.001,1e-10,1e10, true);
			pangolin::Var<double> settings_absVarTH("ui.absVarTH",0.001,1e-10,1e10, true);
			pangolin::Var<double> settings_minRelBS("ui.minRelativeBS",0.1,0,1, false);

			pangolin::Var<int> settings_nMaxFrames("ui.maxFrames",setting_maxFrames, 4,10, false);
			pangolin::Var<double> settings_kfFrequency("ui.kfFrequency",setting_kfGlobalWeight,0.1,3, false);
			pangolin::Var<double> settings_gradHistAdd("ui.minGradAdd",setting_minGradHistAdd,0,15, false);
			#else
			bool settings_showKFCameras = false;
			bool settings_showTrajectory = true;
			bool settings_showFullTrajectory = false;
			bool settings_showActiveConstraints = true;
			bool settings_showAllConstraints = false;
			bool settings_showLiveResidual = false;
			bool settings_showFramesWindow = false;
			bool settings_showFullTracking = false;
			bool settings_showCoarseTracking = false;
			int settings_sparsity = 1;
			double settings_scaledVarTH = 0.001;
			double settings_absVarTH = 0.001;
			double settings_minRelBS = 0.1;
			int settings_nMaxFrames = 7;
			double settings_kfFrequency = setting_kfGlobalWeight;
			double settings_gradHistAdd = setting_minGradHistAdd;
			#endif
			pangolin::Var<int> settings_nPts("ui.activePoints", setting_desiredPointDensity, 50, 5000, false);
			pangolin::Var<int> settings_nCandidates("ui.pointCandidates", setting_desiredImmatureDensity, 50, 5000, false);

			pangolin::Var<bool> settings_groundPoseCalib("ui.GoundPoseCalib", false, false);
			pangolin::Var<bool> settings_resetButton("ui.Reset", false, false);
			pangolin::Var<int> settings_frameID("ui.FrameID", 0, 0, 0, false);
			pangolin::Var<double> settings_trackFps("ui.Track fps", 0, 0, 0, false);
			pangolin::Var<double> settings_mapFps("ui.KF fps", 0, 0, 0, false);

			// callbacks
			// exit
			pangolin::RegisterKeyPressCallback(pangolin::PANGO_CTRL + 'c', [this]() {exit(0); });

			// focus
			pangolin::RegisterKeyPressCallback('f', [this]() {
				focusCamera();
			});

			// move x
			delta_x = 0.0f;
			pangolin::RegisterKeyPressCallback('a', [this]() {
				if (this->settings_pointCloudAlign) delta_x -= 0.01f;
			});
			pangolin::RegisterKeyPressCallback('d', [this]() {
				if (this->settings_pointCloudAlign) delta_x += 0.01f;
			});

			// move y
			delta_y = 0.0f;
			pangolin::RegisterKeyPressCallback('w', [this]() {
				if (this->settings_pointCloudAlign) delta_y -= 0.01f;
			});
			pangolin::RegisterKeyPressCallback('s', [this]() {
				if (this->settings_pointCloudAlign) delta_y += 0.01f;
			});

			// move z
			delta_z = 0.0f;
			pangolin::RegisterKeyPressCallback('z', [this]() {
				if (this->settings_pointCloudAlign) delta_z += 0.01f;
			});
			pangolin::RegisterKeyPressCallback('x', [this]() {
				if (this->settings_pointCloudAlign) delta_z -= 0.01f;
			});

			// scale
			delta_scale = 0.0f;
			pangolin::RegisterKeyPressCallback('q', [this]() {
				if (this->settings_pointCloudAlign) { delta_scale += 0.01f;  if (delta_scale > 0.5f) delta_scale = 0.5f; }
				printf("delta_scale: %f\n", delta_scale);
			});
			pangolin::RegisterKeyPressCallback('e', [this]() {
				if (this->settings_pointCloudAlign) { delta_scale -= 0.01f;  if (delta_scale < -0.5f) delta_scale = -0.5f; }
				printf("delta_scale: %f\n", delta_scale);
			});

			// pc calibrate
			pangolin::RegisterKeyPressCallback('c', [this]() {
				this->settings_groundPoseCalib = true;
			});

			// Default hooks for exiting (Esc) and fullscreen (tab).
			while( /*!pangolin::ShouldQuit() &&*/ running )
			{
				// Clear entire screen
				glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

				if (setting_render_display3D)
				{

					// Activate efficiently by object
					Visualization3D_display.Activate(Visualization3D_camera);

					boost::unique_lock<boost::mutex> lk3d(model3DMutex);
					//pangolin::glDrawColouredCube();
					int refreshed = 0;

					if (this->settings_pointCloudAlign) calcDelta();

					int size_frames = keyframes.size();
					int start_i = std::max(size_frames - MAX_NUM_SHOW_FRAMES, 0);
					for (int i = start_i; i < size_frames; i++)
					{
						KeyFrameDisplay* fh = keyframes[i];

						float blue[3] = { 0,0,1 };

						if (this->settings_showKFCameras) fh->drawCam(1, blue, 0.1, this->settings_pointCloudAlign);

						if (this->settings_pointCloudAlign) fh->setNeedRefreshPC(true);
						refreshed = +(int)(fh->refreshPC(refreshed < 10, this->settings_scaledVarTH, this->settings_absVarTH,
							this->settings_pointCloudMode, this->settings_minRelBS, this->settings_sparsity,
							this->settings_pointCloudAlign && i > size_frames / 2, this->settings_pointCloudAlign));
						fh->drawPC(1, this->settings_pointCloudAlign);
					}

					renderCoordinate();

					if (this->settings_showFollowCam) focusCamera();

					//cinetec mod:Yang save
					if (this->settings_pointCloudSave) {
						settings_pointCloudSave.Reset();
						std::ofstream myfile;
						myfile.open("result_pc.txt");
						myfile << std::setprecision(8);
						for (KeyFrameDisplay* fh : keyframes)
						{
							fh->savePC(myfile, this->settings_pointCloudAlign);
						}
						myfile.close();
					}
					//////////////////////////
					if (this->settings_showCurrentCamera) currentCam->drawCam(2, 0, 0.2, this->settings_pointCloudAlign);
					drawConstraints();
					lk3d.unlock();
					if (this->settings_groundPoseCalib) {
						settings_groundPoseCalib.Reset();
						setting_needGroundCalibration[channel] = true;
					}
				}

				bool no_gl_error = (glGetError() == GL_NO_ERROR);
				openImagesMutex.lock();
				if(no_gl_error && videoImgChanged) 	texVideo.Upload(internalVideoImg->data,GL_BGR,GL_UNSIGNED_BYTE);
				if(no_gl_error && kfImgChanged)		texKFDepth.Upload(internalKFImg->data,GL_BGR,GL_UNSIGNED_BYTE);
				if(no_gl_error && resImgChanged)	texResidual.Upload(internalResImg->data,GL_BGR,GL_UNSIGNED_BYTE);
				videoImgChanged=kfImgChanged=resImgChanged=false;
				openImagesMutex.unlock();



				// frame id
				{
					settings_frameID = currentCam->id;
				}

				// update fps counters
				{
					openImagesMutex.lock();
					float sd=0;
					for(float d : lastNMappingMs) sd+=d;
					settings_mapFps=lastNMappingMs.size()*1000.0f / sd;
					openImagesMutex.unlock();
				}
				{
					model3DMutex.lock();
					float sd=0;
					for(float d : lastNTrackingMs) sd+=d;
					settings_trackFps = lastNTrackingMs.size()*1000.0f / sd;
					model3DMutex.unlock();
				}


				if(setting_render_displayVideo)
				{
					d_video.Activate();
					glColor4f(1.0f,1.0f,1.0f,1.0f);
					texVideo.RenderToViewportFlipY();
				}

				if(setting_render_displayDepth)
				{
					d_kfDepth.Activate();
					glColor4f(1.0f,1.0f,1.0f,1.0f);
					texKFDepth.RenderToViewportFlipY();
				}

				if(setting_render_displayResidual)
				{
					d_residual.Activate();
					glColor4f(1.0f,1.0f,1.0f,1.0f);
					texResidual.RenderToViewportFlipY();
				}


				// update parameters
				this->settings_pointCloudMode = settings_pointCloudMode.Get();
				this->settings_pointCloudAlign = settings_pointCloudAlign.Get();
				this->settings_pointCloudSave = settings_pointCloudSave.Get();
				this->settings_groundPoseCalib = settings_groundPoseCalib.Get();

				this->settings_showFollowCam = settings_showFollowCam.Get();
		
				setting_render_display3D = settings_show3D.Get();
				setting_render_displayDepth = settings_showLiveDepth.Get();
				setting_render_displayVideo = settings_showLiveVideo.Get();

		#ifdef ADV_PARAM
				this->settings_showActiveConstraints = settings_showActiveConstraints.Get();
				this->settings_showAllConstraints = settings_showAllConstraints.Get();
				this->settings_showCurrentCamera = settings_showCurrentCamera.Get();
				this->settings_showKFCameras = settings_showKFCameras.Get();
				this->settings_showTrajectory = settings_showTrajectory.Get();
				this->settings_showFullTrajectory = settings_showFullTrajectory.Get();

				setting_render_displayResidual = settings_showLiveResidual.Get();
				setting_render_renderWindowFrames = settings_showFramesWindow.Get();
				setting_render_plotTrackingFull = settings_showFullTracking.Get();
				setting_render_displayCoarseTrackingFull = settings_showCoarseTracking.Get();

				this->settings_absVarTH = settings_absVarTH.Get();
				this->settings_scaledVarTH = settings_scaledVarTH.Get();
				this->settings_minRelBS = settings_minRelBS.Get();
				this->settings_sparsity = settings_sparsity.Get();

				setting_maxFrames = settings_nMaxFrames.Get();
				setting_kfGlobalWeight = settings_kfFrequency.Get();
				setting_minGradHistAdd = settings_gradHistAdd.Get();
		#else
				this->settings_showActiveConstraints = settings_showActiveConstraints;
				this->settings_showAllConstraints = settings_showAllConstraints;
				this->settings_showCurrentCamera = settings_showCurrentCamera;
				this->settings_showKFCameras = settings_showKFCameras;
				this->settings_showTrajectory = settings_showTrajectory;
				this->settings_showFullTrajectory = settings_showFullTrajectory;

				setting_render_displayResidual = settings_showLiveResidual;
				setting_render_renderWindowFrames = settings_showFramesWindow;
				setting_render_plotTrackingFull = settings_showFullTracking;
				setting_render_displayCoarseTrackingFull = settings_showCoarseTracking;

				this->settings_absVarTH = settings_absVarTH;
				this->settings_scaledVarTH = settings_scaledVarTH;
				this->settings_minRelBS = settings_minRelBS;
				this->settings_sparsity = settings_sparsity;

				setting_maxFrames = settings_nMaxFrames;
				setting_kfGlobalWeight = settings_kfFrequency;
				setting_minGradHistAdd = settings_gradHistAdd;

		#endif
				setting_desiredPointDensity = settings_nPts.Get();
				setting_desiredImmatureDensity = settings_nCandidates.Get();

				if(settings_resetButton.Get())
				{
	    			printf("RESET!\n");
	    			settings_resetButton.Reset();
	    			setting_fullResetRequested[channel] = true;
				}

				// Swap frames and Process Events
				pangolin::FinishFrame();


				if(needReset) reset_internal();
			}


			printf("QUIT Pangolin thread!\n");
			printf("I'll just kill the whole process.\nSo Long, and Thanks for All the Fish!\n");

			exit(0);
		}


		void PangolinDSOViewer::close()
		{
			running = false;
		}

		void PangolinDSOViewer::join()
		{
			runThread.join();
			printf("JOINED Pangolin thread!\n");
		}

		void PangolinDSOViewer::reset()
		{
			needReset = true;
		}

		void PangolinDSOViewer::reset_internal()
		{
			model3DMutex.lock();
			for(size_t i=0; i<keyframes.size();i++) delete keyframes[i];
			keyframes.clear();
			for(size_t i=0; i<allFrames.size();i++) delete allFrames[i];
			allFrames.clear();
			//allFramePoses.clear();
			keyframesByKFID.clear();
			connections.clear();
			model3DMutex.unlock();


			openImagesMutex.lock();
			internalVideoImg->setBlack();
			internalKFImg->setBlack();
			internalResImg->setBlack();
			videoImgChanged= kfImgChanged= resImgChanged=true;
			openImagesMutex.unlock();

			needReset = false;
		}

		void PangolinDSOViewer::drawConstraints()
		{
			if(settings_showAllConstraints)
			{
				// draw constraints
				glLineWidth(1);
				glBegin(GL_LINES);

				glColor3f(0,1,0);
				glBegin(GL_LINES);
				for(unsigned int i=0;i<connections.size();i++)
				{
					if(connections[i].to == 0 || connections[i].from==0) continue;
					int nAct = connections[i].bwdAct + connections[i].fwdAct;
					int nMarg = connections[i].bwdMarg + connections[i].fwdMarg;
					if(nAct==0 && nMarg>0  )
					{
						Sophus::Vector3f t = connections[i].from->getPosition(settings_pointCloudAlign);
						glVertex3f((GLfloat) t[0],(GLfloat) t[1], (GLfloat) t[2]);
						t = connections[i].to->getPosition(settings_pointCloudAlign);
						glVertex3f((GLfloat) t[0],(GLfloat) t[1], (GLfloat) t[2]);
					}
				}
				glEnd();
			}

			if(settings_showActiveConstraints)
			{
				glLineWidth(3);
				glColor3f(0,0,1);
				glBegin(GL_LINES);
				for(unsigned int i=0;i<connections.size();i++)
				{
					if(connections[i].to == 0 || connections[i].from==0) continue;
					int nAct = connections[i].bwdAct + connections[i].fwdAct;

					if(nAct>0)
					{
						Sophus::Vector3f t = connections[i].from->getPosition(settings_pointCloudAlign);
						glVertex3f((GLfloat) t[0],(GLfloat) t[1], (GLfloat) t[2]);
						t = connections[i].to->getPosition(settings_pointCloudAlign);
						glVertex3f((GLfloat) t[0],(GLfloat) t[1], (GLfloat) t[2]);
					}
				}
				glEnd();
			}

			if(settings_showTrajectory)
			{
				float colorRed[3] = {1,0,0};
				glColor3f(colorRed[0],colorRed[1],colorRed[2]);
				glLineWidth(3);

				glBegin(GL_LINE_STRIP);
				for(unsigned int i=0;i<keyframes.size();i++)
				{
					Sophus::Vector3f t = keyframes[i]->getPosition(settings_pointCloudAlign);
					glVertex3f((GLfloat)t[0], (GLfloat)t[1], (GLfloat)t[2]);
				}
				glEnd();
			}

			if(settings_showFullTrajectory)
			{
				float colorGreen[3] = {0,1,0};
				glColor3f(colorGreen[0],colorGreen[1],colorGreen[2]);
				glLineWidth(3);

				glBegin(GL_LINE_STRIP);
				for(unsigned int i=0;i<allFrames.size();i++)
				{
					Sophus::Vector3f t = allFrames[i]->getPosition(settings_pointCloudAlign);
					glVertex3f((GLfloat)t[0], (GLfloat)t[1], (GLfloat)t[2]);
				}
				glEnd();
			}
		}






		void PangolinDSOViewer::publishGraph(const std::map<uint64_t,Eigen::Vector2i> &connectivity)
		{
			if(!setting_render_display3D) return;
			if(disableAllDisplay) return;

			model3DMutex.lock();
			connections.resize(connectivity.size());
			int runningID=0;
			int totalActFwd=0, totalActBwd=0, totalMargFwd=0, totalMargBwd=0;
			for(std::pair<uint64_t,Eigen::Vector2i> p : connectivity)
			{
				int host = (int)(p.first >> 32);
				int target = (int)(p.first & (uint64_t)0xFFFFFFFF);

				assert(host >= 0 && target >= 0);
				if(host == target)
				{
					assert(p.second[0] == 0 && p.second[1] == 0);
					continue;
				}

				if(host > target) continue;

				connections[runningID].from = keyframesByKFID.count(host) == 0 ? 0 : keyframesByKFID[host];
				connections[runningID].to = keyframesByKFID.count(target) == 0 ? 0 : keyframesByKFID[target];
				connections[runningID].fwdAct = p.second[0];
				connections[runningID].fwdMarg = p.second[1];
				totalActFwd += p.second[0];
				totalMargFwd += p.second[1];

				uint64_t inverseKey = (((uint64_t)target) << 32) + ((uint64_t)host);
				Eigen::Vector2i st = connectivity.at(inverseKey);
				connections[runningID].bwdAct = st[0];
				connections[runningID].bwdMarg = st[1];

				totalActBwd += st[0];
				totalMargBwd += st[1];

				runningID++;
			}


			model3DMutex.unlock();
		}
		void PangolinDSOViewer::publishKeyframes(
				std::vector<FrameHessian*> &frames,
				bool final,
				CalibHessian* HCalib)
		{
			if(!setting_render_display3D) return;
			if(disableAllDisplay) return;

			boost::unique_lock<boost::mutex> lk(model3DMutex);

			for(FrameHessian* fh : frames)
			{
				if(keyframesByKFID.find(fh->frameID) == keyframesByKFID.end())
				{
					KeyFrameDisplay* kfd = new KeyFrameDisplay(channel);
					keyframesByKFID[fh->frameID] = kfd;
					keyframes.push_back(kfd);
				}
				keyframesByKFID[fh->frameID]->setFromKF(fh, HCalib);
				keyframesByKFID[fh->frameID]->makeKeyframePC();
			}
		}
		//void PangolinDSOViewer::publishCamPose(FrameShell* frame, CalibHessian* HCalib)
		void PangolinDSOViewer::publishCamPose(FrameShell* frame, CalibHessian* HCalib, char status, int numDropFrame, int camFrameID)
		{
			if(!setting_render_display3D) return;
			if(disableAllDisplay) return;

			boost::unique_lock<boost::mutex> lk(model3DMutex);
			struct timeval time_now;
			gettimeofday(&time_now, NULL);
			lastNTrackingMs.push_back(((time_now.tv_sec-last_track.tv_sec)*1000.0f + (time_now.tv_usec-last_track.tv_usec)/1000.0f));
			if(lastNTrackingMs.size() > 10) lastNTrackingMs.pop_front();
			last_track = time_now;

			if(!setting_render_display3D) return;

			currentCam->setFromF(frame, HCalib);
			//allFramePoses.push_back(frame->camToWorld.translation().cast<float>());
			KeyFrameDisplay* kfd = new KeyFrameDisplay(channel);
			kfd->setFromF(frame, HCalib);
			allFrames.push_back(kfd);
		}

		void PangolinDSOViewer::image_Reductor(const FrameHessian* const fh, MinimalImageB3 *img, int min, int max, Vec10* stats, int tid)
		{
			boost::unique_lock<boost::mutex> lk(openImagesMutex);
			const Eigen::Vector3f* const dI = fh->dI;
			for (int idx = min; idx < max; idx++)
			{
				int c = dI[idx][0] * 0.8f;
				if (c > 255) c = 255;
				img->at(idx) = Vec3b(c, c, c);
			}
		}

		void PangolinDSOViewer::pushLiveFrame(FrameHessian* image)
		{
			if(!setting_render_displayVideo) return;
			if(disableAllDisplay) return;


			if (multiThreading)
			{
				imageReduce.reduce(boost::bind(&PangolinDSOViewer::image_Reductor, this, image, internalVideoImg,
					_1, _2, _3, _4), 0, w*h, 0);
			}
			else
			{
				image_Reductor(image, internalVideoImg, 0, w*h, 0, 0);
				/*for (int i = 0; i<w*h; i++)
					internalVideoImg->data[i][0] =
					internalVideoImg->data[i][1] =
					internalVideoImg->data[i][2] =
					dI[i][0] * 0.8 > 255.0f ? 255.0 : dI[i][0] * 0.8;*/
			}

			videoImgChanged=true;
		}

		bool PangolinDSOViewer::needPushDepthImage()
		{
			return setting_render_displayDepth;
		}
		void PangolinDSOViewer::pushDepthImage(MinimalImageB3* image)
		{

			if(!setting_render_displayDepth) return;
			if(disableAllDisplay) return;

			boost::unique_lock<boost::mutex> lk(openImagesMutex);

			struct timeval time_now;
			gettimeofday(&time_now, NULL);
			lastNMappingMs.push_back(((time_now.tv_sec-last_map.tv_sec)*1000.0f + (time_now.tv_usec-last_map.tv_usec)/1000.0f));
			if(lastNMappingMs.size() > 10) lastNMappingMs.pop_front();
			last_map = time_now;

			memcpy(internalKFImg->data, image->data, w*h*3);
			kfImgChanged=true;
		}

		void PangolinDSOViewer::lockModel3DMutex()
		{
			model3DMutex.lock();
		}

		void PangolinDSOViewer::publishGroundCalibParams()
		{
			int size_frames = keyframes.size();
			int start_i = std::max(size_frames - MAX_NUM_SHOW_FRAMES, 0);
			for (int i = start_i; i < size_frames; i++)
				keyframes[i]->setNeedRefreshPC(true);

			model3DMutex.unlock();
		}

		void PangolinDSOViewer::calcDelta()
		{
			// key frames
			{
				float total_delta_x = 0.0f;
				float total_delta_y = 0.0f;
				float total_delta_z = 0.0f;
				for (int i = 1; i < keyframes.size(); i++)
				{
					Sophus::Matrix4f delta_m = keyframes[i]->camToWorld.matrix().cast<float>() * keyframes[i - 1]->camToWorld.matrix().inverse().cast<float>();
					total_delta_x += fabs(delta_m.col(3)[0]);
					total_delta_y += fabs(delta_m.col(3)[1]);
					total_delta_z += fabs(delta_m.col(3)[2]);
				}

				float current_delta_x = 0.0f;
				float current_delta_y = 0.0f;
				float current_delta_z = 0.0f;
				for (int i = 0; i < keyframes.size(); i++)
				{
					KeyFrameDisplay* fh = keyframes[i];

					if (i > 0)
					{
						Sophus::Matrix4f delta_m = keyframes[i]->camToWorld.matrix().cast<float>() * keyframes[i - 1]->camToWorld.matrix().inverse().cast<float>();
						current_delta_x += fabs(delta_m.col(3)[0]);
						current_delta_y += fabs(delta_m.col(3)[1]);
						current_delta_z += fabs(delta_m.col(3)[2]);
					}

					float delta_x_i = delta_x * current_delta_x / total_delta_x;
					float delta_y_i = delta_y * current_delta_y / total_delta_y;
					float delta_z_i = delta_z * current_delta_z / total_delta_z;

					float delta_scale_x_i = 1.0f + delta_scale * current_delta_x / total_delta_x;
					float delta_scale_y_i = 1.0f + delta_scale * current_delta_y / total_delta_y;
					float delta_scale_z_i = 1.0f + delta_scale * current_delta_z / total_delta_z;

					fh->setDelta(delta_x_i, delta_y_i, delta_z_i, delta_scale_x_i, delta_scale_y_i, delta_scale_z_i);
				}
			}

			// all frames
			{
				float total_delta_x = 0.0f;
				float total_delta_y = 0.0f;
				float total_delta_z = 0.0f;
				for (int i = 1; i < allFrames.size(); i++)
				{
					Sophus::Matrix4f delta_m = allFrames[i]->camToWorld.matrix().cast<float>() * allFrames[i - 1]->camToWorld.matrix().inverse().cast<float>();
					total_delta_x += fabs(delta_m.col(3)[0]);
					total_delta_y += fabs(delta_m.col(3)[1]);
					total_delta_z += fabs(delta_m.col(3)[2]);
				}

				float current_delta_x = 0.0f;
				float current_delta_y = 0.0f;
				float current_delta_z = 0.0f;
				for (int i = 0; i < allFrames.size(); i++)
				{
					KeyFrameDisplay* fh = allFrames[i];

					if (i > 0)
					{
						Sophus::Matrix4f delta_m = allFrames[i]->camToWorld.matrix().cast<float>() * allFrames[i - 1]->camToWorld.matrix().inverse().cast<float>();
						current_delta_x += fabs(delta_m.col(3)[0]);
						current_delta_y += fabs(delta_m.col(3)[1]);
						current_delta_z += fabs(delta_m.col(3)[2]);
					}

					float delta_x_i = delta_x * current_delta_x / total_delta_x;
					float delta_y_i = delta_y * current_delta_y / total_delta_y;
					float delta_z_i = delta_z * current_delta_z / total_delta_z;

					float delta_scale_x_i = 1.0f + delta_scale * current_delta_x / total_delta_x;
					float delta_scale_y_i = 1.0f + delta_scale * current_delta_y / total_delta_y;
					float delta_scale_z_i = 1.0f + delta_scale * current_delta_z / total_delta_z;

					fh->setDelta(delta_x_i, delta_y_i, delta_z_i, delta_scale_x_i, delta_scale_y_i, delta_scale_z_i);
					if (i == allFrames.size() - 1) currentCam->setDelta(delta_x_i, delta_y_i, delta_z_i, delta_scale_x_i, delta_scale_y_i, delta_scale_z_i);
				}
			}
		}

		void PangolinDSOViewer::focusCamera()
		{
			Sophus::Vector3f p = currentCam->getPosition(settings_pointCloudAlign);
			Sophus::Matrix3f q = currentCam->getRotationMatrix(settings_pointCloudAlign);
			Visualization3D_camera.SetProjectionMatrix(pangolin::ProjectionMatrix(2 * w, 2 * h, h, h, w, h, 0.001, 1000));
			Visualization3D_camera.SetModelViewMatrix(pangolin::ModelViewLookAt(p[0], p[1], p[2], p[0] + q.col(2)[0], p[1] + q.col(2)[1], p[2] + q.col(2)[2], -q.col(1)[0], -q.col(1)[1], -q.col(1)[2]));
		}

		void PangolinDSOViewer::renderCoordinate()
		{
			float board_w, board_h;
			if (setting_GCBoardType == 0)
			{
				board_w = setting_GCMarkerSize[0];
				board_h = setting_GCMarkerSize[1];
			}
			else if (setting_GCBoardType == 1)
			{
				board_w = setting_GCChArUco_3X3_SquareLength;
				board_h = setting_GCChArUco_3X3_SquareLength;
			}

			float maxCoordX = board_w * 3.0;
			float maxCoordZ = board_h * 3.0;

			// draw grid lines
			glLineWidth(1.0);
			glBegin(GL_LINES);
			glColor3f(0.0, 0.0, 1.0);
			for (float i = -maxCoordX; i <= maxCoordX; i = i + board_w)
			{
				glVertex3f(i, 0, maxCoordZ);
				glVertex3f(i, 0, -maxCoordZ);
			}
			for (float i = -maxCoordZ; i <= maxCoordZ; i = i + board_h)
			{
				glVertex3f(maxCoordX, 0, i);
				glVertex3f(-maxCoordX, 0, i);
			}
			glEnd();

			// draw original point
			glColor3f(1.0, 0.0, 0.0);
			glPointSize(5);
			glBegin(GL_POINTS);
			glVertex3f(0.0, 0.0, 0.0);
			glEnd();
		}
	}
}