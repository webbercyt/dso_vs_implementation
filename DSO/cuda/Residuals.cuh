#ifndef __RESIDUALS_CUH__
#define __RESIDUALS_CUH__

// .h
//#include "util/globalCalib.h"
//#include "vector"
// 
//#include "util/NumType.h"
//#include <iostream>
//#include <fstream>
//#include "util/globalFuncs.h"
//#include "OptimizationBackend/RawResidualJacobian.h"

// .cpp
//#include "FullSystem/FullSystem.h"
//
//#include "stdio.h"
//#include "util/globalFuncs.h"
//#include <Eigen/LU>
//#include <algorithm>
//#include "IOWrapper/ImageDisplay.h"
//#include "util/globalCalib.h"
//#include <Eigen/SVD>
//#include <Eigen/Eigenvalues>
//
//#include "FullSystem/ResidualProjections.h"
//#include "OptimizationBackend/EnergyFunctional.h"
//#include "OptimizationBackend/EnergyFunctionalStructs.h"
//
//#include "FullSystem/HessianBlocks.h"

#include "cuda/NumType.cuh"
#include "cuda/RawResidualJacobian.cuh"
#include "cuda/EnergyFunctionalStructs.cuh"
#include "cuda/HessianBlocks.cuh"

struct PointHessian;
struct FrameHessian;
struct CalibHessian;

struct EFResidual;


enum ResLocation {
	ResLocation_ACTIVE = 0,
	ResLocation_LINEARIZED,
	ResLocation_MARGINALIZED,
	ResLocation_NONE,
};
enum ResState {
	ResState_IN = 0,
	ResState_OOB,
	ResState_OUTLIER,
};

struct FullJacRowT
{
	Vector2f projectedTo[MAX_RES_PER_POINT];
};

struct PointFrameResidual
{
	EFResidual* efResidual;

	static int instanceCounter;


	ResState state_state;
	double state_energy;
	ResState state_NewState;
	double state_NewEnergy;
	double state_NewEnergyWithOutlier;


	PointHessian* point;
	FrameHessian* host;
	FrameHessian* target;
	RawResidualJacobian* J;


	bool isNew;


	Vector2f projectedTo[MAX_RES_PER_POINT];
	Vec3f centerProjectedTo;
};

struct cuda_PointFrameResidual
{
	cuda_EFResidual efResidual;

	ResState state_state;
	double state_energy;
	ResState state_NewState;
	double state_NewEnergy;
	double state_NewEnergyWithOutlier;
	RawResidualJacobian J;
	bool isNew;
	Vector2f projectedTo[MAX_RES_PER_POINT];
	Vec3f centerProjectedTo;
	double stats0;
	bool toRemove;

	// PointHessian values
	float color[MAX_RES_PER_POINT];			// colors in host frame
	float weights[MAX_RES_PER_POINT];		// host-weights for respective residuals.
	float u, v;
	float idepth_scaled;
	float idepth_zero_scaled;
	float maxRelBaseline;
	bool isGoodResidual;

	// precalc values
	Mat33f PRE_KRKiTll;
	Mat33f PRE_RTll_0;
	Vec2f PRE_aff_mode;
	float PRE_b0_mode;
	Vec3f PRE_KtTll;
	Vec3f PRE_tTll_0;

	//int host_idx;
	float host_frameEnergyTH;	// set dynamically depending on tracking residual

	//int target_idx;
	float target_frameEnergyTH;	// set dynamically depending on tracking residual
};

__device__
Vector3f getInterpolatedElement33(const Vector3f* const mat, const float x, const float y, const int width)
{
	int ix = (int)x;
	int iy = (int)y;
	float dx = x - ix;
	float dy = y - iy;
	float dxdy = dx*dy;
	const Vector3f* bp = mat + ix + iy*width;


	return Vector3f_add4_Vector3f(float_mul_Vector3f(dxdy, *(const Vector3f*)(bp + 1 + width)),
		float_mul_Vector3f((dy - dxdy), *(const Vector3f*)(bp + width)),
		float_mul_Vector3f((dx - dxdy), *(const Vector3f*)(bp + 1)),
		float_mul_Vector3f((1 - dx - dy + dxdy), *(const Vector3f*)(bp)));
}

__device__
bool projectPoint(
	const float &u_pt, const float &v_pt,
	const float &idepth,
	const Mat33f &KRKi, const Vec3f &Kt,
	float &Ku, float &Kv,
	float wM3G, float hM3G)
{
	Vec3f ptp = Vec3f_add(mat33f_mul_vec3f(KRKi, make_Vec3f(u_pt, v_pt, 1)), Vec3f_mul_float(Kt, idepth));
	//Vec3f ptp = KRKi * Vec3f(u_pt, v_pt, 1) + Kt*idepth;

	Ku = ptp.x / ptp.z;
	Kv = ptp.y / ptp.z;
	//Ku = ptp[0] / ptp[2];
	//Kv = ptp[1] / ptp[2];

	return Ku>1.1f && Kv>1.1f && Ku<wM3G && Kv<hM3G;
}

__device__
bool projectPoint(
	const float &u_pt, const float &v_pt,
	const float &idepth,
	const int &dx, const int &dy,
	CalibHessian* const &HCalib,
	const Mat33f &R, const Vec3f &t,
	float &drescale, float &u, float &v,
	float &Ku, float &Kv, Vec3f &KliP, float &new_idepth,
	float wM3G, float hM3G)
{
	KliP = make_Vec3f(
		(u_pt + dx - HCalib->cxl)*HCalib->fxli,
		(v_pt + dy - HCalib->cyl)*HCalib->fyli,
		1);
	//KliP = Vec3f(
	//	(u_pt + dx - HCalib->cxl())*HCalib->fxli(),
	//	(v_pt + dy - HCalib->cyl())*HCalib->fyli(),
	//	1);

	Vec3f ptp = Vec3f_add(mat33f_mul_vec3f(R, KliP), Vec3f_mul_float(t, idepth));
	//Vec3f ptp = R * KliP + t*idepth;

	drescale = 1.0f / ptp.z;
	//drescale = 1.0f / ptp[2];

	new_idepth = idepth*drescale;

	if (!(drescale > 0)) return false;

	u = ptp.x * drescale;
	v = ptp.y * drescale;
	Ku = u*HCalib->fxl + HCalib->cxl;
	Kv = v*HCalib->fyl + HCalib->cyl;
	//u = ptp[0] * drescale;
	//v = ptp[1] * drescale;
	//Ku = u*HCalib->fxl() + HCalib->cxl();
	//Kv = v*HCalib->fyl() + HCalib->cyl();

	return Ku>1.1f && Kv>1.1f && Ku<wM3G && Kv<hM3G;
}

__device__
void takeDataF(cuda_PointFrameResidual* pfr, cuda_EFResidual* ef)
{
	//RawResidualJacobian tmp;
	//tmp = pfr->J;
	//pfr->J = ef->J;
	//ef->J = tmp;
	RawResidualJacobian tmp;
	tmp.resF = pfr->J.resF;
	tmp.Jpdxi0 = pfr->J.Jpdxi0;
	tmp.Jpdxi1 = pfr->J.Jpdxi1;
	tmp.Jpdc0 = pfr->J.Jpdc0;
	tmp.Jpdc1 = pfr->J.Jpdc1;
	tmp.Jpdd = pfr->J.Jpdd;
	tmp.JIdx0 = pfr->J.JIdx0;
	tmp.JIdx1 = pfr->J.JIdx1;
	tmp.JabF0 = pfr->J.JabF0;
	tmp.JabF1 = pfr->J.JabF1;
	tmp.JIdx2 = pfr->J.JIdx2;
	tmp.JabJIdx = pfr->J.JabJIdx;
	tmp.Jab2 = pfr->J.Jab2;

	pfr->J.resF = ef->J.resF;
	pfr->J.Jpdxi0 = ef->J.Jpdxi0;
	pfr->J.Jpdxi1 = ef->J.Jpdxi1;
	pfr->J.Jpdc0 = ef->J.Jpdc0;
	pfr->J.Jpdc1 = ef->J.Jpdc1;
	pfr->J.Jpdd = ef->J.Jpdd;
	pfr->J.JIdx0 = ef->J.JIdx0;
	pfr->J.JIdx1 = ef->J.JIdx1;
	pfr->J.JabF0 = ef->J.JabF0;
	pfr->J.JabF1 = ef->J.JabF1;
	pfr->J.JIdx2 = ef->J.JIdx2;
	pfr->J.JabJIdx = ef->J.JabJIdx;
	pfr->J.Jab2 = ef->J.Jab2;

	ef->J.resF = tmp.resF;
	ef->J.Jpdxi0 = tmp.Jpdxi0;
	ef->J.Jpdxi1 = tmp.Jpdxi1;
	ef->J.Jpdc0 = tmp.Jpdc0;
	ef->J.Jpdc1 = tmp.Jpdc1;
	ef->J.Jpdd = tmp.Jpdd;
	ef->J.JIdx0 = tmp.JIdx0;
	ef->J.JIdx1 = tmp.JIdx1;
	ef->J.JabF0 = tmp.JabF0;
	ef->J.JabF1 = tmp.JabF1;
	ef->J.JIdx2 = tmp.JIdx2;
	ef->J.JabJIdx = tmp.JabJIdx;
	ef->J.Jab2 = tmp.Jab2;


	Vec2f JI_JI_Jd = mat22_mul_vec2(ef->J.JIdx2, ef->J.Jpdd);

	ef->JpJdF.d0 = ef->J.Jpdxi0.d0 * JI_JI_Jd.x + ef->J.Jpdxi1.d0 * JI_JI_Jd.y;
	ef->JpJdF.d1 = ef->J.Jpdxi0.d1 * JI_JI_Jd.x + ef->J.Jpdxi1.d1 * JI_JI_Jd.y;
	ef->JpJdF.d2 = ef->J.Jpdxi0.d2 * JI_JI_Jd.x + ef->J.Jpdxi1.d2 * JI_JI_Jd.y;
	ef->JpJdF.d3 = ef->J.Jpdxi0.d3 * JI_JI_Jd.x + ef->J.Jpdxi1.d3 * JI_JI_Jd.y;
	ef->JpJdF.d4 = ef->J.Jpdxi0.d4 * JI_JI_Jd.x + ef->J.Jpdxi1.d4 * JI_JI_Jd.y;
	ef->JpJdF.d5 = ef->J.Jpdxi0.d5 * JI_JI_Jd.x + ef->J.Jpdxi1.d5 * JI_JI_Jd.y;

	Vec2f v = mat22_mul_vec2(ef->J.JabJIdx, ef->J.Jpdd);
	ef->JpJdF.d6 = v.x;
	ef->JpJdF.d7 = v.y;
}

__device__
void applyRes(cuda_PointFrameResidual* pfr, bool copyJacobians)
{
	if (copyJacobians)
	{
		if (pfr->state_state == ResState::ResState_OOB)
		{
			assert(!pfr->efResidual.isActiveAndIsGoodNEW);
			return;	// can never go back from OOB
		}
		if (pfr->state_NewState == ResState::ResState_IN)// && )
		{
			pfr->efResidual.isActiveAndIsGoodNEW = true;
			takeDataF(pfr, &(pfr->efResidual));
		}
		else
		{
			pfr->efResidual.isActiveAndIsGoodNEW = false;
		}
	}

	pfr->state_state = pfr->state_NewState;
	pfr->state_energy = pfr->state_NewEnergy;
}

__device__
double linearize(cuda_PointFrameResidual* pfr, Vector3f* dIl, CalibHessian* HCalib, int w, float wM3G, float hM3G)
{
	pfr->state_NewEnergyWithOutlier = -1;

	if (pfr->state_state == ResState::ResState_OOB)
	{
		pfr->state_NewState = ResState::ResState_OOB; return pfr->state_energy;
	}
	
	//FrameFramePrecalc* precalc = &(host->targetPrecalc[target->idx]);
	float energyLeft = 0;
	//const Vector3f* dIl = (const Vector3f*)(frameHessians[pfr->host_idx]->dIp[0]);
	const Mat33f &PRE_KRKiTll = pfr->PRE_KRKiTll;
	const Vec3f &PRE_KtTll = pfr->PRE_KtTll;
	const Mat33f &PRE_RTll_0 = pfr->PRE_RTll_0;
	const Vec3f &PRE_tTll_0 = pfr->PRE_tTll_0;
	const float * const color = pfr->color;
	const float * const weights = pfr->weights;
	//const Mat33f &PRE_KRKiTll = precalc->PRE_KRKiTll;
	//const Vec3f &PRE_KtTll = precalc->PRE_KtTll;
	//const Mat33f &PRE_RTll_0 = precalc->PRE_RTll_0;
	//const Vec3f &PRE_tTll_0 = precalc->PRE_tTll_0;
	//const float * const color = point->color;
	//const float * const weights = point->weights;

	Vec2f affLL = pfr->PRE_aff_mode;
	float b0 = pfr->PRE_b0_mode;
	//Vec2f affLL = precalc->PRE_aff_mode;
	//float b0 = precalc->PRE_b0_mode;

	Vec6f d_xi_x, d_xi_y;
	Vec4f d_C_x, d_C_y;
	float d_d_x, d_d_y;
	{
		float drescale, u, v, new_idepth;
		float Ku, Kv;
		Vec3f KliP;

		//return PRE_RTll_0.d00 + PRE_RTll_0.d10 + PRE_RTll_0.d20;
		if (!projectPoint(pfr->u, pfr->v, pfr->idepth_zero_scaled, 0, 0, HCalib,
			PRE_RTll_0, PRE_tTll_0, drescale, u, v, Ku, Kv, KliP, new_idepth, wM3G, hM3G))
		//if (!projectPoint(point->u, point->v, point->idepth_zero_scaled, 0, 0, HCalib,
		//	PRE_RTll_0, PRE_tTll_0, drescale, u, v, Ku, Kv, KliP, new_idepth, wM3G, hM3G))
		{
			pfr->state_NewState = ResState::ResState_OOB; return pfr->state_energy;
		}
		
		pfr->centerProjectedTo = make_Vec3f(Ku, Kv, new_idepth);


		// diff d_idepth
		d_d_x = drescale * (PRE_tTll_0.x - PRE_tTll_0.z * u)*SCALE_IDEPTH*HCalib->fxl;
		d_d_y = drescale * (PRE_tTll_0.y - PRE_tTll_0.z * v)*SCALE_IDEPTH*HCalib->fyl;
		//d_d_x = drescale * (PRE_tTll_0[0] - PRE_tTll_0[2] * u)*SCALE_IDEPTH*HCalib->fxl();
		//d_d_y = drescale * (PRE_tTll_0[1] - PRE_tTll_0[2] * v)*SCALE_IDEPTH*HCalib->fyl();

		// diff calib
		d_C_x.z = drescale*(PRE_RTll_0.d20*u - PRE_RTll_0.d00);
		d_C_x.w = HCalib->fxl * drescale*(PRE_RTll_0.d21 *u - PRE_RTll_0.d01) * HCalib->fyli;
		d_C_x.x = KliP.x * d_C_x.z;
		d_C_x.y = KliP.y * d_C_x.w;
		//d_C_x[2] = drescale*(PRE_RTll_0(2, 0)*u - PRE_RTll_0(0, 0));
		//d_C_x[3] = HCalib->fxl() * drescale*(PRE_RTll_0(2, 1)*u - PRE_RTll_0(0, 1)) * HCalib->fyli();
		//d_C_x[0] = KliP[0] * d_C_x[2];
		//d_C_x[1] = KliP[1] * d_C_x[3];

		d_C_y.z = HCalib->fyl * drescale*(PRE_RTll_0.d20*v - PRE_RTll_0.d10) * HCalib->fxli;
		d_C_y.w = drescale*(PRE_RTll_0.d21*v - PRE_RTll_0.d11);
		d_C_y.x = KliP.x * d_C_y.z;
		d_C_y.y = KliP.y * d_C_y.w;
		//d_C_y[2] = HCalib->fyl() * drescale*(PRE_RTll_0(2, 0)*v - PRE_RTll_0(1, 0)) * HCalib->fxli();
		//d_C_y[3] = drescale*(PRE_RTll_0(2, 1)*v - PRE_RTll_0(1, 1));
		//d_C_y[0] = KliP[0] * d_C_y[2];
		//d_C_y[1] = KliP[1] * d_C_y[3];

		d_C_x.x = (d_C_x.x + u)*SCALE_F;
		d_C_x.y *= SCALE_F;
		d_C_x.z = (d_C_x.z + 1)*SCALE_C;
		d_C_x.w *= SCALE_C;
		//d_C_x[0] = (d_C_x[0] + u)*SCALE_F;
		//d_C_x[1] *= SCALE_F;
		//d_C_x[2] = (d_C_x[2] + 1)*SCALE_C;
		//d_C_x[3] *= SCALE_C;

		d_C_y.x *= SCALE_F;
		d_C_y.y = (d_C_y.y + v)*SCALE_F;
		d_C_y.z *= SCALE_C;
		d_C_y.w = (d_C_y.w + 1)*SCALE_C;
		//d_C_y[0] *= SCALE_F;
		//d_C_y[1] = (d_C_y[1] + v)*SCALE_F;
		//d_C_y[2] *= SCALE_C;
		//d_C_y[3] = (d_C_y[3] + 1)*SCALE_C;


		d_xi_x.d0 = new_idepth*HCalib->fxl;
		d_xi_x.d1 = 0;
		d_xi_x.d2 = -new_idepth*u*HCalib->fxl;
		d_xi_x.d3 = -u*v*HCalib->fxl;
		d_xi_x.d4 = (1 + u*u)*HCalib->fxl;
		d_xi_x.d5 = -v*HCalib->fxl;
		//d_xi_x[0] = new_idepth*HCalib->fxl();
		//d_xi_x[1] = 0;
		//d_xi_x[2] = -new_idepth*u*HCalib->fxl();
		//d_xi_x[3] = -u*v*HCalib->fxl();
		//d_xi_x[4] = (1 + u*u)*HCalib->fxl();
		//d_xi_x[5] = -v*HCalib->fxl();

		d_xi_y.d0 = 0;
		d_xi_y.d1 = new_idepth*HCalib->fyl;
		d_xi_y.d2 = -new_idepth*v*HCalib->fyl;
		d_xi_y.d3 = -(1 + v*v)*HCalib->fyl;
		d_xi_y.d4 = u*v*HCalib->fyl;
		d_xi_y.d5 = u*HCalib->fyl;
		//d_xi_y[0] = 0;
		//d_xi_y[1] = new_idepth*HCalib->fyl();
		//d_xi_y[2] = -new_idepth*v*HCalib->fyl();
		//d_xi_y[3] = -(1 + v*v)*HCalib->fyl();
		//d_xi_y[4] = u*v*HCalib->fyl();
		//d_xi_y[5] = u*HCalib->fyl();
	}


	{
		pfr->J.Jpdxi0 = d_xi_x;
		pfr->J.Jpdxi1 = d_xi_y;
		//J->Jpdxi[0] = d_xi_x;
		//J->Jpdxi[1] = d_xi_y;

		pfr->J.Jpdc0 = make_VecCf(d_C_x.x, d_C_x.y, d_C_x.z, d_C_x.w);
		pfr->J.Jpdc1 = make_VecCf(d_C_y.x, d_C_y.y, d_C_y.z, d_C_y.w);
		//J->Jpdc[0] = d_C_x;
		//J->Jpdc[1] = d_C_y;

		pfr->J.Jpdd = make_Vec2f(d_d_x, d_d_y);
		//J->Jpdd[0] = d_d_x;
		//J->Jpdd[1] = d_d_y;

	}


	float JIdxJIdx_00 = 0, JIdxJIdx_11 = 0, JIdxJIdx_10 = 0;
	float JabJIdx_00 = 0, JabJIdx_01 = 0, JabJIdx_10 = 0, JabJIdx_11 = 0;
	float JabJab_00 = 0, JabJab_01 = 0, JabJab_11 = 0;

	float wJI2_sum = 0;

	for (int idx = 0; idx<patternNum; idx++)
	{
		float Ku, Kv;
		if (!projectPoint(pfr->u + patternP[idx][0], pfr->v + patternP[idx][1], pfr->idepth_scaled, PRE_KRKiTll, PRE_KtTll, Ku, Kv, wM3G, hM3G))
		//if (!projectPoint(point->u + patternP[idx][0], point->v + patternP[idx][1], point->idepth_scaled, PRE_KRKiTll, PRE_KtTll, Ku, Kv, wM3G, hM3G))
		{
			pfr->state_NewState = ResState::ResState_OOB; return pfr->state_energy;
		}

		pfr->projectedTo[idx] = make_Vector2f(Ku, Kv);
		//projectedTo[idx][0] = Ku;
		//projectedTo[idx][1] = Kv;


		Vector3f hitColor = (getInterpolatedElement33(dIl, Ku, Kv, w));
		float residual = hitColor.x - (float)(affLL.x * color[idx] + affLL.y);
		//Vec3f hitColor = (getInterpolatedElement33(dIl, Ku, Kv, wG[0]));
		//float residual = hitColor[0] - (float)(affLL[0] * color[idx] + affLL[1]);



		float drdA = (color[idx] - b0);
		if (!isfinite((float)hitColor.x))
		//if (!std::isfinite((float)hitColor[0]))
		{
			pfr->state_NewState = ResState::ResState_OOB; return pfr->state_energy;
		}


		float w = sqrtf(setting_outlierTHSumComponent / (setting_outlierTHSumComponent + (hitColor.y * hitColor.y + hitColor.z * hitColor.z)));
		//float w = sqrtf(setting_outlierTHSumComponent / (setting_outlierTHSumComponent + hitColor.tail<2>().squaredNorm()));
		w = 0.5f*(w + weights[idx]);



		float hw = fabsf(residual) < setting_huberTH ? 1 : setting_huberTH / fabsf(residual);
		energyLeft += w*w*hw *residual*residual*(2 - hw);

		{
			if (hw < 1) hw = sqrtf(hw);
			hw = hw*w;

			hitColor.y *= hw;
			hitColor.z *= hw;
			//hitColor[1] *= hw;
			//hitColor[2] *= hw;

			pfr->J.resF.data[idx] = residual*hw;
			//J->resF[idx] = residual*hw;

			pfr->J.JIdx0.data[idx] = hitColor.y;
			pfr->J.JIdx1.data[idx] = hitColor.z;
			pfr->J.JabF0.data[idx] = drdA*hw;
			pfr->J.JabF1.data[idx] = hw;
			//J->JIdx[0][idx] = hitColor[1];
			//J->JIdx[1][idx] = hitColor[2];
			//J->JabF[0][idx] = drdA*hw;
			//J->JabF[1][idx] = hw;

			JIdxJIdx_00 += hitColor.y * hitColor.y;
			JIdxJIdx_11 += hitColor.z * hitColor.z;
			JIdxJIdx_10 += hitColor.y * hitColor.z;
			//JIdxJIdx_00 += hitColor[1] * hitColor[1];
			//JIdxJIdx_11 += hitColor[2] * hitColor[2];
			//JIdxJIdx_10 += hitColor[1] * hitColor[2];

			JabJIdx_00 += drdA*hw * hitColor.y;
			JabJIdx_01 += drdA*hw * hitColor.z;
			JabJIdx_10 += hw * hitColor.y;
			JabJIdx_11 += hw * hitColor.z;
			//JabJIdx_00 += drdA*hw * hitColor[1];
			//JabJIdx_01 += drdA*hw * hitColor[2];
			//JabJIdx_10 += hw * hitColor[1];
			//JabJIdx_11 += hw * hitColor[2];

			JabJab_00 += drdA*drdA*hw*hw;
			JabJab_01 += drdA*hw*hw;
			JabJab_11 += hw*hw;


			wJI2_sum += hw*hw*(hitColor.y * hitColor.y + hitColor.z * hitColor.z);
			//wJI2_sum += hw*hw*(hitColor[1] * hitColor[1] + hitColor[2] * hitColor[2]);

			if (setting_affineOptModeA < 0) pfr->J.JabF0.data[idx] = 0;
			if (setting_affineOptModeB < 0) pfr->J.JabF1.data[idx] = 0;
			//if (setting_affineOptModeA < 0) J->JabF[0][idx] = 0;
			//if (setting_affineOptModeB < 0) J->JabF[1][idx] = 0;

		}
	}

	pfr->J.JIdx2 = make_Mat22f(JIdxJIdx_00, JIdxJIdx_10, JIdxJIdx_10, JIdxJIdx_11);
	pfr->J.JabJIdx = make_Mat22f(JabJIdx_00, JabJIdx_01, JabJIdx_10, JabJIdx_11);
	pfr->J.Jab2 = make_Mat22f(JabJab_00, JabJab_01, JabJab_01, JabJab_11);
	//J->JIdx2(0, 0) = JIdxJIdx_00;
	//J->JIdx2(0, 1) = JIdxJIdx_10;
	//J->JIdx2(1, 0) = JIdxJIdx_10;
	//J->JIdx2(1, 1) = JIdxJIdx_11;
	//J->JabJIdx(0, 0) = JabJIdx_00;
	//J->JabJIdx(0, 1) = JabJIdx_01;
	//J->JabJIdx(1, 0) = JabJIdx_10;
	//J->JabJIdx(1, 1) = JabJIdx_11;
	//J->Jab2(0, 0) = JabJab_00;
	//J->Jab2(0, 1) = JabJab_01;
	//J->Jab2(1, 0) = JabJab_01;
	//J->Jab2(1, 1) = JabJab_11;

	pfr->state_NewEnergyWithOutlier = energyLeft;

	if (energyLeft > max(pfr->host_frameEnergyTH, pfr->target_frameEnergyTH) || wJI2_sum < 2)
	{
		energyLeft = max(pfr->host_frameEnergyTH, pfr->target_frameEnergyTH);
		pfr->state_NewState = ResState::ResState_OUTLIER;
	}
	else
	{
		pfr->state_NewState = ResState::ResState_IN;
	}

	pfr->state_NewEnergy = energyLeft;
	return energyLeft;
}


///////////////////////////////////////
/*
extern "C"
void cudaSetFrameFramePrecalc(float* PRE_RTll, float* PRE_KRKiTll, float* PRE_RKiTll, float* PRE_RTll_0, float* PRE_aff_mode, float PRE_b0_mode, float* PRE_tTll, float* PRE_KtTll, float* PRE_tTll_0, float distanceLL, bool profile)
{
	StopWatchInterface *timer = NULL;
	if (profile)
	{
		sdkCreateTimer(&timer);
		sdkStartTimer(&timer);
	}

	if(precalc == NULL) checkCudaErrors(cudaMalloc((void **)&precalc, sizeof(FrameFramePrecalc)));

	checkCudaErrors(cudaMemcpy((void *)&precalc->PRE_RTll, (const void *)PRE_RTll, sizeof(Mat33f), cudaMemcpyHostToDevice));
	checkCudaErrors(cudaMemcpy((void *)&precalc->PRE_KRKiTll, (const void *)PRE_KRKiTll, sizeof(Mat33f), cudaMemcpyHostToDevice));
	checkCudaErrors(cudaMemcpy((void *)&precalc->PRE_RKiTll, (const void *)PRE_RKiTll, sizeof(Mat33f), cudaMemcpyHostToDevice));
	checkCudaErrors(cudaMemcpy((void *)&precalc->PRE_RTll_0, (const void *)PRE_RTll_0, sizeof(Mat33f), cudaMemcpyHostToDevice));

	checkCudaErrors(cudaMemcpy((void *)&precalc->PRE_aff_mode, (const void *)PRE_aff_mode, sizeof(Vec2f), cudaMemcpyHostToDevice));
	checkCudaErrors(cudaMemcpy((void *)&precalc->PRE_b0_mode, (const void *)&PRE_b0_mode, sizeof(float), cudaMemcpyHostToDevice));

	checkCudaErrors(cudaMemcpy((void *)&precalc->PRE_tTll, (const void *)PRE_tTll, sizeof(Vec3f), cudaMemcpyHostToDevice));
	checkCudaErrors(cudaMemcpy((void *)&precalc->PRE_KtTll, (const void *)PRE_KtTll, sizeof(Vec3f), cudaMemcpyHostToDevice));
	checkCudaErrors(cudaMemcpy((void *)&precalc->PRE_tTll_0, (const void *)PRE_tTll_0, sizeof(Vec3f), cudaMemcpyHostToDevice));

	checkCudaErrors(cudaMemcpy((void *)&precalc->distanceLL, (const void *)&distanceLL, sizeof(float), cudaMemcpyHostToDevice));

	if (profile)
	{
		sdkStopTimer(&timer);
		printf("\t\t\t\t\t\t\tcudaSetFrameFramePrecalc(): transfer in FrameFramePrecalc: %f\n", sdkGetTimerValue(&timer));
		sdkResetTimer(&timer);
		sdkDeleteTimer(&timer);
	}
}


extern "C"
void cudaSetFrameFramePrecalc2(void* data, bool profile)
{
	StopWatchInterface *timer = NULL;
	if (profile)
	{
		sdkCreateTimer(&timer);
		sdkStartTimer(&timer);
	}

	if (precalc == NULL) checkCudaErrors(cudaMalloc((void **)&precalc, sizeof(FrameFramePrecalc)));
	checkCudaErrors(cudaMemcpy((void *)precalc, (const void *)data, sizeof(FrameFramePrecalc), cudaMemcpyHostToDevice));

	if (profile)
	{
		sdkStopTimer(&timer);
		printf("\t\t\t\t\t\t\tcudaSetFrameFramePrecalc2(): transfer in FrameFramePrecalc: %f\n", sdkGetTimerValue(&timer));
		sdkResetTimer(&timer);
		sdkDeleteTimer(&timer);
	}
}
*/

#endif // ! __RESIDUALS_CUH__