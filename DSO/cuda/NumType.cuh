#ifndef __NUM_TYPE_CUH__
#define __NUM_TYPE_CUH__


#include <math.h>

// Includes CUDA
#include <cuda_runtime.h>

#if defined(__CUDACC_RTC__)
#define __VECTOR_FUNCTIONS_DECL__ __host__ __device__
#else /* !__CUDACC_RTC__ */
#define __VECTOR_FUNCTIONS_DECL__ static __inline__ __host__ __device__
#endif /* __CUDACC_RTC__ */


#define SSEE(val,idx) (*(((float*)&val)+idx))
#define todouble(x) (x).cast<double>()
#define MatToDynamic(x) MatXX(x)


#define MAX_POINTS_PER_FRAME 2000
#define MAX_RES_PER_POINT 8
#define NUM_THREADS 6
#define CPARS 4



struct Vec2f
{
	float x, y;
};

__VECTOR_FUNCTIONS_DECL__
Vec2f make_Vec2f(float x, float y)
{
	Vec2f t; t.x = x; t.y = y; return t;
}

__VECTOR_FUNCTIONS_DECL__
float Vec2f_dot(const Vec2f a, const Vec2f b)
{
	return a.x * b.x + a.y * b.y;
}


struct Vec2
{
	double x, y;
};

__VECTOR_FUNCTIONS_DECL__
Vec2 make_Vec2(double x, double y)
{
	Vec2 t; t.x = x; t.y = y; return t;
}

__VECTOR_FUNCTIONS_DECL__
Vec2f Vec2f_mul_float(const Vec2f a, const float b)
{
	return make_Vec2f(a.x * b, a.y * b);
}

__VECTOR_FUNCTIONS_DECL__
Vec2f float_mul_Vec2f(const float a, const Vec2f b)
{
	return make_Vec2f(a * b.x, a * b.y);
}

__VECTOR_FUNCTIONS_DECL__
Vec2f Vec2f_add(const Vec2f a, const Vec2f b)
{
	return make_Vec2f(a.x + b.x, a.y + b.y);
}

__VECTOR_FUNCTIONS_DECL__
Vec2f Vec2f_sub(const Vec2f a, const Vec2f b)
{
	return make_Vec2f(a.x - b.x, a.y - b.y);
}


struct Vec3f
{
	float x, y, z;
};

__VECTOR_FUNCTIONS_DECL__
Vec3f make_Vec3f(float x, float y, float z)
{
	Vec3f t; t.x = x; t.y = y; t.z = z; return t;
}

__VECTOR_FUNCTIONS_DECL__
Vec3f Vec3f_mul_float(const Vec3f a, const float b)
{
	return make_Vec3f(a.x * b, a.y * b, a.z * b);
}

__VECTOR_FUNCTIONS_DECL__
Vec3f float_mul_Vec3f(const float a, const Vec3f b)
{
	return make_Vec3f(a * b.x, a * b.y, a * b.z);
}

__VECTOR_FUNCTIONS_DECL__
float Vec3f_dot(const Vec3f a, const Vec3f b)
{
	return a.x * b.x + a.y * b.y + a.z * b.z;
}

__VECTOR_FUNCTIONS_DECL__
Vec3f Vec3f_add(const Vec3f a, const Vec3f b)
{
	return make_Vec3f(a.x + b.x, a.y + b.y, a.z + b.z);
}


struct Vec4f
{
	float x, y, z, w;
};


struct Vec6f
{
	float d0, d1, d2, d3, d4, d5;
};

struct Vec6
{
	double d0, d1, d2, d3, d4, d5;
};


struct Vec8f
{
	float d0, d1, d2, d3, d4, d5, d6, d7;
};


struct Vec10
{
	double d0, d1, d2, d3, d4, d5, d6, d7, d8, d9;
};


struct Mat22f
{
	float d00, d10, d01, d11;
};

__VECTOR_FUNCTIONS_DECL__
Mat22f make_Mat22f(const float d00, const float d01, const float d10, const float d11)
{
	Mat22f t; t.d00 = d00; t.d01 = d01; t.d10 = d10; t.d11 = d11; return t;
}

__VECTOR_FUNCTIONS_DECL__
Vec2f mat22_mul_vec2(const Mat22f m, const Vec2f b)
{
	return make_Vec2f(Vec2f_dot(make_Vec2f(m.d00, m.d01), b), Vec2f_dot(make_Vec2f(m.d10, m.d11), b));
}

struct Mat33f
{
	float d00, d10, d20, d01, d11, d21, d02, d12, d22;
};
typedef Mat33f Matrix3f;

__VECTOR_FUNCTIONS_DECL__
Mat33f make_Mat33f(const float d00, const float d01, const float d02, const float d10, const float d11, const float d12, const float d20, const float d21, const float d22)
{
	Mat33f t; t.d00 = d00; t.d01 = d01; t.d02 = d02; t.d10 = d10; t.d11 = d11; t.d12 = d12; t.d20 = d20; t.d21 = d21; t.d22 = d22;  return t;
}
#define make_Matrix3f make_Mat33f

__VECTOR_FUNCTIONS_DECL__
Vec3f mat33f_mul_vec3f(const Mat33f m, const Vec3f b)
{
	return make_Vec3f(Vec3f_dot(make_Vec3f(m.d00, m.d01, m.d02), b), Vec3f_dot(make_Vec3f(m.d10, m.d11, m.d12), b), Vec3f_dot(make_Vec3f(m.d20, m.d21, m.d22), b));
}

__VECTOR_FUNCTIONS_DECL__
Mat33f inverse_Mat33f(Mat33f m)
{
	float det = m.d00 * (m.d11 * m.d22 - m.d12 * m.d21);
	det -= m.d01 * (m.d10 * m.d22 - m.d12 * m.d20);
	det += m.d02 * (m.d10 * m.d21 - m.d11 * m.d20);
	float idet = 1.0f / det;

	Mat33f ret;
	ret.d00 = (m.d11 * m.d22 - m.d12 * m.d21) * idet;
	ret.d10 = -(m.d10 * m.d22 - m.d12 * m.d20) * idet;
	ret.d20 = (m.d10 * m.d21 - m.d11 * m.d20) * idet;
	ret.d01 = -(m.d01 * m.d22 - m.d02 * m.d21) * idet;
	ret.d11 = (m.d00 * m.d22 - m.d02 * m.d20) * idet;
	ret.d21 = -(m.d00 * m.d21 - m.d01 * m.d20) * idet;
	ret.d02 = (m.d01 * m.d12 - m.d02 * m.d11) * idet;
	ret.d12 = -(m.d00 * m.d12 - m.d02 * m.d10) * idet;
	ret.d22 = (m.d00 * m.d11 - m.d01 * m.d10) * idet;
	return ret;
}
#define inverse_Matrix3f inverse_Mat33f


struct Mat42
{
	double d00, d10, d20, d30, d01, d11, d21, d31;
};


struct Mat66
{
	Vec6 row[6];
};


struct VecC
{
	double x, y, z, w;
};

__VECTOR_FUNCTIONS_DECL__
VecC make_VecC(double x, double y, double z, double w)
{
	VecC t; t.x = x; t.y = y; t.z = z; t.w = w; return t;
}


struct VecCf
{
	float x, y, z, w;
};

__VECTOR_FUNCTIONS_DECL__
VecCf make_VecCf(float x, float y, float z, float w)
{
	VecCf t; t.x = x; t.y = y; t.z = z; t.w = w; return t;
}


struct Vector2f
{
	float x, y;
};

__VECTOR_FUNCTIONS_DECL__
Vector2f make_Vector2f(float x, float y)
{
	Vector2f t; t.x = x; t.y = y; return t;
}


struct Vector3f
{
	float x, y, z;
};

__VECTOR_FUNCTIONS_DECL__
Vector3f make_Vector3f(float x, float y, float z)
{
	Vector3f t; t.x = x; t.y = y; t.z = z; return t;
}

__VECTOR_FUNCTIONS_DECL__
Vector3f Vector3f_add_Vector3f(const Vector3f a, const Vector3f b)
{
	return make_Vector3f(a.x + b.x, a.y + b.y, a.z + b.z);
}

__VECTOR_FUNCTIONS_DECL__
Vector3f Vector3f_add3_Vector3f(const Vector3f a, const Vector3f b, const Vector3f c)
{
	return make_Vector3f(a.x + b.x + c.x, a.y + b.y + c.y, a.z + b.z + c.z);
}

__VECTOR_FUNCTIONS_DECL__
Vector3f Vector3f_add4_Vector3f(const Vector3f a, const Vector3f b, const Vector3f c, const Vector3f d)
{
	return make_Vector3f(a.x + b.x + c.x + d.x, a.y + b.y + c.y + d.y, a.z + b.z + c.z + d.z);
}

__VECTOR_FUNCTIONS_DECL__
Vector3f Vector3f_mul_float(const Vector3f a, const float b)
{
	return make_Vector3f(a.x * b, a.y * b, a.z * b);
}

__VECTOR_FUNCTIONS_DECL__
Vector3f float_mul_Vector3f(const float a, const Vector3f b)
{
	return make_Vector3f(a * b.x, a * b.y, a * b.z);
}


struct VecNRf
{
	float data[8];
};

struct Vec8
{
	double data[8];
};


// transforms points from one frame to another.
struct AffLight
{
	AffLight(double a_, double b_) : a(a_), b(b_) {};
	AffLight() : a(0), b(0) {};

	// Affine Parameters:
	double a,b;	// I_frame = exp(a)*I_global + b. // I_global = exp(-a)*(I_frame - b).
};

__VECTOR_FUNCTIONS_DECL__
Vec2 fromToVecExposure(float exposureF, float exposureT, AffLight g2F, AffLight g2T)
{
	if (exposureF == 0 || exposureT == 0)
	{
		exposureT = exposureF = 1;
		//printf("got exposure value of 0! please choose the correct model.\n");
		//assert(setting_brightnessTransferFunc < 2);
	}

	double a = exp(g2T.a - g2F.a) * exposureT / exposureF;
	double b = g2T.b - a*g2F.b;
	return make_Vec2(a, b);
}

__VECTOR_FUNCTIONS_DECL__
Vec2 vec(AffLight aff)
{
	return make_Vec2(aff.a, aff.b);
}

#endif // !__NUM_TYPE_CUH__
