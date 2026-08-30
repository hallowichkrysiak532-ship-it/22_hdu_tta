#ifndef __PUBLIC_MATH_H
#define __PUBLIC_MATH_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#define FP_ZERO_2         0
#define FP_SUBNORMAL_2    4
#define FP_NORMAL_2       5
#define FP_INFINITE_2     3
#define FP_NAN_2          7

//宏定义
#define M_E			 2.7182818284590452354
#define M_LOG2E		 1.4426950408889634074
#define M_LOG10E	 0.43429448190325182765
#define M_LN2		 0.69314718055994530942
#define M_LN10		 2.30258509299404568402
#define M_PI		 3.14159265358979323846		//圆周率pi
#define M_2PI        6.28318530717958647692		//圆周率2*pi
#ifndef M_PI_2
 #define M_PI_2		 1.57079632679489661923		//圆周率pi/2
#endif
#define M_PI_4		 0.78539816339744830962		//圆周率pi/4
#define M_1_PI		 0.31830988618379067154		//圆周率1/pi
#define M_2_PI		 0.63661977236758134308		//圆周率2/pi
#define M_2_SQRTPI	 1.12837916709551257390		//2/sqrt(pi)
#define M_SQRT2		 1.41421356237309504880		//2的平方根
#define M_SQRT1_2	 0.70710678118654752440		//2的平方根的倒数
#define M_DEG_TO_RAD 0.01745329251994
#define M_RAD_TO_DEG 57.2957795130823

#define M_E_F			((float)M_E)
#define M_LOG2E_F		((float)M_LOG2E)
#define M_LOG10E_F	 	((float)M_LOG10E)
#define M_LN2_F		 	((float)M_LN2)
#define M_LN10_F		((float)M_LN10)
#define M_PI_F		 	((float)M_PI)
#define M_2PI_F        	((float)M_2PI)
#define M_PI_2_F		((float)M_PI_2)
#define M_PI_4_F		((float)M_PI_4)
#define M_1_PI_F		((float)M_1_PI)
#define M_2_PI_F		((float)M_2_PI)
#define M_2_SQRTPI_F	((float)M_2_SQRTPI)
#define M_SQRT2_F		((float)M_SQRT2)
#define M_SQRT1_2_F	 	((float)M_SQRT1_2)
#define M_DEG_TO_RAD_F 	((float)M_DEG_TO_RAD)
#define M_RAD_TO_DEG_F 	((float)M_RAD_TO_DEG)

#define	EARTH_RADIUS	6371393.0

//一阶低通滤波器参数
typedef struct{
	unsigned char  not_first_input;		//第一次输入不进行滤波
	float alpha;
	float output;	
}lowpass_filter_1p_t;




typedef struct tagQuaternionCoor_t
{
	float a;
	float b;
	float c;
	float d;
}quaternionCoor_t;
typedef struct tag_vector3
{
	float x;
	float y;
	float z;
}_vector3;


typedef struct tag_nav_t
{
	float n;
	float e;
	float d;
}_nav_t;


//方差期望计算
typedef struct{
	unsigned char not_first_input;		//第一次输入不计算方差期望
	float E;
	float Var;
}variance_expectation_t;

float sq(const float value);
unsigned char is_zero(const float value);
float vector3_length(_vector3 vector);
float constrain_float(float value, float min, float max);
float lowpass_filter_1p_fliter(lowpass_filter_1p_t* filter, float sample, float cutoff_freq, float dt);
void Math_UpdateEVar(float input, variance_expectation_t *var_expe, unsigned short bufferSize);

float Angle_Conversion_180(float angle);
float Angle_Conversion_360(float angle);


//矢量乘法
float vector_multiplication(float* A, float* B, int n);
void matrix_multiply_const(float* A, float b, int m, int n, float* C);
//矩阵乘法
void matrix_multiply(float* A, float* B, int m, int p, int n, float* C);
//矩阵加法
void matrix_addition(float* A, float* B, int m, int n, float* C);
//矩阵减法
void matrix_subtraction(float* A, float* B, int m, int n, float* C);
//矩阵取逆
int matrix_inversion(float* A, int n, float* AInverse);
//矩阵转置
void matrix_transpose(float* A, int m, int n, float* C);


void NorthEastToForwardRight(float n, float e, float yaw, float *forward, float *right);
void ForwardRightToNorthEast(float f, float r, float yaw, float *north, float *east);


#endif
