#include "public_math.h"
#include "float.h"

/*********************************************************************************
*函数说明：平方运算
*输入参数：值v
*输出参数：无
*返回参数：结果
*********************************************************************************/
float sq(const float value)
{
    return value*value;
}
///三维向量
/*********************************************************************************
*函数说明：计算三维向量的长度（范数）
*输入参数：三维向量vector
*输出参数：无
*返回参数：向量的长度
*********************************************************************************/
float vector3_length(_vector3 vector) 
{			
	return sqrt(sq(vector.x)+sq(vector.y)+sq(vector.z));
}

/*********************************************************************************
*函数说明：判断浮点数是否等于0
*输入参数：判断值fVal
*输出参数：无
*返回参数：1：等于0,0：不等于0
*********************************************************************************/
unsigned char is_zero(const float value) 
{
    return fabsf(value) < FLT_EPSILON ? 1 : 0;
}


/*********************************************************************************
*函数说明：限幅函数
*输入参数：输入数据value, 最小值min, 最大值max
*输出参数：限制幅值后的值
*********************************************************************************/
float constrain_float(float value, float min, float max)
{
	if(fpclassify(value) == FP_NAN_2)
		return (min + max) * 0.5f;
	
	if(min >= max)
		return value;
	
	if(value > max)
		value = max;
	else if(value < min)
		value = min;
	else{}
		
	return value;
}

int16_t constrain_int16(int16_t value, int16_t min, int16_t max)
{
	if(min >= max)
		return value;
	
	if(value > max)
		value = max;
	else if(value < min)
		value = min;
	else{}	
		
	return value;
}
int32_t constrain_int32(int32_t value, int32_t min, int32_t max)
{
	if(min >= max)
		return value;
	
	if(value > max)
		value = max;
	else if(value < min)
		value = min;
	else{}	
		
	return value;
}


/*********************************************************************************
*函数说明：一阶低通滤波函数，利用当前采样值计算滤波值
*输入参数：滤波器指针* filter，采样值sample, 截止频率cutoff_freq，采样周期
*输出参数：无
*返回参数：滤波后的数据
*********************************************************************************/
float lowpass_filter_1p_fliter(lowpass_filter_1p_t* filter, float sample, float cutoff_freq, float dt) 
{
    if (cutoff_freq < 0.0f || is_zero(cutoff_freq) || dt < 0.0f || is_zero(dt)) {
        filter->output = sample;
        return sample;
    }

	if(1.0f/dt < 2.0f*cutoff_freq){	//采样频率必须大于截至频率的两倍
		filter->output = sample;
        return sample;	
	}
	
	if(!filter->not_first_input)
	{		//第一次输入不进行滤波, 计算滤波参数
		float rc = 2.0f*(float)M_PI*cutoff_freq*dt/(1.0f+2.0f*(float)M_PI*cutoff_freq*dt);
		filter->not_first_input = 1;
		filter->output = sample;
		filter->alpha = constrain_float(rc, 0.0f, 1.0f);
        return sample;		
	}

	filter->output = filter->alpha*sample + (1-filter->alpha)*filter->output;
    return filter->output;
}



/*********************************************************************************
*函数说明：统计函数，使用迭代的方法计算方差和期望
*输入参数：输入数据input，方差期望计算值*var_expe，计算长度bufferSize
*输出参数：无
*返回参数：无
*********************************************************************************/
void Math_UpdateEVar(float input, variance_expectation_t *var_expe, unsigned short bufferSize)
{
	if(var_expe->not_first_input != 1)
	{
		var_expe->not_first_input = 1;
		var_expe->E = input;
		var_expe->Var = 200;
	}
	else
	{
		var_expe->E += (input - var_expe->E)/bufferSize;
		var_expe->Var = (bufferSize-2)*(var_expe->Var)/(bufferSize-1)+(input - var_expe->E)*(input - var_expe->E)/bufferSize;
	}
}



float Angle_Conversion_180(float angle)
{
	float result;
	result = angle;
	while(result > 180.0f)
	{
		result = result-360.0f;
	}
	
	while(result < -180.0f)
	{
		result = result+360.0f;
	}
	
	return result;
}


float Angle_Conversion_360(float angle)
{
	float result;
	result = angle;
	while(result > 360.0f)
	{
		result = result-360.0f;
	}
	
	while(result < 0.0f)
	{
		result = result+360.0f;
	}
	
	return result;
}


//矢量乘法
float vector_multiplication(float* A, float* B, int n)
{
	float result = 0;
	int i;
	for(i=0;i<n;i++)
	{
		result += A[i]*B[i];
	}
	return result;
}

void matrix_multiply_const(float* A, float b, int m, int n, float* C)
{
	int i, j;
  	for(i=0;i<m;i++)
    	for(j=0;j<n;j++)
      		C[n*i+j]=A[n*i+j]*b;
}

//矩阵乘法
void matrix_multiply(float* A, float* B, int m, int p, int n, float* C)
{

    int i, j, k;
    for (i=0; i<m; i++)
        for(j=0; j<n; j++)
        {
            C[n*i+j]=0;
            for (k=0; k<p; k++)
                C[n*i+j]= C[n*i+j]+A[p*i+k]*B[n*k+j];
        }
}



//矩阵加法
void matrix_addition(float* A, float* B, int m, int n, float* C)

{

	int i, j;
  	for(i=0;i<m;i++)
    	for(j=0;j<n;j++)
      		C[n*i+j]=A[n*i+j]+B[n*i+j];
}


//矩阵减法
void matrix_subtraction(float* A, float* B, int m, int n, float* C)
{
	int i, j;
  	for(i=0;i<m;i++)
    	for(j=0;j<n;j++)
      		C[n*i+j]=A[n*i+j]-B[n*i+j];
}

//矩阵取逆
int matrix_inversion(float* A, int n, float* AInverse)
{
	int i, j, iPass, imx, icol, irow;
	float det, temp, pivot, factor = 0.0f;
	float* ac = (float*)malloc(n*n*sizeof(float));
	if(ac == NULL)
	{
		return 0;
	}
  	det = 1;
  	for(i = 0; i < n; i++)
  	{
    	for (j = 0; j < n; j++)
    	{
      		AInverse[n*i+j] = 0;
      		ac[n*i+j] = A[n*i+j];
    	}
    	AInverse[n*i+i] = 1;
  	}
  	for(iPass = 0; iPass < n; iPass++)
  	{
    	imx = iPass;
    	for (irow = iPass; irow < n; irow++)
    	{
      		if (fabs(A[n*irow+iPass]) > fabs(A[n*imx+iPass])) imx = irow;
    	}


		if (imx != iPass)
	    {
			for (icol = 0; icol < n; icol++)
	      	{
	        	temp = AInverse[n*iPass+icol];
	        	AInverse[n*iPass+icol] = AInverse[n*imx+icol];
	        	AInverse[n*imx+icol] = temp;
	        	if (icol >= iPass)
	        	{
	          		temp = A[n*iPass+icol];
	          		A[n*iPass+icol] = A[n*imx+icol];
	          		A[n*imx+icol] = temp;
	        	}
	      	}
	    }

	    pivot = A[n*iPass+iPass];
	    det = det * pivot;
	    if (det == 0)
	    {
			free(ac);
	      	return 0;
	    }

	    for (icol = 0; icol < n; icol++)
	    {

			AInverse[n*iPass+icol] = AInverse[n*iPass+icol] / pivot;
	      	if (icol >= iPass) A[n*iPass+icol] = A[n*iPass+icol] / pivot;
	    }

	    for (irow = 0; irow < n; irow++)
	    {

			if (irow != iPass) factor = A[n*irow+iPass];
	      	for (icol = 0; icol < n; icol++)
	      	{
	        	if (irow != iPass)
	        	{
	          		AInverse[n*irow+icol] -= factor * AInverse[n*iPass+icol];
	          		A[n*irow+icol] -= factor * A[n*iPass+icol];
	        	}
	      	}
	    }
  	}

  	free(ac);
  	return 1;
}


//矩阵转置
void matrix_transpose(float* A, int m, int n, float* C)
{
    int i, j;
    for (i=0; i<m; i++)
        for(j=0; j<n; j++)
            C[m*j+i]=A[n*i+j];
}



void NorthEastToForwardRight(float n, float e, float yaw, float *forward, float *right)
{
	float cos_yaw,sin_yaw;
//	cos_yaw = arm_cos_f32(yaw*M_DEG_TO_RAD_F);
//	sin_yaw = arm_sin_f32(yaw*M_DEG_TO_RAD_F);
	
	cos_yaw = cos(yaw*M_DEG_TO_RAD_F);
	sin_yaw = sin(yaw*M_DEG_TO_RAD_F);
	
	*forward = cos_yaw*n+sin_yaw*e;
	*right = cos_yaw*e-sin_yaw*n;
}



void ForwardRightToNorthEast(float f, float r, float yaw, float *north, float *east)
{
	float cos_yaw,sin_yaw;
//	cos_yaw = arm_cos_f32(yaw*M_DEG_TO_RAD_F);
//	sin_yaw = arm_sin_f32(yaw*M_DEG_TO_RAD_F);
	
	cos_yaw = cos(yaw*M_DEG_TO_RAD_F);
	sin_yaw = sin(yaw*M_DEG_TO_RAD_F);
	
	*east = cos_yaw*r+sin_yaw*f;
	*north = cos_yaw*f-sin_yaw*r;
	
}


