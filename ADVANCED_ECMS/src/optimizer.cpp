#include <iostream>
#include <math.h>
#include <optimizer.hpp>
#include <eigen3/Eigen/Dense>

using namespace std;
using namespace LBK;

// battery configuration
float OptimizerUtils::Qmax = 3200;

OptimizerUtils::OptimizerUtils(float lambda, float tau, float iter)
{
	OptimizerUtils::lambda = lambda;
	OptimizerUtils::tau = tau;
	OptimizerUtils::iter = iter;
	OptimizerUtils::Qmax = Qmax;
	float * f_array = OptimizerUtils::parameter;
	*f_array = 5;
	*(f_array+1)=3;
	*(f_array+2)=0.5;
}

float F_rl(float velocity, float* parameter)
{
	return parameter[0] + velocity*parameter[1] + pow(velocity,2)*parameter[2];
}

float OptimizerUtils::Costmodeling()
{
// using lagrangian method cost and ADMM cost
}

float OptimizerUtils::SOC_penalty(float soc, float soc_L, float soc_H)
{
	float x_soc = (soc-(soc_L + soc_H)/2)/(soc_H-soc_L);
	return 1-(1-0.7*x_soc)*pow(x_soc, 3);
}

float OptimizerUtils::SOC_rate(float Voc, float Rint, float P_bat)
{
	return (-Voc + sqrt(pow(Voc,2)-4*P_bat*Rint))/(2*Rint*OptimizerUtils::Qmax);
}

