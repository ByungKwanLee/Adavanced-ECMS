#include <iostream>
#include <math.h>
#include <VehicleInfo.hpp>
#include <eigen3/Eigen/Dense>

using namespace std;
using namespace LBK;

float VehicleInfo::velocity = 0;
float VehicleInfo::accel = 0;

VehicleInfo::VehicleInfo()
{
	float * parameter=VehicleInfo::parameter;
	parameter[0]=5;
	parameter[1]=3;
	parameter[2]=0.1;
}

float VehicleInfo::Frl()
{
	float velocity = VehicleInfo::velocity;
	float* parameter = VehicleInfo::parameter;
	return parameter[0] + velocity*parameter[1] + pow(velocity,2)*parameter[2];
}