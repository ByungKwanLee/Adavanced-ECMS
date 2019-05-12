#include <iostream>
#include <math.h>
#include <VehicleInfo.hpp>
#include <eigen3/Eigen/Dense>

using namespace std;
using namespace LBK;

float VehicleInfo::velocity = 0; // [60km/h -> m/s]
float VehicleInfo::accel = 0; // [m/s^2]
float VehicleInfo::mass = 1530.9; //[kg]
float VehicleInfo::parameter[3] = {88.6,0.14,0.36};
float VehicleInfo::wheel_radius=0.27; //[m]

float VehicleInfo::Frl()
{
	float velocity = VehicleInfo::velocity;
	float* parameter = VehicleInfo::parameter;
	return parameter[0] + velocity*parameter[1] + pow(velocity,2)*parameter[2];
}


float VehicleInfo::P_d()
{
	return (VehicleInfo::Frl()
	+VehicleInfo::mass*VehicleInfo::accel)
	*VehicleInfo::velocity; // [Nm/s, J/s]
}
