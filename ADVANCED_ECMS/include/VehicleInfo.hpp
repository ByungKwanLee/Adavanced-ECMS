#include <iostream>

using namespace std;

namespace LBK
{

class VehicleInfo
{

public:
	// variable
	static float velocity;
	static float accel;
	static float mass;  // [kg]
	static float parameter[3];
	static float wheel_radius;

	// calculation function
	static float Frl();
	static float P_d();
	
	// real time variable
	static float velocity_rt;
	static float velocity_pre_rt;
	static float accel_rt;
};


};