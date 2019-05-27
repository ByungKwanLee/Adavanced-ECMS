#include <iostream>
#include <fstream>
#include <vector>
#include <optimizer.hpp>
#include <ros/ros.h>

namespace LBK
{

class ECMS_performance
{

public:
    std::vector<float> dSOC_per;
    std::vector<float> SOC_per;
    std::vector<float> gear_per;
    std::vector<float> mode_per;
    std::vector<float> mass_per;
	void do_performance(Optimizer & obj_optimizer, string method, float time_rt);
	void csvwrite();

	ECMS_performance();
	static std::vector<float> velocity_cycle;
	static std::vector<float> accel_cycle;
	




};

}