#include <iostream>
#include <fstream>
#include <vector>
#include <optimizer.hpp>
#include <ros/ros.h>



class ECMS_performance
{

public:

	std::vector<float> Ti;
	std::vector<float> Wi;
	std::vector<float> T1;
	std::vector<float> W1;

    std::vector<float> dSOC_per;
    std::vector<float> SOC_per;
    std::vector<float> gear_per;
    std::vector<float> mode_per;
    std::vector<float> mass_per;
    std::vector<float> lambda_per;
    std::vector<float> mu_per;
    std::vector<float> raw_per;

    float SOC_init;
	void do_performance(Optimizer & obj_optimizer, string method, float time_rt);
	void csvwrite(string method);

	ECMS_performance(float SOC);
	static std::vector<float> velocity_cycle;
	static std::vector<float> accel_cycle;
	




};

