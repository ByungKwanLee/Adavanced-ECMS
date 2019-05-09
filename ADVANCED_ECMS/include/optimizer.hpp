#include <iostream>
#include <VehicleInfo.hpp>
#include <vector>

using namespace std;


namespace LBK
{


class Optimizer
{

private:
	float lambda;
	float tau;
	float iter;
	float Costmodeling();

public:
	Optimizer(float lambda, float tau, float iter);
	
};



}

