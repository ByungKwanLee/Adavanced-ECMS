#include <iostream>
#include <optimizer.hpp>
#include <math.h>

using namespace std;
using namespace LBK;

int main(){

	float lambda = 0.1;
	float tau = 1/pow(lambda,2);
	float iter = 100.;
	VehicleInfo vehicle_info;
	// OptimizerUtils bird(lambda, tau, iter);
	cout << vehicle_info.Frl() << endl;
}