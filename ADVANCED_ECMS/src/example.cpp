#include <iostream>
#include <optimizer.hpp>
#include <math.h>
#include <data_load.hpp>

using namespace std;
using namespace LBK;

int main(){
	vector<vector<float>> array = get_2d_data("example.csv");
	float lambda = 0.1;
	float tau = 1/pow(lambda,2);
	float iter = 100.;
	VehicleInfo vehicle_info;
	// OptimizerUtils bird(lambda, tau, iter);
	cout << vehicle_info.Frl() << endl;
}