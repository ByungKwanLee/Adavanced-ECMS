#include <iostream>
#include <optimizer.hpp>
#include <math.h>
#include <data_loader.hpp>

// getcwd
// #include <unistd.h>
// std::string GetCurrentWorkingDir( void ) {
//   char buff[FILENAME_MAX];
//   getcwd( buff, FILENAME_MAX );
//   std::string current_working_dir(buff);
//   return current_working_dir;
// }

using namespace std;
using namespace LBK;

int main(){
	vector<vector<float>> array = get_2d_data("src/ADVANCED_ECMS/data/example.csv");
	float lambda = 0.1;
	float tau = 1/pow(lambda,2);
	float iter = 100.;
	VehicleInfo vehicle_info;
	// OptimizerUtils bird(lambda, tau, iter);
	cout << vehicle_info.Frl() << endl;
}