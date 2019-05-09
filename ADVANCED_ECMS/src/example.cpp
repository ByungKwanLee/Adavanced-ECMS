#include <iostream>
#include <optimizer.hpp>
#include <math.h>
#include <data_loader.hpp>
#include <ICEMG.hpp>
#include <typeinfo>  // typeid::name operator

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


   float * p =interp_Tool::interpolate_2d(MG::MG_mapRPM, MG::MG_mapTrq, MG::MG_mapData, 201, 300);

	float lambda = 0.1;
	float tau = 1/pow(lambda,2);
	float iter = 100.;

	cout << p[0] << ", " << p[1] << endl;
    delete[] p;
}