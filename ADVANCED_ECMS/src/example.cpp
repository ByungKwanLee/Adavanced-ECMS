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

	vector<float> xData{1,2,3,4,5,6,10};
   int x_size = xData.size();
   int min_ind    =  std::min_element(xData.begin(), xData.end()) - xData.begin();
   float min_data = *std::min_element(xData.begin(), xData.end());
   std::for_each(xData.begin(), xData.end(), [](float& d) { d*=-3;});
   
   for(vector<float>::iterator it = xData.begin(); it != xData.end(); it++)
   {
   		cout << *it << endl;
   }

	float lambda = 0.1;
	float tau = 1/pow(lambda,2);
	float iter = 100.;


	cout << VehicleInfo::P_d() << endl;
}