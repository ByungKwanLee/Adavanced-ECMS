#include <iostream>
#include <VehicleInfo.hpp>
#include <math.h>
#include <ICEMG.hpp>
#include <interp_tool.hpp>
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


// EV of W1, T1, Eta1 estimation
cout << "Tool::W1_EV() : " <<endl<<Tool::W1_EV() << endl<< endl;
cout << "Tool::T1_EV() : " <<endl<<Tool::T1_EV() << endl<< endl;
cout << "Tool::Eta1_EV() : " <<endl<<Tool::Eta1_EV() <<endl<< endl;

// EV of P estimation
cout << "Tool::P1elec_EV() : " <<endl<<Tool::P1elec_EV()<< endl<< endl;
cout << "Tool::PbatD_EV() : " <<endl<<Tool::PbatD_EV()<< endl<< endl;
cout << "Tool::dSOC_EV() : " <<endl<<Tool::dSOC_EV()<< endl<< endl;

// HEV of Wi, Ti
// cout <<"Tool::Wi_HEV() : " << endl<<Tool::Wi_HEV() << endl<< endl;
// cout << "Tool::Wi_HEV_rep() : " <<endl<<Tool::Wi_HEV_rep() << endl<< endl;
// cout << "Tool::Ti_max() : " <<endl<<Tool::Ti_max() << endl<< endl;
// cout <<"Tool::Ti_HEV() : " << endl<<Tool::Ti_HEV() << endl<< endl;
// cout <<"Tool::FC_HEV() : " << endl<<Tool::FC_HEV() << endl<< endl;

// HEV of W1 T1 Eta1
// cout <<"Tool::W1_HEV() : " << endl<<Tool::W1_HEV() << endl<< endl;
// cout <<"Tool::T1_HEV() : " << endl<<Tool::T1_HEV() << endl<< endl;
// cout << "Tool::Eta1_HEV() : " <<endl<<Tool::Eta1_HEV() << endl<< endl;

// HEV of P estimation
// cout <<"Tool::P1elec_HEV() : " << endl<<Tool::P1elec_HEV() << endl<< endl;
// cout <<"Tool::PbatD_HEV() : " << endl<<Tool::PbatD_HEV() << endl<< endl;
// cout << "Tool::dSOC_HEV() : " <<endl<<Tool::dSOC_HEV()<< endl<< endl;

}