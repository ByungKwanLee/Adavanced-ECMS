#include <iostream>
#include <VehicleInfo.hpp>
#include <math.h>
#include <ICEMG.hpp>
#include <interp_tool.hpp>
#include <typeinfo>  // typeid::name operator
#include <std_msgs/Float32.h>
#include <ros/ros.h>

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


void accel_callback(const std_msgs::Float32::ConstPtr & accel_data)
{	

	VehicleInfo::accel_rt = accel_data->data;
	VehicleInfo::velocity_rt = VehicleInfo::velocity_pre_rt + VehicleInfo::accel_rt * 0.01;
	VehicleInfo::velocity_pre_rt = VehicleInfo::velocity_rt;
	cout << "velocity and accel : "<< VehicleInfo::velocity_rt <<", "<< VehicleInfo::accel_rt << endl;
} 


int main(int argc, char ** argv){

ros::init(argc, argv, "Advanced_ecms_node");
ros::NodeHandle nh;
ros::Subscriber sub = nh.subscribe("/accel", 1, accel_callback);

ros::Time start = ros::Time::now();
MG::SOC = 0.5;
VehicleInfo::velocity =  60.4/3.6;
VehicleInfo::accel = -2.3;
Tool::ICEMG_parameter_update();
ros::Time end = ros::Time::now();

ros::spin();
// Processing time
// cout << "Processing time : "<<(end - start).toSec() <<endl<<endl;

// P_d
// cout<< "Tool::P_d() : "<< VehicleInfo::P_d() <<endl<<endl;

// Voc, Rint
// cout << "Tool::Voc() : " << Tool::Voc(false) <<endl<<endl; // guranteed
// cout << "Tool::Rint() : " << Tool::Rint(false) <<endl<<endl; // guranteed

// EV of W1, T1, Eta1 estimation
// cout << "Tool::W1_EV() : " <<endl<<Tool::W1_EV(false) << endl<< endl; // guranteed
// cout << "Tool::T1_EV() : " <<endl<<Tool::T1_EV(false) << endl<< endl; // guranteed
// cout << "Tool::Eta1_EV() : " <<endl<<Tool::Eta1_EV(false) <<endl<< endl; // guranteed

// EV of P estimation
// cout << "Tool::P1elec_EV() : " <<endl<<Tool::P1elec_EV(false)<< endl<< endl; //guranteed
// cout << "Tool::PbatD_EV() : " <<endl<<Tool::PbatD_EV(false)<< endl<< endl; //guranteed
// cout << "Tool::dSOC_EV() : " <<endl<<Tool::dSOC_EV(false)<< endl<< endl; //guranteed

// HEV of Wi, Ti
// cout <<"Tool::Wi_HEV() : " << endl<<Tool::Wi_HEV(false) << endl<< endl; //guranteed
// cout << "Tool::Wi_HEV_rep() : " <<endl<<Tool::Wi_HEV_rep(false) << endl<< endl; // guranteed
// cout << "Tool::Ti_max() : " <<endl<<Tool::Ti_max(false) << endl<< endl; //guranteed
// cout <<"Tool::Ti_HEV() : " << endl<<Tool::Ti_HEV(false).size() << endl<< endl; //guranteed
// cout <<"Tool::FC_HEV() : " << endl<<Tool::FC_HEV(false).size() << endl<< endl; //guranteed

// HEV of W1 T1 Eta1
// cout <<"Tool::W1_HEV() : " << endl<<Tool::W1_HEV(false).size() << endl<< endl; // guranteed
// cout <<"Tool::T1_HEV() : " << endl<<Tool::T1_HEV(false).size() << endl<< endl; // guranteed
// cout << "Tool::Eta1_HEV() : " <<endl<<Tool::Eta1_HEV(false).size() << endl<< endl; // guranteed

// HEV of P estimation
// cout <<"Tool::P1elec_HEV() : " << endl<<Tool::P1elec_HEV(false).size() << endl<< endl; // guranteed
// cout <<"Tool::PbatD_HEV() : " << endl<<Tool::PbatD_HEV(false).size() << endl<< endl; // guranteed
// cout << "Tool::dSOC_HEV() : " <<endl<<Tool::dSOC_HEV(false).size()<< endl<< endl; // guranteed

}