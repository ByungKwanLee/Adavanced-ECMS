// C++ library 
#include <iostream>
#include <math.h>
#include <typeinfo>  // typeid::name operator

// Advanced ECMS library
#include <VehicleInfo.hpp>
#include <ICEMG.hpp>
#include <interp_tool.hpp>
#include <optimizer.hpp>

// ROS library
#include <ros/ros.h>
#include <std_msgs/Float32.h>

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

// check sum for callback 
int i = 0;

void accel_callback(const std_msgs::Float32::ConstPtr & accel_data)
{	
	VehicleInfo::velocity_update(accel_data->data);
	// cout << "velocity and accel : "<< VehicleInfo::velocity_rt <<", "<< VehicleInfo::accel_rt << endl;
	i += 1;
} 

void accel_nocallback()
{	
	// friction for ground and wind
	VehicleInfo::velocity_update(-0.3);
	// cout << "velocity and accel : "<< VehicleInfo::velocity_rt <<", "<< VehicleInfo::accel_rt << endl;
}


int main(int argc, char ** argv){

// ros setting for making node
ros::init(argc, argv, "Advanced_ecms_node");
ros::NodeHandle nh;
ros::Subscriber sub = nh.subscribe("/accel", 1, accel_callback);
ros::Publisher pub_vel = nh.advertise<std_msgs::Float32>("/velocity", 1);
ros::Publisher pub_soc = nh.advertise<std_msgs::Float32>("/soc", 1);

MG::SOC = 0.6;
VehicleInfo::velocity =  11.76*3.6/3.6;
VehicleInfo::accel = 4;
Tool::ICEMG_parameter_update();

// Optimizer obj_optimizer(-491,1,1,1);

// obj_optimizer.optimal_method("L");

// cout << obj_optimizer.Lagrangian_Costmodeling("EV") << endl;
// cout << obj_optimizer.Lagrangian_Costmodeling("HEV")<< endl;
// cout << obj_optimizer.ADMM_Costmodeling("EV") << endl;
// cout << obj_optimizer.ADMM_Costmodeling("HEV") << endl;

// // L method
// float * minimum_EV_L = obj_optimizer.minimum_EV("L");
// float ** minimum_HEV_L = obj_optimizer.minimum_HEV("L");

// // ADMM method
// float * minimum_EV_ADMM = obj_optimizer.minimum_EV("ADMM");
// float ** minimum_HEV_ADMM = obj_optimizer.minimum_HEV("ADMM");


// Processing time
// cout << "Optimal Mode : " <<std::get<0>(obj_optimizer.optimal_method("L"))
// << ", Optimal gear : " << std::get<1>(obj_optimizer.optimal_method("L"))
// << ", grid number : " <<std::get<2>(obj_optimizer.optimal_method("L"))
// <<", cost : " <<std::get<3>(obj_optimizer.optimal_method("L"))
// <<endl<<endl;


// real time processing part
// ros::Rate rate(50); // HZ
// while(ros::ok())
// {	
// 	ros::Time start = ros::Time::now();
// 	int i_copy = i;

// 	// ros spin for subscribing rt var : accel
// 	ros::spinOnce();

// 	// if no callback, then deceleration for friction
// 	if (i == i_copy) accel_nocallback();

// 	// update from rt var -> optimizer var
// 	VehicleInfo::for_optimizer();

// 	// publish velocity
// 	std_msgs::Float32 msg_vel; 
// 	msg_vel.data = VehicleInfo::velocity_rt;
// 	pub_vel.publish(msg_vel);

// 	Tool::ICEMG_parameter_update();
// 	// limitation of int byte
// 	if (i > 10000) i = 0;

// 	ros::Time end = ros::Time::now();

// 	obj_optimizer.optimal_method("L");

// 	// Processing time
// 	cout << "Optimal Mode : " <<std::get<0>(obj_optimizer.optimal_inform)
// 	<< ", Optimal gear : " << std::get<1>(obj_optimizer.optimal_inform)
// 	<< ", grid number : " <<std::get<2>(obj_optimizer.optimal_inform)
// 	<<", cost : " <<std::get<3>(obj_optimizer.optimal_inform)
// 	<<", SOC : "<<MG::SOC
// 	<<", velocity : " <<VehicleInfo::velocity
// 	<<", accel : " << VehicleInfo::accel
// 	<<endl<<endl;

// 	// publish SOC
// 	std_msgs::Float32 msg_SOC; 
// 	msg_SOC.data = MG::SOC;
// 	pub_soc.publish(msg_SOC);

// 	// obj_optimizer.SOC_update();

// 	// sleep thread
// 	rate.sleep();
// }








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