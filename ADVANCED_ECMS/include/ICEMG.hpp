#include <iostream>
#include <eigen3/Eigen/Dense>
#include <math.h>
// 
#include <vector>

using namespace std;

namespace LBK
{

class ICE
{

public:
	// TM, FD ratio, Eff
	static vector<float> TM_Ratio;
	static Eigen::VectorXf TMR; // type casting from std to eigen
	static float TM_Eff;
	static float FD_EFF;
	static float FD_Ratio;

	static vector<float> En_minRPM;
	static vector<float> En_minTrq;
	static vector<float> En_maxRPM;
	static vector<float> En_maxTrq;

	static vector<float> En_mapRPM;
	static vector<float> En_mapTrq;
	static vector<vector<float>> En_mapData;


	// HEV
	static float Wi_HEV;
	static float Ti_min;
	static float Ti_max;
	static float Ti_HEV;

	// FC
	static float FC_HEV;

	// 
	static void init_TM_Ratio();

};

class MG
{
public:

	// battery configuration
	static float Voc;
	static float Rint;
	static float P_bat;
	static float Bat_NumCell;
	static float P_aux;

	static float Bat_Quantity;
	static float SOC;
	static vector<float> Bat_indexRint;
	static vector<float> Bat_indexSoc;
	static vector<float> Bat_indexVoc;

	// Motor map
	static vector<float> MG_mapTrq;
	static vector<float> MG_mapRPM;
	static vector<vector<float>> MG_mapData;
	static vector<float> MG_maxTrq;
	static vector<float> MG_maxRPM;


	// EV 
	static Eigen::VectorXf W1_EV;
	static Eigen::VectorXf T1_EV;
	static Eigen::VectorXf Eta1_EV; // efficiency of motor power
	static Eigen::VectorXf P1elec_EV;
	static float PbatD_EV;
	static float dSOC_EV;

	// HEV
	static float W1_HEV;
	static float T1_HEV;
	static float Eta1_HEV; // efficiency of motor power
	static float P1elec_HEV;
	static float PbatD_HEV;
	static float dSOC_HEV;

	MG();
	static vector<float> rpm2rs(vector<float> rpm);
	static Eigen::VectorXf power(Eigen::VectorXf W1, Eigen::VectorXf T1, Eigen::VectorXf Eta1);
	static Eigen::VectorXf Eta1_func(Eigen::VectorXf W1, Eigen::VectorXf T1);
};


}