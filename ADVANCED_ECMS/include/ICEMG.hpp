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
	static Eigen::VectorXf Wi_HEV;
	static Eigen::MatrixXf Wi_HEV_rep; // special
	static Eigen::VectorXf Ti_max;
	static Eigen::MatrixXf Ti_HEV;

	// FC
	static Eigen::MatrixXf FC_HEV;

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
	static Eigen::VectorXf PbatD_EV;
	static Eigen::VectorXf dSOC_EV;

	// HEV
	static Eigen::MatrixXf W1_HEV;
	static Eigen::MatrixXf T1_HEV;

	static Eigen::MatrixXf Eta1_HEV; // efficiency of motor power
	static Eigen::MatrixXf P1elec_HEV;
	static Eigen::MatrixXf PbatD_HEV;
	static Eigen::MatrixXf dSOC_HEV;

	MG();
	
};

class Tool
{
public :
	static const int NumGrid; 
	static vector<float> rpm2rs(vector<float> rpm);
	static Eigen::MatrixXf FC_HEV();
	static Eigen::MatrixXf Ti_HEV();
	static Eigen::VectorXf Ti_max();

	static Eigen::VectorXf W1_EV();
	static Eigen::VectorXf T1_EV();
	static Eigen::VectorXf Wi_HEV();
	static Eigen::MatrixXf Wi_HEV_rep();
	static Eigen::MatrixXf W1_HEV();
	static Eigen::MatrixXf T1_HEV();
	
	static Eigen::VectorXf PbatD_EV();
	static Eigen::MatrixXf PbatD_HEV();

	static Eigen::VectorXf dSOC_EV();
	static Eigen::MatrixXf dSOC_HEV();
	static Eigen::VectorXf P1elec_EV();
	static Eigen::MatrixXf P1elec_HEV();
	static Eigen::VectorXf Eta1_EV();
	static Eigen::MatrixXf Eta1_HEV();

};


}