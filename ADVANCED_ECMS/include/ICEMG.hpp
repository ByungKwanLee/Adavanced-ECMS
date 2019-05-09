#include <iostream>
#include <eigen3/Eigen/Dense>
#include <math.h>
#include <vector>

using namespace std;

namespace LBK
{

class ICE
{

public:

	static float TM_Ratio;
	static float TM_Eff;
	static float FD_EFF;
	static float RF_Ratio;
};

class MG
{
public:

	static float Voc;
	static float Rint;
	static float P_bat;
	static float Bat_NumCell;

	static float Bat_Quantity;
	static float SOC;
	static vector<float> Bat_indexRint;
	static vector<float> Bat_indexSoc;
	static vector<float> Bat_indexVoc;

	static vector<float> MG_mapTrq;
	static vector<float> MG_mapRPM;
	static vector<float> MG_maxTrq;
	static vector<float> MG_maxRPM;
	static vector<vector<float>> MG_mapData;

	MG();
	float SOC_rate();
};


class interp_Tool
{

public:
	static float interpolate_1d(vector<float> & xData, vector<float> & yData, 
		float x, bool extrapolation);
	static float interpolate_2d(vector<float> & xData, vector<float> & yData, vector<vector<float>> & zData, 
	vector<float> x, vector<float> y);
};


}