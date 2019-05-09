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
	float ICE;

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
	static vector<vector<float>> MG_mapData;

	MG();
	float SOC_rate();
};


class Est_Tool
{

public:
	static float interpolate_1d(vector<float> & xData, vector<float> & yData, 
		float x, bool extrapolation);
	// static float interpolate_2d
};


}