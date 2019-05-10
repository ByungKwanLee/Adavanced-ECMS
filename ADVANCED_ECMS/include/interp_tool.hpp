#include <iostream>
#include <eigen3/Eigen/Dense>
#include <vector>

using namespace std;

namespace LBK
{

class interp_Tool
{

public:
	static float interpolate_1d(vector<float> & xData, vector<float> & yData, 
		float x, bool extrapolation);
	static float  interpolate_2d(vector<float> & xData, vector<float> & yData, 
		vector<vector<float>> & zData, float x, float y);
};

}

