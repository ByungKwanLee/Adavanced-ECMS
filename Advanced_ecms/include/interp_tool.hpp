#include <iostream>
#include <vector>
#include <eigen3/Eigen/Dense>

using namespace std;



class interp_Tool
{

public:
	static float interpolate_1d(vector<float> & xData, vector<float> & yData, 
		float x, bool extrapolation);
	static float  interpolate_2d(vector<float> & xData, vector<float> & yData, 
		vector<vector<float>> & zData, float x, float y);
};



