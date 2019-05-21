#include <iostream>
#include <vector>
#include <assert.h>
#include <eigen3/Eigen/Dense>
#include <typeinfo>
#include <map>
#include <tuple>

using namespace std;


namespace LBK
{


class Optimizer
{

private:
	float lambda;
	float mu;
	float raw;
	float max_iter;

public:
	std::tuple<string, int, int, float> optimal_inform;
	
	Optimizer(float lambda, float mu, float raw, float max_iter);
	Eigen::MatrixXf Lagrangian_Costmodeling(string mode);
	Eigen::MatrixXf ADMM_Costmodeling(string mode);
	vector<float> minimum_EV(string method);
	vector<vector<float>> minimum_HEV(string method);
	std::tuple<string, int, int, float> optimal_method(string method);
	void SOC_update();

	void En_FC_rt(bool print = false);
	float En_FC_sum;
	float En_FC_instant;

	
};



}

