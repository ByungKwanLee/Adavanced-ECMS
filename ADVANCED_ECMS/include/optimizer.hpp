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
	float mu;
	float raw;
	float max_iter;
	float correction;

public:
	std::tuple<string, int, int, float> optimal_inform;
	
	Optimizer(float lambda, float mu, float raw, float max_iter);
	Eigen::MatrixXf Lagrangian_Costmodeling(string mode);
	Eigen::MatrixXf ADMM_Costmodeling(string mode);
	vector<float> minimum_EV(string method);
	vector<vector<float>> minimum_HEV(string method);
	void optimal_method(string method);
	void SOC_correction(bool update = false);
	void optimizer(string method);

	void En_FC_rt(bool print = false);
	float En_FC_sum;
	float En_FC_instant;
	float lambda;
	float lambda_init;

	
};



}

