#include <iostream>
#include <typeinfo>
#include <map>
#include <tuple>
#include <math.h>
#include <vector>
#include <assert.h>
#include <eigen3/Eigen/Dense>


using namespace std;





class Optimizer
{

private:
	float max_iter;
	

public:
	float correction;
	std::tuple<string, int, int, float> optimal_inform;
	
	Optimizer(float lambda, float mu, float max_iter);
	Eigen::MatrixXf Lagrangian_Costmodeling(string mode);
	Eigen::MatrixXf ADMM_Costmodeling(string mode);
	vector<float> minimum_EV(string method);
	vector<vector<float>> minimum_HEV(string method);
	void optimal_method(string method);
	void SOC_correction(bool update = false);
	void optimizer(string method);
	void Regenerative_optimal(string method);

	void En_FC_rt(bool print = false);
	float En_FC_sum;
	float En_FC_instant;
	float lambda;
	float lambda_init;
	float mu_init;
	float mu;
	float raw_init;
	float raw;

	
};





