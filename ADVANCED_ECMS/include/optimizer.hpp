#include <iostream>
#include <vector>
#include <assert.h>
#include <eigen3/Eigen/Dense>

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
	Optimizer(float lambda, float mu, float raw, float max_iter);
	Eigen::MatrixXf Lagrangian_Costmodeling(string mode);
	Eigen::MatrixXf ADMM_Costmodeling(string mode);
	

	
};



}

