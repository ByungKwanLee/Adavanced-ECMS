#include <iostream>
#include <vector>
#include <eigen3/Eigen/Dense>

using namespace std;


namespace LBK
{


class Optimizer
{

private:
	float mu;
	float raw;
	float max_iter;
	

public:
	Optimizer(float mu, float raw, float max_iter);
	Eigen::MatrixXf ADMM_Costmodeling();
	static void ADMM_Optimizer();
	
};



}

