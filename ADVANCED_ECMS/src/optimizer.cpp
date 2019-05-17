#include <iostream>
#include <math.h>
#include <optimizer.hpp>
#include <ICEMG.hpp>


using namespace std;
using namespace LBK;


Optimizer::Optimizer(float mu, float raw, float max_iter)
{
	Optimizer::mu = mu;
	Optimizer::raw = raw;
	Optimizer::max_iter = max_iter;
}

Eigen::MatrixXf Optimizer::ADMM_Costmodeling()
{
	Eigen::MatrixXf H = MG::dSOC_HEV + Optimizer::mu * Eigen::MatrixXf::Ones(MG::dSOC_HEV.rows(), MG::dSOC_HEV.cols());
	return ICE::FC_HEV + Optimizer::raw * H.cwiseProduct(H);
}

void Optimizer::ADMM_Optimizer()
{
	
}

