#include <iostream>
#include <math.h>
#include <optimizer.hpp>
#include <ICEMG.hpp>


using namespace std;
using namespace LBK;


Optimizer::Optimizer(float lambda, float mu, float raw, float max_iter)
{
	Optimizer::lambda = lambda;
	Optimizer::mu = mu;
	Optimizer::raw = raw;
	Optimizer::max_iter = max_iter;
}

Eigen::MatrixXf Optimizer::Lagrangian_Costmodeling(string mode)
{
	assert( (mode == "EV" || mode == "HEV" ) && "Choose in [""EV"", ""HEV""]" );
	
	if (mode == "EV")
	{
		return Optimizer::lambda * (MG::Bat_Quantity*3600) *MG::dSOC_EV;
	}
	else
	{
		return ICE::FC_HEV + Optimizer::lambda *  (MG::Bat_Quantity*3600) * MG::dSOC_HEV;
	}
}


Eigen::MatrixXf Optimizer::ADMM_Costmodeling(string mode)
{
	assert( (mode == "EV" || mode =="HEV" ) && "Choose in [""EV"", ""HEV""]" );

	if (mode == "EV" )
	{
		Eigen::VectorXf H = (MG::Bat_Quantity * 3600) * MG::dSOC_EV + Optimizer::mu * Eigen::VectorXf::Ones(MG::dSOC_EV.size());
		return Optimizer::raw * H.cwiseProduct(H);
	}
	else
	{
		Eigen::MatrixXf H = (MG::Bat_Quantity * 3600) * MG::dSOC_HEV + Optimizer::mu * Eigen::MatrixXf::Ones(MG::dSOC_HEV.rows(), MG::dSOC_HEV.cols());
		return ICE::FC_HEV + Optimizer::raw * H.cwiseProduct(H);	
	}
	
}