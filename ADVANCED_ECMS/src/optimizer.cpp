#include <iostream>
#include <math.h>
#include <optimizer.hpp>
#include <ICEMG.hpp>


using namespace std;
using namespace LBK;

template <typename T>
struct NaN_include
{
  bool operator() (T a, T b) const
  {
    if (std::isnan(a))
    {
      return false; // Assume NaN is greater than *any* non-NaN value.
    }
    if (std::isnan(b))
    {
      return true; // Assume *any* non-NaN value is smaller than NaN.
    }
    return (a < b);
  }
};


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


float * Optimizer::minimum_EV(string method)
{
	assert( (method == "L" || method == "ADMM" ) && "Choose in [""L"", ""ADMM""]" );

	Eigen::VectorXf v;
	if( method == "L" )
	{
		v = Optimizer::Lagrangian_Costmodeling("EV");
	}
	else
	{
		v = Optimizer::ADMM_Costmodeling("EV");
	}

	vector<float> v_(v.data(), v.data()+v.size());
	vector<float>::iterator v_iterator = std::min_element(v_.begin(), v_.end(), NaN_include<float>());

	// Dynamic Allocation
	float * output = new float[2];
	output[0] = *v_iterator;
	output[1] = int(v_iterator - v_.begin());
	return output;
}

float ** Optimizer::minimum_HEV(string method)
{
	assert( (method == "L" || method == "ADMM" ) && "Choose in [""L"", ""ADMM""]" );

	Eigen::MatrixXf v;
	if (method=="L")
	{
		v = Optimizer::Lagrangian_Costmodeling("HEV");

	}
	else
	{
		v = Optimizer::ADMM_Costmodeling("HEV");
	}
	
	// Dynamic Allocation
	float ** output = new float * [v.rows()];
	for(int ind=0; ind < v.rows(); ind++)
	{
		output[ind] = new float[2];	
	}

	for(int ind=0; ind < v.rows(); ind++)
	{
		Eigen::VectorXf row_vec = v.row(ind);
		vector<float> row_std(row_vec.data(), row_vec.data() + row_vec.size());
		vector<float>::iterator v_min_row = std::min_element(row_std.begin(), row_std.end(), NaN_include<float>());
		int v_min_ind =  v_min_row - row_std.begin();
		output[ind][0] = *v_min_row;
		output[ind][1] = v_min_ind;
	}

	return output;
}