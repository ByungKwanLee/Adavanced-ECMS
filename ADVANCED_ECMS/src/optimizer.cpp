#include <iostream>
#include <math.h>
#include <optimizer.hpp>
#include <ICEMG.hpp>
#include <VehicleInfo.hpp>

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

struct min_by_value
{
	bool operator() (std::pair<int, float> a, std::pair<int, float> b) 
	{
		if (std::isnan(a.second))
	    {
	      return false; // Assume NaN is greater than *any* non-NaN value.
	    }
	    if (std::isnan(b.second))
	    {
	      return true; // Assume *any* non-NaN value is smaller than NaN.
	    }
		return a.second < b.second;
	}
};


Optimizer::Optimizer(float lambda, float mu, float raw, float max_iter)
{
	Optimizer::lambda_init = lambda;
	Optimizer::mu_init = mu;
	Optimizer::raw_init = raw;
	Optimizer::max_iter = max_iter;
	Optimizer::En_FC_instant = Optimizer::En_FC_sum = 0;
}


Eigen::MatrixXf Optimizer::Lagrangian_Costmodeling(string mode)
{
	assert( (mode == "EV" || mode == "HEV" ) && "Choose in [""EV"", ""HEV""]" );
	
	if (mode == "EV")
	{
		return Optimizer::lambda *MG::dSOC_EV;
	}
	else
	{
		return ICE::FC_HEV + Optimizer::lambda  * MG::dSOC_HEV;
	}
}


Eigen::MatrixXf Optimizer::ADMM_Costmodeling(string mode)
{
	assert( (mode == "EV" || mode =="HEV" ) && "Choose in [""EV"", ""HEV""]" );

	if (mode == "EV" )
	{
		Eigen::VectorXf H =  MG::dSOC_EV + Optimizer::mu * Eigen::VectorXf::Ones(MG::dSOC_EV.size());
		return Optimizer::raw * H.cwiseProduct(H);
	}
	else
	{
		Eigen::MatrixXf H = MG::dSOC_HEV + Optimizer::mu * Eigen::MatrixXf::Ones(MG::dSOC_HEV.rows(), MG::dSOC_HEV.cols());
		return ICE::FC_HEV + Optimizer::raw * H.cwiseProduct(H);	
	}
	
}


vector<float> Optimizer::minimum_EV(string method)
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

	vector<float> output(v.data(), v.data()+v.size());
	return output;
}

vector<vector<float>> Optimizer::minimum_HEV(string method)
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

	vector<vector<float>> output(2, vector<float>(v.rows()));
	vector<float>::iterator v_min_row;
	vector<float> A, B;
	for(int ind=0; ind < v.rows(); ind++)
	{
		Eigen::VectorXf row_vec = v.row(ind);
		vector<float> row_std(row_vec.data(), row_vec.data() + row_vec.size());
		v_min_row = std::min_element(row_std.begin(), row_std.end(), NaN_include<float>());
		A.push_back(v_min_row - row_std.begin());
		B.push_back(*v_min_row);
	}

	for(vector<vector<float>>::iterator it = output.begin(); it != output.end(); it++)
	{
			(*it) = it == output.begin() ? A : B;
	}
	return output;
}

void Optimizer::Regenerative_optimal(string method)
{
	assert( (method == "L" || method == "ADMM" ) && "Choose in [""L"", ""ADMM""]" );
	vector<float> minimum_EV;
	if(method == "L")
	{
		// L method
		minimum_EV  = Optimizer::minimum_EV("L");
	}
	else
	{
		// ADMM method
		minimum_EV  = Optimizer::minimum_EV("ADMM");
	}


	vector<float>::iterator  min_EV_Regen = std::min_element(minimum_EV.begin(), minimum_EV.end(), NaN_include<float>());
	int min_EV_Regen_ind = min_EV_Regen - minimum_EV.begin();

	assert( isnan(*min_EV_Regen) == 0 && "No cost and No soc, too much demand");


	std::tuple<string, int, int, float> minimum_pair("EV",  
		min_EV_Regen_ind + 1, -2, *min_EV_Regen);

	Optimizer::optimal_inform = minimum_pair;
}

void Optimizer::optimal_method(string method)
{
	assert( (method == "L" || method == "ADMM" ) && "Choose in [""L"", ""ADMM""]" );

	vector<float> minimum_EV;
	vector<vector<float>> minimum_HEV;

	if(method == "L")
	{
		// L method
		minimum_EV  = Optimizer::minimum_EV("L");
		minimum_HEV = Optimizer::minimum_HEV("L");
	}
	else
	{
		// ADMM method
		minimum_EV  = Optimizer::minimum_EV("ADMM");
		minimum_HEV = Optimizer::minimum_HEV("ADMM");
	}

	vector<float> minimum_HEV_ind=minimum_HEV[0];
	vector<float> minimum_HEV_val=minimum_HEV[1];
	vector<vector<float>> minimum_ALL(minimum_EV.size(), vector<float>(2));
	for(vector<vector<float>>::iterator it = minimum_ALL.begin(); it != minimum_ALL.end(); it++)
	{
		std::vector<float> v{minimum_EV[it-minimum_ALL.begin()], minimum_HEV_val[it-minimum_ALL.begin()]};
		(*it) = v;
	}

	std::vector<float> cost_min; // cost min (1,2,3,4,5,6)
	std::vector<float> mode; // EV or HEV (1,2,3,4,5,6)
	std::vector<float>::iterator min_row_ind_iter; // index iterator
	for(vector<vector<float>>::iterator it = minimum_ALL.begin(); it!=minimum_ALL.end(); it++)
	{
		min_row_ind_iter = std::min_element((*it).begin(), (*it).end(), NaN_include<float>());
		mode.push_back(min_row_ind_iter-(*it).begin());
		cost_min.push_back(*min_row_ind_iter);
	}

	int cost_min_min_ind = std::min_element(cost_min.begin(), cost_min.end(), NaN_include<float>()) - cost_min.begin();

	assert( isnan(cost_min[cost_min_min_ind]) == 0 && "No cost and No soc, too much demand" );

	if( mode[cost_min_min_ind] == 0 ) // EV
	{
		std::tuple<string, int, int, float> minimum_pair("EV",  
			cost_min_min_ind + 1, -1, cost_min[cost_min_min_ind]);
		Optimizer::optimal_inform=minimum_pair;
	}
	else
	{
		std::tuple<string, int, int, float> minimum_pair("HEV",  
			cost_min_min_ind + 1, minimum_HEV_ind[cost_min_min_ind], cost_min[cost_min_min_ind]);
		Optimizer::optimal_inform=minimum_pair;
	}
}

void Optimizer::SOC_correction(bool update)
{
	Optimizer::correction = std::get<0>(Optimizer::optimal_inform) == "EV" ? 
	MG::dSOC_EV(std::get<1>(Optimizer::optimal_inform)-1) : 
	MG::dSOC_HEV(std::get<1>(Optimizer::optimal_inform)-1, std::get<2>(Optimizer::optimal_inform));

	if(update) MG::SOC = MG::SOC + correction * VehicleInfo::time_rt;
}


void Optimizer::En_FC_rt(bool print)
{
	Optimizer::En_FC_instant = std::get<0>(Optimizer::optimal_inform) == "EV" ? 0 : 
	ICE::FC_HEV(std::get<1>(Optimizer::optimal_inform)-1, std::get<2>(Optimizer::optimal_inform));

	assert(Optimizer::En_FC_instant >= 0 && "No En_FC should be positive");

	Optimizer::En_FC_sum += Optimizer::En_FC_instant;
	if (print) cout << "[Sum/instant] Engine FC : "<< 
		Optimizer::En_FC_sum<<", "
		<<Optimizer::En_FC_instant << endl<<endl;
}

void Optimizer::optimizer(string method)
{

	assert( (method == "L" || method == "ADMM" ) && "Choose in [""L"", ""ADMM""]" );

	Optimizer::lambda = Optimizer::lambda_init;
	Optimizer::mu = Optimizer::mu_init;
	Optimizer::raw = Optimizer::raw_init;

	if(method == "L")
	{
		for(int ind = 0; ind < Optimizer::max_iter; ind ++)
		{
			if( VehicleInfo::accel > 0 )
			{
				Optimizer::optimal_method("L");
			}
			else
			{
				Optimizer::Regenerative_optimal("L");
			}
			Optimizer::SOC_correction();
			Optimizer::lambda += MG::Bat_Quantity * 3600 * Optimizer::correction;	
		}
		Optimizer::SOC_correction(true);
		return;
	}
	else
	{
		
		for(int ind = 0; ind < Optimizer::max_iter; ind ++)
		{
			if( VehicleInfo::accel > 0 )
			{
				Optimizer::optimal_method("ADMM");
			}
			else
			{
				Optimizer::Regenerative_optimal("ADMM");
			}
			Optimizer::SOC_correction();
			Optimizer::mu += Optimizer::correction;
			Optimizer::raw = 1.2*Optimizer::raw;
		}
		Optimizer::SOC_correction(true);
		return;
		
	}
}