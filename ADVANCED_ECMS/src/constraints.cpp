#include <ICEMG.hpp>
#include <interp_tool.hpp>

using namespace std;
using namespace LBK;

// constraints start. //
float Tool::Wi_HEV_min() {return *std::min_element(ICE::En_mapRPM.begin(), ICE::En_mapRPM.end());}
float Tool::Wi_HEV_max() {return *std::max_element(ICE::En_mapRPM.begin(), ICE::En_mapRPM.end());}
float Tool::W1_HEV_max()  {return *std::max_element(MG::MG_maxRPM.begin(), MG::MG_maxRPM.end());}
float Tool::W1_EV_max()  {return *std::max_element(MG::MG_maxRPM.begin(), MG::MG_maxRPM.end());}

Eigen::VectorXf Tool::T1_EV_max()
{
	Eigen::VectorXf v(MG::W1_EV.size());
	for (int ind = 0; ind<v.size();ind++)
	{
		v(ind) = interp_Tool::interpolate_1d(MG::MG_maxRPM,MG::MG_maxTrq, abs(MG::W1_EV(ind)), false);
	}

	return v;
}

Eigen::MatrixXf Tool::Ti_HEV_max()
{
	Eigen::MatrixXf v(ICE::Wi_HEV_rep.rows(), ICE::Wi_HEV_rep.cols());
	for(int ind=0; ind<v.rows(); ind++)
	{
		for(int ind2=0; ind2<v.cols(); ind2++)
		{
			v(ind, ind2) = interp_Tool::interpolate_1d(ICE::En_maxRPM, ICE::En_maxTrq, ICE::Wi_HEV_rep(ind, ind2), false);
		}
	}

	return v;
}

Eigen::MatrixXf Tool::T1_HEV_max()
{
	Eigen::MatrixXf v(MG::W1_HEV.rows(), MG::W1_HEV.cols());
	for(int ind=0; ind<v.rows(); ind++)
	{
		for(int ind2=0; ind2<v.cols(); ind2++)
		{
			v(ind, ind2) = interp_Tool::interpolate_1d(MG::MG_maxRPM, MG::MG_maxTrq, abs(MG::W1_HEV(ind, ind2)), false);
		}
	}

	return v;
}
// constraints end. //