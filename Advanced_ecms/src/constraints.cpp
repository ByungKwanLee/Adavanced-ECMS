#include <ICEMG.hpp>
#include <interp_tool.hpp>

using namespace std;


float Tool::Wi_HEV_min() {return *std::min_element(ICE::En_mapRPM.begin(), ICE::En_mapRPM.end());}
float Tool::Wi_HEV_max() {return *std::max_element(ICE::En_mapRPM.begin(), ICE::En_mapRPM.end());}
float Tool::W1_HEV_max()  {return *std::max_element(MG::MG_maxRPM.begin(), MG::MG_maxRPM.end());}
float Tool::W1_EV_max()  {return *std::max_element(MG::MG_maxRPM.begin(), MG::MG_maxRPM.end());}

Eigen::VectorXf Tool::T1_EV_max(bool update)
{
	Eigen::VectorXf v(MG::W1_EV.size());

	#pragma omp pararrel for
	for (int ind = 0; ind<v.size();ind++)
	{
		v(ind) = interp_Tool::interpolate_1d(MG::MG_maxRPM,MG::MG_maxTrq, abs(MG::W1_EV_constr(ind)), false);
	}

	if(update) MG::T1_EV_max = v;

	return v;
}