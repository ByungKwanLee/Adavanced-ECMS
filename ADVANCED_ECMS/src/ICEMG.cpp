#include <ICEMG.hpp>
#include <interp_tool.hpp>
#include <data_loader.hpp>
#include <VehicleInfo.hpp>

using namespace std;
using namespace LBK;

// ICE configuration
vector<float> ICE::TM_Ratio{4.212, 2.637, 1.8, 1.386, 1, 0.772 };
Eigen::VectorXf ICE::TMR = Eigen::Map<Eigen::VectorXf> (ICE::TM_Ratio.data(), ICE::TM_Ratio.size()); // type casting from std to eigen

float ICE::TM_Eff = 0.97;
float ICE::FD_EFF = 0.97;
float ICE::FD_Ratio = 4.11;

vector<float> En_minRPM = get_1d_data("src/ADVANCED_ECMS/data/En_minRPM");
vector<float> En_minTrq = get_1d_data("src/ADVANCED_ECMS/data/En_minTrq");
vector<float> En_maxRPM = get_1d_data("src/ADVANCED_ECMS/data/En_maxRPM");
vector<float> En_maxTrq = get_1d_data("src/ADVANCED_ECMS/data/En_maxTrq");

vector<float> En_mapTrq = get_1d_data("src/ADVANCED_ECMS/data/En_mapTrq");
vector<vector<float>> En_mapData = get_2d_data("src/ADVANCED_ECMS/data/En_mapData");

// float Ti_max = interp_Tool::interpolate_1d();





//  battery configuration
float MG::SOC=0.6;
float MG::P_bat = 32000;
float MG::Bat_NumCell = 168;
float MG::Bat_Quantity = 6.5;
float MG::P_aux = 400; // [V]

vector<float> MG::Bat_indexVoc = get_1d_data("src/ADVANCED_ECMS/data/Bat_indexVoc");
vector<float> MG::Bat_indexSoc = get_1d_data("src/ADVANCED_ECMS/data/Bat_indexSOC");
vector<float> MG::Bat_indexRint = get_1d_data("src/ADVANCED_ECMS/data/Bat_indexRint");

// Motor map
vector<float> MG::MG_mapTrq = get_1d_data("src/ADVANCED_ECMS/data/MG_mapTrq");
vector<float> MG::MG_mapRPM = MG::rpm2rs( get_1d_data("src/ADVANCED_ECMS/data/MG_mapRPM") );
vector<float> MG::MG_maxTrq = get_1d_data("src/ADVANCED_ECMS/data/MG_maxTrq");
vector<float> MG::MG_maxRPM = MG::rpm2rs( get_1d_data("src/ADVANCED_ECMS/data/MG_maxRPM") );
vector<vector<float>> MG::MG_mapData = get_2d_data("src/ADVANCED_ECMS/data/MG_mapData");

// Voc Rint estimation
float MG::Voc  = MG::Bat_NumCell * interp_Tool::interpolate_1d(MG::Bat_indexSoc, MG::Bat_indexRint, MG::SOC, false);
float MG::Rint = MG::Bat_NumCell * interp_Tool::interpolate_1d(MG::Bat_indexSoc, MG::Bat_indexVoc,  MG::SOC, false);

// EV of W1, T1, Eta1 estimation
Eigen::VectorXf MG::W1_EV = VehicleInfo::velocity/VehicleInfo::wheel_radius*ICE::FD_Ratio*ICE::TMR;
Eigen::VectorXf MG::T1_EV = (VehicleInfo::mass*VehicleInfo::accel + VehicleInfo::Frl())
					*(VehicleInfo::wheel_radius
						/(ICE::FD_Ratio*ICE::FD_EFF)
						/ICE::TM_Eff) * ICE::TMR.cwiseInverse();
Eigen::VectorXf MG::Eta1_EV = MG::Eta1_func(MG::W1_EV, MG::T1_EV);


// //  EV of P estimation
Eigen::VectorXf MG::P1elec_EV = MG::power(MG::W1_EV, MG::T1_EV, MG::Eta1_EV);

// float MG::PbatD_EV = MG::P1elec_EV + MG::P_aux;
// float MG::dSOC_EV = (-MG::Voc + sqrt(pow(MG::Voc, 2)-4 * MG::PbatD_EV * MG::Rint)) / (2 * MG::Rint * MG::Bat_Quantity * 3600);

// // HEV of W1 T1 Eta1
// float MG::W1_HEV = MG::W1_EV;
// // float MG::T1_HEV = //repmat;
// float MG::Eta1_HEV = interp_Tool::interpolate_2d(MG::MG_mapRPM, MG::MG_mapTrq, MG::MG_mapData, 
// 						abs(MG::W1_HEV), abs(MG::T1_HEV));

// // HEV of P estimation
// float MG::P1elec_HEV = MG::W1_HEV * MG::T1_HEV * pow(MG::Eta1_HEV, (MG::W1_HEV*MG::T1_HEV>=0) ? 1 : -1);
// float MG::PbatD_HEV = MG::P1elec_HEV + MG::P_aux;
// float MG::dSOC_HEV = (-MG::Voc + sqrt(pow(MG::Voc, 2)-4 * MG::PbatD_HEV * MG::Rint)) / (2 * MG::Rint * MG::Bat_Quantity * 3600);

Eigen::VectorXf MG::power(Eigen::VectorXf W1, Eigen::VectorXf T1, Eigen::VectorXf Eta1)
{

	Eigen::VectorXf W1T1 = W1.cwiseProduct(T1);
	Eigen::VectorXf v(6);	
	for (int ind =0; ind < v.size(); ind++)
	{
		float sign =(W1T1(ind) >=0) ? 1 : -1;
		v(ind) = W1T1(ind) * pow (Eta1(ind), sign);
	}
	return v;
}

// estimate eta function using interpolation 2d function
Eigen::VectorXf MG::Eta1_func(Eigen::VectorXf W1, Eigen::VectorXf T1)
{
	Eigen::VectorXf v(6);

	for (int ind =0; ind < v.size(); ind++)
	{
		v(ind) = interp_Tool::interpolate_2d(MG::MG_mapRPM, MG::MG_mapTrq, MG::MG_mapData, 
						abs(W1(ind)), abs(T1(ind)));
		ind+=1;
	}
	return v;
}

// rpm 2 rs function
vector<float> MG::rpm2rs(vector<float> rpm)
{
	std::for_each(rpm.begin(), rpm.end(), [](float& x) {x*=M_PI/30;});
	return rpm;
}