#include <ICEMG.hpp>
#include <interp_tool.hpp>
#include <data_loader.hpp>
#include <VehicleInfo.hpp>

using namespace std;
using namespace LBK;

// setting constant value of Tool NumGrid
const int Tool::NumGrid = 100;

// ICE configuration
vector<float> ICE::TM_Ratio{4.212, 2.637, 1.8, 1.386, 1, 0.772 };
Eigen::VectorXf ICE::TMR = Eigen::Map<Eigen::VectorXf> (ICE::TM_Ratio.data(), ICE::TM_Ratio.size()); // type casting from std to eigen

float ICE::TM_Eff = 0.97;
float ICE::FD_EFF = 0.97;
float ICE::FD_Ratio = 4.11;

vector<float> ICE::En_minRPM = get_1d_data("src/ADVANCED_ECMS/data/En_minRPM");
vector<float> ICE::En_minTrq = get_1d_data("src/ADVANCED_ECMS/data/En_minTrq");
vector<float> ICE::En_maxRPM = get_1d_data("src/ADVANCED_ECMS/data/En_maxRPM");
vector<float> ICE::En_maxTrq = get_1d_data("src/ADVANCED_ECMS/data/En_maxTrq");
vector<float> ICE::En_mapRPM = get_1d_data("src/ADVANCED_ECMS/data/En_mapRPM"); //to be added
vector<float> ICE::En_mapTrq = get_1d_data("src/ADVANCED_ECMS/data/En_mapTrq");
vector<vector<float>> ICE::En_mapData = get_2d_data("src/ADVANCED_ECMS/data/En_mapData");

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
vector<float> MG::MG_mapRPM = Tool::rpm2rs( get_1d_data("src/ADVANCED_ECMS/data/MG_mapRPM") );
vector<float> MG::MG_maxTrq = get_1d_data("src/ADVANCED_ECMS/data/MG_maxTrq");
vector<float> MG::MG_maxRPM = Tool::rpm2rs( get_1d_data("src/ADVANCED_ECMS/data/MG_maxRPM") );
vector<vector<float>> MG::MG_mapData = get_2d_data("src/ADVANCED_ECMS/data/MG_mapData");

// Voc Rint estimation
float MG::Voc  = MG::Bat_NumCell * interp_Tool::interpolate_1d(MG::Bat_indexSoc, MG::Bat_indexRint, MG::SOC, false);
float MG::Rint = MG::Bat_NumCell * interp_Tool::interpolate_1d(MG::Bat_indexSoc, MG::Bat_indexVoc,  MG::SOC, false);

// EV of W1, T1, Eta1 estimation
Eigen::VectorXf MG::W1_EV = Tool::W1_EV();
Eigen::VectorXf MG::T1_EV = Tool::T1_EV();
Eigen::VectorXf MG::Eta1_EV = Tool::Eta1_EV();

// //  EV of P estimation
Eigen::VectorXf MG::P1elec_EV = Tool::P1elec_EV();
Eigen::VectorXf MG::PbatD_EV = Tool::PbatD_EV();
Eigen::VectorXf MG::dSOC_EV = Tool::dSOC_EV();

// HEV of Wi, Ti
Eigen::VectorXf ICE::Wi_HEV = Tool::Wi_HEV(); //effective to get T_max;
Eigen::MatrixXf ICE::Wi_HEV_rep = Tool::Wi_HEV_rep(); // special
Eigen::VectorXf ICE::Ti_max = Tool::Ti_max();
Eigen::MatrixXf ICE::Ti_HEV = Tool::Ti_HEV(); // 6*100
Eigen::MatrixXf ICE::FC_HEV = Tool::FC_HEV();

// // HEV of W1 T1 Eta1
Eigen::MatrixXf MG::W1_HEV = Tool::W1_HEV();
Eigen::MatrixXf MG::T1_HEV = Tool::T1_HEV();
Eigen::MatrixXf MG::Eta1_HEV = Tool::Eta1_HEV();


// // HEV of P estimation
Eigen::MatrixXf MG::P1elec_HEV = Tool::P1elec_HEV();
Eigen::MatrixXf MG::PbatD_HEV = Tool::PbatD_HEV();
Eigen::MatrixXf MG::dSOC_HEV = Tool::dSOC_HEV();



// Tool class function
Eigen::VectorXf Tool::W1_EV() {return VehicleInfo::velocity/VehicleInfo::wheel_radius*ICE::FD_Ratio*ICE::TMR;}
Eigen::VectorXf Tool::T1_EV() {return (VehicleInfo::mass*VehicleInfo::accel + VehicleInfo::Frl()) 
	* ( VehicleInfo::wheel_radius / (ICE::FD_Ratio*ICE::FD_EFF*ICE::TM_Eff) ) * ICE::TMR.cwiseInverse();}


Eigen::MatrixXf Tool::W1_HEV() {return ICE::Wi_HEV_rep;}
Eigen::MatrixXf Tool::T1_HEV() {return MG::T1_EV.replicate<1, Tool::NumGrid>()-ICE::Ti_HEV;}


Eigen::VectorXf Tool::Wi_HEV() {return MG::W1_EV;}
Eigen::MatrixXf Tool::Wi_HEV_rep() {return ICE::Wi_HEV.replicate<1,Tool::NumGrid >();}
Eigen::MatrixXf Tool::Ti_HEV()
{

	Eigen::MatrixXf v(ICE::TMR.size(), Tool::NumGrid);
	for(int gear=0; gear<ICE::TMR.size() ;gear++)
	{	
		v.row(gear) = Eigen::VectorXf::LinSpaced(Tool::NumGrid, 0, ICE::Ti_max(gear));
	}

	return v;

}

Eigen::VectorXf Tool::Ti_max()
{
	Eigen::VectorXf v(ICE::Wi_HEV.size());
	for (int ind = 0; ind<v.size();ind++)
	{
		v(ind) = interp_Tool::interpolate_1d(ICE::En_maxRPM,ICE::En_maxTrq, ICE::Wi_HEV(ind), false);
	}

	return v;
}

///

//constraints start.

float Tool::Wi_HEV_min() {return *std::min_element(ICE::En_mapRPM.begin(), ICE::En_mapRPM.end());}
float Tool::Wi_HEV_max() {return *std::max_element(ICE::En_mapRPM.begin(), ICE::En_mapRPM.end());}
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

// constraints end.

///

Eigen::MatrixXf Tool::FC_HEV()
{

	Eigen::MatrixXf v(ICE::Wi_HEV_rep.rows(), ICE::Wi_HEV_rep.cols());

	for (int ind =0; ind < v.rows(); ind++)
	{
		for(int ind2=0; ind2 <v.cols(); ind2++){
			v(ind, ind2) = interp_Tool::interpolate_2d(ICE::En_mapRPM, ICE::En_mapTrq, ICE::En_mapData, 
				abs(ICE::Wi_HEV_rep(ind, ind2)), abs(ICE::Ti_HEV(ind, ind2)));
		}
	}
	return v;
}

Eigen::VectorXf Tool::PbatD_EV() {return MG::P1elec_EV + MG::P_aux * Eigen::VectorXf::Ones(MG::P1elec_EV.size());}
Eigen::MatrixXf Tool::PbatD_HEV() {return MG::P1elec_HEV + MG::P_aux * Eigen::MatrixXf::Ones(MG::P1elec_HEV.rows(), MG::P1elec_HEV.cols());}



Eigen::VectorXf Tool::dSOC_EV()
{
	Eigen::VectorXf v(MG::PbatD_EV.size());

	for (int ind = 0; ind<v.size(); ind++)
	{
		v(ind) =  (-MG::Voc + sqrt(pow(MG::Voc, 2)-4 * MG::PbatD_EV(ind) * MG::Rint)) / (2 * MG::Rint * MG::Bat_Quantity * 3600);
	}
	return v;
}

Eigen::MatrixXf Tool::dSOC_HEV()
{
	Eigen::MatrixXf v(MG::PbatD_HEV.rows(), MG::PbatD_HEV.cols());

	for (int ind = 0; ind<v.rows(); ind++)
	{
		for (int ind2=0;ind2<v.cols(); ind2++)
		{
			v(ind, ind2) =  (-MG::Voc + sqrt(pow(MG::Voc, 2)-4 * MG::PbatD_HEV(ind, ind2) * MG::Rint)) / (2 * MG::Rint * MG::Bat_Quantity * 3600);
		}
		
	}
	return v;
}


Eigen::VectorXf Tool::P1elec_EV()
{

	Eigen::VectorXf W1T1 = MG::W1_EV.cwiseProduct(MG::T1_EV);
	Eigen::VectorXf v(W1T1.size());	
	for (int ind =0; ind < v.size(); ind++)
	{
		int sign = (W1T1(ind) >=0) ? 1 : -1;
		v(ind) = W1T1(ind) * pow (MG::Eta1_EV(ind), sign);
	}
	return v;
}

Eigen::MatrixXf Tool::P1elec_HEV()
{

	Eigen::MatrixXf W1T1 = MG::W1_HEV.cwiseProduct(MG::T1_HEV);
	Eigen::MatrixXf v(W1T1.rows(), W1T1.cols());	
	for (int ind =0; ind < v.rows(); ind++)
	{
		for(int ind2=0; ind2<v.cols();ind2++){

			int sign = (W1T1(ind, ind2) >=0) ? 1 : -1;
			v(ind, ind2) = W1T1(ind, ind2) * pow (MG::Eta1_HEV(ind, ind2), sign);
		}

	}
	return v;
}


Eigen::VectorXf Tool::Eta1_EV()
{
	Eigen::VectorXf v(MG::W1_EV.size());

	for (int ind =0; ind < v.size(); ind++)
	{
		v(ind) = interp_Tool::interpolate_2d(MG::MG_mapRPM, MG::MG_mapTrq, MG::MG_mapData, 
						abs(MG::W1_EV(ind)), abs(MG::T1_EV(ind)));
	}
	return v;
}

Eigen::MatrixXf Tool::Eta1_HEV()
{

	Eigen::MatrixXf v(MG::W1_HEV.rows(), MG::W1_HEV.cols());

	for (int ind =0; ind < v.rows(); ind++)
	{
		for(int ind2=0; ind2 <v.cols(); ind2++){
			v(ind, ind2) = interp_Tool::interpolate_2d(MG::MG_mapRPM, MG::MG_mapTrq, MG::MG_mapData, 
				abs(MG::W1_HEV(ind, ind2)), abs(MG::T1_HEV(ind, ind2)));
		}
	}
	return v;
}

vector<float> Tool::rpm2rs(vector<float> rpm)
{
	std::for_each(rpm.begin(), rpm.end(), [](float& x) {x*=M_PI/30;});
	return rpm;
}