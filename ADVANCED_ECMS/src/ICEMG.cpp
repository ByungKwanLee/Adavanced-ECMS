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

float ICE::TM_Eff = 0.9;
float ICE::FD_EFF = 0.97;
float ICE::FD_Ratio = 4.11;

vector<float> ICE::En_minRPM = get_1d_data("src/ADVANCED_ECMS/data/En_minRPM");
vector<float> ICE::En_minTrq = get_1d_data("src/ADVANCED_ECMS/data/En_minTrq");
vector<float> ICE::En_maxRPM = get_1d_data("src/ADVANCED_ECMS/data/En_maxRPM");
vector<float> ICE::En_maxTrq = get_1d_data("src/ADVANCED_ECMS/data/En_maxTrq");
vector<float> ICE::En_mapRPM = get_1d_data("src/ADVANCED_ECMS/data/En_mapRPM"); //to be added
vector<float> ICE::En_mapTrq = get_1d_data("src/ADVANCED_ECMS/data/En_mapTrq");
vector<vector<float>> ICE::En_mapData = get_2d_data("src/ADVANCED_ECMS/data/En_mapData");
vector<vector<float>> ICE::En_FC = get_2d_data("src/ADVANCED_ECMS/data/En_FC");

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
Eigen::VectorXf MG::W1_EV = Tool::W1_EV(false);
Eigen::VectorXf MG::T1_EV = Tool::T1_EV(false);
Eigen::VectorXf MG::Eta1_EV = Tool::Eta1_EV(false);

// EV of P estimation
Eigen::VectorXf MG::P1elec_EV = Tool::P1elec_EV(false);
Eigen::VectorXf MG::PbatD_EV = Tool::PbatD_EV(false);
Eigen::VectorXf MG::dSOC_EV = Tool::dSOC_EV(false);

// HEV of Wi, Ti
Eigen::VectorXf ICE::Wi_HEV = Tool::Wi_HEV(false); //effective to get T_max;
Eigen::MatrixXf ICE::Wi_HEV_rep = Tool::Wi_HEV_rep(false); // special
Eigen::VectorXf ICE::Ti_max = Tool::Ti_max(false);
Eigen::MatrixXf ICE::Ti_HEV = Tool::Ti_HEV(false); // 6*100
Eigen::MatrixXf ICE::FC_HEV = Tool::FC_HEV(false);

// HEV of W1 T1 Eta1
Eigen::MatrixXf MG::W1_HEV = Tool::W1_HEV(false);
Eigen::MatrixXf MG::T1_HEV = Tool::T1_HEV(false);
Eigen::MatrixXf MG::Eta1_HEV = Tool::Eta1_HEV(false);

// HEV of P estimation
Eigen::MatrixXf MG::P1elec_HEV = Tool::P1elec_HEV(false);
Eigen::MatrixXf MG::PbatD_HEV = Tool::PbatD_HEV(false);
Eigen::MatrixXf MG::dSOC_HEV = Tool::dSOC_HEV(false);



// Tool class function

// EV of W1, T1, Eta1 estimation
Eigen::VectorXf Tool::W1_EV(bool update) 
{
	Eigen::VectorXf v;
	v = VehicleInfo::velocity/VehicleInfo::wheel_radius*ICE::FD_Ratio*ICE::TMR;
	if(update) MG::W1_EV = v;
	return v;
 } // nothing makes effect


Eigen::VectorXf Tool::T1_EV(bool update) {

	Eigen::VectorXf v;
	v = (VehicleInfo::mass*VehicleInfo::accel + VehicleInfo::Frl())  
	* ( VehicleInfo::wheel_radius / (ICE::FD_Ratio*ICE::FD_EFF*ICE::TM_Eff) ) * ICE::TMR.cwiseInverse();
	if(update) MG::T1_EV = v;
	return v;
} //nothing makes effect


Eigen::VectorXf Tool::Eta1_EV(bool update)
{
	Eigen::VectorXf v(MG::W1_EV.size());

	for (int ind =0; ind < v.size(); ind++)
	{
		v(ind) = interp_Tool::interpolate_2d(MG::MG_mapRPM, MG::MG_mapTrq, MG::MG_mapData, 
						abs(MG::W1_EV(ind)), abs(MG::T1_EV(ind)));
	}

	if(update) MG::Eta1_EV = v;
	return v;
} // W1_EV, T1_EV makes effect


// EV of P estimation
Eigen::VectorXf Tool::P1elec_EV(bool update)
{

	Eigen::VectorXf W1T1 = MG::W1_EV.cwiseProduct(MG::T1_EV);
	Eigen::VectorXf v(W1T1.size());	
	for (int ind =0; ind < v.size(); ind++)
	{
		int sign = (W1T1(ind) >=0) ? 1 : -1;
		v(ind) = W1T1(ind) * pow (MG::Eta1_EV(ind), sign);
	}

	if(update) MG::P1elec_EV = v;
	return v;
} // W1_EV, T1_EV, Eta1_EV makes effect

Eigen::VectorXf Tool::PbatD_EV(bool update)
{
	Eigen::VectorXf v;
	v = MG::P1elec_EV + MG::P_aux * Eigen::VectorXf::Ones(MG::P1elec_EV.size());
	if(update) MG::PbatD_EV = v;
	return v;

} // P1elec_EV makes effect

Eigen::VectorXf Tool::dSOC_EV(bool update)
{
	Eigen::VectorXf v(MG::PbatD_EV.size());

	for (int ind = 0; ind<v.size(); ind++)
	{
		v(ind) =  (-MG::Voc + sqrt(pow(MG::Voc, 2)-4 * MG::PbatD_EV(ind) * MG::Rint)) / (2 * MG::Rint * MG::Bat_Quantity * 3600);
	}

	if(update) MG::dSOC_EV = v;

	return v;
} // PbatD_EV makes effect


// HEV of Wi, Ti
Eigen::VectorXf Tool::Wi_HEV(bool update)
{
	Eigen::VectorXf v;
	v = MG::W1_EV;
	if(update) ICE::Wi_HEV = v;
	return v;
} // W1_EV makes effect

Eigen::MatrixXf Tool::Wi_HEV_rep(bool update)
{
	Eigen::MatrixXf v;
	v = ICE::Wi_HEV.replicate<1,Tool::NumGrid >();
	if(update) ICE::Wi_HEV_rep = v;
	return v;
} // W1_HEV makes effect

Eigen::VectorXf Tool::Ti_max(bool update)
{
	Eigen::VectorXf v(ICE::Wi_HEV.size());
	for (int ind = 0; ind<v.size();ind++)
	{
		v(ind) = interp_Tool::interpolate_1d(ICE::En_maxRPM,ICE::En_maxTrq, ICE::Wi_HEV(ind), false);
	}

	if(update) ICE::Ti_max = v;
	return v;
} //Wi_HEV makes effect

Eigen::MatrixXf Tool::Ti_HEV(bool update)
{

	Eigen::MatrixXf v(ICE::TMR.size(), Tool::NumGrid);
	for(int gear=0; gear<ICE::TMR.size() ;gear++)
	{	
		v.row(gear) = Eigen::VectorXf::LinSpaced(Tool::NumGrid, 0, ICE::Ti_max(gear));
	}

	if(update) ICE::Ti_HEV = v;
	return v;
} // Ti_max makes effect

Eigen::MatrixXf Tool::FC_HEV(bool update)
{

	Eigen::MatrixXf v(ICE::Wi_HEV_rep.rows(), ICE::Wi_HEV_rep.cols());

	for (int ind =0; ind < v.rows(); ind++)
	{
		for(int ind2=0; ind2 <v.cols(); ind2++){
			v(ind, ind2) = interp_Tool::interpolate_2d(ICE::En_mapRPM, ICE::En_mapTrq, ICE::En_FC, 
				ICE::Wi_HEV_rep(ind, ind2), ICE::Ti_HEV(ind, ind2));
		}
	}

	if(update) ICE::FC_HEV = v;
	return v;
} //Wi_HEV_rep, Ti_HEV makes effect



// HEV of W1 T1 Eta1
Eigen::MatrixXf Tool::W1_HEV(bool update) 
{
	Eigen::MatrixXf v;
	v = ICE::Wi_HEV_rep;
	if(update) MG::W1_HEV = v;
	return v;
} // Wi_HEV_rep makes effect

Eigen::MatrixXf Tool::T1_HEV(bool update) 
{
	Eigen::MatrixXf v;
	v = MG::T1_EV.replicate<1, Tool::NumGrid>()-ICE::Ti_HEV;
	if(update) MG::T1_HEV = v;
	return v;
} // T1_EV, Ti_HEV makes effect

Eigen::MatrixXf Tool::Eta1_HEV(bool update)
{

	Eigen::MatrixXf v(MG::W1_HEV.rows(), MG::W1_HEV.cols());

	for (int ind =0; ind < v.rows(); ind++)
	{
		for(int ind2=0; ind2 <v.cols(); ind2++){
			v(ind, ind2) = interp_Tool::interpolate_2d(MG::MG_mapRPM, MG::MG_mapTrq, MG::MG_mapData, 
				abs(MG::W1_HEV(ind, ind2)), abs(MG::T1_HEV(ind, ind2)));
		}
	}
	if(update) MG::Eta1_HEV = v;
	return v;
} //W1_HEV, T1_HEV makes effect



// HEV of P estimation
Eigen::MatrixXf Tool::P1elec_HEV(bool update)
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
	if(update) MG::P1elec_HEV = v;
	return v;
} //W1_HEV, T1_HEV, Eta1_HEV makes effect


Eigen::MatrixXf Tool::PbatD_HEV(bool update)
{
	Eigen::MatrixXf v;
	v = MG::P1elec_HEV + MG::P_aux * Eigen::MatrixXf::Ones(MG::P1elec_HEV.rows(), MG::P1elec_HEV.cols());
	if(update) MG::PbatD_HEV = v;
	return v;
} // PbatD_HEV makes effect


Eigen::MatrixXf Tool::dSOC_HEV(bool update)
{
	Eigen::MatrixXf v(MG::PbatD_HEV.rows(), MG::PbatD_HEV.cols());

	for (int ind = 0; ind<v.rows(); ind++)
	{
		for (int ind2=0;ind2<v.cols(); ind2++)
		{
			v(ind, ind2) =  (-MG::Voc + sqrt(pow(MG::Voc, 2)-4 * MG::PbatD_HEV(ind, ind2) * MG::Rint)) / (2 * MG::Rint * MG::Bat_Quantity * 3600);
		}
		
	}
	if(update) MG::dSOC_HEV = v;
	return v;
} // PbatD_HEV makes effect




// constraints start. //

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
// constraints end. //






// fundamental tool
vector<float> Tool::rpm2rs(vector<float> rpm)
{
	std::for_each(rpm.begin(), rpm.end(), [](float& x) {x*=M_PI/30;});
	return rpm;
}