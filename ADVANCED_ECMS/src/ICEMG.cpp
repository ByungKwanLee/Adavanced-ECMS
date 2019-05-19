#include <ICEMG.hpp>
#include <interp_tool.hpp>
#include <data_loader.hpp>
#include <VehicleInfo.hpp>


using namespace std;
using namespace LBK;

// ICE configuration
vector<float> ICE::TM_Ratio{4.212, 2.637, 1.8, 1.386, 1, 0.772 };
Eigen::VectorXf ICE::TMR = Eigen::Map<Eigen::VectorXf> (ICE::TM_Ratio.data(), ICE::TM_Ratio.size()); // type casting from std to eigen

float ICE::TM_Eff = 0.9;
float ICE::FD_EFF = 0.97;
float ICE::FD_Ratio = 4.11;

vector<float> ICE::En_minRPM = Tool::rpm2rs(get_1d_data("src/ADVANCED_ECMS/data/En_minRPM"));
vector<float> ICE::En_minTrq = get_1d_data("src/ADVANCED_ECMS/data/En_minTrq");
vector<float> ICE::En_maxRPM = Tool::rpm2rs(get_1d_data("src/ADVANCED_ECMS/data/En_maxRPM"));
vector<float> ICE::En_maxTrq = get_1d_data("src/ADVANCED_ECMS/data/En_maxTrq");
vector<float> ICE::En_mapRPM = Tool::rpm2rs(get_1d_data("src/ADVANCED_ECMS/data/En_mapRPM")); //to be added
vector<float> ICE::En_mapTrq = get_1d_data("src/ADVANCED_ECMS/data/En_mapTrq");
vector<vector<float>> ICE::En_mapData = get_2d_data("src/ADVANCED_ECMS/data/En_mapData");
vector<vector<float>> ICE::En_FC = get_2d_data("src/ADVANCED_ECMS/data/En_FC");

//  battery configuration
float MG::SOC = 0.6;
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
float MG::Voc  = Tool::Voc(false);
float MG::Rint = Tool::Rint(false);

// EV of W1, T1, Eta1 estimation
Eigen::VectorXf MG::W1_EV = Tool::W1_EV(false);
Eigen::VectorXf MG::W1_EV_constr = Tool::W1_EV_constr(false);

float MG::W1_EV_max = Tool::W1_EV_max(); //constraints
Eigen::VectorXf MG::T1_EV_max = Tool::T1_EV_max(); //constraints

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


// constraints of MG
float MG::W1_HEV_max = Tool::W1_HEV_max(); //constraints

// constraints of ICE
float ICE::Wi_HEV_min = Tool::Wi_HEV_min(); //constraints
float ICE::Wi_HEV_max = Tool::Wi_HEV_max(); //constraints

