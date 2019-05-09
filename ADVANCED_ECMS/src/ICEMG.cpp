#include <ICEMG.hpp>
#include <data_loader.hpp>

using namespace std;
using namespace LBK;

// ICE configuration
float ICE::TM_Ratio = 4.11;
float ICE::TM_Eff = 0.97;
float FD_EFF = 0.97;
float FD_Ratio = 4.11;

//  motor and battery configuration
float MG::SOC=0.6;
float MG::P_bat = 32000;
float MG::Bat_NumCell = 168;
float MG::Bat_Quantity = 6.5;

vector<float> MG::Bat_indexVoc = get_1d_data("src/ADVANCED_ECMS/data/Bat_indexVoc");
vector<float> MG::Bat_indexSoc = get_1d_data("src/ADVANCED_ECMS/data/Bat_indexSOC");
vector<float> MG::Bat_indexRint = get_1d_data("src/ADVANCED_ECMS/data/Bat_indexRint");

vector<float> MG_mapTrq = get_1d_data("src/ADVANCED_ECMS/data/MG_mapTrq");
vector<float> MG_mapRPM = get_1d_data("src/ADVANCED_ECMS/data/MG_mapRPM");
vector<float> MG_maxTrq = get_1d_data("src/ADVANCED_ECMS/data/MG_maxTrq");
vector<float> MG_maxRPM = get_1d_data("src/ADVANCED_ECMS/data/MG_maxRPM");
vector<vector<float>> MG::MG_mapData = get_2d_data("src/ADVANCED_ECMS/data/MG_mapData");

float MG::Rint = MG::Bat_NumCell * interp_Tool::interpolate_1d(MG::Bat_indexSoc, MG::Bat_indexVoc,  MG::SOC, false);
float MG::Voc  = MG::Bat_NumCell * interp_Tool::interpolate_1d(MG::Bat_indexSoc, MG::Bat_indexRint, MG::SOC, false);

float MG::SOC_rate()
{
	float Voc = MG::Voc;
	float Rint = MG::Rint;
	float P_bat = MG::P_bat;
	return (-Voc + sqrt(pow(MG::Voc,2)-4*MG::P_bat*MG::Rint))/(2*MG::Rint*MG::Bat_Quantity);
}

float interp_Tool::interpolate_1d(vector<float> & xData, vector<float> & yData, 
	float x, bool extrapolation)
{
   int size = xData.size();

   int i = 0;
   if ( x >= xData[size - 2] )
   {
      i = size - 2;
   }
   else
   {
      while ( x > xData[i+1] ) i++;
   }
   float xL = xData[i], yL = yData[i], xR = xData[i+1], yR = yData[i+1];
   if ( !extrapolation )
   {
      if ( x < xL ) yR = yL;
      if ( x > xR ) yL = yR;
   }

   float dydx = ( yR - yL ) / ( xR - xL );

   return yL + dydx * ( x - xL );
}


float interp_Tool::interpolate_2d(vector<float> & xData, vector<float> & yData, vector<vector<float>> & zData, 
	vector<float> x, vector<float> y)
{
   int x_size = xData.size();
   int y_size = yData.size();
   int min_ind    =  std::min_element(xData.begin(), xData.end()) - xData.begin();
   float min_data = *std::min_element(xData.begin(), xData.end());


}