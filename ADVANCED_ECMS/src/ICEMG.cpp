#include <ICEMG.hpp>
#include <data_loader.hpp>

using namespace std;
using namespace LBK;

//  motor and battery configuration
float MG::SOC=0.6;
float MG::P_bat = 32000;
float MG::Bat_NumCell = 168;
float MG::Bat_Quantity = 6.5;

vector<float> MG::Bat_indexVoc = get_1d_data("src/ADVANCED_ECMS/data/Bat_indexVoc");
vector<float> MG::Bat_indexSoc = get_1d_data("src/ADVANCED_ECMS/data/Bat_indexSOC");
vector<float> MG::Bat_indexRint = get_1d_data("src/ADVANCED_ECMS/data/Bat_indexRint");
vector<vector<float>> MG::MG_mapData = get_2d_data("src/ADVANCED_ECMS/data/MG_mapData");

float MG::Rint = Est_Tool::interpolate_1d(MG::Bat_indexSoc, MG::Bat_indexVoc,  MG::SOC, false);
float MG::Voc  = Est_Tool::interpolate_1d(MG::Bat_indexSoc, MG::Bat_indexRint, MG::SOC, false);

float MG::SOC_rate()
{
	float Voc = MG::Voc;
	float Rint = MG::Rint;
	float P_bat = MG::P_bat;
	return (-Voc + sqrt(pow(MG::Voc,2)-4*MG::P_bat*MG::Rint))/(2*MG::Rint*MG::Bat_Quantity);
}

float Est_Tool::interpolate_1d(vector<float> & xData, vector<float> & yData, 
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