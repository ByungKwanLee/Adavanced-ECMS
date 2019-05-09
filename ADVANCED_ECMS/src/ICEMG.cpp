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

vector<float> MG::MG_mapTrq = get_1d_data("src/ADVANCED_ECMS/data/MG_mapTrq");
vector<float> MG::MG_mapRPM = get_1d_data("src/ADVANCED_ECMS/data/MG_mapRPM");
vector<float> MG::MG_maxTrq = get_1d_data("src/ADVANCED_ECMS/data/MG_maxTrq");
vector<float> MG::MG_maxRPM = get_1d_data("src/ADVANCED_ECMS/data/MG_maxRPM");
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


float* interp_Tool::interpolate_2d(vector<float> & xData, vector<float> & yData, vector<vector<float>> & zData, 
	float x, float y)
{
	int x_size = xData.size();
	int y_size = yData.size();

	vector<float> xData_(xData.begin(), xData.end()), yData_(yData.begin(), yData.end());
	std::for_each(xData_.begin(), xData_.end(), [x](float& d) { d-=x; d=abs(d);});
	std::for_each(yData_.begin(), yData_.end(), [y](float& d) { d-=y; d=abs(d);});

	int min_ind_x    =  std::min_element(xData_.begin(), xData_.end()) - xData_.begin();
	float min_data_x = *std::min_element(xData_.begin(), xData_.end());
	xData_[min_ind_x]=pow(2,100);
	int sec_ind_x    =  std::min_element(xData_.begin(), xData_.end()) - xData_.begin();
	float sec_data_x = *std::min_element(xData_.begin(), xData_.end());

	int min_ind_y    =  std::min_element(yData_.begin(), yData_.end()) - yData_.begin();
	float min_data_y = *std::min_element(yData_.begin(), yData_.end());
	yData_[min_ind_y]=pow(2,100);
	int sec_ind_y    =  std::min_element(yData_.begin(), yData_.end()) - yData_.begin();
	float sec_data_y = *std::min_element(yData_.begin(), yData_.end());

	float interpol_y =min_data_x*(min_data_y*zData[min_ind_x][min_ind_y] + sec_data_y*zData[min_ind_x][sec_ind_y])/(min_data_y+sec_data_y) 
	+ sec_data_x*(min_data_y*zData[sec_ind_x][min_ind_y]  + sec_data_y*zData[sec_ind_x][sec_ind_y])/(min_data_y+sec_data_y);
	interpol_y/=(min_data_x+sec_data_x);

	float interpol_x = min_data_y*(min_data_x*zData[min_ind_x][min_ind_y]  + sec_data_x*zData[sec_ind_x][min_ind_y])/(min_data_x+sec_data_x)
	+ sec_data_y*(min_data_x*zData[min_ind_x][sec_ind_y]  + sec_data_x*zData[sec_ind_x][sec_ind_y])/(min_data_x+sec_data_x);
	interpol_x/=(min_data_y+sec_data_y);

	float *p = new float[2];
	p[0] = interpol_x;
	p[1] = interpol_y;

	return p;

}