#include <interp_tool.hpp>
#include <vector>

using namespace std;
using namespace LBK;

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


float interp_Tool::interpolate_2d(vector<float> & xData, vector<float> & yData, 
	vector<vector<float>> & zData, float x, float y)
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



	return (interpol_x + interpol_y) /2;

}