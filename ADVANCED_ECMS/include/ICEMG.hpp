#include <iostream>
#include <eigen3/Eigen/Dense>
#include <math.h>
#include <vector>

namespace LBK
{

class ICE
{

public:
	float ICE;





};

class MG
{

public:

	float Voc;
	float Rint;
	float P_bat;
	static float Qmax;
	static float SOC;

	MG();
	MG(float Voc, float Rint, float P_bat);
	float SOC_rate();


};


}