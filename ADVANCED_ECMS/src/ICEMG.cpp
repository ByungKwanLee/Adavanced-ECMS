#include <iostream>
#include <math.h>
#include <ICEMG.hpp>
#include <eigen3/Eigen/Dense>

using namespace std;
using namespace LBK;

// battery configuration
float MG::Qmax = 3200;

MG::MG(float Voc, float Rint, float P_bat)
{
	MG::Voc=Voc;
	MG::Rint=Rint;
	MG::P_bat=P_bat;
}

float MG::SOC_rate()
{
	float Voc = MG::Voc;
	float Rint = MG::Rint;
	float P_bat = MG::P_bat;
	return (-Voc + sqrt(pow(Voc,2)-4*P_bat*Rint))/(2*Rint*MG::Qmax);
}