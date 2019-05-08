#include <iostream>

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
	
	MG(float Voc, float Rint, float P_bat);
	float SOC_rate();


};


}