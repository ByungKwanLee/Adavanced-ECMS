#include <iostream>

using namespace std;


namespace LBK{


class OptimizerUtils
{

private:
	float lambda;
	float tau;
	float iter;
	float Costmodeling();
	float parameter[3];

public:
	static float Qmax;
	OptimizerUtils(){};
	OptimizerUtils(float lambda, float tau, float iter);
	float F_rl(float velocity, float* parameter);
	float SOC_penalty(float soc, float soc_L, float soc_H);
	float SOC_rate(float Voc, float Rint, float P_bat);
	float Hamiltonian();
	void Efficient_optimizer();



};

};