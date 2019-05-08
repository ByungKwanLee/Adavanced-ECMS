#include <iostream>
#include <VehicleInfo.hpp>
using namespace std;


namespace LBK
{


class OptimizerUtils
{

private:
	float lambda;
	float tau;
	float iter;
	float Costmodeling();
	

public:
	OptimizerUtils();
	OptimizerUtils(float lambda, float tau, float iter);
	
	float Hamiltonian();
	void Efficient_optimizer();



};

}