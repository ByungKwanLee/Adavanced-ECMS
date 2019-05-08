#include <iostream>
#include <math.h>
#include <optimizer.hpp>
#include <eigen3/Eigen/Dense>

using namespace std;
using namespace LBK;




OptimizerUtils::OptimizerUtils(float lambda, float tau, float iter)
{
	OptimizerUtils::lambda = lambda;
	OptimizerUtils::tau = tau;
	OptimizerUtils::iter = iter;
	OptimizerUtils::Qmax = Qmax;
}


float OptimizerUtils::Costmodeling()
{
// using lagrangian method cost and ADMM cost
}



