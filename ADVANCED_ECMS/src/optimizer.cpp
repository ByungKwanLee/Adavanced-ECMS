#include <iostream>
#include <math.h>
#include <optimizer.hpp>
#include <eigen3/Eigen/Dense>

using namespace std;
using namespace LBK;

Optimizer::Optimizer(float lambda, float tau, float iter)
{
	Optimizer::lambda = lambda;
	Optimizer::tau = tau;
	Optimizer::iter = iter;
}

