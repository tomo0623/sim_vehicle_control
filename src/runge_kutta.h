#ifndef _RUNGEKUTTA_H_
#define _RUNGEKUTTA_H_

#include <iostream>
#include <cmath>
#include <vector>
#include <array>

#include "Eigen/Dense"
#include "param.h"

//------------------------------------------------------------------------
template <class DIFF>
class RungeKutta
{
private:
	const double dt;
	const double hdt;
	DIFF diff;

public:
	RungeKutta(double _dt) : dt(_dt), hdt(_dt * 0.5) {};
	Eigen::VectorXd operator()(double t, Eigen::VectorXd x, Eigen::VectorXd u)
	{
		Eigen::VectorXd k1(kNumStateX), k2(kNumStateX), k3(kNumStateX), k4(kNumStateX);
		k1 = diff(t, x, u);
		k2 = diff(t + hdt, x + k1 * hdt, u);
		k3 = diff(t + hdt, x + k2 * hdt, u);
		k4 = diff(t, x + k3 * dt, u);
		x = x + (k1 + k2 * 2.0 + k3 * 2.0 + k4) * (dt / 6.0);
		return x;
	}
};
#endif // _RUNGEKUTTA_H_