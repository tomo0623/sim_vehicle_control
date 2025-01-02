#ifndef _DYNAMICS_H_
#define _DYNAMICS_H_

#include <iostream>
#include <cmath>
#include <vector>
#include <array>

#include "Eigen/Dense"
#include "param.h"

// ダイナミクスClass(関数オブジェクト)
class Dynamics
{
public:
	class StateEquation
	{
	private:
		Eigen::VectorXd CalcSEQ(double, Eigen::VectorXd, Eigen::VectorXd); // 状態方程式計算関数
	public:
		Eigen::VectorXd operator()(double t, Eigen::VectorXd x, Eigen::VectorXd u)
		{
			return CalcSEQ(t, x, u);
		}
	};
	class OutputEquation
	{
	private:
		Eigen::VectorXd CalcOEQ(double, Eigen::VectorXd, Eigen::VectorXd); // 出力方程式計算関数
	public:
		Eigen::VectorXd operator()(double t, Eigen::VectorXd x, Eigen::VectorXd u)
		{
			return CalcOEQ(t, x, u);
		}
	};
};

#endif // _DYNAMICS_H_