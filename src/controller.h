#ifndef _CONTROLLER_H_
#define _CONTROLLER_H_

#include <iostream>
#include <cmath>
#include <vector>
#include <array>

#include <armadillo>
#include "Eigen/Dense"
#include "param.h"

// 制御器Class
class Controller
{
private:
	Eigen::VectorXd CalcCtrlInput(double t, Eigen::VectorXd); // 制御入力計算関数

public:
	Eigen::VectorXd u = Eigen::VectorXd::Zero(kNumInputU); // 入力ベクトル
	arma::vec xp; // 目標軌道列X
	arma::vec yp; // 目標軌道列Y

	// コンストラクタ
	Controller(Eigen::VectorXd u0)
	{
		u = u0;
	}
	Controller()
	{
	}

	// 関数オブジェクト
	Eigen::VectorXd operator()(double t, Eigen::VectorXd y)
	{
		return CalcCtrlInput(t, y); // 制御計算
	}

	// 制御入力モニタ
	double GetValues(int idx)
	{
		return u[idx];
	}
	Eigen::VectorXd GetValues()
	{
		return u;
	}

	// 疑似Planner
	void PseudoPlanner();

};
#endif // _CONTROLLER_H_