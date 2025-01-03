#ifndef _CONTROLLER_H_
#define _CONTROLLER_H_

#include <iostream>
#include <cmath>
#include <vector>
#include <array>

#include <armadillo>
#include "Eigen/Dense"
#include "param.h"

#include "FilterCCF.h"

// 
enum CtrlMode {
	PID_LINE,
	PID_SIN,
	TSCF_LINE,
	TSCF_SIN
};

// 制御器Class
class Controller
{
private:
	Eigen::VectorXd CalcCtrlInput(double t, Eigen::VectorXd); // 制御入力計算関数

public:
	Eigen::Vector2d ctrl_param = Eigen::VectorXd::Zero(2); // 制御パラメータ
	Eigen::VectorXd u = Eigen::VectorXd::Zero(kNumInputU); // 入力ベクトル
	arma::vec xp; // 目標軌道列X
	arma::vec yp; // 目標軌道列Y
	FilterCCF ADF;
	enum CtrlMode mode;
	

	// コンストラクタ
	Controller(Eigen::VectorXd u0, CtrlMode user_mode = PID_LINE, std::vector<double> user_param = {0.,0.})
	{
		u = u0;

		mode = user_mode;
		std::cout << "ctrl mode : " << mode << std::endl;
		ctrl_param(0) = user_param[0];
		ctrl_param(1) = user_param[1];
		// ctrl_param(0) = 0.0001; // PID仮調整
		// ctrl_param(1) = 0.01; // PID仮調整
		std::cout << "ctrl param : " << ctrl_param.transpose() << std::endl;
		
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

	// 制御パラメータ設定
	void SetCtrlParam(Eigen::Vector2d user_param)
	{
		ctrl_param = user_param;
	}

};
#endif // _CONTROLLER_H_