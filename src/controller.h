#ifndef _CONTROLLER_H_
#define _CONTROLLER_H_

#include <iostream>
#include <cmath>
#include <vector>
#include <array>

#include <armadillo>
#include "Eigen/Dense"
#include "param.h"

#include "filterCCF.h"

//
enum CtrlMode
{
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
	Eigen::VectorXd CalcCtrlInputTSCF_by_s(double t, Eigen::VectorXd); // 制御入力計算関数

public:
	Eigen::Vector2d ctrl_param = Eigen::VectorXd::Zero(2); // 制御パラメータ
	Eigen::VectorXd u = Eigen::VectorXd::Zero(kNumInputU); // 入力ベクトル
	arma::vec xp;										   // 目標軌道列X
	arma::vec yp;										   // 目標軌道列Y
	arma::vec curvature;								   // 目標軌道曲率
	arma::vec angle;									   // 目標軌道方向角
	arma::vec length;									   // 目標軌道累積距離
	Eigen::VectorXd outNNS = Eigen::VectorXd::Zero(5);	   // 最近傍点探索パラメータ(最近傍補間参照点との偏差, 最近傍補間IDX, 直行条件, ベクトル方向確認, reserve)
	Eigen::VectorXd ref_param = Eigen::VectorXd::Zero(6);  // 最近傍点参照パラメータ(偏差, 方向角, 曲率, 累積距離, 参照座標X, 参照座標Y)
	FilterCCF ADF;
	enum CtrlMode mode;

	// コンストラクタ
	Controller(Eigen::VectorXd u0, CtrlMode user_mode = PID_LINE, std::vector<double> user_param = {0., 0.})
	{
		u = u0;

		mode = user_mode;
		std::cout << "ctrl mode : " << mode << std::endl;
		ctrl_param(0) = user_param[0];
		ctrl_param(1) = user_param[1];
		// ctrl_param(0) = 0.0001; // PID仮調整
		// ctrl_param(1) = 0.01; // PID仮調整
		std::cout << "ctrl param : " << ctrl_param.transpose() << std::endl;

		// 疑似プランナー・軌道パラメータ計算実行(TSCF-時間軸：走行距離版用処理)
		PseudoPlanner();
		CalcTrajectoryParams();
	}
	Controller()
	{
	}

	// 関数オブジェクト
	Eigen::VectorXd operator()(double t, Eigen::VectorXd y)
	{
		if(mode<4)
		{
			return CalcCtrlInput(t, y); // 制御計算(教材用)
		}
		else
		{
			return CalcCtrlInputTSCF_by_s(t, y); // 制御計算(状態量s：走行距離に関する運動学モデルベースのTSCF)
		}
		
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

	// 軌道パラメータ計算
	void CalcTrajectoryParams();

	// 参照パラメータ計算
	void CalcReferenceParams(const double, const double);

	// 制御パラメータ設定
	void SetCtrlParam(Eigen::Vector2d user_param)
	{
		ctrl_param = user_param;
	}
};
#endif // _CONTROLLER_H_