#include <iostream>
#include <fstream>
#include <cmath>
#include <cfloat>
#include <vector>
#include <array>

#include <armadillo>
#include "Eigen/Dense"

#include "param.h"
#include "controller.h"
#include "plant_model.h"


// 暫定のデバッグ用グローバル変数（注意）
extern double g_dbg_info[15];

// 疑似Planner関数
void Controller::PseudoPlanner()
{
	xp = arma::regspace(0,10,2000);
	yp = arma::ones(size(xp))*5.0; // 直線軌道
	// yp = arma::sin(xp/100-0)*5.0+5.0; // sinカーブ軌道

	// std::cout<<xp<<std::endl;
	// std::cout<<yp<<std::endl;
	return;
}

// // 偏差パラメータ計算関数
// Eigen::VectorXd CalcErrorParam()
// {
// 	Eigen::VectorXd err_param = Eigen::VectorXd::Zero(4);

// 	return err_param;
// }


// 制御入力計算関数
// x = [posx, posy, theta]^T
// u = [delta, ref_posx, ref_posy, tracking_error]^T
// y = [posx, posy, theta, vx, vy, gamma]^T
Eigen::VectorXd Controller::CalcCtrlInput(double t, Eigen::VectorXd y)
{

	double tracking_err = 0., diff_tracking_err = 0.;
	// 暫定目標軌道
	Eigen::Vector3d ref_vec;
	if(0)
	{
		// 直線
		ref_vec(0) = 5.0;
		ref_vec(1) = 0.0; // 一階微分
		ref_vec(2) = 0.0; // 二階微分
	}
	else
	{
		// sinカーブ
		ref_vec(0) = 5.0*sin(y(0)/20.0)-5.0;
		ref_vec(1) = 5.0*cos(y(0)/20.0)/20.0; // 一階微分
		ref_vec(2) = -5.0*sin(y(0)/20.0)/20.0/20.0; // 二階微分
	}
	

	// 制御偏差
	tracking_err = ref_vec(0) - y(1);

	// 制御入力設定
	if(0)
	{
		// PID
		Eigen::Vector2d adf_out = ADF(tracking_err);
		diff_tracking_err = adf_out(1);
		u(0) = ctrl_param(0)*tracking_err + ctrl_param(1)*diff_tracking_err;
		u(1) = y(0);
		u(2) = ref_vec(0);
		u(3) = tracking_err;
	}
	else
	{
		// TSCF
		double omega = 0.1;
		double zeta = 1.0;

		double f1 = 2.0*zeta*omega;
        double f2 = omega*omega;
		double eta = ref_vec(2) - f1*(tan(y(2))-ref_vec(1)) - f2*(y(1)-ref_vec(0));

		u(0) = atan2(kWheelBase*eta*cos(y(2))*cos(y(2))*cos(y(2)),1);
		u(1) = y(0);
		u(2) = ref_vec(0);
		u(3) = tracking_err;
	}

	// デバッグ情報の仮取得
	g_dbg_info[0] = 0.;
	g_dbg_info[1] = 0.;
	g_dbg_info[2] = 0.;
	g_dbg_info[3] = 0.;
	g_dbg_info[4] = 0.;
	g_dbg_info[5] = 0.;
	g_dbg_info[6] = 0.;
	g_dbg_info[7] = 0.;
	g_dbg_info[8] = 0.;
	g_dbg_info[9] = 0.;
	g_dbg_info[10] = 0.;
	g_dbg_info[11] = 0.;
	g_dbg_info[12] = 0.;
	g_dbg_info[13] = 0.;
	g_dbg_info[14] = 0.;

	// std::cout<<"control : " << t <<std::endl;
	// std::cout<<u<<std::endl;
	return u;
}
