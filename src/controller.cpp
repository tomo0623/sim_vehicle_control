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

// 関数プロトタイプ宣言
void NNS(arma::vec, arma::vec, double, double, Eigen::VectorXd &);

// 疑似Planner関数
void Controller::PseudoPlanner()
{
	xp = arma::regspace(0, 5, 2000);
	// yp = arma::ones(size(xp)) * 5.0; // 直線軌道
	yp = arma::sin(xp / 20 - 0) * 5.0 - 5.0; // sinカーブ軌道

	// std::cout<<xp<<std::endl;
	// std::cout<<yp<<std::endl;
	return;
}

// 軌道パラメータ計算
void Controller::CalcTrajectoryParams()
{
	// 目標軌道の各ベクトルの1階/2階微分の計算(diff取る前に要素コピーでvec拡大)
	arma::vec x = xp;
	arma::vec y = yp;
	arma::vec dx = arma::diff(x);
	arma::vec dy = arma::diff(y);
	if (1)
	{
		// 後方拡張
		dx.insert_rows(dx.size(), 1);
		dx(dx.size() - 1) = dx(dx.size() - 2);
		dy.insert_rows(dy.size(), 1);
		dy(dy.size() - 1) = dy(dy.size() - 2);
	}
	else
	{
		// 前方拡張
		dx.insert_rows(0, 1);
		dx(0) = dx(1);
		dy.insert_rows(0, 1);
		dy(0) = dy(1);
	}
	arma::vec dx2 = arma::diff(dx);
	arma::vec dy2 = arma::diff(dy);
	if (1)
	{
		// 後方拡張
		dx2.insert_rows(dx2.size(), 1);
		dx2(dx2.size() - 1) = dx2(dx2.size() - 2);
		dy2.insert_rows(dy2.size(), 1);
		dy2(dy2.size() - 1) = dy2(dy2.size() - 2);
	}
	else
	{
		// 前方拡張
		dx2.insert_rows(0, 1);
		dx2(0) = dx2(1);
		dy2.insert_rows(0, 1);
		dy2(0) = dy2(1);
	}

	// 軌道方向角算出
	arma::vec theta = arma::atan2(dy, dx);
	angle = theta;

	// 軌道曲率算出
	arma::vec kappa(dx2.size(), arma::fill::zeros);
	for (int i = 0; i < kappa.size(); i++)
	{
		kappa(i) = (dx(i) * dy2(i) - dy(i) * dx2(i)) / std::pow(dx(i) * dx(i) + dy(i) * dy(i), 3.0 / 2.0);
	}
	curvature = kappa;

	// 累積距離
	arma::vec s(dx.size(), arma::fill::zeros);
	for (int i = 1; i < s.size(); i++)
	{
		s(i) = s(i - 1) + sqrt(dx(i) * dx(i) + dy(i) * dy(i));
	}
	length = s;

	// std::cout<<theta<<std::endl;
	// std::cout<<kappa<<std::endl;
	// std::cout<<length<<std::endl;

	return;
}

// 参照パラメータ計算
void Controller::CalcReferenceParams(const double xk, const double yk)
{
	// 目標軌道の最近傍点を探索
	NNS(xp, yp, xk, yk, outNNS);

	// 最近傍点補間IDXから目標軌道の方向角/曲率/累積距離/座標を補間
	arma::vec t = arma::regspace(0, xp.size() - 1);
	arma::vec x_interp(1), y_interp(1);
	arma::vec kappa_interp(1), theta_interp(1), s_interp(1);
	arma::vec t_search(1);
	t_search(0) = outNNS(1);
	arma::interp1(t, angle, t_search, theta_interp);
	arma::interp1(t, curvature, t_search, kappa_interp);
	arma::interp1(t, length, t_search, s_interp);
	arma::interp1(t, xp, t_search, x_interp);
	arma::interp1(t, yp, t_search, y_interp);

	// 出力セット
	ref_param(0) = outNNS(0);
	ref_param(1) = theta_interp(0);
	ref_param(2) = kappa_interp(0);
	ref_param(3) = s_interp(0);
	ref_param(4) = x_interp(0);
	ref_param(5) = y_interp(0);

	// std::cout << outNNS(1) << std::endl;
	// std::cout << ref_param << std::endl;

	return;
}

// 制御入力計算関数
// x = [posx, posy, theta]^T
// u = [delta, ref_posx, ref_posy, tracking_error]^T
// y = [posx, posy, theta, vx, vy, gamma]^T
Eigen::VectorXd Controller::CalcCtrlInput(double t, Eigen::VectorXd y)
{

	double tracking_err = 0., diff_tracking_err = 0.;
	// 暫定目標軌道
	Eigen::Vector3d ref_vec;
	if (mode == PID_LINE || mode == TSCF_LINE)
	{
		// 直線
		ref_vec(0) = 5.0;
		ref_vec(1) = 0.0; // 一階微分
		ref_vec(2) = 0.0; // 二階微分
	}
	else
	{
		// sinカーブ
		ref_vec(0) = 5.0 * sin(y(0) / 20.0) - 5.0;
		ref_vec(1) = 5.0 * cos(y(0) / 20.0) / 20.0;			// 一階微分
		ref_vec(2) = -5.0 * sin(y(0) / 20.0) / 20.0 / 20.0; // 二階微分
	}

	// 制御偏差
	tracking_err = ref_vec(0) - y(1);

	// 制御入力設定
	if (mode == PID_LINE || mode == PID_SIN)
	{
		// PID
		Eigen::Vector2d adf_out = ADF(tracking_err);
		diff_tracking_err = adf_out(1);
		u(0) = ctrl_param(0) * tracking_err + ctrl_param(1) * diff_tracking_err;
		u(1) = y(0);
		u(2) = ref_vec(0);
		u(3) = tracking_err;
	}
	else
	{
		// TSCF
		// double omega = 0.1;
		// double zeta = 1.0;
		double omega = ctrl_param(0);
		double zeta = ctrl_param(1);

		double f1 = 2.0 * zeta * omega;
		double f2 = omega * omega;
		double eta = ref_vec(2) - f1 * (tan(y(2)) - ref_vec(1)) - f2 * (y(1) - ref_vec(0));

		u(0) = atan2(kWheelBase * eta * cos(y(2)) * cos(y(2)) * cos(y(2)), 1);
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
