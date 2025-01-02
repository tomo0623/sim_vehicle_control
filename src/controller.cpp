#include <iostream>
#include <fstream>
#include <cmath>
#include <cfloat>
#include <vector>
#include <array>

#include "Eigen/Dense"

#include "param.h"
#include "controller.h"
#include "plant_model.h"

// 暫定のデバッグ用グローバル変数（注意）
extern double g_dbg_info[15];

// 制御入力計算関数
// x = [posx, posy, theta]^T
// u = [delta, ref_posx, ref_posy, tracking_error]^T
// y = [posx, posy, theta, vx, vy, gamma]^T
Eigen::VectorXd Controller::CalcCtrlInput(double t, Eigen::VectorXd y)
{

	// 制御入力設定
	u(0) = 5. / kR2D;
	u(1) = 0.;
	u(2) = 0.;
	u(3) = 0.;

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
