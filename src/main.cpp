//------------------------------------------------------------------------
#include <iostream>
#include <fstream>
#include <iomanip>
#include <cmath>
#include <cfloat>
#include <vector>
#include <array>
#include <string>

#include <armadillo>
#include "Eigen/Dense"

#include "param.h"
#include "runge_kutta.h"
#include "dynamics.h"
#include "controller.h"
#include "plant_model.h"

#include "FilterCCF.h"


// 暫定のデバッグ用グローバル変数（注意）
double g_dbg_info[15] = {0.};

// 関数プロトタイプ宣言
int python_graph_plotter(std::string);

// ユーザー設定取得部
void GetUserSetting(int& user_mode, std::vector<double>& user_param)
{
	// 制御モード取得
	std::cout << "制御モード選択 -> 0:PID-直線追従, 1:PID-SIN追従, 2:TSCF-直線追従, 3:TSCF-SIN追従" << std::endl;
    std::cin >> user_mode;
	while(std::cin.fail() || user_mode < 0 || user_mode > 3)
    {
		if (std::cin.fail())
		{
			std::cin.clear();
			std::cin.ignore(256, '\n');
			std::cout << "数値入力してください" << std::endl;
			std::cin >> user_mode;
		}
		else if(user_mode < 0 || user_mode > 3) 
		{
			std::cout << "0~3の整数値入力をしてください" << '\n';
			std::cin >> user_mode;
		}
    }
    std::cout << "制御モード : " << user_mode << std::endl;

	// 制御パラメータ取得
	if(user_mode<2)
	{
		std::cout << "制御パラメータ設定(スペース区切り) -> Pゲイン Dゲイン" << std::endl;
	}
	else
	{
		std::cout << "制御パラメータ設定(スペース区切り) -> 固有角周波数ω 減衰係数ζ" << std::endl;
	}

	std::cin >> user_param[0] >> user_param[1];
	while(std::cin.fail() || user_param[0] < 0 || user_param[1] < 0)
    {
		if (std::cin.fail())
		{
			std::cin.clear();
			std::cin.ignore(256, '\n');
			std::cout << "数値入力してください" << std::endl;
			std::cin >> user_param[0] >> user_param[1];
		}
		else if(user_param[0] < 0 || user_param[1] < 0) 
		{
			std::cout << "非負の数値を入力してください" << '\n';
			std::cin >> user_param[0] >> user_param[1];
		}
    }
	std::cout << "制御パラメータ : " << user_param[0] << "," << user_param[1] << std::endl;
}

// シミュレーション処理メイン部
// x = [posx, posy, theta]^T
// u = [delta, ref_posx, ref_posy, tracking_error]^T
// y = [posx, posy, theta, vx, vy, gamma]^T
template <class Method, class OEQ>
void Simulator(Method f, OEQ h, std::string filename)
{

	// SIM変数
	double t = 0;
	std::ofstream ofs(filename);
	std::cout << filename << std::endl;

	// 状態・入力変数
	Eigen::VectorXd x = Eigen::VectorXd::Zero(kNumStateX);
	Eigen::VectorXd u = Eigen::VectorXd::Zero(kNumInputU);
	Eigen::VectorXd y = Eigen::VectorXd::Zero(kNumOutputY);

	// CUIで制御設定変更
	int user_mode = 0;
	std::vector<double> user_param(2);
	GetUserSetting(user_mode, user_param);

	// 制御器クラス
	Controller Ctrl(u, static_cast<CtrlMode>(user_mode), user_param);
	// Controller Ctrl(u, static_cast<CtrlMode>(1));
	
	// 状態/出力初期化 x0 = [posx, posy, theta]^T
	x << 0., 0., 0. / kR2D;
	y = h(t, x, u);

	// シミュレーション処理
	for (int i = 0; i < kNumMaxSimCount; i++)
	{
		// 時間更新
		t += kSmplTimeSim;

		// 制御計算
		if (i % kNumSimCtrl == 0)
		{
			u = Ctrl(t, y);
		}

		// ルンゲクッタ更新
		x = f(t, x, u);
		y = h(t, x, u);

		// ロギング
		ofs << t << ",";
		for (auto tmp : x)
		{
			ofs << std::scientific << std::setprecision(15) << tmp << ",";
		}
		for (auto tmp : u)
		{
			ofs << std::scientific << std::setprecision(15) << tmp << ",";
		}
		for (auto tmp : y)
		{
			ofs << std::scientific << std::setprecision(15) << tmp << ",";
		}
		// dbg
		for (auto tmp : g_dbg_info)
		{
			ofs << std::scientific << std::setprecision(15) << tmp << ",";
		}

		ofs << std::endl;
	}
}

//------------------------------------------------------------------------
int main(void)
{
	std::string filename = "simout.csv";

	// RK4シミュレーション
	Simulator(RungeKutta<Dynamics::StateEquation>(kSmplTimeSim), Dynamics::OutputEquation(), filename);
	std::cout << "シミュレーション実行完了" << std::endl;

	// グラフプロット(python matplotlib流用)
	std::cout << "グラフプロット (python matplotlib流用)" << std::endl;
	python_graph_plotter(filename);

	std::cout << "全処理完了" << std::endl;


	// 疑似プランナー動作チェック
	// Controller Ctrl;
	// Ctrl.PseudoPlanner();

	// // フィルタ動作チェック
	// FilterCCF ADF(0.05, 0.02, 0.01);
	// arma::vec vec_sin_curve = arma::sin(arma::regspace(0,0.01,10))*5.0;
	// Eigen::Vector2d filter_output;
	// std::ofstream ofs("filter_test.csv");
	// for(int i=0; i < vec_sin_curve.size(); i++)
	// {
	// 	// std::cout << vec_sin_curve[i] << std::endl;
	// 	filter_output = ADF(vec_sin_curve[i]);
	// 	ofs << vec_sin_curve[i] << ",";
	// 	ofs << filter_output[0] << ",";
	// 	ofs << filter_output[1] << std::endl;
	// }

}