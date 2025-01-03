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

	// 制御器クラス
	Controller Ctrl(u);

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

	// // RK4シミュレーション
	// Simulator(RungeKutta<Dynamics::StateEquation>(kSmplTimeSim), Dynamics::OutputEquation(), filename);
	// std::cout << "シミュレーション実行完了" << std::endl;

	// // グラフプロット(python matplotlib流用)
	// std::cout << "グラフプロット (python matplotlib流用)" << std::endl;
	// python_graph_plotter(filename);

	// std::cout << "全処理完了" << std::endl;


	// Controller Ctrl;
	// Ctrl.PseudoPlanner();

	// // フィルタ動作チェック
	// FilterCCF ADF(0.05, 0.02, 0.01);
	// arma::vec vec_sin_curve = arma::sin(arma::regspace(0,0.01,10));
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