#include "matplotlibcpp.h"
#include <vector>
#include <cmath>

#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "Eigen/Dense"

#include "../src/param.h"

namespace plt = matplotlibcpp;

// lineをdelimiterで分割する関数
std::vector<std::string> split(std::string &line, char delimiter)
{
    std::istringstream stream(line);
    std::string field;
    std::vector<std::string> result;
    while (getline(stream, field, delimiter))
    {
        result.push_back(field);
    }
    return result;
}

// 2次元STLコンテナからEigen行列に変換する関数
template <class S>
Eigen ::Matrix<S, Eigen ::Dynamic, Eigen ::Dynamic>
vector2matrix(const std ::vector<std ::vector<S>> &in)
{
    unsigned int x = in.size();
    unsigned int y = in[0].size();
    std ::vector<S> t;
    t.reserve(x * y);
    for (auto i : in)
    {
        t.insert(t.end(), i.begin(), i.end());
    }
    return (Eigen ::Map<Eigen ::Matrix<S, Eigen ::Dynamic, Eigen ::Dynamic>>(&t[0], y, x)).transpose();

    /*
    // vectorにコピー
	Eigen::VectorXd eig_a = Eigen::VectorXd::Zero(5);
	std::vector<double> a(5);
	memcpy(&a, &eig_a(0), sizeof(double)*eig_a.size());
    */
}

// グラフプロット関数
int python_graph_plotter(std::string filename)
{

    // CSV読み込み
    std::vector<std::vector<double>> data;
    std::ifstream ifs(filename);

    if (ifs)
    {
        std::string line;

        // 一行目がラベルの場合
        // getline(ifs, line);
        // std::vector<std::string> str_vec = split(line, ',');

        while (getline(ifs, line))
        {
            std::vector<double> data_vec;
            std::vector<std::string> str_vec = split(line, ',');
            for (auto &&s : str_vec)
            {
                data_vec.push_back(std::stod(s));
            }
            data.push_back(data_vec);
        }
    }

    // Eigen行列に差し替え
    Eigen::MatrixXd data_matrix = vector2matrix(data);
    // std::cout << data_matrix.col(0) << std::endl;

    // ステアリミット
    Eigen::Vector2d tmp_time = {data_matrix(0, 0), data_matrix(data_matrix.rows() - 1, 0)};
    Eigen::Vector2d tmp_str_up = {kLimSteerAngle, kLimSteerAngle};
    Eigen::Vector2d tmp_str_lo = -tmp_str_up;
    // std::cout << data_matrix(data_matrix.rows()-1,0) << std::endl;

    // グラフ描画 : 走行軌道
    plt::figure();
    plt::plot(data_matrix.col(5), data_matrix.col(6), {{"label", "ref"}, {"color", "g"}, {"linestyle", "--"}, {"linewidth", "2.0"}});
    plt::plot(data_matrix.col(1), data_matrix.col(2), {{"label", "act"}, {"color", "r"}, {"linestyle", "-"}});
    plt::title("Vehicle Trajectory");
    plt::legend();
    plt::xlabel("X [m]");
    plt::ylabel("Y [m]");
    plt::grid(1);

    // グラフ描画 : 状態・入力
    plt::figure();
    plt::suptitle("States, Inputs");
    plt::subplot(2, 2, 1);
    plt::plot(data_matrix.col(0), data_matrix.col(1));
    plt::ylabel("posx [m]");
    plt::xlabel("time [s]");
    plt::grid(1);
    plt::subplot(2, 2, 2);
    plt::plot(data_matrix.col(0), data_matrix.col(2));
    plt::ylabel("posy [m]");
    plt::xlabel("time [s]");
    plt::grid(1);
    plt::subplot(2, 2, 3);
    plt::plot(data_matrix.col(0), data_matrix.col(3));
    plt::ylabel("theta [rad]");
    plt::xlabel("time [s]");
    plt::grid(1);
    plt::subplot(2, 2, 4);
    plt::plot(data_matrix.col(0), data_matrix.col(4));
    plt::plot(tmp_time, tmp_str_up, "r--");
    plt::plot(tmp_time, tmp_str_lo, "r--");
    plt::ylabel("steer [rad]");
    plt::xlabel("time [s]");
    plt::grid(1);

    // グラフ描画 : 制御誤差
    plt::figure();
    plt::suptitle("Control states");
    plt::subplot(2, 2, 1);
    plt::plot(data_matrix.col(0), data_matrix.col(7));
    plt::ylabel("tracking error [m]");
    plt::xlabel("time [s]");
    plt::grid(1);
    plt::subplot(2, 2, 2);
    plt::plot(data_matrix.col(0), data_matrix.col(12));
    plt::ylabel("vy [m/s]");
    plt::xlabel("time [s]");
    plt::grid(1);
    plt::subplot(2, 2, 3);
    plt::plot(data_matrix.col(0), data_matrix.col(13));
    plt::ylabel("gamma [rad/s]");
    plt::xlabel("time [s]");
    plt::grid(1);
    plt::subplot(2, 2, 4);
    plt::plot(data_matrix.col(0), data_matrix.col(4));
    plt::plot(tmp_time, tmp_str_up, "r--");
    plt::plot(tmp_time, tmp_str_lo, "r--");
    plt::ylabel("steer [rad]");
    plt::xlabel("time [s]");
    plt::grid(1);

    plt::show();

    return 0;
}