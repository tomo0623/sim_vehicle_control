// nelder_mead法による最近傍点探索

#include <iostream>
#include <fstream>
#include <cmath>
#include <cfloat>
#include <vector>
#include <array>

#include <armadillo>
#include "Eigen/Dense"

#include "nelder-mead.h"

// 評価関数構造体(グローバル変数)
class ObjectiveParam
{
public:
    arma::vec xp;
    arma::vec yp;
    double xk;
    double yk;
};
ObjectiveParam g_ObjParam;

// 評価関数
double objective_func(const std::vector<double> &xopt)
{
    // 評価値
    double res = 0.0;

    // 探索解における内挿座標を計算
    arma::vec t = arma::regspace(0, g_ObjParam.xp.size() - 1);
    arma::vec x_interp(1), y_interp(1);
    arma::vec t_search(1);

    t_search(0) = xopt[0];
    arma::interp1(t, g_ObjParam.xp, t_search, x_interp);
    arma::interp1(t, g_ObjParam.yp, t_search, y_interp);

    // ノルム誤差
    res = pow(x_interp(0) - g_ObjParam.xk, 2) + pow(y_interp(0) - g_ObjParam.yk, 2);

    // ステップモニター
    // std::cout << "objective_func : " << res << std::endl;
    // std::cout << "x_int : " << x_interp(0) << std::endl;
    // std::cout << "y_int : " << y_interp(0) << std::endl;

    return res;
}

// 探索処理
void NNS(arma::vec xp, arma::vec yp, double xk, double yk, Eigen::VectorXd &outNNS)
{

    // 探索用Class設定
    g_ObjParam.xp = xp;
    g_ObjParam.yp = yp;
    g_ObjParam.xk = xk;
    g_ObjParam.yk = yk;

    // X座標をベースに探索初期値(浮動小数IDX)を設定
    arma::vec diff_xpos = g_ObjParam.xp - g_ObjParam.xk;
    diff_xpos = arma::abs(diff_xpos);
    // std::cout << diff_xpos.index_min() << std::endl;
    // std::cout << obj.xp(diff_xpos.index_min()) << std::endl;

    // 探索初期解
    std::vector<double> x0 = std::vector<double>{double(diff_xpos.index_min())};

    // 探索処理
    std::vector<double> xopt = nelder_mead::find_min(objective_func, x0, false, {}, 0.001, 0.001);
    // std::cout << "x0 : " << x0[0] <<  std::endl;
    // std::cout << "xopt : " << xopt[0] <<  std::endl;

    // 内挿補間IDXによる最近傍参照点
    arma::vec t = arma::regspace(0, g_ObjParam.xp.size() - 1);
    arma::vec x_interp(1), y_interp(1);
    arma::vec t_search(1);
    t_search(0) = double(xopt[0]);
    arma::interp1(t, g_ObjParam.xp, t_search, x_interp);
    arma::interp1(t, g_ObjParam.yp, t_search, y_interp);
    // std::cout << x_interp(0) <<  std::endl;
    // std::cout << y_interp(0) <<  std::endl;

    // 直行条件/ベクトル方向
    double x_tar, y_tar, dxp, dyp, xo2k, yo2k;
    double cond_dot, cond_cross, dist;
    int idxL, idxU;

    idxL = std::floor(xopt[0]);

    if (xopt[0] - idxL < 1e-3)
    {
        x_tar = g_ObjParam.xp(idxL);
        y_tar = g_ObjParam.yp(idxL);
        dxp = g_ObjParam.xp(idxL + 1) - x_tar;
        dyp = g_ObjParam.yp(idxL + 1) - y_tar;
    }
    else
    {
        x_tar = x_interp(0);
        y_tar = y_interp(0);
        dxp = g_ObjParam.xp(std::ceil(xopt[0])) - x_tar;
        dyp = g_ObjParam.yp(std::ceil(xopt[0])) - y_tar;
    }

    xo2k = g_ObjParam.xk - x_tar;
    yo2k = g_ObjParam.yk - y_tar;
    cond_dot = dxp * xo2k + dyp * yo2k;   // 直行条件
    cond_cross = dxp * yo2k - xo2k * dyp; // ベクトル方向確認

    // 最近傍補間距離
    dist = sqrt(pow(x_interp(0) - g_ObjParam.xk, 2) + pow(y_interp(0) - g_ObjParam.yk, 2));
    dist = dist * arma::sign(dist);
    // std::cout << dist <<  std::endl;

    // 出力セット
    outNNS(0) = dist;            // 最近傍補間参照点との偏差
    outNNS(1) = double(xopt[0]); // 最近傍補間IDX
    outNNS(2) = cond_dot;        // 直行条件
    outNNS(3) = cond_cross;      // ベクトル方向確認

    return;

}