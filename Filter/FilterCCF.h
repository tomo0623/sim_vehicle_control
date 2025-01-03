#ifndef _FILTERCCF_H_
#define _FILTERCCF_H_

#include <iostream>
#include <cmath>
#include <vector>
#include <array>

#include "Eigen/Dense"

// 可制御正準形（Controllable Canonical Form）に基づく簡易フィルタ
// 今回は一次LPFと一次ADFの合成伝達関数としてのみ実装
class FilterCCF
{
private:
    Eigen::Matrix2d matAc, matCc, matAd;
    Eigen::Vector2d vecBc, vecBd, x_old, x_new;

    double tau_lpf, tau_adf, stime;
public:
	// コンストラクタ
	FilterCCF(double tmp_tau_lpf = 0.05, double tmp_tau_adf = 0.02, double tmp_stime = 0.01)
	{
        // 時定数等の設定
        tau_lpf = tmp_tau_lpf;
        tau_adf = tmp_tau_adf;
        stime = tmp_stime;

        // 初期化
        x_old << 0.0, 0.0;
        x_new << 0.0, 0.0;

        // 連続時間状態方程式の行列生成
        matAc << 0.0, 1.0, -1.0/(tau_lpf*tau_adf), -(tau_lpf+tau_adf)/(tau_lpf*tau_adf);
        vecBc << 0.0, 1.0;
        matCc << 1.0/(tau_lpf*tau_adf), 0.0, 0.0, 1.0/(tau_lpf*tau_adf);
        // 離散時間状態方程式の行列生成(オイラー近似)
        matAd = (Eigen::Matrix2d::Identity(2,2) + matAc*stime);
        vecBd = vecBc*stime;
	}

    // 関数オブジェクト
	Eigen::Vector2d operator()(double in)
	{
        x_old = x_new;
        x_new = matAd*x_old + vecBd*in; // 次サンプリング用のオイラー更新計算
		return matCc*x_old; // 出力方程式
	}

};

#endif // _FILTERCCF_H_