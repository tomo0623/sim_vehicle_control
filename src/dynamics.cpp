#include <iostream>
#include <cmath>
#include <vector>
#include <array>

#include "Eigen/Dense"
#include "param.h"
#include "dynamics.h"
#include "plant_model.h"

// 定数定義
VehicleModel car;

// 状態方程式計算関数
// x = [posx, posy, theta]^T
// u = [delta, ref_posx, ref_posy, tracking_error]^T
// y = [posx, posy, theta, vx, vy, gamma]^T
Eigen::VectorXd Dynamics::StateEquation::CalcSEQ(double t, Eigen::VectorXd x, Eigen::VectorXd u)
{
    Eigen::VectorXd dx(kNumStateX);
    double steer;

    // 操舵角セット
    steer = u(0);
    if (steer > kLimSteerAngle)
    {
        steer = kLimSteerAngle;
    }
    else if (steer < -kLimSteerAngle)
    {
        steer = -kLimSteerAngle;
    }

    // 車両の非線形キネマティクス
    dx(0) = car.speed * cos(x(2));
    dx(1) = car.speed * sin(x(2));
    dx(2) = car.speed / car.wheelbase * tan(steer);

    // std::cout<<"dynamics : " << t <<std::endl;
    // std::cout<<dx<<std::endl;
    return dx;
}

// 出力方程式計算関数
// x = [posx, posy, theta]^T
// u = [delta, ref_posx, ref_posy, tracking_error]^T
// y = [posx, posy, theta, vx, vy, gamma]^T
Eigen::VectorXd Dynamics::OutputEquation::CalcOEQ(double t, Eigen::VectorXd x, Eigen::VectorXd u)
{
    Eigen::VectorXd y = Eigen::VectorXd::Zero(kNumOutputY);

    // 制御検証SIMのため全状態可観測を仮定
    for (int i = 0; i < x.size(); i++)
    {
        y(i) = x(i);
    }

    // 直交座標速度と角速度の再計算
    double steer;
    steer = u(0);
    if (steer > kLimSteerAngle)
    {
        steer = kLimSteerAngle;
    }
    else if (steer < -kLimSteerAngle)
    {
        steer = -kLimSteerAngle;
    }
    y(x.size()) = car.speed * cos(x(2));
    y(x.size() + 1) = car.speed * sin(x(2));
    y(x.size() + 2) = car.speed / car.wheelbase * tan(steer);
    y(x.size() + 3) = car.speed;

    // std::cout<<"outputs : " << t <<std::endl;
    // std::cout<<y<<std::endl;
    return y;
}
