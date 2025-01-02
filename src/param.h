#ifndef _PARAM_H_
#define _PARAM_H_

// 定数マクロ
constexpr int kNumStateX = 3;  // 状態数
constexpr int kNumInputU = 4;  // 入力数
constexpr int kNumOutputY = 6; // 出力数

constexpr double kSmplTimeSim = 0.01;                          // サンプリング時間(シミュレータ)
constexpr double kSmplTimeCtrl = 0.01;                         // サンプリング時間(制御器)
constexpr int kNumSimCtrl = int(kSmplTimeCtrl / kSmplTimeSim); // 制御割込カウント周期

constexpr double kMaxSimTime = 100.0;                               // シミュレーション時間
constexpr double kNumMaxSimCount = int(kMaxSimTime / kSmplTimeSim); // シミュレーション時間

constexpr double kR2D = 180. / 3.14159265359;

// 車両定数パラメータ
constexpr double kWheelBase = 2.5;            // ホイールベース [m]
constexpr double kVehicleSpeed = 30. / 3.6;   // 走行速度 [m/s]
constexpr double kLimSteerAngle = 30. / kR2D; // 最大舵角 [rad]

#endif // _PARAM_H_