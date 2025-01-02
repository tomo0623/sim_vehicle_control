#ifndef _PLANTMODEL_H_
#define _PLANTMODEL_H_

#include "Eigen/Dense"

// 車両パラメータモデル()
class VehicleModel
{
private:
public:
    double wheelbase = kWheelBase;
    double speed = kVehicleSpeed;

    // コンストラクタ
    VehicleModel()
    {
    }
};

#endif // _PLANTMODEL_H_