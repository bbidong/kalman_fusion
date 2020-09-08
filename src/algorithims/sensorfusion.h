#pragma once

#include "interface/measurement_package.h"
#include "kalmanfilter.h"

class SensorFusion {
public:
    SensorFusion();  // 初始化函数, 初始化测量矩阵H和测量误差R
    ~SensorFusion();

    void Process(MeasurementPackage measurement_pack);
    KalmanFilter kf_;

private:
    bool is_initialized_;
    long last_timestamp_;
    Eigen::MatrixXd H_;
    Eigen::MatrixXd R_camera_;
    Eigen::MatrixXd R_lidar_;
    Eigen::MatrixXd R_radar_;


};
