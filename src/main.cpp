#include <string>
#include <vector>
#include <fstream>
#include <iostream>
#include <sstream>
#include <iomanip>
#include "Eigen/Eigen"
#include "interface/ground_truth_package.h"
#include "interface/measurement_package.h"
#include "algorithims/sensorfusion.h"

int main(int argc, char* argv[]) {

    // 设置测量数据的路径
    std::string input_file_name = "data/measure.txt";

    // 打开数据，若失败则输出失败信息，返回-1，并终止程序
    std::ifstream input_file(input_file_name.c_str(), std::ifstream::in);
    if (!input_file.is_open()) {
        std::cout << "Failed to open file named : " << input_file_name << std::endl;
        return -1;
    }


    // measurement_pack_list：毫米波雷达/激光雷达/相机实际测得的数据。数据包含测量值和时间戳，即融合算法的输入。
    std::vector<MeasurementPackage> measurement_pack_list;

    // 通过while循环将雷达测量值和真值全部读入内存，存入measurement_pack_list
    std::string line;

    std::ofstream out_file;
    out_file.open("bin/fusion_result.txt");
    while (getline(input_file, line)) {
        float sensor_type;
        MeasurementPackage meas_package;
        std::istringstream iss(line);
        float timestamp;

        // 读取当前行的第一个元素，代表传感器类型
        iss >> sensor_type;

        if (sensor_type<3) {

            // 该行第二个元素为测量值x，第三个元素为测量值y，第四个元素为时间戳
            if (sensor_type==1)  meas_package.sensor_type_ = MeasurementPackage::CAMERA;
            else if (sensor_type==2) meas_package.sensor_type_ = MeasurementPackage::LIDAR;

            meas_package.raw_measurements_ = Eigen::VectorXd(2);
            float x;
            float y;
            iss >> x;
            iss >> y;
            meas_package.raw_measurements_ << x, y;

            iss >> timestamp;
            meas_package.timestamp_ = timestamp;

            measurement_pack_list.push_back(meas_package);
        }

        else if (sensor_type == 3) {
            // 毫米波雷达数据 Radar data
            // 该行第二个元素为距离pho，第三个元素为角度phi，第四个元素为径向速度pho_dot，第五个元素为时间戳
            meas_package.sensor_type_ = MeasurementPackage::RADAR;
            meas_package.raw_measurements_ = Eigen::VectorXd(3);
            float rho;
            float phi;
            float rho_dot;
            iss >> rho;
            iss >> phi;
            iss >> rho_dot;
            meas_package.raw_measurements_ << rho, phi, rho_dot;
            iss >> timestamp;
            meas_package.timestamp_ = timestamp;
            measurement_pack_list.push_back(meas_package);
        }

    }

    // 循环读取完所有数据
    std::cout << "Success to load data." << std::endl;

    // 开始部署跟踪算法,初始化R和H
    SensorFusion fuser;

    for (size_t i = 0; i < measurement_pack_list.size(); ++i) {
        fuser.Process(measurement_pack_list[i]);
        Eigen::Vector4d x_out = fuser.kf_.GetX();  // 返回状态

        std::cout << "x " << x_out(0)
                  << " y " << x_out(1)
                  << " vx " << x_out(2)
                  << " vy " << x_out(3)
                  << std::endl;
        // 把结果写到文件里
        out_file<<'x'<<' '<<x_out(0)<<' '<<'y'<<' '<<x_out(1)<<' '<<"vx"<<' '<<x_out(2)<<' '<<"vy"<<' '<<x_out(3)<<std::endl;
    }
    out_file.close();
}