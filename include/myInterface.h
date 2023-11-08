
#ifndef __MYINTERFACE_H
#define __MYINTERFACE_H


#include <iostream>
#include <vector>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

#include <eigen3/Eigen/Core>
#include "my_utility.h"


using namespace std;

// class Trajectory{
// public:
//     Trajectory(){}
//     std::vector<double> ts;
//     std::vector<v3d> eulers, trans;
// };


struct NoiseManager {

    /// Gyroscope white noise (rad/s/sqrt(hz))
    double sigma_w = 1.6968e-04;
    /// Gyroscope white noise covariance
    double sigma_w_2 = pow(sigma_w, 2);

    /// Gyroscope random walk (rad/s^2/sqrt(hz))
    double sigma_wb = 1.9393e-05;
    /// Gyroscope random walk covariance
    double sigma_wb_2 = pow(sigma_wb, 2);

    /// Accelerometer white noise (m/s^2/sqrt(hz))
    double sigma_a = 2.0000e-3;
    /// Accelerometer white noise covariance
    double sigma_a_2 = pow(sigma_a, 2);

    /// Accelerometer random walk (m/s^3/sqrt(hz))
    double sigma_ab = 3.0000e-03;
    /// Accelerometer random walk covariance
    double sigma_ab_2 = pow(sigma_ab, 2);

};


class ImuData{
public:
    ImuData(){}
    ImuData(double ts, Eigen::Vector3d acc, Eigen::Vector3d gyro) : ts(ts), acc(acc), gyro(gyro) {}
    double ts;
    Eigen::Vector3d acc, gyro;
};


class MyParam{
public:
    MyParam():sim_freq_imu(100), sim_freq_cam(10)
    {
        sim_distance_threshold = 0.01;               // when simulate, skip some distance at the beginning.
        sim_traj_path = "/default/input/path";
    }

public:
    /// IMU noise (gyroscope and accelerometer)
    NoiseManager imu_noises;
    double sim_freq_imu;
    double sim_freq_cam;
    string sim_traj_path;
    double sim_distance_threshold;
    const double gravity_mag = 9.81;
};


void loadControlPoints(const std::string filename, std::vector<Eigen::VectorXd>& traj_data);
void saveImu(const std::string filename, const std::vector<ImuData>& imus);

#endif

