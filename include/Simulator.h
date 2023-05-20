#ifndef _SIMULATOR_H__
#define _SIMULATOR_H__

#include <eigen3/Eigen/Core>
#include <fstream>
#include <opencv2/core/core.hpp>
#include <random>
#include <sstream>
#include <string>
#include <memory>

#include "myInterface.h"
#include "BsplineSE3.h"



class Simulator {
public:
    Simulator(MyParam param);
    bool ok() { return is_running_; }
    bool get_next_imu(double &time_imu, Eigen::Vector3d &wm, Eigen::Vector3d &am);


    MyParam param_;
    //===================================================================
    // State related variables
    //===================================================================

    /// Our loaded trajectory data (timestamp(s), q_GtoI, p_IinG)
    std::vector<Eigen::VectorXd> traj_data_;

    /// Our b-spline trajectory
    std::shared_ptr<ov_core::BsplineSE3> spline_;

    /// Mersenne twister PRNG for measurements (IMU)
    std::mt19937 gen_meas_imu;

    
    // If our simulation is running
    bool is_running_;

    //===================================================================
    // Simulation specific variables
    //===================================================================

    /// Current timestamp of the system
    double timestamp_;

    /// Last time we had an IMU reading
    double timestamp_last_imu_;

    /// Last time we had an CAMERA reading
    double timestamp_last_cam_;


    /// Our running acceleration bias
    Eigen::Vector3d true_bias_accel = Eigen::Vector3d::Zero();

    /// Our running gyroscope bias
    Eigen::Vector3d true_bias_gyro = Eigen::Vector3d::Zero();

    // Our history of true biases
    bool has_skipped_first_bias = false;        // TODO: why this...
    std::vector<double> hist_true_bias_time;
    std::vector<Eigen::Vector3d> hist_true_bias_accel;
    std::vector<Eigen::Vector3d> hist_true_bias_gyro;

};


#endif
