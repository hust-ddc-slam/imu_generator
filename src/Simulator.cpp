
#include "Simulator.h"


// init simulator.
Simulator::Simulator(MyParam param):param_(param)
{
    loadControlPoints(param_.sim_traj_path, traj_data_);

    spline_ = std::make_shared<ov_core::BsplineSE3>();
    spline_->feed_trajectory(traj_data_);
    
    // Set all our timestamps as starting from the minimum spline time
    timestamp_ = spline_->get_start_time();
    timestamp_last_imu_ = spline_->get_start_time();
    timestamp_last_cam_ = spline_->get_start_time();

    // Get the pose at the current timestep
    Eigen::Matrix3d R_GtoI_init;
    Eigen::Vector3d p_IinG_init;
    bool success_pose_init = spline_->get_pose(timestamp_, R_GtoI_init, p_IinG_init);
    if (!success_pose_init) {
        PRINT_ERROR("[SIM]: unable to find the first pose in the spline...\n");
        std::exit(EXIT_FAILURE);
    }
    
    // Find the timestamp that we move enough to be considered "moved"
    double distance = 0.0;
    double distancethreshold = param_.sim_distance_threshold;
    while (true) {

        // Get the pose at the current timestep
        Eigen::Matrix3d R_GtoI;
        Eigen::Vector3d p_IinG;
        bool success_pose = spline_->get_pose(timestamp_, R_GtoI, p_IinG);
        
        // Check if it fails
        if (!success_pose) {
            PRINT_ERROR("[SIM]: unable to find jolt in the groundtruth data to initialize at\n");
            std::exit(EXIT_FAILURE);
        }

        // Append to our scalar distance
        distance += (p_IinG - p_IinG_init).norm();
        p_IinG_init = p_IinG;

        // Now check if we have an acceleration, else move forward in time
        if (distance > distancethreshold) {
            break;
        } 
        else {
            timestamp_ += 1.0 / param_.sim_freq_cam;
            timestamp_last_imu_ += 1.0 / param_.sim_freq_cam;
            timestamp_last_cam_ += 1.0 / param_.sim_freq_cam;
        }
    }
    PRINT_DEBUG("[SIM]: moved %.3f seconds into the dataset where it starts moving\n", timestamp_ - spline_->get_start_time());


    // Append the current true bias to our history. TODO:
    is_running_ = true;

    // generate noise random number. TODO:

}


bool Simulator::get_next_imu(double &time_imu, Eigen::Vector3d &wm, Eigen::Vector3d &am) {
    // Return if the camera measurement should go before us. TODO:
    // if (timestamp_last_cam_ + 1.0 / param_.sim_freq_cam < timestamp_last_imu_ + 1.0 / param_.sim_freq_imu){
    //     cout << "Error. TS!" << endl;
    //     return false;
    // }

    // Else lets do a new measurement!!!
    timestamp_last_imu_ += 1.0 / param_.sim_freq_imu;
    timestamp_ = timestamp_last_imu_;
    time_imu = timestamp_last_imu_;

    // Current state values
    Eigen::Matrix3d R_GtoI;
    Eigen::Vector3d p_IinG, w_IinI, v_IinG, alpha_IinI, a_IinG;

    // STEP 1. Get the pose, velocity, and acceleration
    // NOTE: we get the acceleration between our two IMU
    // NOTE: this is because we are using a constant measurement model for integration
    bool success_accel = spline_->get_acceleration(timestamp_, R_GtoI, p_IinG, w_IinI, v_IinG, alpha_IinI, a_IinG);
    
    // If failed, then that means we don't have any more spline
    // Thus we should stop the simulation
    if (!success_accel) {
        is_running_ = false;
        return false;
    }


    // STEP 2. Convert to IMU frame
    // Transform omega and linear acceleration into imu frame
    Eigen::Vector3d omega_inI = w_IinI;
    Eigen::Vector3d gravity;
    gravity << 0.0, 0.0, param_.gravity_mag;
    Eigen::Vector3d accel_inI = R_GtoI * (a_IinG + gravity);

    // -> generate IMU noise. TODO.
    double dt = 1.0 / param_.sim_freq_imu;
    std::normal_distribution<double> w(0,1);
    if (has_skipped_first_bias) {
        // Move the biases forward in time
        true_bias_gyro(0) += param_.imu_noises.sigma_wb * std::sqrt(dt) * w(gen_meas_imu);
        true_bias_gyro(1) += param_.imu_noises.sigma_wb * std::sqrt(dt) * w(gen_meas_imu);
        true_bias_gyro(2) += param_.imu_noises.sigma_wb * std::sqrt(dt) * w(gen_meas_imu);
        true_bias_accel(0) += param_.imu_noises.sigma_ab * std::sqrt(dt) * w(gen_meas_imu);
        true_bias_accel(1) += param_.imu_noises.sigma_ab * std::sqrt(dt) * w(gen_meas_imu);
        true_bias_accel(2) += param_.imu_noises.sigma_ab * std::sqrt(dt) * w(gen_meas_imu);

        // Append the current true bias to our history
        hist_true_bias_time.push_back(timestamp_last_imu_);
        hist_true_bias_gyro.push_back(true_bias_gyro);
        hist_true_bias_accel.push_back(true_bias_accel);
    }
    has_skipped_first_bias = true;

    // Add with noise.
    wm(0) = omega_inI(0) + true_bias_gyro(0) + param_.imu_noises.sigma_w / std::sqrt(dt) * w(gen_meas_imu);
    wm(1) = omega_inI(1) + true_bias_gyro(1) + param_.imu_noises.sigma_w / std::sqrt(dt) * w(gen_meas_imu);
    wm(2) = omega_inI(2) + true_bias_gyro(2) + param_.imu_noises.sigma_w / std::sqrt(dt) * w(gen_meas_imu);
    am(0) = accel_inI(0) + true_bias_accel(0) + param_.imu_noises.sigma_a / std::sqrt(dt) * w(gen_meas_imu);
    am(1) = accel_inI(1) + true_bias_accel(1) + param_.imu_noises.sigma_a / std::sqrt(dt) * w(gen_meas_imu);
    am(2) = accel_inI(2) + true_bias_accel(2) + param_.imu_noises.sigma_a / std::sqrt(dt) * w(gen_meas_imu);

    return true;
}
