
#include "myInterface.h"

void loadControlPoints(const string filename, std::vector<Eigen::VectorXd>& traj_data){
    cout << "--> Load control points from: " << filename << endl;

    std::ifstream file;
    file.open(filename);
    if(!file){
        cerr << "Cannot open file: " << filename << endl;
        std::abort();
    }

    // Loop through each line of this file
    std::string current_line;
    while (std::getline(file, current_line)) {
      // Skip if we start with a comment
        if (!current_line.find("#"))
            continue;
        int i=0;
        std::istringstream s(current_line);
        std::string field;
        Eigen::Matrix<double, 8, 1> data;
        
        // Loop through this line (timestamp(s) tx ty tz qx qy qz qw)
        while (std::getline(s, field, ',')) {
            // Skip if empty
            if (field.empty() || i >= data.rows())
            continue;
            // save the data to our vector
            data(i) = std::atof(field.c_str());
            i++;
        }
        // Only a valid line if we have all the parameters
        if (i > 7) {                        // format: ts, x,y,z, qx,qy,qz,qw
            traj_data.push_back(data);
        }
    }
    // Finally close the file
    file.close();
    cout << "<-- Loaded: " << traj_data.size() << " control points." << endl;
}


void saveImu(const std::string filename, const std::vector<ImuData>& imus){
    std::ofstream save_points;
    save_points.open(filename.c_str());
    save_points << "# ORB-slam3's format: time[ns], wx(rad/s), wy, wz, ax(m/s2), ay, az" << endl;
    for (int i = 0; i < imus.size(); ++i)    {
        ImuData imu = imus[i];
        // double time = imu.ts;
        size_t time = size_t(imu.ts * 1e9);
        // Eigen::Vector3d gyro = imu.gyro;
        Eigen::Vector3d gyro = imu.gyro * 3.141592 / 180.0f;
        Eigen::Vector3d acc = imu.acc;

        save_points << std::fixed           // no scientific notation
                    << time << ","
                    << gyro(0) << "," << gyro(1) << "," << gyro(2) << ","
                    << acc(0) << "," << acc(1) << "," << acc(2)
                    << endl;
    }
    save_points.close();
}
