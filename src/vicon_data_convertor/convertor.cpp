#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <Eigen/Geometry>

using namespace std;


string input_imu_filename = "/home/larrydong/codeGit/extrinsics_eskf/imu_generator/data/imu_vicon/1109-rig-new3.csv";
string output_trajectory_filename = "/home/larrydong/codeGit/extrinsics_eskf/imu_generator/data/imu_vicon/new3-trajectory.csv";


int main() {
    
    std::ifstream dataFile(input_imu_filename);
    if (!dataFile.is_open()) {
        std::cerr << "Cannot open file:" << input_imu_filename << endl;
        return 1;
    }

    std::string line;
    std::vector<std::string> lines;
    int frequency = 0;                      // vicon data frequency.

    // 读取并丢弃前5行无效数据
    for (int i = 0; i < 5; ++i) {
        std::getline(dataFile, line);
        if(i==1)
            frequency = atoi(line.c_str());
    }
    cout << "==> Vicon Sample Frequnecy: " << frequency << endl;

    // 读取有效数据并存储到lines中
    while (std::getline(dataFile, line)) {
        lines.push_back(line);
    }

    // 关闭文件
    dataFile.close();

    // 创建Eigen矩阵来存储转化后的数据
    Eigen::MatrixXd data(lines.size(), 7);

    // 解析原始数据并转化为目标格式
    for (int i = 0; i < lines.size(); ++i) {
        double rx, ry, rz, tx, ty, tz;
        int index, sub_frame;
        sscanf(lines[i].c_str(), "%d,%d,%lf,%lf,%lf,%lf,%lf,%lf", &index, &sub_frame, &rx, &ry, &rz, &tx, &ty, &tz);
        // cout << "rx" << rx << ", ry" << ry << ", rz " << rz << ", \n"
        //      << "tx " << tx << ", ty " << ty << ", tz " << tz << endl;
        Eigen::Vector3d euler_angles(rx, ry, rz);
        euler_angles = euler_angles / 180 * 3.1415926;
        Eigen::Quaterniond quaternion;
        quaternion = Eigen::AngleAxisd(euler_angles.norm(), euler_angles.normalized());

        // openvins trajectory format: ts,tx,ty,tz,qw,qx,qy,qz(Halmiton)
        data(i, 0) = tx*1e-2;
        data(i, 1) = ty*1e-2;
        data(i, 2) = tz*1e-2;
        data(i, 3) = quaternion.w();    // Do not need to convert to JPL, because openvins's load convert from Halmiton to JPL then.
        data(i, 4) = quaternion.x();
        data(i, 5) = quaternion.y();
        data(i, 6) = quaternion.z();
    }

    // save to outputfile 
    std::ofstream save_points;
    save_points.open(output_trajectory_filename.c_str());
    save_points << "# openvins imu generation format: ts,tx,ty,tz,qw,qx,qy,qz" << endl;
    for (int i = 0; i < data.rows(); i++){
        save_points << i * 1.0f / frequency << "," 
                    << data(i, 0) << "," << data(i, 1) << "," << data(i, 2) << "," << data(i, 3) << "," 
                    << data(i, 4) << "," << data(i, 5) << "," << data(i, 6) << std::endl;
    }
    save_points.close();
    cout <<"<== Saved control done. Number: " << data.rows() << endl;

    return 0;
}

