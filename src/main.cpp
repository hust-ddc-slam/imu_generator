
#include <iostream>
#include <fstream>
#include <string>
#include <vector>

#include "BsplineSE3.h"
#include "myInterface.h"
#include "Simulator.h"

using namespace std;


// Define the function to be called when ctrl-c (SIGINT) is sent to process
void signal_callback_handler(int signum) { std::exit(signum); }


// 
int main(int argc, char **argv){
    // Settings. Verbosity, and params.
    std::string verbosity = "ALL";         // ALL = 0, DEBUG = 1, INFO = 2, WARNING = 3, ERROR = 4, SILENT = 5
    ov_core::Printer::setPrintLevel(verbosity);


    MyParam my_param;                       // init MyParam, includes IMU settings.
    my_param.sim_traj_path = "/home/larrydong/codeGit/extrinsics_eskf/imu_generator/data/imu_vicon/rig05-trajectory.csv";   // input quart format.
    Simulator simulator(my_param);          // initializa simulator.
    
    vector<ImuData> imus;
    cout << "Generating IMU..." << endl;
    while(simulator.ok()){
        double time_imu;
        Eigen::Vector3d wm, am;
        bool imu_ok = simulator.get_next_imu(time_imu, wm, am);
        if(imu_ok){
            ImuData imu(time_imu, am, wm);
            imus.push_back(imu);
        }
        else{
            cerr << "IMU is not okay!" << endl;
        }
    }

    cout << "Generated imu: " << imus.size() << endl;
    string imu_save_path = "/home/larrydong/codeGit/extrinsics_eskf/imu_generator/data/imu_vicon/simulation-05.csv";
    saveImu(imu_save_path, imus);

    cout << "==> Done" << endl;
    return 0;
}


