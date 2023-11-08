
#include <iostream>
#include <vector>
#include <fstream>
#include <eigen3/Eigen/Core>

#include "spline.h"

using namespace std;

typedef Eigen::Vector3d v3d;

class Pose{
public:
    Pose(){}
    Pose(v3d eularAngle, v3d translation) : eular(eularAngle), trans(translation){}
    void print(void) { cout << "[Pose]: angle: " << eular.transpose() << ", trans: " << trans.transpose() << endl; }
    v3d eular;
    v3d trans;
};


class Trajectory{
public:
    Trajectory(){}
    void print(void) { 
        for(int i=0; i<ts.size(); ++i){
            cout << "[Traj], ts: " << ts[i] << endl;
            poses[i].print();
        }
    }
    vector<double> ts;
    vector<Pose> poses;
};


void saveTrajectory(const Trajectory& traj, string filename){
    ofstream of(filename);
    of << "# ts, roll, pitch, yaw(angle), tx(m), ty, tz" << endl;
    for(int i=0; i<traj.ts.size(); ++i){
        double ts = traj.ts[i];
        v3d e = traj.poses[i].eular;
        v3d t = traj.poses[i].trans;
        of << ts << "," << e[0] << "," << e[1] << "," << e[2] << "," << t[0] << "," << t[1] << "," << t[2] << endl;
    }
    of.close();
    cout << "Saved " << traj.ts.size() << " data to file: " << filename << endl;
}



Trajectory interpTrajectory(const Trajectory& input, double ts_step){
    Trajectory output;
    double tbegin = input.ts[0];
    double tend = input.ts.back();
    vector<double> roll, pitch, yaw, X, Y, Z, T;
    for(int i=0; i<input.ts.size(); ++i){
        const Pose& p = input.poses[i];
        T.push_back(input.ts[i]);
        X.push_back(p.trans[0]);
        Y.push_back(p.trans[1]);
        Z.push_back(p.trans[2]);
        roll.push_back(p.eular[0]);
        pitch.push_back(p.eular[1]);
        yaw.push_back(p.eular[2]);
    }
    // tk::spline spX(T, X, tk::spline::cspline_hermite);
    // tk::spline spY(T, Y, tk::spline::cspline_hermite);
    // tk::spline spZ(T, Z, tk::spline::cspline_hermite);
    tk::spline spX(T, X);
    tk::spline spY(T, Y);
    tk::spline spZ(T, Z);
    tk::spline spRoll(T, roll);
    tk::spline spPitch(T, pitch);
    tk::spline spYaw(T, yaw);

    for (double t = input.ts[0]; t < input.ts.back(); t+=ts_step){
        output.poses.emplace_back(v3d(spRoll(t), spPitch(t), spYaw(t)), v3d(spX(t), spY(t), spZ(t)));
        output.ts.push_back(t);
    }
    return output;
}



int main(void){

    // // Interp angle;
    // // Step 1. Generate control points;
    vector<v3d> eularAngles{        // unit: degree
        v3d(0,0,0),
        v3d(0,0,0),
        v3d(0,0,0),
        v3d(0,0,0),
        v3d(5,5,0),
        v3d(10,0,0),
        v3d(30,0,0),
        v3d(10,0,0),
        v3d(0,5,60),
        v3d(10,0,0),
        v3d(30,0,0),
        v3d(10,0,0),
        v3d(0,0,0)
    };
    vector<v3d> translations{       // unit: m
        v3d(0,0,0),
        v3d(0,0,0),
        v3d(0,0,0),
        v3d(0,0,0),
        v3d(2,0,0),
        v3d(8,0,0),
        v3d(1,2,0),
        v3d(0,4,0),
        v3d(0,9,0),
        v3d(0,4,3),
        v3d(0,0,6),
        v3d(0,0,1),
        v3d(0,0,0)
    };
    vector<double> ts{0,1,2,3,4,5,6,7,8,9,10,11,12};

    assert(ts.size() == eularAngles.size() && ts.size() == translations.size());
    Trajectory ctrl_trajectory;
    for(int i=0; i<ts.size(); ++i){
        ctrl_trajectory.ts.push_back(ts[i]);
        ctrl_trajectory.poses.emplace_back(eularAngles[i], translations[i]);
    }

    // Step 2. Interp.
    Trajectory gen_trajectory = interpTrajectory(ctrl_trajectory, 1e-2);

    // cout << "-------------------------------" << endl;
    // cout << "Interped trajectory." << endl;
    // gen_trajectory.print();


    // // Step 3. Save output.
    string filename = "trajectory.csv";
    saveTrajectory(gen_trajectory, filename);
    
    return 0;
}
