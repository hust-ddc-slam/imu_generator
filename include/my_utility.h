
#ifndef _UTILITY_H_
#define _UTILITY_H_

#define Gravity (9.8f)
#define SampleRate (100.0f)

#include <eigen3/Eigen/Core>

typedef Eigen::Vector3d v3d;
typedef Eigen::Vector3f v3f;

namespace utility{

inline v3d deg2rad(const Eigen::Vector3d& v){
    const double scale = M_PI/180.0;
    Eigen::Vector3d out;
    out[0] = v[0]*scale;
    out[1] = v[1]*scale;
    out[2] = v[2]*scale;
    return out;
}
inline v3d rad2deg(const Eigen::Vector3d& v){
    const double scale = 180.0/M_PI;
    Eigen::Vector3d out;
    out[0] = v[0]*scale;
    out[1] = v[1]*scale;
    out[2] = v[2]*scale;
    return out;
}

}
#endif
