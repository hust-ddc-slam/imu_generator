# imu_generator
Generate IMU data based on control points

Using a BSpline interp for trajectory, and generate IMU with noise/bias.  
This code is modified from openvins.

## Usage
```bash
cd imu_generator
mkdir build && cd build 
cmake .. && make

./gen_imu
```

## Principle
