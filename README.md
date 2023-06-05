# imu_generator
Generate IMU data based on control points

Using a BSpline interp for trajectory, and generate IMU with noise/bias.  
This code is modified from openvins.

## Usage
```bash
cd imu_generator
mkdir build && cd build 
cmake .. && make

# 修改main的输入输出文件路径后，运行
./gen_imu
```

## 简介
根据轨迹的控制点，生成IMU的数据。  

**轨迹格式**  

轨迹格式采用：ts,tx,ty,tz,qw,qx,qy,qz  
即：时间(s)，平移的xyz(m)，旋转四元数(Halmiton格式)，全部为：轨迹的姿态在世界系下的表示。
(Attention: Openvins's using JPL format. Need a convertion.)

**轨迹插值与数据生成**  

轨迹插值采用前、后各2个控制点，通过四元数上的BSpline插值得到。  
这部分的核心代码采用了 openvins 的版本，原理详见：[openvins中的IMU数据仿真](https://blog.csdn.net/tfb760/article/details/130259267)

数据生成，即根据插值后的数据，生成IMU的角速度和加速度（和噪声+bias）

## 代码
`BsplineSE3.h/cpp`为四元数的插值，直接来源openvins  
`print.h/cpp`,`quat_ops.h`为输出和四元数操作相关函数，直接复制了openvins  
`my_utility.h`和`myInterface.h/cpp`为自己定义的数据格式，不适用openvins的接口  
`Simulator.h/cpp`为仿真的主要代码，在openvins上改了接口和部分格式

**代码流程备忘**  

代码核心函数：Simulator的初始化、载入控控制点 `loadControlPoints`、生成IMU `get_next_imu`

1. 在初始化`Simulator`时，就已经载入了控制点数据并根据控制点，生成了插值后的轨迹；
2. 初始化`Simulator`时，相关IMU的设置也已经载入，在`MyParam`中；
3. 在`main`函数中不断生成IMU数据，并保存输出


## TODO:
- IMU的bias和noise还没有从openvins中添加过来



