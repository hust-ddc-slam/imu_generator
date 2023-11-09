# IMU数据仿真程序

# 1、imu_generator
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
总共需要修改的地方：
1. `myInterface.h`中的`sim_freq_imu`和`sim_distance_threshold`，分别表示：仿真输出的IMU频率，以及初始运动距离后开始仿真；
2. `main.cpp`中的文件输出与输出路径；


**轨迹格式**  
轨迹格式采用：ts,tx,ty,tz,qw,qx,qy,qz  
即：时间(s)，平移的xyz(m)，旋转四元数(Hamilton格式)，全部为：轨迹的姿态在世界系下的表示。  
(注意：openvins采用的是JPL四元数，因此输入Hamilton时，在`myInterface`中的代码进行了转化：xyz取负号，qw换位置；同时，若需要输出pose，则也需要转回Hamilton形式。)

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

# 2、vicon_data_convertor
这个程序将一个vicon导出数据，转成用于上述`imu_generator`的数据格式。

## Usage
修改：输入（vicon数据）、输出（trajectory的轨迹）的文件路径。  

## 注意事项
输入数据格式（vicon）：index,sub_frame,RX,RY,RZ(XYZ欧拉角？TODO：需确认是从哪个系到哪个系）,TX,TY,TZ(mm)  
轨迹的存储格式是：ts,tx,ty,tz(m),qw,qx,qy,qz(Hamilton)，与上述格式一致。

# 3、matlab_plot
matlab用于绘图的代码。

# Issue：
1. convertor部分的欧拉角转化不清楚；坐标系是谁到谁还不清楚；
2. VICON数据二次求导得到加速度噪声太大，是否需要对原始数据进行适当的滤波？

更新日期：2023.11.08
