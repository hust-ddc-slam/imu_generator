
% This code compare an REAL xsens IMU output, and Simulated IMU by trajectory from Openvins.
% also, the vicon's trajectory is plotted.

clc;clear;close all;

base_data_folder = "/home/larrydong/codeGit/extrinsics_eskf/imu_generator/data/imu_vicon/";

xsens_record_ahead = 0;            % skip seconds data.


%% TODO
% IMU and VICON's coordinate. Extrinsics. TODO:
% R_vicon2imu = [1,0,0; 0,1,0; 0,0,1];


%% Plot Xsens IMU
imu_filename = "MT_0080003F51_004-000.txt";

filename = base_data_folder + imu_filename;
% format: PacketCounter,SampleTimeFine,Acc_X,Acc_Y,Acc_Z,Gyr_X,Gyr_Y,Gyr_Z
skip_line = 14;
data = readmatrix(filename,'Range', skip_line);


packages = data(:,1);
assert(length(packages)==packages(end)-packages(1)+1);  % avoid missing packat in data sampling.
times = (data(:,2)-data(1,2))/1e4;       % 10ms -> second. SampleTimeFine is x10 ms.
% skip offset-data
I0 = find(times >= xsens_record_ahead);  
I0 = I0(1);
times = times(I0:end) - times(I0);
packages = data(I0:end, 1);
accs = data(I0:end, 3:5);
gyros = data(I0:end, 6:8);
N = length(packages);
% real_imu_frequency = 1/ ((times(end)-times(1))/(length(times)-1))

figure("Name","Xsens");
subplot(211);
plot(times,accs(:,1),'r-'); hold on;
plot(times,accs(:,2),'g-');
plot(times,accs(:,3),'b-');
legend('acc-x','y','z');
xlabel('time (s)');
ylabel('acc (m/s)');
title('Accleration (from Xsens)');

subplot(212);
plot(times,gyros(:,1),'r-'); hold on;
plot(times,gyros(:,2),'g-');
plot(times,gyros(:,3),'b-');
legend('gyro-x','gy','gz');
xlabel('time (s)');
ylabel('gyro (deg/s)');
title('Gyroscope (from Xsens)');



%% Plot openvins simulation
imu_filename = "simulation-05.csv";
% ORB-slam3's format: time[ns], wx(rad/s), wy, wz, ax(m/s2), ay, az

filename = base_data_folder + imu_filename;
skip_line = 2;
data = readmatrix(filename,'Range', skip_line);
times = (data(:,1)-data(1,1))/1e9;       % ns -> second
I0 = 1;                                 % vicon data do not need an offset
times = times(I0:end) - times(I0);
accs = data(I0:end, 5:7);
gyros = data(I0:end, 2:4)*180/pi;
N = length(packages);

figure("Name","simulation");
subplot(211);
plot(times,accs(:,1),'r-'); hold on;
plot(times,accs(:,2),'g-');
plot(times,accs(:,3),'b-');
legend('acc-x','y','z');
xlabel('time (s)');
ylabel('acc (m/s)');
title('Accleration (from openvins)');

subplot(212);
plot(times,gyros(:,1),'r-'); hold on;
plot(times,gyros(:,2),'g-');
plot(times,gyros(:,3),'b-');
legend('gyro-x','gy','gz');
xlabel('time (s)');
ylabel('gyro (deg/s)');
title('Gyroscopd (from openvins)');







