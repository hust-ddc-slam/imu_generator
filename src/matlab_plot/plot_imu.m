

clc;clear;
close all;

% load trajectory.
filename = '/home/larrydong/imu_ws/src/imu_data_simulation/generate_imu/build/imu_data.csv';
data = readmatrix(filename,'Range', 2);
% time, qw, qx, qy, qz, tx, ty, tz, gryo-x, gy, gz, acc-x, y, z
% 1   ,  2,  3,  4,  5,  6,  7,  8,      9, 10, 11,    12,13, 14
ts = data(:,1);
wx = data(:,9);
wy = data(:,10);
wz = data(:,11);
ax = data(:,12);
ay = data(:,13);
az = data(:,14);
w = data(:,2);
x = data(:,3);
y = data(:,4);
z = data(:,5);
tx = data(:,6);
ty = data(:,7);
tz = data(:,8);

figure(2);

subplot(311);
plot(ts,w,'r', 'LineWidth', 1); hold on;
plot(ts,x,'g', 'LineWidth', 1);
plot(ts,y,'b', 'LineWidth', 1);
plot(ts,z, 'Color',[247 177 229]/255, 'LineWidth', 2);
legend('w','x','y','z');
xlabel('time (s)');
ylabel('quarterion');
title('Quarterion (from IMU simulation, GT)');
% 
% plot(ts,tx,'r'); hold on;
% plot(ts,ty,'g');
% plot(ts,tz,'b'); 
% legend('translation-x','y','z');
% xlabel('time (s)');
% ylabel('translation (m)');
% title('translation (from IMU simulation, GT)');


subplot(312);
plot(ts,wx,'r'); hold on;
plot(ts,wy,'g');
plot(ts,wz,'b'); 
legend('gyro-x','y','z');
xlabel('time (s)');
ylabel('w (deg/s)');
title('Angular velocity(from IMU simulation)');

subplot(313);
plot(ts,ax,'r'); hold on;
plot(ts,ay,'g');
plot(ts,az,'b');
legend('ax','ay','az');
xlabel('time (s)');
ylabel('acceleration (m/s2)');
title('Acceleration (from IMU simulation)');
hold off;
