
clc;clear; 
close all;

% load trajectory.
%% original
filename = '/home/larrydong/imu_ws/src/imu_simulation/data/trajectory.csv';
data = readmatrix(filename,'Range', 2);
ts = data(:,1);
roll = data(:,2);
pitch = data(:,3);
yaw = data(:,4);
tx = data(:,5);
ty = data(:,6);
tz = data(:,7);

figure(1);
subplot(211); hold off;
plot(ts,roll,'r'); hold on;
plot(ts,pitch,'g');
plot(ts,yaw,'b'); 
legend('roll','pitch','yaw');
xlabel('time (s)');
ylabel('angle (deg)');  
title('Eular angle (trajectory. Body 2 IMU)');

subplot(212);
plot(ts,tx,'r'); hold on;
plot(ts,ty,'g');
plot(ts,tz,'b');
legend('tx','ty','tz');
xlabel('time (s)');
ylabel('translation (m)');
title('Translation');

%% show control points
filename = '/home/larrydong/imu_ws/src/imu_simulation/data/trajectory_control_points.csv';
data = readmatrix(filename,'Range', 2);
ts = data(:,1);
roll = data(:,2);
pitch = data(:,3);
yaw = data(:,4);
tx = data(:,5);
ty = data(:,6);
tz = data(:,7);
subplot(211);
scatter(ts,roll,'ro' ,'HandleVisibility','off')
scatter(ts,pitch,'go','HandleVisibility','off')
scatter(ts,yaw,'bo','HandleVisibility','off')
subplot(212);
plot(ts,tx,'ro','HandleVisibility','off')
plot(ts,ty,'go','HandleVisibility','off')
plot(ts,tz,'bo','HandleVisibility','off')



%% test
% filename = '/home/larrydong/imu_ws/src/imu_data_simulation/generate_imu/build/test_imu.csv';
% data = readmatrix(filename,'Range', 2);
% ts = data(:,1);
% roll = data(:,2);
% pitch = data(:,3);
% yaw = data(:,4);
% tx = data(:,5);
% ty = data(:,6);
% tz = data(:,7);
% 
% 
% subplot(211);
% plot(ts,roll,'r--'); hold on;
% plot(ts,pitch,'g--');
% plot(ts,yaw,'b--'); 
% legend('roll','pitch','yaw');
% xlabel('time (s)');
% ylabel('angle (deg)');  
% title('Eular angle');
% 
% subplot(212);
% plot(ts,tx,'r--'); hold on;
% plot(ts,ty,'g--');
% plot(ts,tz,'b--');
% legend('tx','ty','tz');
% xlabel('time (s)');
% ylabel('translation (m)');
% title('Translation');
% 
