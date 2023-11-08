
clc;clear;
% close all;

% ts, roll, pitch, yaw(angle)
% filename = '/home/larrydong/imu_ws/src/imu_data_simulation/generate_imu/build/imu_estimation_euler.csv';
% data = readmatrix(filename,'Range', 2);
% ts = data(:,1);
% roll = data(:,2);
% pitch = data(:,3);
% yaw = data(:,4);
% 
% figure(10);
% subplot(211); hold off;
% plot(ts,roll,'r','LineWidth',1); hold on;
% plot(ts,pitch,'g','LineWidth',1);
% plot(ts,yaw,'b','LineWidth',1); 
% legend('roll','pitch','yaw');
% xlabel('time (s)');
% ylabel('angle (deg)');  
% title('Eular angle (c++)');

filename = '/home/larrydong/imu_ws/src/imu_data_simulation/generate_imu/build/imu_estimation_quarts.csv';
data = readmatrix(filename,'Range', 2);
ts = data(:,1);
w = data(:,2);
x = data(:,3);
y = data(:,4);
z = data(:,5);
figure(10);
subplot(211); hold off;
plot(ts,w,'r','LineWidth',1); hold on;
plot(ts,x,'g','LineWidth',1);
plot(ts,y,'b','LineWidth',1); 
plot(ts,z,'Color',[247 177 229]/255,'LineWidth',1.5); 
legend('w','x','y','z');
xlabel('time (s)');
ylabel('quaterion');  
title('Quaterion (c++)');


% filename = '/home/larrydong/imu_ws/src/imu_data_simulation/generate_imu/build/python_simulation_quarts.txt';
% data = readmatrix(filename,'Range', 1);
% w = data(:,1);
% x = data(:,2);
% y = data(:,3);
% z = data(:,4);
% 
% figure(10);
% subplot(212); hold off;
% plot(ts,w,'r','LineWidth',1); hold on;
% plot(ts,x,'g','LineWidth',1);
% plot(ts,y,'b','LineWidth',1); 
% plot(ts,z,'k','LineWidth',1); 
% legend('w','x','y','z');
% xlabel('time (s)');
% ylabel('quaterion');  
% title('Quaterion (python)');

% 


% 
% filename = '/home/larrydong/imu_ws/src/imu_data_simulation/generate_imu/build/python_simulation.csv';
% data = readmatrix(filename,'Range', 1);
% roll = data(:,1);
% pitch = data(:,2);
% yaw = data(:,3);
% 
% subplot(212); hold off;
% plot(ts,roll,'r','LineWidth',1); hold on;
% plot(ts,pitch,'g','LineWidth',1);
% plot(ts,yaw,'b','LineWidth',1); 
% legend('roll','pitch','yaw');
% xlabel('time (s)');
% ylabel('angle (deg)');  
% title('Eular angle (python)');

% 
% subplot(212);
% plot(ts,tx,'r'); hold on;
% plot(ts,ty,'g');
% plot(ts,tz,'b');
% legend('tx','ty','tz');
% xlabel('time (s)');
% ylabel('translation (m)');
% title('Translation');