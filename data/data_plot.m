clc
close all;
clear all;

sensor_type = [];
time = [];
x_all = []; 
x_est_all = []; 
error_all = [];
NIS_radar_all = []; 
NIS_laser_all = []; 

%% load data from txt
% sensor_type	px	py	vx	vy	
% estimate_x	estimate_y	estimate_vx	estimate_vy	
% rmse_x	rmse_y	rmse_vx	rmse_vy	NIS_lidar	NIS_radar
filename = 'data_record2.txt';
delimiterIn = '\t';
headerlinesIn = 1;
file_data = importdata(filename,delimiterIn,headerlinesIn);
data = file_data.data;
sensor_type = data(:,2);
time = data(:,1);
time = (time - time(1))/1e6;
x_all = data(:,2:5);
x_est_all = data(:, 6:9);
error_all = data(:, 10:13);
NIS_radar_all = data(:, 14);
NIS_laser_all = data(:, 15);
sensor_type = file_data.textdata(2:end,1);

%%
laser_idx = find(string(sensor_type) == 'L');
radar_idx = find(string(sensor_type) == 'R');
laser_px = x_all(laser_idx,1);
laser_py = x_all(laser_idx,2);
radar_px = x_all(radar_idx,1);
radar_py = x_all(radar_idx,2);

real_px = x_all(:,1);
real_py = x_all(:,2);

est_px = x_est_all(:,1);
est_py = x_est_all(:,2);
%% plot figure
figure;
hold on
plot(real_px(1:5:end), real_py(1:5:end), '-o', 'MarkerSize', 5, 'Linewidth', 1)
plot(est_px(1:5:end), est_py(1:5:end), '-o', 'MarkerSize', 5, 'Linewidth', 1)
plot(laser_px(1:3:end), laser_py(1:3:end),'x', 'MarkerSize', 5)
plot(radar_px(1:3:end), radar_py(1:3:end),'x', 'MarkerSize', 5)

xlabel('X position (m)')
ylabel('Y Position (m)')
legend('Actual position', 'est', 'Laser meas', 'Radar meas')
grid on
%%
figure;
subplot(2,1,1)
hold on
plot(time,NIS_laser_all)
plot([0,25],[7.815,7.815])
xlabel('Time (s)')
ylabel('Lidar NIS')
grid on
subplot(2,1,2)
hold on
plot(time,NIS_radar_all)
plot([0,25],[7.815,7.815])
xlabel('Time (s)')
ylabel('Radar NIS')
grid on
%%
real_vx = x_all(:,3);
real_vy = x_all(:,4);

est_vx = x_est_all(:,3);
est_vy = x_est_all(:,4);

figure;
hold on
plot(time, real_px, '-r')
plot(time(1:8:end), est_px(1:8:end), '.-r')
plot(time, real_py, '-g')
plot(time(1:8:end), est_py(1:8:end), '.-g')
plot(time, real_vx, '-b')
plot(time(1:8:end), est_vx(1:8:end), '.-b')
plot(time, real_vy, '-m')
plot(time(1:8:end), est_vy(1:8:end), '.-m')
xlabel('Time (s)')
legend('px', 'px est', 'py', 'py est', 'vx', 'vx est', 'vy', 'vy est');
grid on

%% 
error_x = error_all(:,1);
error_y = error_all(:,2);
error_vx = error_all(:,3);
error_vy = error_all(:,4);

figure;
hold on
plot(time, error_x, 'Linewidth', 1)
plot(time, error_y, 'Linewidth', 1)
plot(time, error_vx, 'Linewidth', 1)
plot(time, error_vy, 'Linewidth', 1)
xlabel('Time (s)')
ylabel('RMSE')
legend('error x', 'error y', 'error vx', 'error vy')
grid on





