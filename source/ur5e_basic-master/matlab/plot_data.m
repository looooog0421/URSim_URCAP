%% 测试UR5e实时跟踪效果
%% matlab浏览到当前目录 F5运行
clc;
close all;
clear;

%% plot sin test data
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure; 
hold on;
grid on; grid minor;

data_path = '../data/';
suffix_traj = 'sin_test.txt';
path = [data_path , suffix_traj];
result = load(path);
plot(result(:,1), result(:,4),'LineWidth', 1.2);
plot(result(:,1), result(:,7),'LineWidth', 1.2);

legend('real','ref'); grid on; grid minor;