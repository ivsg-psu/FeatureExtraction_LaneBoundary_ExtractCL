% script_test_fcn_ExtractCL_extractCL_WhiteStrip
% This script tests the main function fcn_ExtractCL_extractCL_WhiteStrip
% by generating synthetic point cloud data and reference poses.

% Clear environment
close all;
clear;
clc;

% Add function path if needed
% addpath(genpath('./Functions'));

%% Create synthetic point cloud data
N = 1000;
X = linspace(0, 20, N)';
Y = 2*sin(0.3 * X);
Z = zeros(N,1);
Intensity = 50 + 30*randn(N,1); % Add noise

% Assume intensity is higher at "lane marker" region
marker_idx = abs(Y) < 0.2;
Intensity(marker_idx) = Intensity(marker_idx) + 50;

% Add ST coordinates
Station = X;
Lateral = Y;

pointcloud_array = [X, Y, Z, Intensity, zeros(N,4), Station, Lateral];

%% Create synthetic reference pose (straight path)
Ref_Pose = [X, Y, Z, zeros(N,1), zeros(N,1), zeros(N,1), Station];

%% Set parameters
s_width = 1.0;
s_res = 0.1;
t_res = 0.05;
min_pts = 100;
fig_num = 1;

%% Call the function
[XYZSTE_Center_Line_Array, HistoryData] = fcn_ExtractCL_extractCL_WhiteStrip( ...
    pointcloud_array, s_width, s_res, t_res, min_pts, Ref_Pose, fig_num);

%% Visualization
figure(2);
plot(X, Y, 'k.'); hold on;
if ~isempty(XYZSTE_Center_Line_Array)
    plot(XYZSTE_Center_Line_Array(:,1), XYZSTE_Center_Line_Array(:,2), 'ro', 'LineWidth', 2);
end
xlabel('X [m]'); ylabel('Y [m]'); axis equal;
title('Extracted Center Line vs Raw Points');
legend('Raw Points', 'Extracted Center Line');
