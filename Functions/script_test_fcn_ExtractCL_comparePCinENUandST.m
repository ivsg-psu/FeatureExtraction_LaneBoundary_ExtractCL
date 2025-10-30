% script_test_fcn_ExtractCL_comparePCinENUandST.m
% Tests fcn_ExtractCL_comparePCinENUandST.m
%
% Revision history
%     2025-10-30 - xfc5113@psu.edu
%     -- wrote the code originally

%% Purpose
% This script tests fcn_ExtractCL_comparePCinENUandST, which visualizes and
% compares LiDAR point clouds in both ENU and ST coordinate systems within a
% specified S (station) range.

%% Setup
clc; clear; close all;

% Load example data
flag_do_curve = 1;
if flag_do_curve
    % Load default dataset (straight)
    [VehiclePose_Example, PointCloud_ENU_Example] = fcn_ExtractCL_loadMatData('Data','VehiclePoseExample.mat','PointCloudENUExample_Curve.mat');
else
    % Load example dataset (curve) 
    [VehiclePose_Example, PointCloud_ENU_Example] = fcn_ExtractCL_loadMatData();
end
 %

% Step 1: Project ENU → ST
fig_num = -1;
[pointCloud_ST_cell, ref_station, Seg] = ...
    fcn_ExtractCL_projectPC_ENUToST(PointCloud_ENU_Example, VehiclePose_Example, fig_num);

% Step 2: Lateral filtering
T_range = [-3, 3];
pointCloud_ST_filtered_cell = fcn_ExtractCL_filterPCinT(pointCloud_ST_cell, T_range);

%% Step 3: Compare pointclouds in ENU vs ST
disp('Step 3: Comparing point cloud representations in ENU and ST coordinate systems...');

ref_traj = [VehiclePose_Example(:,1:3), ref_station];
S_range = [0 100];
fig_num = fig_num + 20;

% Function call
fcn_ExtractCL_comparePCinENUandST(pointCloud_ST_filtered_cell, ref_traj, S_range, fig_num);

%% Verification
% Verify the figure was created
assert(ishandle(fig_num) && strcmp(get(fig_num,'Type'),'figure'), 'Expected figure not created.');
axObjs = get(fig_num,'Children');
assert(~isempty(axObjs), 'No axes found in the figure.');

% Verify S_range values are valid
assert(S_range(1) < S_range(2), 'Invalid S_range definition.');

% Print summary
fprintf('  fcn_ExtractCL_comparePCinENUandST executed successfully.\n');
fprintf('   Compared S range: [%.2f, %.2f]\n', S_range(1), S_range(2));

% Optional annotation
figure(fig_num);
sgtitle('Comparison of point cloud in ENU and ST coordinate systems');
