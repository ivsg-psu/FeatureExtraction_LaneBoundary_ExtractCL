%% script_test_fcn_ExtractCL_filterPointCloudInROI.m
% Tests fcn_ExtractCL_filterPointCloudInROI.m

% Revision history:
%   2025_06_23 by Xinyu Cao
%   -- Initial test script creation for point cloud ROI filtering

%% Set up the workspace
close all;
clc;
clear;

%% Generate synthetic LiDAR data
N_scans = 3;
points_per_scan = 1000;
LiDAR_PointCloud_Cell = cell(N_scans,1);
rng(0); % for reproducibility
for i = 1:N_scans
    X = randn(points_per_scan,1)*10;
    Y = randn(points_per_scan,1)*5;
    Z = rand(points_per_scan,1)*2;
    Intensity = rand(points_per_scan,1);
    LiDAR_PointCloud_Cell{i} = [X Y Z Intensity];
end
LiDAR_Data_struct.PointCloud = LiDAR_PointCloud_Cell;

%% Test 1: Cartesian ROI
ROI.mode = 'cartesian';
ROI.X_lim = [-5 5];
ROI.Y_lim = [-3 3];
ROI.Z_lim = [0.5 1.5];

filtered_struct = fcn_ExtractCL_filterPointCloudInROI(LiDAR_Data_struct, ROI);
filtered_cell = fcn_ExtractCL_filterPointCloudInROI(LiDAR_PointCloud_Cell, ROI);

assert(iscell(filtered_cell));
assert(isstruct(filtered_struct));
assert(isfield(filtered_struct,'PointCloud'));
assert(isequal(length(filtered_struct.PointCloud), N_scans));

scan1 = filtered_cell{1};
assert(all(scan1(:,1) >= -5 & scan1(:,1) <= 5));
assert(all(scan1(:,2) >= -3 & scan1(:,2) <= 3));
assert(all(scan1(:,3) >= 0.5 & scan1(:,3) <= 1.5));

%% Test 2: Polar ROI
ROI.mode = 'polar';
ROI.R_lim = [0 8];
ROI.Theta_lim = [-90 90];
ROI.Z_lim = [0 2];

filtered_polar = fcn_ExtractCL_filterPointCloudInROI(LiDAR_PointCloud_Cell, ROI);
scan2 = filtered_polar{2};
R2 = hypot(scan2(:,1), scan2(:,2));
Theta2 = atan2d(scan2(:,2), scan2(:,1));
assert(all(R2 >= 0 & R2 <= 8));
assert(all(Theta2 >= -90 & Theta2 <= 90));

%% Test 3: Fast mode (fig_num = -1)
ROI.mode = 'cartesian';
fig_num = -1;
filtered_fast = fcn_ExtractCL_filterPointCloudInROI(LiDAR_Data_struct, ROI, fig_num);
assert(iscell(filtered_fast.PointCloud));

%% Test 4: No Z limit
ROI = struct('mode','cartesian','X_lim',[-5 5],'Y_lim',[-3 3]);
filtered_nz = fcn_ExtractCL_filterPointCloudInROI(LiDAR_PointCloud_Cell, ROI);
scan3 = filtered_nz{3};
assert(all(scan3(:,1) >= -5 & scan3(:,1) <= 5));
assert(all(scan3(:,2) >= -3 & scan3(:,2) <= 3));

%% Test 5: Invalid mode
ROI.mode = 'invalid_mode';
try
    fcn_ExtractCL_filterPointCloudInROI(LiDAR_PointCloud_Cell, ROI);
    error('Expected error for invalid mode did not occur.');
catch ME
    assert(contains(ME.message,'Unsupported ROI.mode'));
end

%% Done
fprintf('All tests passed for fcn_ExtractCL_filterPointCloudInROI.m\n');