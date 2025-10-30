% script_test_fcn_ExtractCL_plotCenterLineXY.m
% Tests fcn_ExtractCL_plotCenterLineXY.m
%
% Revision history
%     2025-10-30 - xfc5113@psu.edu
%     -- wrote the code originally

%% Purpose
% This script verifies that fcn_ExtractCL_plotCenterLineXY can plot the mapping
% van reference trajectory alongside LiDAR point cloud samples in ENU.

%% Setup
clc; clear; close all;

% Load example data (supports either no-arg or folder-arg loader)
try
    [VehiclePose_Example, PointCloud_ENU_Example] = fcn_ExtractCL_loadMatData();
catch
    dataFolder = fullfile(pwd,'Data');
    [VehiclePose_Example, PointCloud_ENU_Example] = fcn_ExtractCL_loadMatData(dataFolder);
end

% Basic sanity checks on loaded data
assert(~isempty(VehiclePose_Example), 'VehiclePose_Example is empty.');
assert(iscell(PointCloud_ENU_Example) && ~isempty(PointCloud_ENU_Example), ...
    'PointCloud_ENU_Example must be a non-empty cell array.');

%% Prepare inputs to the plot function
N_frames = length(PointCloud_ENU_Example);
assert(N_frames >= 1, 'PointCloud_ENU_Example must contain at least 1 frame.');

fig_num = 1;

plotFormat.Color = [1 0 0];
plotFormat.Marker = '.';
plotFormat.MarkerSize = 10;
plotFormat.LineStyle = '-';
plotFormat.LineWidth = 3;

frames_to_view = 1:round(N_frames/2);
PointCloud_ENU_toBeView = PointCloud_ENU_Example(frames_to_view);
PointCloud_ENU_toBeView_Array = cell2mat(PointCloud_ENU_toBeView);

% Additional sanity checks on concatenated array
assert(isnumeric(PointCloud_ENU_toBeView_Array) && size(PointCloud_ENU_toBeView_Array,2) >= 3, ...
    'PointCloud_ENU_toBeView_Array must be numeric with at least 3 columns [X Y Z ...].');

%% Function Call
fcn_ExtractCL_plotCenterLineXY( ...
    VehiclePose_Example, ...
    PointCloud_ENU_toBeView_Array, ...
    plotFormat, ...
    fig_num);

title('LiDAR pointcloud vs reference trajectory in ENU');

%% Verification
% Verify the figure exists and has at least one line object
assert(ishandle(fig_num) && strcmp(get(fig_num,'Type'),'figure'), 'Figure was not created.');
axesChildren = findobj(get(fig_num,'CurrentAxes'),'Type','line');
assert(~isempty(axesChildren), 'No line objects found in the plot. Plot may have failed.');

disp('fcn_ExtractCL_plotCenterLineXY test completed successfully.');
