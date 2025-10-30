% script_test_fcn_ExtractCL_separateLaneMarkers.m
% Tests fcn_ExtractCL_separateLaneMarkers.m
%
% Revision history
%     2025-10-30 - xfc5113@psu.edu
%     -- wrote the code originally

%% Purpose
% This script tests fcn_ExtractCL_separateLaneMarkers, which classifies lane
% marker points into left/right lane boundaries based on their lateral (T)
% distribution, and identifies islands and outliers using robust MAD-based
% thresholds.

%% Setup
clc; clear; close all;

% Load example data
try
    [VehiclePose_Example, PointCloud_ENU_Example] = fcn_ExtractCL_loadMatData();
catch
    dataFolder = fullfile(pwd,'Data');
    [VehiclePose_Example, PointCloud_ENU_Example] = fcn_ExtractCL_loadMatData(dataFolder);
end

% Step 1: Project ENU → ST
fig_num = 6;
[pointCloud_ST_cell, ref_station, Seg] = ...
    fcn_ExtractCL_projectPC_ENUToST(PointCloud_ENU_Example, VehiclePose_Example, fig_num);

% Step 2: Lateral filtering
T_range = [-3, 3];
pointCloud_ST_filtered_cell = fcn_ExtractCL_filterPCinT(pointCloud_ST_cell, T_range);

% Step 3: Extract lane markers
s_length = 5;
N_s      = 10;
s_res    = s_length/N_s;
t_res    = 0.01;
min_pts  = 500;
fig_num  = -1;
pointCloud_ST_array = cell2mat(pointCloud_ST_filtered_cell);
[XYZST_lane_markers_array, HistoryData] = ...
    fcn_ExtractCL_extractLaneMarkers(pointCloud_ST_array, s_length, s_res, t_res, min_pts, Seg, fig_num);

%% Step 4: Separate left/right lane markers
disp('Step 4: Separating left and right lane markers...');
[LaneMarkers, islands, outliers] = fcn_ExtractCL_separateLaneMarkers(XYZST_lane_markers_array); %#ok<NASGU,ASGLU>

%% Verification
% Check outputs
assert(isstruct(LaneMarkers), 'LaneMarkers output must be a struct.');
assert(all(isfield(LaneMarkers, {'LaneMarkerLeft','LaneMarkerRight'})), ...
    'LaneMarkers struct missing expected fields.');

assert(isnumeric(islands) && size(islands,2) >= 5, ...
    'islands must be a numeric array with at least 5 columns [X Y Z S T].');
assert(isnumeric(outliers) && size(outliers,2) >= 5, ...
    'outliers must be a numeric array with at least 5 columns [X Y Z S T].');

% Display summary
fprintf('   fcn_ExtractCL_separateLaneMarkers executed successfully.\n');
fprintf('   Left markers:  %d points\n', size(LaneMarkers.LaneMarkerLeft,1));
fprintf('   Right markers: %d points\n', size(LaneMarkers.LaneMarkerRight,1));
fprintf('   Islands:       %d points\n', size(islands,1));
fprintf('   Outliers:      %d points\n', size(outliers,1));

% Optional visualization
figure(10);
clf; 
hold on; grid on; axis equal;
scatter(pointCloud_ST_array(:,1), pointCloud_ST_array(:,2), 20, pointCloud_ST_array(:,4),'filled','DisplayName','Point cloud');
scatter(LaneMarkers.LaneMarkerLeft(:,1), LaneMarkers.LaneMarkerLeft(:,2), 20, 'r', 'filled','DisplayName','Left side lane markers');
scatter(LaneMarkers.LaneMarkerRight(:,1), LaneMarkers.LaneMarkerRight(:,2), 20, 'green', 'filled','DisplayName','Right lane markers');
if ~isempty(islands)
    scatter(islands(:,4), islands(:,5), 60, 'm', '*', 'DisplayName','Islands');
end
if ~isempty(outliers)
    scatter(outliers(:,4), outliers(:,5), 60, 'k', 's','DisplayName','Outliers');
end
xlabel('S [m]');
ylabel('T [m]');
legend('Location', 'best')
title('Separated Lane Markers in ST Frame');
