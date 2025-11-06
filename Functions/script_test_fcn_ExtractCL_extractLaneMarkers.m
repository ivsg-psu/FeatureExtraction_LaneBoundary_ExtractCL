% script_test_fcn_ExtractCL_extractLaneMarkers.m
% Tests fcn_ExtractCL_extractLaneMarkers.m
%
% Revision history
%     2025-10-30 - xfc5113@psu.edu
%     -- wrote the code originally
%     2025_11_05 - xfc5113@psu.edu
%     -- added corner-case validation

%% TO DO:
% -- add fail case and bad layout template
% Future tests should include:
%   (1) intentionally corrupted or incomplete S-strips to test robustness
%   (2) irregular pattern spacing (e.g., missing double yellow or mixed types)
%   (3) mismatched S/T resolution and small sample count to confirm stability

%% Purpose
% This script tests fcn_ExtractCL_extractLaneMarkers, which extracts lane marker
% points from the organized LiDAR point cloud (in ST coordinates) using
% intensity extrema filtering and pattern matching over sliding S-windows.

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
fig_num = 5;
[pointCloud_ST_cell, ref_station, Seg] = ...
    fcn_ExtractCL_projectPC_ENUToST(PointCloud_ENU_Example, VehiclePose_Example, fig_num);

% Step 2: Lateral filtering
T_range = [-3, 3];
pointCloud_ST_filtered_cell = fcn_ExtractCL_filterPCinT(pointCloud_ST_cell, T_range);

%% Step 3: Extract lane markers
disp('Step 3: Extracting center line based on intensity extrema and patterns...');

% Initialize parameters
s_length = 5;       % window length [m]
N_s      = 10;      % number of bins per window
s_res    = s_length/N_s;
t_res    = 0.01;    % lateral resolution [m]
min_pts  = 500;     % minimum points per S-strip
fig_num  = -1;      % suppress plotting by default

% Convert cell array to matrix
pointCloud_ST_array = cell2mat(pointCloud_ST_filtered_cell); %#ok<NASGU>

% Function call
[XYZST_lane_markers_array, HistoryData] = ...
    fcn_ExtractCL_extractLaneMarkers(pointCloud_ST_array, s_length, s_res, t_res, min_pts, Seg, fig_num); %#ok<NASGU>

%% Verification
% Check output validity
assert(~isempty(XYZST_lane_markers_array) && isnumeric(XYZST_lane_markers_array), ...
    'Output XYZST_lane_markers_array must be a non-empty numeric array.');
assert(isstruct(HistoryData) && isfield(HistoryData, 'LanePattern'), ...
    'HistoryData must be a struct containing field LanePattern.');

fprintf('   fcn_ExtractCL_extractLaneMarkers executed successfully.\n');
fprintf('   Extracted %d lane marker points.\n', size(XYZST_lane_markers_array,1));

% Optional visualization
if fig_num > 0
    figure(fig_num);
    clf;
    hold on; grid on; axis equal;
   
    scatter(pointCloud_ST_array(:,1), pointCloud_ST_array(:,2), 20, pointCloud_ST_array(:,4),'filled','DisplayName','Point cloud');
    scatter(XYZST_lane_markers_array(:,1), XYZST_lane_markers_array(:,2), 20, 'red','filled','DisplayName','Detected lane markers');
    xlabel('X-East [m]');
    ylabel('Y-North [m]');
    title('Extracted Lane Marker Points');
end

%% Corner Cases
%% Corner Case 1
disp('Corner Case 1: Empty point cloud input...');
try
    [XYZST_LaneMarkers_Array, HistoryData] = ...
        fcn_ExtractCL_extractLaneMarkers([], 0.5, 0.25, 0.05, 10, Seg, -1);
    assert(isempty(XYZST_LaneMarkers_Array), 'Expected empty output for empty input.');
    fprintf('  Passed: empty input handled gracefully.\n');
catch ME
    fprintf('  Failed: %s\n', ME.message);
end
%% Corner Case 2
disp('Corner Case 2: Too few points...');
small_pc = rand(5,10);  % only 5 points
small_pc(:,9) = linspace(0,1,5);
small_pc(:,10) = linspace(-0.5,0.5,5);
try
    [XYZST_LaneMarkers_Array, HistoryData] = ...
        fcn_ExtractCL_extractLaneMarkers(small_pc, 0.5, 0.25, 0.05, 10, Seg, -1);
    assert(isempty(XYZST_LaneMarkers_Array), 'Expected no detections for sparse input.');
    fprintf('  Passed: sparse input handled.\n');
catch ME
    fprintf('  Failed: %s\n', ME.message);
end

%% Corner Case 3
disp('Corner Case 3: Invalid input layout...');
bad_pc = rand(100,8);
try
    fcn_ExtractCL_extractLaneMarkers(bad_pc, 0.5, 0.25, 0.05, 10, Seg, -1);
    error('Expected error for invalid input layout not thrown.');
catch ME
    assert(contains(ME.identifier,'BadInput'), 'Unexpected error identifier.');
    fprintf('  Passed: invalid input layout rejected correctly.\n');
end
