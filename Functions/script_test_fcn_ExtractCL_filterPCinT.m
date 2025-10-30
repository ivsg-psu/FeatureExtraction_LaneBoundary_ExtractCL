% script_test_fcn_ExtractCL_filterPCinT.m
% Tests fcn_ExtractCL_filterPCinT.m
%
% Revision history
%     2025-10-30 - xfc5113@psu.edu
%     -- wrote the code originally

%% Purpose
% This script tests fcn_ExtractCL_filterPCinT, which filters the LiDAR point
% cloud data in the (S,T) coordinate frame by a given lateral range T_range
% to reduce the amount of data for subsequent processing.

%% Setup
clc; clear; close all;

% Load example data
try
    [VehiclePose_Example, PointCloud_ENU_Example] = fcn_ExtractCL_loadMatData();
catch
    dataFolder = fullfile(pwd,'Data');
    [VehiclePose_Example, PointCloud_ENU_Example] = fcn_ExtractCL_loadMatData(dataFolder);
end

% Step 1: project ENU → ST
fig_num = 3;
[pointCloud_ST_cell, ref_station, Seg] = ...
    fcn_ExtractCL_projectPC_ENUToST(PointCloud_ENU_Example, VehiclePose_Example, fig_num);

% Confirm valid projection output
assert(iscell(pointCloud_ST_cell) && ~isempty(pointCloud_ST_cell), ...
    'pointCloud_ST_cell must be a non-empty cell array.');

%% Step 2: lateral filtering
disp('Step 2: Filtering the processing region in the lateral direction to reduce the amount of data to be processed....');

% Define lateral range (in meters)
T_range = [-3, 3];

% Function call
pointCloud_ST_filtered_cell = fcn_ExtractCL_filterPCinT(pointCloud_ST_cell, T_range);

%% Verification
% Basic checks
assert(iscell(pointCloud_ST_filtered_cell), 'Output must be a cell array.');
assert(length(pointCloud_ST_filtered_cell) == length(pointCloud_ST_cell), ...
    'Output cell array length must match input.');

% Sample check: verify T values are within bounds
nonemptyCells = find(~cellfun(@isempty, pointCloud_ST_filtered_cell), 1);
if ~isempty(nonemptyCells)
    testIdx = nonemptyCells(1);
    T_values = pointCloud_ST_filtered_cell{testIdx}(:,10);
    assert(all(T_values >= T_range(1) & T_values <= T_range(2)), ...
        'Filtered T values exceed the specified range.');
end

% Display summary
fprintf('fcn_ExtractCL_filterPCinT executed successfully.\n');
fprintf('T_range applied: [%.1f, %.1f]\n', T_range(1), T_range(2));
fprintf('Number of filtered frames: %d\n', length(pointCloud_ST_filtered_cell));

% Optional visualization
if ~isempty(pointCloud_ST_filtered_cell)
    figure(fig_num + 1);
    clf;
    hold on; grid on; axis equal;
    for k = 1:min(500, length(pointCloud_ST_filtered_cell))
        
        pts = pointCloud_ST_filtered_cell{k};
        if isempty(pts)
            continue;
        end
        scatter(pts(:,9), pts(:,10), 20, pts(:,4), 'filled');
        hold on;
    end
    xlabel('S [m]');
    ylabel('T [m]');
    title('Filtered LiDAR points in ST frame');
end
