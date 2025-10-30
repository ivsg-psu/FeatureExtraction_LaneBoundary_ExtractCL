% script_test_fcn_ExtractCL_projectPC_ENUToST.m
% Tests fcn_ExtractCL_projectPC_ENUToST.m
%
% Revision history
%     2025-10-30 - xfc5113@psu.edu
%     -- wrote the code originally

%% Purpose
% This script tests fcn_ExtractCL_projectPC_ENUToST, which projects LiDAR
% point cloud data from the ENU coordinate system into the station–lateral
% (S,T) frame defined by the reference vehicle trajectory.

%% Setup
clc; clear; close all;

% Load example data
try
    [VehiclePose_Example, PointCloud_ENU_Example] = fcn_ExtractCL_loadMatData();
catch
    dataFolder = fullfile(pwd,'Data');
    [VehiclePose_Example, PointCloud_ENU_Example] = fcn_ExtractCL_loadMatData(dataFolder);
end

% Check data validity
assert(~isempty(VehiclePose_Example), 'VehiclePose_Example is empty.');
assert(iscell(PointCloud_ENU_Example) && ~isempty(PointCloud_ENU_Example), ...
    'PointCloud_ENU_Example must be a non-empty cell array.');

% Select figure number
fig_num = 2;

%% Function Call
disp('Step 1: Projecting LiDAR point cloud to station-lateral (S,T) frame...');
[pointCloud_ST_cell, ref_station, Seg] = ...
    fcn_ExtractCL_projectPC_ENUToST(PointCloud_ENU_Example, VehiclePose_Example, fig_num);

%% Verification
% Basic checks
assert(iscell(pointCloud_ST_cell) && ~isempty(pointCloud_ST_cell), ...
    'Output pointCloud_ST_cell must be a non-empty cell array.');
assert(isnumeric(ref_station) && ~isempty(ref_station), ...
    'ref_station must be a numeric vector.');
assert(isstruct(Seg), 'Seg must be a struct.');

% Check structure fields (if defined)
expectedFields = {'S_ref','T_ref','Heading_ref'};
missingFields = setdiff(expectedFields, fieldnames(Seg));
if ~isempty(missingFields)
    warning('Seg is missing expected fields: %s', strjoin(missingFields, ', '));
end

% Display summary
fprintf(' fcn_ExtractCL_projectPC_ENUToST executed successfully.\n');
fprintf('   Number of ST frames: %d\n', length(pointCloud_ST_cell));
fprintf('   Reference stations: [%.2f, %.2f]\n', min(ref_station), max(ref_station));

% Optional: visualize the first few ST frames
if ~isempty(pointCloud_ST_cell)
    figure(fig_num + 1);
    clf;
    hold on; grid on; axis equal;
    for k = 1:min(500, length(pointCloud_ST_cell))
        
        pts = pointCloud_ST_cell{k};
        if isempty(pts)
            continue;
        end
        scatter(pts(:,9), pts(:,10), 20, pts(:,4), 'filled');
        hold on;
    end
    xlabel('S [m]');
    ylabel('T [m]');
    title('Sample projection results in ST frame');
end
