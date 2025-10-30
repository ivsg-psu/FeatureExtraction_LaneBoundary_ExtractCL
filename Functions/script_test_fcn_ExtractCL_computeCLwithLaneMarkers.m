% script_test_fcn_ExtractCL_computeCLwithLaneMarkers.m
% Tests fcn_ExtractCL_computeCLwithLaneMarkers.m
%
% Revision history
%     2025-10-30 - xfc5113@psu.edu
%     -- wrote the code originally

%% Purpose
% This script tests fcn_ExtractCL_computeCLwithLaneMarkers, which computes the
% lane (or road) center line using previously detected lane markers and
% historical pattern data. The method determines lateral offsets between
% lane-marker clusters and outputs a center line in station–lateral (S,T)
% coordinates.

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
fig_num = 7;
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

% Step 4: Separate lane markers
[LaneMarkers, islands, outliers] = fcn_ExtractCL_separateLaneMarkers(XYZST_lane_markers_array); %#ok<ASGLU>

%% Step 5: Compute lane center line
disp('Step 5: Computing lane center line from detected markers...');
mode = 'left'; % use left lane markers as reference
[RoadCenterLine, LaneMarkerCenterLine] = ...
    fcn_ExtractCL_computeCLwithLaneMarkers(LaneMarkers, mode, HistoryData, t_res, Seg); %#ok<NASGU>

%% Verification
% Check outputs
assert(~isempty(RoadCenterLine) && isnumeric(RoadCenterLine), ...
    'RoadCenterLine must be a non-empty numeric array.');
assert(size(RoadCenterLine,2) >= 2, ...
    'RoadCenterLine should contain at least [S, T] columns.');

fprintf(' fcn_ExtractCL_computeCLwithLaneMarkers executed successfully.\n');
fprintf('   Center line points: %d\n', size(RoadCenterLine,1));


%% Plot detected road center line in ENU coordinate system (Show point cloud)
plotFormat.Color = [1 0 0];
plotFormat.Marker = '.';
plotFormat.MarkerSize = 10;
plotFormat.LineStyle = '-';
plotFormat.LineWidth = 3;
fig_num = 50;
fcn_ExtractCL_plotCenterLineXY(RoadCenterLine, pointCloud_ST_array, plotFormat, fig_num)
title('Extracted road center line (ENU Coordinates)', 'FontSize', 22);
%% Plot detected road center line in LLA coordinate system (Not show pointcloud)
setenv('MATLABFLAG_PLOTROAD_ALIGNMATLABLLAPLOTTINGIMAGES_LAT','-0.0000015');
setenv('MATLABFLAG_PLOTROAD_ALIGNMATLABLLAPLOTTINGIMAGES_LON','0.000005');

reference_latitude = 40.86368573;
reference_longitude = -77.83592832;
reference_altitude = 344.189;
ref_baseStationLLA = [reference_latitude, reference_longitude, reference_altitude];
plotFormat.Color = [1 0 0];
plotFormat.Marker = '.';
plotFormat.MarkerSize = 10;
plotFormat.LineStyle = '-';
plotFormat.LineWidth = 3;
fig_num = fig_num + 5;
fcn_ExtractCL_plotCenterLineLL(RoadCenterLine,ref_baseStationLLA,plotFormat,fig_num)
title('Extracted road center line (LLA Coordinates)', 'FontSize', 22);


