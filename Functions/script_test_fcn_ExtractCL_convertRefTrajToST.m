% script_test_fcn_ExtractCL_convertRefTrajToST.m
% Tests fcn_ExtractCL_convertRefTrajToST.m
%
% Revision history
%     2025-11-05 - xfc5113@psu.edu
%     -- wrote the code originally

%% Purpose
% This script tests fcn_ExtractCL_convertRefTrajToST, which constructs
% segment-wise geometry (tangent, normal, and station) from a reference
% vehicle trajectory. The output is used in LiDAR → ST projection.
%
% It includes:
%   (1) Real trajectory test (loaded from Data)
%   (2) Synthetic straight-line trajectory
%   (3) Synthetic circular trajectory
%   (4) Degenerate / corner cases (zero-length or single-point)

%% Setup
clc; clear; close all;

addpath(genpath(pwd));  % ensure functions are visible

%% Case 1: Load real example trajectory from Data
disp('Case 1: Testing with real VehiclePose_Example.mat data...');

% Load reference vehicle pose
[VehiclePose_Example, ~] = fcn_ExtractCL_loadMatData('Data', ...
    'VehiclePoseExample.mat','PointCloudENUExample.mat');

% Run conversion function
ST_struct_real = fcn_ExtractCL_convertRefTrajToST(VehiclePose_Example, 0);

% Basic verification
assert(isstruct(ST_struct_real), 'Output is not a struct.');
assert(all(isfield(ST_struct_real, {'traj_XY','ref_station','seg_tangent','seg_normal'})), ...
    'Missing expected fields in output struct.');
assert(length(ST_struct_real.ref_station) == size(ST_struct_real.traj_XY,1), ...
    'ref_station size mismatch.');
assert(all(ST_struct_real.segment_length > 0), ...
    'Some segment lengths are non-positive.');

fprintf('  Case 1 passed: Real data conversion successful. %d segments generated.\n', ...
    length(ST_struct_real.segment_length));

%% Case 2: Synthetic straight-line trajectory
disp('Case 2: Testing with straight-line synthetic trajectory...');

x = (0:1:50)'; 
y = zeros(size(x));
Ref_Pose_straight = [x y];

ST_struct_straight = fcn_ExtractCL_convertRefTrajToST(Ref_Pose_straight, 0);

% Expected results
expected_length = diff(ST_struct_straight.ref_station);
assert(all(abs(expected_length - 1) < 1e-9), 'Station spacing not uniform for straight line.');
assert(all(abs(ST_struct_straight.seg_tangent(:,1) - 1) < 1e-9), 'Tangent X should be 1 for straight line.');
assert(all(abs(ST_struct_straight.seg_tangent(:,2)) < 1e-9), 'Tangent Y should be 0 for straight line.');
fprintf('  Case 2 passed: Straight-line trajectory test successful.\n');

%% Case 3: Synthetic circular trajectory
disp('Case 3: Testing with circular trajectory...');

theta = linspace(0, pi/2, 40)';
R = 10;
x = R * cos(theta);
y = R * sin(theta);
Ref_Pose_circle = [x y];

ST_struct_circle = fcn_ExtractCL_convertRefTrajToST(Ref_Pose_circle, 0);

% Basic checks
assert(length(ST_struct_circle.ref_station) == length(theta), ...
    'Station vector length mismatch.');
assert(all(ST_struct_circle.segment_length > 0), ...
    'Segment length invalid for circular path.');
fprintf('  Case 3 passed: Circular trajectory test successful.\n');

%% Case 4: Degenerate and corner cases
disp('Case 4: Testing degenerate inputs...');

% (a) Only one point
try
    fcn_ExtractCL_convertRefTrajToST([0 0], 0);
    error('Expected error for single-point input not thrown.');
catch ME
    assert(contains(ME.identifier,'TooFewSamples'), 'Unexpected error type for single-point input.');
    fprintf('  Subcase (a) passed: single-point input correctly rejected.\n');
end

% (b) Two identical points
try
    fcn_ExtractCL_convertRefTrajToST([0 0; 0 0], 0);
    error('Expected non-fatal error or zero-length segment warning not thrown.');
catch ME
    fprintf('  Subcase (b): identical-points case handled with message: %s\n', ME.message);
end

fprintf('\nAll test cases for fcn_ExtractCL_convertRefTrajToST executed successfully.\n');
