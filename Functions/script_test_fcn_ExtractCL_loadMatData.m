% script_test_fcn_ExtractCL_loadMatData.m
% Tests fcn_ExtractCL_loadMatData.m
%
% Revision history
%     2025-10-30 - xfc5113@psu.edu
%     -- wrote the code originally

%% Purpose:
% This script tests the data-loading function fcn_ExtractCL_loadMatData,
% which should load example variables such as VehiclePose_Example and
% PointCloud_ENU_Example from the /Data directory.

%% Setup
clc;
clear;
close all;

% Define expected file paths
dataFolder = fullfile(pwd,'Data');
expectedFiles = {'VehiclePoseExample.mat','PointCloudENUExample.mat'};

%% Function Call
% Run the function under test
[VehiclePose_Example, PointCloud_ENU_Example] = fcn_ExtractCL_loadMatData(dataFolder);

%% Verification
% Check loaded variables
disp('Loaded variables:');
whos VehiclePose_Example PointCloud_ENU_Example

% Basic assertions
assert(~isempty(VehiclePose_Example), 'VehiclePose_Example is empty!');
assert(~isempty(PointCloud_ENU_Example), 'PointCloud_ENU_Example is empty!');
fprintf('fcn_ExtractCL_loadMatData test passed.\n');
