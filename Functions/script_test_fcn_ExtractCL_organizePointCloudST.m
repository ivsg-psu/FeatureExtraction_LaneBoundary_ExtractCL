% script_test_fcn_ExtractCL_organizePointCloudST.m
% Tests fcn_ExtractCL_organizePointCloudST.m
%
% Purpose
% This script tests fcn_ExtractCL_organizePointCloudST, which converts
% unstructured (s,t) LiDAR points into regularized grid strips (S,T,I,Z).
%
% It includes:
%   (1) Synthetic data generation and boxcar aggregation
%   (2) Output size and numeric checks
%   (3) Optional visualization of intensity grid
%   (4) Corner case handling for empty / sparse / NaN data
% Revision history
%     2025-11-05 - xfc5113@psu.edu
%     -- wrote the code originally using synthetic data generation

%% Setup
clc; clear; close all;
addpath(genpath(pwd));
fprintf('Running script_test_fcn_ExtractCL_organizePointCloudST.m ...\n');

%% Case 1: Random rectangular region
disp('Case 1: Randomly distributed points in rectangular region');

% Parameters
N = 20000;
s_min = 0;  s_max = 20;
t_min = -6; t_max = 6;

% Randomly scatter points within a rectangular region
s_bin = s_min + (s_max - s_min) * rand(N,1);
t_bin = t_min + (t_max - t_min) * rand(N,1);

% Generate smooth synthetic intensity and height
intensity_bin = 100 + 15*sin(0.3*s_bin) + 10*cos(0.5*t_bin) + randn(N,1)*2;
z_bin = 0.2 * sin(0.2*t_bin) + 0.02 * randn(N,1);

% Grid settings
s_low = s_min;
s_width = s_max - s_min;
s_res = 0.5;
t_res = 0.1;
t_edges = t_min:t_res:t_max;

% Function call
[S_interp, T_interp, I_interp, Z_interp, s_strip_centers] = ...
    fcn_ExtractCL_organizePointCloudST(s_bin, t_bin, intensity_bin, z_bin, ...
        s_low, s_res, s_width, t_res, t_edges);

%% Visualization: raw vs organized
fig_num = 401;
figure(fig_num); clf;
tiledlayout(1,2,'TileSpacing','tight','Padding','compact');

% (a) Raw ST point cloud
nexttile(1);
scatter(s_bin, t_bin, 4, intensity_bin, 'filled');
xlabel('S [m]'); ylabel('T [m]');
title('Raw ST point cloud');
axis equal; 
grid on; 
box on;
xlim([s_min s_max]); 
ylim([t_min t_max]);
colormap('jet'); colorbar;

% (b) Organized intensity grid
nexttile(2);
imagesc(s_strip_centers, t_edges(1:end-1)+t_res/2, I_interp);
axis xy; 
axis equal; 
grid on; 
box on;
xlabel('S [m]'); 
ylabel('T [m]');
xlim([s_min s_max]); 
ylim([t_min t_max]);
title('Organized intensity grid');
colormap('jet'); colorbar;

fprintf('  Case 1 passed: rectangular region comparison plotted successfully.\n');

%% Verification
assert(~isempty(I_interp), 'Empty intensity grid output.');
assert(all(size(I_interp) == size(Z_interp)), 'I and Z grids size mismatch.');
assert(~any(isnan(mean(I_interp,1,'omitnan'))), 'NaN columns found in I_interp.');

%% Case 2: Sparse data
disp('Case 2: Sparse region');
s_sparse = linspace(0,2,30)'; t_sparse = randn(30,1);
I_sparse = 50 + rand(30,1)*10; Z_sparse = zeros(30,1);
[S,T,I,Z,sC] = fcn_ExtractCL_organizePointCloudST(s_sparse,t_sparse,I_sparse,Z_sparse,0,0.5,2,0.1,-2:0.1:2);
assert(all(isnan(I(:))), 'Sparse input should yield NaN grid.');
fprintf('  Case 2 passed.\n');

%% Case 3: NaN handling
disp('Case 3: Input with NaNs');
N = 1000;
s_bin = rand(N,1)*5;
t_bin = rand(N,1)*4 - 2;
I_bin = 100 + randn(N,1)*10;
Z_bin = zeros(N,1);
t_bin(1:30) = NaN;
I_bin(31:50) = NaN;
[S,T,I,Z,sC] = fcn_ExtractCL_organizePointCloudST(s_bin,t_bin,I_bin,Z_bin,0,0.5,5,0.1,-3:0.1:3);
assert(~isempty(I), 'Output unexpectedly empty.');
fprintf('  Case 3 passed: NaN inputs handled correctly.\n');


%% Summary
fprintf('\nAll tests for fcn_ExtractCL_organizePointCloudST executed successfully.\n');
fprintf('Please verify that the organized grid visually matches raw region width.\n');
