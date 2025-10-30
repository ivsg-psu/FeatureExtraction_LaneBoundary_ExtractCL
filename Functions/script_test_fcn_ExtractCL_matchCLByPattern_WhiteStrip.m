%% script_test_fcn_ExtractCL_matchCLByPattern_WhiteStrip
% This script tests the function fcn_ExtractCL_matchCLByPattern_WhiteStrip
% by creating a synthetic intensity map with a solid white line in the center.
%
% Author: Xinyu Cao
% Date: 2025-06-27

close all;
clear;

% Parameters
T_range = -0.5:0.01:0.5;  % lateral range (T)
S_range = 0:0.01:1.0;     % longitudinal range (S)
T_resolution = 0.01;

% Create synthetic intensity image with a single vertical stripe (like a lane marker)
intensity_data = 10*ones(length(S_range), length(T_range));
true_centerline_T_index = round(length(T_range)/2);
intensity_data(:, true_centerline_T_index) = 100;  % solid white strip in center

% Define a pattern template for the lane (e.g., Gaussian or symmetric bump)
pattern_width = 9;
pattern_template = zeros(pattern_width,1);
pattern_template(round(pattern_width/2)) = 1;

% Single-strip fallback pattern
single_strip_template = pattern_template;

% Optional inputs
fig_num = 101; % set to -1 to suppress plot
HistoryData = [];
pointcloud_in_bin = [];
s_strip_edges = [0 1];
t_strip_edges = [T_range(1) T_range(end)];

% Run the function
[center_line_mask, pattern_template_cell, extrema_filter_cell, ...
    best_pattern_template, best_fit_errors] = ...
    fcn_ExtractCL_matchCLByPattern_WhiteStrip(...
        intensity_data, ...
        pattern_template, ...
        T_resolution, ...
        single_strip_template, ...
        fig_num);

% Visualize results
figure(102); clf;
subplot(1,2,1);
imagesc(T_range, S_range, intensity_data); axis xy;
title('Input Intensity Image');
xlabel('T [m]'); ylabel('S [m]'); colorbar;

subplot(1,2,2);
imagesc(T_range, S_range, center_line_mask); axis xy;
title('Detected Centerline Mask');
xlabel('T [m]'); ylabel('S [m]'); colorbar;

% Confirm match quality
fprintf('Mean fit error: %.4f\n', mean(best_fit_errors,'omitnan'));
