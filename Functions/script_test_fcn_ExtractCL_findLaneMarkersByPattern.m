%% script_test_fcn_ExtractCL_findLaneMarkersByPattern.m
% Tests fcn_ExtractCL_findLaneMarkersByPattern.m% 
% Purpose:
%   Validate pattern-based lane marker detection for multiple configurations.
%   Cases:
%     1) White + double yellow
%     2) Single both sides (new)
%     3) Dual single-strip markers
%     4) Super-wide mixed region (single + single + double yellow)
%     5) Corner and robustness checks
%
% Revision history:
%   2025_11_05 - Xinyu Cao
%   -- Added new 'single_both_sides' pattern as Case 2
%   -- Added 1 m zero padding to all synthetic samples
%


%% Setup
close all; clc; clear;
addpath(genpath(pwd));

t_res = 0.02;        % Lateral resolution
pad_m = 1.0;         % 1 m zero padding each side
pad_idx = round(pad_m / t_res);
templateUpdateCount = 0;

%% ===== CASE 1: White + Double Yellow =====
disp('Case 1: White + Double Yellow...');
lane_type = 'left_double_yellow_right_white';
pt = fcn_ExtractCL_createLanePattern(lane_type, t_res);
L = numel(pt);
Lp = L + 2*pad_idx;
T_range = ((0:Lp-1) - pad_idx)' * t_res;

sig = zeros(Lp,1);
sig(pad_idx + (1:L)) = pt;

g = exp(-((-5:5).^2)/(2*1.5^2)); g = g/sum(g);
base = conv(sig, g', 'same');
base = 80 + 70*base;

N_T = Lp; N_S = 8;
intensity_data = repmat(base, 1, N_S) + 3*randn(N_T, N_S);
t_profile = repmat(T_range, 1, N_S);

fig_num = 101;
[mask, pattern_cell, extrema_cell, best_template, best_errors] = ...
    fcn_ExtractCL_findLaneMarkersByPattern(intensity_data, pt, t_res, templateUpdateCount, t_profile, fig_num);

figure(fig_num+1); clf;
tiledlayout(1,2,'TileSpacing','compact');
nexttile; imagesc(1:N_S, T_range, intensity_data); axis xy;
xlabel('Strip index'); ylabel('T [m]');
title('Case 1: Input intensity (white + double yellow)'); colorbar;
nexttile; imagesc(1:N_S, T_range, mask); axis xy;
xlabel('Strip index'); ylabel('T [m]');
title('Detected mask'); colorbar; colormap('gray');
assert(any(mask(:)), 'No lane markers detected in Case 1.');
fprintf('  Case 1 passed.\n\n');


%% ===== CASE 2: Single Both Sides =====
disp('Case 2: Single Both Sides...');
lane_type = 'single_both_sides';
pt_both = fcn_ExtractCL_createLanePattern(lane_type, t_res);
L = numel(pt_both);
Lp = L + 2*pad_idx;
T_range = ((0:Lp-1) - pad_idx)' * t_res;

sig = zeros(Lp,1);
sig(pad_idx + (1:L)) = pt_both;

g = exp(-((-5:5).^2)/(2*1.2^2)); g = g/sum(g);
base = conv(sig, g', 'same');
base = 85 + 70*base;

N_T = Lp; N_S = 8;
intensity_data = repmat(base, 1, N_S) + 3*randn(N_T, N_S);
t_profile = repmat(T_range, 1, N_S);

fig_num = 201;
[mask, pattern_cell, extrema_cell, best_template, best_errors] = ...
    fcn_ExtractCL_findLaneMarkersByPattern(intensity_data, pt_both, t_res, templateUpdateCount, t_profile, fig_num);

figure(fig_num+1); clf;
tiledlayout(1,2,'TileSpacing','compact');
nexttile; imagesc(1:N_S, T_range, intensity_data); axis xy;
xlabel('Strip index'); ylabel('T [m]');
title('Case 2: Input intensity (single both sides)'); colorbar;
nexttile; imagesc(1:N_S, T_range, mask); axis xy;
xlabel('Strip index'); ylabel('T [m]');
title('Detected mask'); colorbar; colormap('gray');
assert(any(mask(:)), 'No lane markers detected in Case 2.');
fprintf('  Case 2 passed.\n\n');


%% ===== CASE 3: Find left_double_yellow_right_white in mixed layouts =====
disp('Case 3: Two lanes with different layout...');

pt_mix = fcn_ExtractCL_createLanePattern('left_double_yellow_right_white', t_res);
L = numel(pt_mix);
Lp = 2*L + 2*pad_idx;
T_range = ((0:Lp-1) - pad_idx)' * t_res;

sig = zeros(Lp,1);
ptA = fcn_ExtractCL_createLanePattern('single_strip', t_res);
LA = numel(ptA);
sig(pad_idx + (1:LA)) = max(sig(pad_idx + (1:LA)), ptA);

shift = pad_idx + round(3.0/t_res);
idxShift = shift + (1:L); 
idxShift = idxShift(idxShift <= Lp);
sig(idxShift) = max(sig(idxShift), pt_mix(1:numel(idxShift)));

g = exp(-((-8:8).^2)/(2*2.0^2)); g = g/sum(g);
base = conv(sig, g', 'same');
base = 75 + 70*base;

N_T = Lp; N_S = 6;
intensity_data = repmat(base, 1, N_S) + 3*randn(N_T, N_S);
t_profile = repmat(T_range, 1, N_S);

fig_num = 301;
[mask, pattern_cell, extrema_cell, best_template, best_errors] = ...
    fcn_ExtractCL_findLaneMarkersByPattern(intensity_data, pt_mix, t_res, templateUpdateCount, t_profile, fig_num);

figure(fig_num+1); clf;
tiledlayout(1,2,'TileSpacing','compact');
nexttile; imagesc(1:N_S, T_range, intensity_data); axis xy;
xlabel('Strip index'); ylabel('T [m]');
title('Case 3: Input intensity (mixed types)'); colorbar;
nexttile; imagesc(1:N_S, T_range, mask); axis xy;
xlabel('Strip index'); ylabel('T [m]');
title('Detected mask'); colorbar; colormap('gray');
assert(any(mask(:)), 'No lane markers detected in Case 4.');
fprintf('  Case 4 passed.\n\n');


%% ===== CASE 4: Find two single lane markers in mixed layouts =====
% NOTE: This is a fail case that need to be fixed later
disp('Case 4: Find two single lane markers in mixed layouts...');

pt_mix = fcn_ExtractCL_createLanePattern('single_both_sides', t_res);
L = numel(pt_mix);
Lp = 2*L + 2*pad_idx;
T_range = ((0:Lp-1) - pad_idx)' * t_res;

sig = zeros(Lp,1);
ptA = fcn_ExtractCL_createLanePattern('single_strip', t_res);
LA = numel(ptA);
sig(pad_idx + (1:LA)) = max(sig(pad_idx + (1:LA)), ptA);

shift = pad_idx + round(3.0/t_res);
idxShift = shift + (1:L); 
idxShift = idxShift(idxShift <= Lp);
sig(idxShift) = max(sig(idxShift), pt_mix(1:numel(idxShift)));

g = exp(-((-8:8).^2)/(2*2.0^2)); g = g/sum(g);
base = conv(sig, g', 'same');
base = 75 + 70*base;

N_T = Lp; N_S = 6;
intensity_data = repmat(base, 1, N_S) + 3*randn(N_T, N_S);
t_profile = repmat(T_range, 1, N_S);

fig_num = 401;
[mask, pattern_cell, extrema_cell, best_template, best_errors] = ...
    fcn_ExtractCL_findLaneMarkersByPattern(intensity_data, pt_mix, t_res, templateUpdateCount, t_profile, fig_num);

figure(fig_num+1); clf;
tiledlayout(1,2,'TileSpacing','compact');
nexttile; imagesc(1:N_S, T_range, intensity_data); axis xy;
xlabel('Strip index'); ylabel('T [m]');
title('Case 3: Input intensity (mixed types)'); colorbar;
nexttile; imagesc(1:N_S, T_range, mask); axis xy;
xlabel('Strip index'); ylabel('T [m]');
title('Detected mask'); colorbar; colormap('gray');
assert(any(mask(:)), 'No lane markers detected in Case 4.');
fprintf('  Case 4 passed.\n\n');

%% ===== CASE 5: Corner / Robustness =====
disp('Case 5: Corner and robustness checks...');

% (a) Empty data
try
    fcn_ExtractCL_findLaneMarkersByPattern([], pt, t_res, 0, []);
    error('Expected error not thrown for empty data.');
catch ME
    assert(contains(ME.message, 'non-empty numeric 2-D'), 'Unexpected error for empty data.');
end

% (b) t_profile size mismatch
try
    fcn_ExtractCL_findLaneMarkersByPattern(ones(8,5), pt, t_res, 0, ones(9,5));
    error('Expected size mismatch error not triggered.');
catch ME
    assert(contains(ME.message, 'same size'), 'Missing/incorrect t_profile size error.');
end

% (c) Bad T_resolution
try
    fcn_ExtractCL_findLaneMarkersByPattern(ones(10,3), pt, -0.1, 0, ones(10,3));
    error('Expected bad T_resolution error not triggered.');
catch ME
    assert(contains(ME.message, 'positive finite scalar'), 'Missing/incorrect T_resolution error.');
end

%% Summary
fprintf('\nAll tests passed for fcn_ExtractCL_findLaneMarkersByPattern.m\n');
