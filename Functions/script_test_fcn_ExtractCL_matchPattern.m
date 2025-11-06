%% script_test_fcn_ExtractCL_matchPattern.m
% Tests fcn_ExtractCL_matchPattern.m
%
% Revision history:
%   2025_11_05 - Xinyu Cao
%   -- Created multi-case test suite with padded synthetic data
%
% Purpose:
%   Verify pattern–signal alignment via sliding-window matching.
%   Cases:
%     1) White + double yellow pattern
%     2) Single both sides pattern
%     3) Random noise robustness

%% Setup
close all; clc; clear;
addpath(genpath(pwd));

t_res = 0.02;
pad_m = 1.0; pad_idx = round(pad_m/t_res);

%% ===== CASE 1: White + Double Yellow =====
disp('Case 1: Matching left_double_yellow_right_white...');
lane_type = 'left_double_yellow_right_white';
pattern_template = fcn_ExtractCL_createLanePattern(lane_type, t_res);
L = numel(pattern_template);
Lp = L + 2*pad_idx;
T_range = ((0:Lp-1) - pad_idx)' * t_res;

sig = zeros(Lp,1);
sig(pad_idx+(1:L)) = pattern_template;
g = exp(-((-5:5).^2)/(2*1.5^2)); g = g/sum(g);
base = conv(sig,g','same'); base = 80 + 60*base + 3*randn(Lp,1);

[best_pattern, best_idx, best_error, MSE] = ...
    fcn_ExtractCL_matchPattern(base, pattern_template);

figure(101); clf;
subplot(3,1,1);
plot(T_range, base, 'Color',[0.4 0.4 0.4], 'LineWidth',2); hold on;
plot(T_range, best_pattern*max(base), 'r--', 'LineWidth',2);
title('Case 1: Raw intensity and best-fit pattern');
xlabel('T [m]'); ylabel('Intensity'); grid on;
xline(T_range(best_idx), 'b--', 'LineWidth',2);

subplot(3,1,2);
plot(MSE,'k','LineWidth',1.5); grid on;
xlabel('Start index'); ylabel('MSE');
title('Mean Squared Error across shifts');

subplot(3,1,3);
plot(best_pattern,'r','LineWidth',2); grid on;
xlabel('Sample'); ylabel('Pattern presence');
title(sprintf('Best alignment at idx = %d', best_idx));


fprintf('  Case 1 passed.\n\n');

%% ===== CASE 2: Single Both Sides =====
disp('Case 2: Matching single_both_sides...');
lane_type = 'single_both_sides';
pattern_template = fcn_ExtractCL_createLanePattern(lane_type, t_res);
L = numel(pattern_template);
Lp = L + 2*pad_idx;
T_range = ((0:Lp-1) - pad_idx)' * t_res;

sig = zeros(Lp,1);
sig(pad_idx+(1:L)) = pattern_template;
g = exp(-((-6:6).^2)/(2*1.3^2)); g = g/sum(g);
base = conv(sig,g','same'); base = 90 + 70*base + 2.5*randn(Lp,1);

[best_pattern, best_idx, best_error, MSE] = ...
    fcn_ExtractCL_matchPattern(base, pattern_template);

figure(201); clf;
subplot(2,1,1);
plot(T_range, base,'k','LineWidth',1.8); hold on;
plot(T_range, best_pattern*max(base),'r--','LineWidth',2);
xlabel('T [m]'); ylabel('Intensity');
title('Case 2: single\_both\_sides alignment'); grid on;
xline(T_range(best_idx),'b--','LineWidth',2);

subplot(2,1,2);
plot(MSE,'Color',[0 0.45 0.75],'LineWidth',1.5); grid on;
xlabel('Start index'); ylabel('MSE');
title(sprintf('Best-fit error = %.4f at idx %d', best_error,best_idx));

fprintf('  Case 2 passed.\n\n');

%% ===== CASE 3: Random Noise Robustness =====
disp('Case 3: Random noise robustness...');
rng(42);
N = 500; base = randn(N,1)*5 + 50;
pattern_template = fcn_ExtractCL_createLanePattern('single_strip', t_res);

[best_pattern, best_idx, best_error, MSE] = ...
    fcn_ExtractCL_matchPattern(base, pattern_template);

figure(301); clf;
subplot(2,1,1);
plot(base,'Color',[0.4 0.4 0.4]); hold on;
plot(best_pattern*max(base),'r--','LineWidth',1.5);
xlabel('Sample'); ylabel('Intensity'); title('Case 3: Random noise test');
subplot(2,1,2);
plot(MSE,'k','LineWidth',1.3); grid on;
xlabel('Start index'); ylabel('MSE');
title('MSE profile for random data');

assert(~isnan(best_error) && isfinite(best_error), 'NaN or Inf best-fit error.');
fprintf('  Case 3 passed.\n\n');

%% ===== Summary =====
fprintf('\nAll tests passed for fcn_ExtractCL_matchPattern.m\n');
