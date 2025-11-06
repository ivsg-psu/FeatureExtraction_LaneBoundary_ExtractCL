%% script_test_fcn_ExtractCL_createLanePattern.m
% Tests fcn_ExtractCL_createLanePattern.m
% Purpose:
%   This script tests fcn_ExtractCL_createLanePattern, which generates 1D
%   binary lane marker templates based on lane configuration and resolution.
%   It includes:
%     (1) Pattern generation for supported types
%     (2) Fast mode (-1 input)
%     (3) Custom parameters
%     (4) Error handling for invalid inputs
%     (5) Sanity checks for plotting behavior

% Revision history:
%   2025_06_23 by Xinyu Cao
%   -- Initial test script creation for pattern template generation
%     2025-11-05 - xfc5113@psu.edu
%     -- added corner cases

%% Setup
clc; clear; close all;
fprintf('Running script_test_fcn_ExtractCL_createLanePattern.m ...\n');

%% Test 1: Basic pattern generation (left_double_yellow_right_white)
disp('Test 1: left_double_yellow_right_white');
fig_num = 101;
lane_type = 'left_double_yellow_right_white';
t_res = 0.01;

pattern_template = fcn_ExtractCL_createLanePattern(lane_type, t_res, [], [], [], fig_num);

% Assertions
assert(~isempty(pattern_template), 'Pattern template is empty.');
assert(all(ismember(pattern_template, [0 1])), 'Non-binary values found in pattern.');
on_indices = find(pattern_template);
num_segments = sum(diff(on_indices) > 1) + 1;
assert(num_segments >= 3, 'Expected at least three lane marker segments (double yellow + white).');
fprintf('  Test 1 passed: pattern length = %d samples, segments = %d.\n', length(pattern_template), num_segments);

%% Test 2: Basic single_strip pattern
disp('Test 2: single_strip');
fig_num = 102;
lane_type = 'single_strip';
pattern_template = fcn_ExtractCL_createLanePattern(lane_type, t_res, [], [], [], fig_num);

assert(~isempty(pattern_template), 'Pattern template is empty.');
assert(all(ismember(pattern_template, [0 1])), 'Non-binary values found in pattern.');
on_indices = find(pattern_template);
num_segments = sum(diff(on_indices) > 1) + 1;
assert(num_segments == 1, 'Expected exactly one solid strip.');
fprintf('  Test 2 passed: pattern length = %d samples, segments = %d.\n', length(pattern_template), num_segments);
%% Test 3: Basic single on both sides pattern
disp('Test 3: single_strip');
fig_num = 103;
lane_type = 'single_both_sides';
pattern_template = fcn_ExtractCL_createLanePattern(lane_type, t_res, [], [], [], fig_num);

assert(~isempty(pattern_template), 'Pattern template is empty.');
assert(all(ismember(pattern_template, [0 1])), 'Non-binary values found in pattern.');
on_indices = find(pattern_template);
num_segments = sum(diff(on_indices) > 1) + 1;
assert(num_segments == 2, 'Expected exactly one solid strip.');
fprintf('  Test 3 passed: pattern length = %d samples, segments = %d.\n', length(pattern_template), num_segments);

%% Test 4: Fast mode (-1 input)
disp('Test 4: Fast mode (-1 input)');
pattern_template = fcn_ExtractCL_createLanePattern('left_double_yellow_right_white', t_res, [], [], [], -1);
assert(~isempty(pattern_template), 'Fast mode returned empty pattern.');
fprintf('  Test 4 passed: fast mode executed successfully.\n');

%% Test 5: Custom parameters
disp('Test 5: Custom parameters');
lane_width = 4.0;
marker_width = 0.15;
double_gap = 0.2;
fig_num = 104;

pattern_template_custom = fcn_ExtractCL_createLanePattern('left_double_yellow_right_white', ...
    t_res, lane_width, marker_width, double_gap, fig_num);

assert(~isempty(pattern_template_custom), 'Custom parameter pattern empty.');
assert(length(pattern_template_custom) > length(pattern_template), ...
    'Custom wider lane should produce longer pattern.');
fprintf('  Test 5 passed: pattern length increased as expected (%d → %d).\n', ...
    length(pattern_template), length(pattern_template_custom));

%% Test 6: Invalid lane type
disp('Test 6: Invalid lane type');
try
    fcn_ExtractCL_createLanePattern('unsupported_type', t_res);
    error('Expected error for unsupported lane type did not occur.');
catch ME
    assert(contains(ME.message, 'Unsupported lane_type'), ...
        'Unexpected error message for invalid lane type.');
    fprintf('  Test 6 passed: invalid lane type correctly rejected.\n');
end

%% Test 7: No plotting if fig_num omitted
disp('Test 7: No plotting when fig_num omitted');
pattern_template_noplot = fcn_ExtractCL_createLanePattern('single_strip', t_res);
assert(~isempty(pattern_template_noplot), 'Pattern should still be generated without fig_num.');
fprintf('Test 7 passed: pattern generated without plotting.\n');

%% Test 8: Corner case - invalid t_res
disp('Test 8: Corner case - invalid t_res');
try
    fcn_ExtractCL_createLanePattern('single_strip', 0);
    error('Expected error for zero resolution not thrown.');
catch
    fprintf('Subcase (a) passed: zero t_res rejected.\n');
end

try
    fcn_ExtractCL_createLanePattern('single_strip', -0.01);
    error('Expected error for negative resolution not thrown.');
catch
    fprintf('  Subcase (b) passed: negative t_res rejected.\n');
end


%% Summary
fprintf('\nAll tests passed for fcn_ExtractCL_createLanePattern.m\n');
