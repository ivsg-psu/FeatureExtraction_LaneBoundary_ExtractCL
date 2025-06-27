%% script_test_fcn_ExtractCL_createLanePattern.m
% Tests fcn_ExtractCL_createLanePattern.m

% Revision history:
%   2025_06_23 by Xinyu Cao
%   -- Initial test script creation for pattern template generation

%% Set up the workspace
close all;
clc;
clear;

%% Test 1: Basic pattern generation for left_double_yellow_right_white
fig_num = 101;
lane_type = 'left_double_yellow_right_white';
t_res = 0.01;

pattern_template = fcn_ExtractCL_createLanePattern(lane_type, t_res, [], [], [], fig_num);

% Check length of pattern
assert(length(pattern_template) > 0);

% Check binary values only
assert(all(ismember(pattern_template, [0 1])));

% Ensure multiple segments present (at least 3 separated by 0s)
on_indices = find(pattern_template);
diff_on = diff(on_indices);
num_segments = sum(diff_on > 1) + 1;
assert(num_segments >= 3);

%% Test 2: Basic pattern generation for single_strip
fig_num = 102;
lane_type = 'single_strip';
t_res = 0.01;

pattern_template = fcn_ExtractCL_createLanePattern(lane_type, t_res, [], [], [], fig_num);

% Check length of pattern
assert(length(pattern_template) > 0);

% Check binary values only
assert(all(ismember(pattern_template, [0 1])));

% Should only be one segment
on_indices = find(pattern_template);
diff_on = diff(on_indices);
num_segments = sum(diff_on > 1) + 1;
assert(num_segments == 1);

%% Test 3: Fast mode (-1 input)
fig_num = 103;
lane_type = 'left_double_yellow_right_white';
t_res = 0.01;

pattern_template = fcn_ExtractCL_createLanePattern(lane_type, t_res, [], [], [], -1);

% Check output
assert(length(pattern_template) > 0);

%% Test 4: Custom parameters
lane_width = 4.0;
marker_width = 0.15;
double_gap = 0.2;
fig_num = 104;

pattern_template = fcn_ExtractCL_createLanePattern(...
    'left_double_yellow_right_white', t_res, lane_width, marker_width, double_gap, fig_num);

assert(length(pattern_template) > 0);

%% Test 5: Invalid lane type
try
    pattern_template = fcn_ExtractCL_createLanePattern('unsupported_type', t_res);
    error('Expected error for unsupported lane type did not occur.');
catch ME
    assert(contains(ME.message, 'Unsupported lane_type'));
end

%% Test 6: No plotting if fig_num is empty
lane_type = 'single_strip';
pattern_template = fcn_ExtractCL_createLanePattern(lane_type, t_res);
assert(length(pattern_template) > 0);

%% Done
fprintf('All tests passed for fcn_ExtractCL_createLanePattern.m\n');
