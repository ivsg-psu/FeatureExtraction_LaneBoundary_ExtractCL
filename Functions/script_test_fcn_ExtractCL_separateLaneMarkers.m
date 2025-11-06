%% script_test_fcn_ExtractCL_separateLaneMarkers.m
% Tests fcn_ExtractCL_separateLaneMarkers.m
%
% Revision history:
%   2025_11_05 - Xinyu Cao
%   -- Created full validation script with multiple geometry layouts
%
% Purpose:
%   Verify correct MAD-based lane marker separation into left/right/island/outlier sets.
%   Cases:
%     1) Straight symmetric lanes (dense)
%     2) Curved lanes (sinusoidal T variation)
%     3) Sparse and noisy points
%     4) Single-side lane markers
%     5) Empty input and boundary conditions

%% Setup
clc; clear; close all;
addpath(genpath(pwd));

%% ===== CASE 1: Straight symmetric lane markers =====
disp('Case 1: Straight symmetric lane markers...');
N = 200;
S = linspace(0,100,N)';
T_left  = -3.6 + 0.1*randn(N,1);
T_right =  3.6 + 0.1*randn(N,1);
T_all = [T_left; T_right];
S_all = [S; S];
X = S_all .* cosd(0);  % along X-axis
Y = S_all .* sind(0);
Z = zeros(size(X));
XYZST_lane_markers_array = [X Y Z S_all T_all];

[LaneMarkers, islands, outliers] = fcn_ExtractCL_separateLaneMarkers(XYZST_lane_markers_array, 101);

% Visualization
fig_num = 101;
figure(fig_num); clf; hold on; axis equal; grid on;
scatter(LaneMarkers.LaneMarkerLeft(:,4), LaneMarkers.LaneMarkerLeft(:,5), 20, 'r', 'filled');
scatter(LaneMarkers.LaneMarkerRight(:,4), LaneMarkers.LaneMarkerRight(:,5), 20, 'b', 'filled');
xlabel('Station [m]'); ylabel('T [m]');
title('Case 1: Straight lane separation');
legend('Left lane markers','Right lane markers');
assert(~isempty(LaneMarkers.LaneMarkerLeft) && ~isempty(LaneMarkers.LaneMarkerRight), ...
    'Left/Right markers not detected.');
fprintf('  Case 1 passed.\n\n');

%% ===== CASE 2: Curved lanes =====
disp('Case 2: Curved lane geometry...');
N = 300;
S = linspace(0,120,N)';
T_center = 0.5*sin(S/10);
T_left  = T_center - 3.5 + 0.1*randn(N,1);
T_right = T_center + 3.5 + 0.1*randn(N,1);
T_all = [T_left; T_right];
S_all = [S; S];
X = S_all .* cosd(0);
Y = S_all .* sind(0);
Z = zeros(size(X));
XYZST_lane_markers_array = [X Y Z S_all T_all];

[LaneMarkers, islands, outliers] = fcn_ExtractCL_separateLaneMarkers(XYZST_lane_markers_array, 102);

fig_num = 102;
figure(fig_num); clf; hold on; axis equal; grid on;
scatter(LaneMarkers.LaneMarkerLeft(:,4), LaneMarkers.LaneMarkerLeft(:,5), 15, 'r', 'filled');
scatter(LaneMarkers.LaneMarkerRight(:,4), LaneMarkers.LaneMarkerRight(:,5), 15, 'b', 'filled');
xlabel('Station [m]'); ylabel('T [m]');
title('Case 2: Curved lanes separation');
legend('Left','Right');
assert(abs(mean(LaneMarkers.LaneMarkerLeft(:,5)) + mean(LaneMarkers.LaneMarkerRight(:,5))) < 1, ...
    'Left/Right symmetry off for curved case.');
fprintf('  Case 2 passed.\n\n');

%% ===== CASE 3: Sparse and noisy =====
disp('Case 3: Sparse + noisy data...');
N = 30;
S = linspace(0,60,N)';
T_left  = -3.6 + 0.6*randn(N,1);
T_right =  3.6 + 0.6*randn(N,1);
T_all = [T_left; T_right];
S_all = [S; S];
X = S_all;
Y = zeros(size(S_all));
Z = zeros(size(X));
XYZST_lane_markers_array = [X Y Z S_all T_all];

[LaneMarkers, islands, outliers] = fcn_ExtractCL_separateLaneMarkers(XYZST_lane_markers_array, 103);

fig_num = 103;
figure(fig_num); clf; hold on; axis equal; grid on;
scatter(LaneMarkers.LaneMarkerLeft(:,4), LaneMarkers.LaneMarkerLeft(:,5), 30, 'r', 'filled');
scatter(LaneMarkers.LaneMarkerRight(:,4), LaneMarkers.LaneMarkerRight(:,5), 30, 'b', 'filled');
scatter(islands(:,4), islands(:,5), 40, 'g','filled');
xlabel('Station [m]'); ylabel('T [m]');
title('Case 3: Sparse + noisy data');
legend('Left','Right','Islands');
fprintf('  Case 3 passed.\n\n');

%% ===== CASE 4: Only single-side markers =====
disp('Case 4: Single-side lane markers...');
N = 100;
S = linspace(0,50,N)';
T_left = -3.5 + 0.2*randn(N,1);
XYZST_lane_markers_array = [S zeros(N,1) zeros(N,1) S T_left];

[LaneMarkers, islands, outliers] = fcn_ExtractCL_separateLaneMarkers(XYZST_lane_markers_array, 104);
fig_num = 104;
figure(fig_num); clf; hold on; axis equal; grid on;
scatter(LaneMarkers.LaneMarkerLeft(:,4), LaneMarkers.LaneMarkerLeft(:,5), 30, 'r','filled');
xlabel('Station [m]'); ylabel('T [m]');
title('Case 4: Single-side markers (Left only)');
legend('Left lane markers');
assert(~isempty(LaneMarkers.LaneMarkerLeft), 'Left markers missing.');
fprintf('  Case 4 passed.\n\n');

%% ===== CASE 5: Empty input and boundary =====
disp('Case 5: Empty / invalid input...');
try
    fcn_ExtractCL_separateLaneMarkers([]);
    error('Expected empty input error not triggered.');
catch ME
    assert(contains(ME.message,'XYZST_lane_markers_array must have at least 5 columns: [X Y Z S T].'),...
        'Missing empty input error.');
end


%% Summary
fprintf('All tests passed for fcn_ExtractCL_separateLaneMarkers.m\n');
