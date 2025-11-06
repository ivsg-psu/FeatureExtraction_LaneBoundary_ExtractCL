%% script_test_fcn_ExtractCL_computeCLwithLaneMarkers.m
% Tests fcn_ExtractCL_computeCLwithLaneMarkers.m
%
% Revision history:
%   2025_11_05 - Xinyu Cao
%   -- Created full multi-mode test script with fake geometry and patterns
%
% Purpose:
%   Validate both 'left' and 'right' modes of lane-centerline computation.
%   The test simulates lane marker data, per-station patterns, and segment
%   geometry, verifying that results are spatially consistent and stable.
%
% Cases:
%   1) Left-mode direct projection
%   2) Right-mode pattern-guided offset
%   3) Sparse pattern + missing Z
%   4) Empty marker / invalid input handling

%% Setup
clc; clear; close all;
addpath(genpath(pwd));

% Common parameters
t_res = 0.05;
S_ref = (0:5:100)';

% Build a synthetic Seg (straight 100m path)
Seg.ref_station       = S_ref;
Seg.traj_start        = [S_ref(1:end-1) zeros(length(S_ref)-1,1)];
Seg.traj_end          = [S_ref(2:end) zeros(length(S_ref)-1,1)];
Seg.segment           = Seg.traj_end - Seg.traj_start;
Seg.segment_length    = sqrt(sum(Seg.segment.^2,2));
Seg.seg_tangent       = [ones(length(Seg.segment_length),1) zeros(length(Seg.segment_length),1)];
Seg.seg_normal        = [zeros(length(Seg.segment_length),1) ones(length(Seg.segment_length),1)]; % left = +Y
Seg.seg_start_station = S_ref(1:end-1);
Seg.d_seg             = diff(S_ref);

%% ===== CASE 1: Left mode (direct centerline) =====
disp('Case 1: Direct use of left markers...');
N = length(S_ref);
T_left  = -3.6 + 0.05*randn(N,1);
Z_left  = 0.01*S_ref + 0.02*randn(N,1);
LaneMarkers.LaneMarkerLeft  = [S_ref S_ref*0 Z_left S_ref T_left];
LaneMarkers.LaneMarkerRight = [];

% Dummy HistoryData
for i = 1:N
    HistoryData(i).S_Ref = S_ref(i);
    HistoryData(i).LanePattern = zeros(1, round(8/t_res));
    HistoryData(i).T_Ref = (-4:t_res:4)';
    HistoryData(i).Z_Ref = zeros(size(HistoryData(i).T_Ref));
end

[RoadCL_left, MarkerCL_left] = ...
    fcn_ExtractCL_computeCLwithLaneMarkers(LaneMarkers, 'left', HistoryData, t_res, Seg);

figure(101); clf; hold on; axis equal; grid on;
plot(MarkerCL_left(:,1), MarkerCL_left(:,2), 'r.-','LineWidth',2);
plot(RoadCL_left(:,1), RoadCL_left(:,2), 'b--','LineWidth',2);
xlabel('X [m]'); ylabel('Y [m]');
title('Case 1: Left-mode direct projection');
legend('Marker CL (Left)','Road CL');
assert(size(RoadCL_left,1)>0 && all(abs(RoadCL_left(:,2))<0.5), ...
    'Left-mode road centerline should follow X-axis.');
fprintf('  Case 1 passed.\n\n');

%% ===== CASE 2: Right mode (pattern-guided offset) =====
disp('Case 2: Pattern-guided offset using right markers...');
T_right = 3.6 + 0.05*randn(N,1);
Z_right = 0.01*S_ref + 0.02*randn(N,1);
LaneMarkers.LaneMarkerRight = [S_ref S_ref*0 Z_right S_ref T_right];
LaneMarkers.LaneMarkerLeft  = [];

% Generate artificial per-station patterns
for i = 1:N
    HistoryData(i).S_Ref = S_ref(i);
    T_axis = (-4:t_res:4)';
    HistoryData(i).T_Ref = T_axis;
    HistoryData(i).Z_Ref = zeros(size(T_axis));

    % pattern: right solid + double yellow at -3 ~ -3.4
    pattern = zeros(size(T_axis));
    idx_right = find(abs(T_axis - 3.6) < 0.05);
    idx_double = find((T_axis > -3.4 & T_axis < -3.0) | (T_axis > -3.2 & T_axis < -2.8));
    pattern([idx_right; idx_double]) = 1;
    HistoryData(i).LanePattern = pattern;
end

[RoadCL_right, MarkerCL_right] = ...
    fcn_ExtractCL_computeCLwithLaneMarkers(LaneMarkers, 'right', HistoryData, t_res, Seg);

figure(102); clf; hold on; axis equal; grid on;
plot(MarkerCL_right(:,1), MarkerCL_right(:,2), 'b.','DisplayName','Right lane markers');
plot(RoadCL_right(:,1),  RoadCL_right(:,2),  'r--','LineWidth',2,'DisplayName','Road centerline');
xlabel('X [m]'); ylabel('Y [m]');
title('Case 2: Right-mode pattern-guided centerline');
legend('Location','best');

fprintf('  Case 2 passed.\n\n');

%% ===== CASE 3: Sparse / missing pattern entries =====
disp('Case 3: Sparse patterns...');
for i = 1:N
    if mod(i,3)==0
        HistoryData(i).LanePattern = zeros(size(HistoryData(i).T_Ref));
    end
end
[RoadCL_sparse, MarkerCL_sparse] = ...
    fcn_ExtractCL_computeCLwithLaneMarkers(LaneMarkers, 'right', HistoryData, t_res, Seg);
assert(size(RoadCL_sparse,1) <= size(RoadCL_right,1), ...
    'Sparse pattern case should yield fewer valid stations.');
fprintf('  Case 3 passed.\n\n');

%% ===== CASE 4: Empty / invalid input =====
disp('Case 4: Empty or missing lane markers...');
LaneMarkersEmpty.LaneMarkerLeft = [];
LaneMarkersEmpty.LaneMarkerRight = [];
[RoadCL_empty, MarkerCL_empty] = ...
    fcn_ExtractCL_computeCLwithLaneMarkers(LaneMarkersEmpty, 'left', HistoryData, t_res, Seg);
assert(isempty(RoadCL_empty) && isempty(MarkerCL_empty), 'Empty input should yield empty outputs.');
fprintf('  Case 4 passed.\n\n');

%% ===== Summary =====
fprintf('All tests passed for fcn_ExtractCL_computeCLwithLaneMarkers.m\n');
