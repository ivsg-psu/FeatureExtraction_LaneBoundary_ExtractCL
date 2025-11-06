%% script_test_fcn_ExtractCL_projectPC_STToENU.m
% Tests fcn_ExtractCL_projectPC_STToENU.m
%
% Revision history:
%   2025_11_05 - Xinyu Cao
%   -- Created test script using straight and curved reference paths
%
% Purpose:
%   Verify correct projection of (S,T) points back to ENU coordinates
%   using segment-based geometry structure.

%% Setup
clc; clear; close all;

%% ===== Case 1: Straight path baseline =====
disp('Case 1: Straight path, T offset test...');

% Build a straight reference trajectory (along X axis)
X = (0:1:100)'; 
Y = zeros(size(X));
Z = zeros(size(X));
Yaw = zeros(size(X));
Ref_Pose = [X Y Z zeros(length(X),3)];  % Column 7 = station
Seg = fcn_ExtractCL_convertRefTrajToST(Ref_Pose);

% Generate test (S,T) points along the path
S_extrema = (10:10:90)';                   % every 10 m along S
T_extrema = [0; 1; 2; -1; 0; -2; 0; 1; 0];  % varied lateral offsets
XY_extrema = fcn_ExtractCL_projectPC_STToENU(S_extrema, T_extrema, Seg);

% Visualization
fig_num = 101;
figure(fig_num); clf; hold on; axis equal; grid on;
plot(Seg.traj_start(:,1), Seg.traj_start(:,2), 'k-', 'LineWidth',2, 'DisplayName','Reference line');
scatter(XY_extrema(:,1), XY_extrema(:,2), 60, T_extrema, 'filled','DisplayName','Projected points');
colorbar; xlabel('X [m]'); ylabel('Y [m]');
title('Case 1: Straight path projection');
legend('Location','best');

% Verification
assert(size(XY_extrema,1)==length(S_extrema), 'Output size mismatch.');
assert(all(isfinite(XY_extrema(:))), 'Non-finite values in XY_extrema.');
fprintf('  Case 1 passed.\n\n');

%% ===== Case 2: Curved path =====
disp('Case 2: Quarter-circle path...');
theta = linspace(0,pi/2,50)';
R = 50;
X = R*cos(theta);
Y = R*sin(theta);
Z = zeros(size(X));
Ref_Pose = [X Y Z zeros(length(X),3) cumsum([0; sqrt(diff(X).^2+diff(Y).^2)])];
Seg = fcn_ExtractCL_convertRefTrajToST(Ref_Pose);

% Define test points slightly inside/outside the curve
S_extrema = linspace(0,Ref_Pose(end,7),20)';
T_extrema = 2*sin(linspace(0,pi/2,20))';   % vary lateral offset
XY_extrema = fcn_ExtractCL_projectPC_STToENU(S_extrema, T_extrema, Seg);

% Visualization
fig_num = 102;
figure(fig_num); clf; hold on; axis equal; grid on;
plot(X,Y,'k--','LineWidth',2,'DisplayName','Reference path');
scatter(XY_extrema(:,1), XY_extrema(:,2),40,T_extrema,'filled','DisplayName','Projected (S,T)');
xlabel('X [m]'); ylabel('Y [m]');
title('Case 2: Quarter-circle projection test');
colorbar; legend('Location','best');

% Verification
assert(abs(norm(XY_extrema(1,:)-[X(1) Y(1)]))<5, 'Projection start deviates too much.');
assert(size(XY_extrema,2)==2,'XY_extrema must be Nx2.');
fprintf('  Case 2 passed.\n\n');

%% ===== Case 3: Edge handling (outside S range) =====
disp('Case 3: Out-of-range S values...');
S_extrema = [-10; 0; 50; 110];   % include outside bounds
T_extrema = [0; 1; -1; 0];
XY_extrema = fcn_ExtractCL_projectPC_STToENU(S_extrema, T_extrema, Seg);

% Visualization
fig_num = 103;
figure(fig_num); clf; hold on; axis equal; grid on;
plot(X,Y,'k-','LineWidth',2);
scatter(XY_extrema(:,1), XY_extrema(:,2),60,'r','filled');
xlabel('X [m]'); ylabel('Y [m]');
title('Case 3: Out-of-range S handling');
legend('Reference path','Projected points');

% Verification
assert(size(XY_extrema,1)==4, 'Output count mismatch for edge case.');
fprintf('  Case 3 passed.\n\n');


%% Summary
fprintf('All tests passed for fcn_ExtractCL_projectPC_STToENU.m\n');
