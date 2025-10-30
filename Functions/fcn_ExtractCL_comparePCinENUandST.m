function fcn_ExtractCL_comparePCinENUandST(pointCloud_ST_cell, ref_traj, S_range, fig_num)
% fcn_ExtractCL_comparePCinENUandST
% This function visualizes the comparison between LiDAR point clouds and a
% reference trajectory in both the ENU (global) and ST (station–lateral)
% coordinate frames.
%
% The function extracts the point cloud points and reference trajectory data
% within a specified S-range and plots their spatial relationships.
%
% FORMAT:
%
%      fcn_ExtractCL_comparePCinENUandST(...
%           pointCloud_ST_cell,...
%           ref_traj,...
%           S_range,...
%           fig_num);
%
% INPUTS:
%
%      pointCloud_ST_cell: {N_frames x 1} cell array
%          Each cell contains one LiDAR point cloud organized in ST format,
%          where column 9 = S (station) and column 10 = T (lateral offset).
%
%      ref_traj: [N x 5] numeric matrix
%          Reference trajectory, columns ordered as:
%              [X_enu, Y_enu, Z_enu, S_station, T_lateral]
%
%      S_range: [1 x 2] vector
%          Range of S values [S_min, S_max] defining region of interest.
%
%      fig_num: scalar integer
%          Figure number for plotting the comparison results.
%
% OUTPUTS:
%
%      None (visualization only)
%
% DEPENDENCIES:
%
%      None (pure MATLAB)
%
% EXAMPLES:
%
%      % Example usage:
%      fcn_ExtractCL_comparePCinENUandST(pointCloud_ST_cell, ref_traj, [0 50], 101);
%
% This function was written on 2025-10-28 by Xinyu Cao
% Questions or comments? xfc5113@psu.edu
%
% Revision history:
%      2025-10-28 - Xinyu Cao - initial version, formatted per IVSG style
% -------------------------------------------------------------------------

flag_do_debug = 0; % Flag to print debug messages
flag_check_inputs = 1; % Flag to check input validity

% Tell user where we are
if flag_do_debug
    st = dbstack;
    fprintf(1,'STARTING function: %s, in file: %s\n',st(1).name,st(1).file);
end


%% check input arguments
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   _____                   _                  
%  |_   _|                 | |                 
%    | |  _ __  _ __  _   _| |_ ___            
%    | | | '_ \| '_ \| | | | __/ __|           
%   _| |_| | | | |_) | |_| | |_\__ \           
%  |_____|_| |_| .__/ \__,_|\__|___/           
%              | |                             
%              |_|                             
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if flag_check_inputs
    % Basic input validation
    assert(iscell(pointCloud_ST_cell), 'Input pointCloud_ST_cell must be a cell array.');
    assert(isnumeric(ref_traj) && size(ref_traj,2) >= 4, 'ref_traj must be an N x 5 numeric matrix.');
    assert(isnumeric(S_range) && numel(S_range)==2, 'S_range must be a 1x2 numeric vector.');
    assert(isscalar(fig_num), 'fig_num must be a scalar integer.');
end


%% Main code starts here
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   __  __       _
%  |  \/  |     (_)
%  | \  / | __ _ _ _ __
%  | |\/| |/ _` | | '_ \
%  | |  | | (_| | | | | |
%  |_|  |_|\__,_|_|_| |_|
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Extract S range bounds
s_min = S_range(1);
s_max = S_range(2);

% Clamp range to valid trajectory span
traj_s = ref_traj(:,4);
s_min = max(s_min, min(traj_s));
s_max = min(s_max, max(traj_s));

% Merge cell array into a single matrix
pointCloud_ST_array = cell2mat(pointCloud_ST_cell);

% Filter point cloud by S range (column 9)
pointCloud_ST_inSRange = pointCloud_ST_array(...
    pointCloud_ST_array(:,9) >= s_min & pointCloud_ST_array(:,9) <= s_max, :);

% Filter reference trajectory by same S range
ref_traj_inSRange = ref_traj(traj_s >= s_min & traj_s <= s_max, :);


figure(fig_num)
clf

% ===== Subplot 1: ENU view =====
subplot(2,1,1);
scatter(pointCloud_ST_inSRange(:,1), pointCloud_ST_inSRange(:,2), ...
    20, pointCloud_ST_inSRange(:,4), 'filled', 'DisplayName', 'LiDAR points');
hold on
grid on

plot(ref_traj_inSRange(:,1), ref_traj_inSRange(:,2), ...
    'Color',[0.85 0.33 0.1], 'LineWidth',4, 'DisplayName','Reference path');

scatter(ref_traj_inSRange(1,1), ref_traj_inSRange(1,2), ...
    100,'red','filled','DisplayName','Path start','MarkerEdgeColor','k');

scatter(ref_traj_inSRange(end,1), ref_traj_inSRange(end,2), ...
    100,'green','filled','DisplayName','Path end','MarkerEdgeColor','k');

axis equal
xlabel('X-East [m]')
ylabel('Y-North [m]')
title('Point cloud in ENU frame')
legend('Location','best');


% ===== Subplot 2: ST view =====
subplot(2,1,2);
scatter(pointCloud_ST_inSRange(:,9), pointCloud_ST_inSRange(:,10), ...
    20, pointCloud_ST_inSRange(:,4),'filled','DisplayName','LiDAR points');
hold on
grid on

plot(ref_traj_inSRange(:,4), zeros(size(ref_traj_inSRange(:,4))), ...
    'Color',[0.85 0.33 0.1],'LineWidth',4,'DisplayName','Reference path');

scatter(ref_traj_inSRange(1,4), 0, ...
    100,'red','filled','DisplayName','Path start','MarkerEdgeColor','k');

scatter(ref_traj_inSRange(end,4), 0, ...
    100,'green','filled','DisplayName','Path end','MarkerEdgeColor','k');

axis equal
xlabel('Station [m]')
ylabel('Lateral Offset [m]')
title('Point cloud in ST frame')
legend('Location','best');

%% Debug summary
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   _____       _
%  |  __ \     | |
%  | |  | | ___| |__  _   _  __ _
%  | |  | |/ _ \ '_ \| | | |/ _` |
%  | |__| |  __/ |_) | |_| | (_| |
%  |_____/ \___|_.__/ \__,_|\__, |
%                            __/ |
%                           |___/
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if flag_do_debug
    fprintf('S range used: [%.2f, %.2f]\n', s_min, s_max);
    fprintf('Number of points in range: %d\n', size(pointCloud_ST_inSRange,1));
end

if flag_do_debug
    fprintf(1,'ENDING function: %s, in file: %s\n\n',st(1).name,st(1).file);
end

end % Ends main function
