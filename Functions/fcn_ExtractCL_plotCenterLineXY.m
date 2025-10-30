function fcn_ExtractCL_plotCenterLineXY(ref_traj_points, pointcloud_array, plotFormat, fig_num, varargin)
% fcn_ExtractCL_plotCenterLineXY
% Plots the extracted lane centerline and raw LiDAR point cloud in the ENU XY plane.
%
% FORMAT:
%
%      fcn_ExtractCL_plotCenterLineXY(...
%           ref_traj_points,...
%           pointcloud_array,...
%           plotFormat,...
%           fig_num);
%
% INPUTS:
%
%      ref_traj_points: Nx3 array or more columns
%          [X Y Z ...] reference trajectory in ENU coordinates. The Z or
%          additional columns are ignored in this function.
%
%      pointcloud_array: Mx8 array
%          [X Y Z Intensity ...] raw LiDAR point cloud in ENU coordinates.
%
%      plotFormat: string or struct
%          Plot format specification passed directly to
%          fcn_plotRoad_plotXY for visual styling.
%
%      fig_num: integer
%          Figure number to be used for plotting. If the figure exists,
%          it will be cleared before drawing.
%
% OUTPUTS:
%
%      None (function generates XY scatter plot of point cloud and
%      overlays the extracted lane centerline).
%
% DEPENDENCIES:
%
%      fcn_plotRoad_plotXY
%
% EXAMPLES:
%
%      See the script: script_test_fcn_ExtractCL_plotCenterLineXY
%      for a complete usage example.
%
% This function was written on 2025_06_26 by X. Cao
% Questions or comments? xfc5113@psu.edu
%
% Revision history:
%      2025_08_03 - xfc5113@psu.edu
%     -- wrote the code originally
%      2025_10_28 - xfc5113@psu.edu
%      -- reformatted comments and headings following repository template
%      -- added input checks for array dimensions and types
%
% TO DO
%      - Add optional colorbar scaling for intensity

%% Setup
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   _____                   _
%  |_   _|                 | |
%    | |  _ __  _ __  _   _| |_ ___
%    | | | '_ \| '_ \| | | | __/ __|
%   _| |_| | | | |_) | |_| | |_\__ \
%  |_____|_| |_| .__/ \__,_|\__|___/
%              | |
%              |_|
% See: http://patorjk.com/software/taag/#p=display&f=Big&t=Inputs
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Number of arguments
if nargin ~= 4
    error('fcn_ExtractCL_plotCenterLineXY:IncorrectNargin',...
        'Expected 4 inputs: ref_traj_points, pointcloud_array, plotFormat, fig_num.');
end

% ref_traj_points checks
if ~isnumeric(ref_traj_points) || ~ismatrix(ref_traj_points) || isempty(ref_traj_points)
    error('fcn_ExtractCL_plotCenterLineXY:BadRefTraj',...
        'ref_traj_points must be a non-empty numeric 2-D array (Nx3 or Nx5).');
end
ncols_ref = size(ref_traj_points,2);
if ~(ncols_ref >= 3)
    error('fcn_ExtractCL_plotCenterLineXY:BadRefTrajCols',...
        'ref_traj_points must have at least 3 columns. Got %d.', ncols_ref);
end

% pointcloud_array checks
if ~isnumeric(pointcloud_array) || ~ismatrix(pointcloud_array) || isempty(pointcloud_array)
    error('fcn_ExtractCL_plotCenterLineXY:BadPointCloud',...
        'pointcloud_array must be a non-empty numeric 2-D array.');
end
ncols_pc = size(pointcloud_array,2);
if ncols_pc < 4
    error('fcn_ExtractCL_plotCenterLineXY:BadPointCloudCols',...
        'pointcloud_array must have 4 columns [X Y Z Intensity]. Got %d.', ncols_pc);
end


% Prepare reference trajectory and point cloud data
XY_ref_traj = ref_traj_points(:, 1:2);
XY_pointcloud = pointcloud_array(:, 1:2);
intensity_pointcloud = pointcloud_array(:, 4);

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

% Initialize figure
figure(fig_num);
clf;
hold on;
grid on;
axis equal;

% Plot raw LiDAR point cloud intensity map
scatter(XY_pointcloud(:,1), XY_pointcloud(:,2), 20, intensity_pointcloud, 'filled');

% Overlay the extracted lane centerline
h_plot = fcn_plotRoad_plotXY(XY_ref_traj, plotFormat, fig_num); %#ok<NASGU>

% Configure plot appearance
% title('Extracted Lane Centerline in ENU XY Plane', 'FontSize', 20);
xlabel('X-East [m]');
ylabel('Y-North [m]');
legend('Point Cloud', 'Reference Trajectory');

%% Plot the results (for debugging)?
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


end % Ends main function
