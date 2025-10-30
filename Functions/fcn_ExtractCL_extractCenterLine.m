function RoadCenterLine = fcn_ExtractCL_extractCenterLine(PointCloud_ENU_Example, VehiclePose_ENU_Example, T_range, varargin)
% fcn_ExtractCL_extractCenterLine
% -------------------------------------------------------------------------
% Purpose:
%   Perform the full road center line or lane base line extraction pipeline from ENU LiDAR
%   point cloud data using ENU→ST projection, lateral filtering, lane-marker
%   extraction, and left/right separation. The final output RoadCenterLine
%   depends on extraction mode ('left' or 'right').
%
% FORMAT:
%
%   LaneCenterLine = fcn_ExtractCL_extractCenterLine(...
%        PointCloud_ENU_Example, VehiclePose_ENU_Example, T_range, ...
%        (s_length), (N_s), (t_res), (min_pts), (mode), (fig_num));
%
% INPUTS (required):
%   PointCloud_ENU_Example : Nx3 or Nx4 numeric, ENU points [E N U (I)].
%   VehiclePose_ENU_Example: Mx6 numeric, vehicle poses [X Y Z Roll Pitch Yaw].
%   T_range                : 1x2 numeric, lateral filter range [Tmin Tmax] in ST.
%
% OPTIONAL INPUTS (via varargin, in order):
%   s_length : scalar, longitudinal strip length (m).          (default: 5)
%   N_s      : scalar, number of S bins across s_length.       (default: 10)
%              -> s_res = s_length / N_s
%   t_res    : scalar, lateral ST grid resolution (m).         (default: 0.01)
%   min_pts  : scalar, minimum points to form a lane marker.    (default: 500)
%   mode     : 'double' or 'single' (string/char).             (default: 'double')
%   fig_num  : figure number (positive integer to plot).       (default: -1)
%
% OUTPUT:
%   LaneCenterLine : Kx5 numeric [X Y Z S T], extracted lane center line.
%
% DEPENDENCIES:
%   fcn_ExtractCL_projectPC_ENUToST
%   fcn_ExtractCL_filterPCinT
%   fcn_ExtractCL_extractLaneMarkers
%   fcn_ExtractCL_separateLaneMarkers
%   fcn_ExtractCL_computeCLwithLaneMarkers
%
% EXAMPLE:
%   See script_test_fcn_ExtractCL_extractCenterLine.m
%
% This function was written on 2025_10_29 by X. Cao
% Questions or comments? xfc5113@psu.edu
%
% Revision history:
%   2025_10_29 - xfc5113@psu.edu
%   -- write the code originally
%
% TO DO:
%   -- add road center line clean feature
%   -- segment the center line
%
% -------------------------------------------------------------------------

%% Debug and Configuration Flags
flag_do_debug    = 0;  % Verbose prints
flag_do_plot     = 0;  % Final visualization
flag_check_input = 1;  % Input checks

if flag_do_debug
    st = dbstack; %#ok<*UNRCH>
    fprintf(1,'STARTING function: %s, in file: %s\n', st(1).name, st(1).file);
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   _____                   _
%  |_   _|                 | |
%    | |  _ __  _ __  _   _| |_ ___
%    | | | '_ \| '_ \| | | | __/ __|
%   _| |_| | | | |_) | |_| | |_\__ \
%  |_____|_| |_| .__/ \__,_|\__|___/
%              | |
%              |_|
% See: http://patorjk.com/software/taag/#p=display&f=Big&t=Inputs
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Input Checking and Varargin Parsing
if flag_check_input
    if nargin < 3
        error('Incorrect number of input arguments. Expected at least 3.');
    end
    if isempty(PointCloud_ENU_Example)
        error('PointCloud_ENU_Example is empty');
    end
    if isempty(VehiclePose_ENU_Example) || size(VehiclePose_ENU_Example,2) < 6
        error('VehiclePose_ENU_Example must be Mx6 numeric [X Y Z Roll Pitch Yaw].');
    end
    if ~isvector(T_range) || numel(T_range) ~= 2
        error('T_range must be a 1x2 vector [Tmin Tmax].');
    end
end

% Defaults
s_length_default = 5;
N_s_default      = 10;
t_res_default    = 0.01;
min_pts_default  = 500;
mode_default     = 'left';
fig_num_default  = -1;

% Pull from varargin in order if provided
s_length = s_length_default;
N_s      = N_s_default;
t_res    = t_res_default;
min_pts  = min_pts_default;
mode     = mode_default;
fig_num  = fig_num_default;

if ~isempty(varargin)
    if numel(varargin) >= 1 && ~isempty(varargin{1}), s_length = varargin{1}; end
    if numel(varargin) >= 2 && ~isempty(varargin{2}), N_s      = varargin{2}; end
    if numel(varargin) >= 3 && ~isempty(varargin{3}), t_res    = varargin{3}; end
    if numel(varargin) >= 4 && ~isempty(varargin{4}), min_pts  = varargin{4}; end
    if numel(varargin) >= 5 && ~isempty(varargin{5}), mode     = varargin{5}; end
    if numel(varargin) >= 6 && ~isempty(varargin{6}), fig_num  = varargin{6}; end
end

% Derived parameter
s_res = s_length / N_s;

% Validate mode and fig_num
if ~(ischar(mode) || isstring(mode))
    error('mode must be a string: ''left'' or ''right''.');
end
% Plot only if fig_num is a positive integer
if isscalar(fig_num) && (fig_num == fix(fig_num)) && (fig_num > 0)
    flag_do_plot = 1;
else
    flag_do_plot = 0;
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   __  __       _
%  |  \/  |     (_)
%  | \  / | __ _ _ _ __
%  | |\/| |/ _` | | '_ \
%  | |  | | (_| | | | | |
%  |_|  |_|\__,_|_|_| |_|
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Main Code Starts Here

% Step 1: ENU → ST projection
[pointCloud_ST_cell, ref_station, Seg] = ...
    fcn_ExtractCL_projectPC_ENUToST(PointCloud_ENU_Example, VehiclePose_ENU_Example, fig_num); %#ok<ASGLU>

% Step 2: Lateral (T) filtering in ST
pointCloud_ST_filtered_cell = fcn_ExtractCL_filterPCinT(pointCloud_ST_cell, T_range);

% Step 3: Lane markers via extrema filtering / pattern matching in ST
pointCloud_ST_array = cell2mat(pointCloud_ST_filtered_cell); %#ok<NASGU>
[XYZST_lane_markers_array, HistoryData] = ... %#ok<NASGU>
    fcn_ExtractCL_extractLaneMarkers(pointCloud_ST_array, s_length, s_res, t_res, min_pts, Seg, fig_num);

% Step 4: Separate left/right lane markers
[LaneMarkers, islands, outliers] = fcn_ExtractCL_separateLaneMarkers(XYZST_lane_markers_array); %#ok<NASGU,ASGLU>

% Step 5: Compute lane center line from markers depending on mode
% mode = 'left', compute the road center line based on the left side lane markers 
[RoadCenterLine, ~] = fcn_ExtractCL_computeCLwithLaneMarkers(LaneMarkers, 'left', HistoryData, t_res,Seg);
% mode = 'right', compute the road center line based on the right side lane
% marekrs and corresponding lane layout pattern
% [RoadCenterLine, ~] = fcn_ExtractCL_computeCLwithLaneMarkers(LaneMarkers, 'right', HistoryData, t_res,Seg);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   _____       _
%  |  __ \     | |
%  | |  | | ___| |__  _   _  __ _
%  | |  | |/ _ \ '_ \| | | |/ _` |
%  | |__| |  __/ |_) | |_| | (_| |
%  |_____/ \___|_.__/ \__,_|\__, |
%                            __/ |
%                           |___/
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Debug / Visualization
if flag_do_plot
    figure(fig_num); clf; hold on; grid on; axis equal;
    title('Lane Center Line Extraction (ST domain)');
    xlabel('S (m)'); ylabel('T (m)');

    if isfield(LaneMarkers,'LaneMarkerLeft') && ~isempty(LaneMarkers.LaneMarkerLeft)
        plot(LaneMarkers.LaneMarkerLeft(:,4),  LaneMarkers.LaneMarkerLeft(:,5),  '.', 'DisplayName','Left Marker');
    end
    if isfield(LaneMarkers,'LaneMarkerRight') && ~isempty(LaneMarkers.LaneMarkerRight)
        plot(LaneMarkers.LaneMarkerRight(:,4), LaneMarkers.LaneMarkerRight(:,5), '.', 'DisplayName','Right Marker');
    end
    if ~isempty(LaneCenterLine)
        plot(LaneCenterLine(:,4), LaneCenterLine(:,5), '-', 'LineWidth', 2, 'DisplayName','Center Line');
    end
    legend('show','Location','best');
end

if flag_do_debug
    fprintf(1,'ENDING function: %s, in file: %s\n\n', st(1).name, st(1).file);
end

end
