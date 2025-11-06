function [XYZSTE_Center_Line_Array, HistoryData] = fcn_ExtractCL_extractCL_WhiteStrip(pointcloud_array, s_width, s_res, t_res, min_pts, Ref_Pose, varargin)
% fcn_ExtractCL_extractCenterLine_WhiteStrip
% Extracts the centerline from organized LiDAR intensity strips by remapping them into ST grid,
% applying extrema filters and pattern matching.
%
% FORMAT:
%   [XYZ_Center_Line_Array, Pattern_Cell, Extrema_Filter_Cell, HistoryData] = fcn_ExtractCL_extractCenterLine_WhiteStrip(...
%       pointcloud_array, s_width, s_res, t_res, min_pts, Ref_Pose, (fig_num), (HistoryData))
%
% INPUTS:
%   pointcloud_array: Nx10 array.
%
%   s_width: scalar
%     Width of each s-bin (e.g., 0.5).
%
%   s_res: scalar
%     Resolution in s-direction.
%
%   t_res: scalar
%     Resolution in t-direction.
%
%   min_pts: scalar
%     Minimum number of points to process a strip.
%
%   Ref_Pose: Mx7 array
%     Reference trajectory with [X, Y, Z, ..., Yaw, Station].
%
%   (OPTIONAL) fig_num: scalar
%     Figure number for debug plotting.
%
%   (OPTIONAL) HistoryData: struct
%     Structure containing previous pattern/extrema matching information.
%
% OUTPUTS:
%   XYZ_Center_Line_Array: Kx6 array
%     Extracted centerline points with [X, Y, Z, S, T, pattern_error].
%
%
%   HistoryData: struct
%     Output history struct.



%% Debugging and Input checks
flag_max_speed = 0;
flag_do_debug = 0;
flag_check_inputs = 1;

% If last varargin is -1, enter max speed mode
if nargin >= 7 && isequal(varargin{end}, -1)
    flag_max_speed = 1;
    flag_do_debug = 0;
    flag_check_inputs = 0;
else
    % Check if environment variables override debug flags
    MATLABFLAG_EXTRACTCL_FLAG_CHECK_INPUTS = getenv("MATLABFLAG_EXTRACTCL_FLAG_CHECK_INPUTS");
    MATLABFLAG_EXTRACTCL_FLAG_DO_DEBUG = getenv("MATLABFLAG_EXTRACTCL_FLAG_DO_DEBUG");
    if ~isempty(MATLABFLAG_EXTRACTCL_FLAG_CHECK_INPUTS)
        flag_check_inputs = str2double(MATLABFLAG_EXTRACTCL_FLAG_CHECK_INPUTS);
    end
    if ~isempty(MATLABFLAG_EXTRACTCL_FLAG_DO_DEBUG)
        flag_do_debug = str2double(MATLABFLAG_EXTRACTCL_FLAG_DO_DEBUG);
    end
end

% Optional: debug print start
if flag_do_debug
    st = dbstack;
    fprintf(1, 'STARTING function: %s, in file: %s\n', st(1).name, st(1).file);
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
% See: http://patorjk.com/software/taag/#p=display&f=Big&t=Inputs
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
flag_do_plot = 0;
fig_num = -1;
if nargin > 6
    fig_num = varargin{1};
end
if fig_num >= 1
    flag_do_plot = 1;
end

HistoryData = struct(...
    'CenterLine', {}, ...
    'LanePattern', {}, ...
    'ExtremaFilter', {}, ...
    'T_Ref', {});
flag_use_history_data = 0;
if nargin > 7
    HistoryData = varargin{2};
end
if ~isempty(HistoryData)
    flag_use_history_data = 1;
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
%% Extract fields from point cloud
s = pointcloud_array(:,9);
t = pointcloud_array(:,10);
intensity = pointcloud_array(:,4);
z = pointcloud_array(:,3);


%% S-binning
s_min = floor(min(s) / s_width) * s_width;
s_max = ceil(max(s) / s_width) * s_width;
s_bin_edges = s_min:s_width:s_max;
N_bins = length(s_bin_edges) - 1;

%% Initialization
XYZSTE_center_line_Cell = cell(N_bins, 1);
Pattern_Cell = {};
Extrema_Filter_Cell = {};
T_Ref_Cell = {};

original_pattern_template = fcn_ExtractCL_createLanePattern('left_double_yellow_right_white', t_res);
single_strip_template = fcn_ExtractCL_createLanePattern('single_strip', t_res);
pattern_template = original_pattern_template;

best_pattern_template = nan;
XYZ_center_line_array = [];

%% Process each S-bin
for ith_bin = 1:N_bins
    s_low = s_bin_edges(ith_bin);
    s_high = s_bin_edges(ith_bin + 1);
    point_idx = (s >= s_low & s < s_high);
    pointcloud_in_bin = pointcloud_array(point_idx,:);
    s_bin = s(point_idx);
    t_bin = t(point_idx);
    z_bin = z(point_idx);
    intensity_bin = intensity(point_idx);
    t_min = floor(min(t_bin) / t_res) * t_res;
    t_max = ceil(max(t_bin) / t_res) * t_res;
    t_strip_edges = t_min:t_res:(t_max - t_res);
    % pattern_length = length(pattern_template);
    % if length(t_strip_edges) < pattern_length
    %     continue;
    % end
    % 
    % s_range = max(s_bin) - min(s_bin);
    % num_points_in_bin = sum(point_idx);
    % % Check whether there are enough number of points in the current bin
    % if (num_points_in_bin < min_pts) || (s_range < 0.25*s_width)
    %     continue;
    % end
    % 
    % s_grid_edge_low = 0;
    % s_gird_edge_high = s_width;
    % if s_range < 0.8 * s_width
    %     s_low_new = min(s_bin);
    %     s_high_new = max(s_bin);
    %     if s_high_new - s_low_new < s_res
    %         continue;
    %     end
    %     s_grid_edge_low = floor((s_low_new - s_low)/s_res) * s_res;
    %     s_gird_edge_high = ceil((s_high_new - s_low)/s_res) * s_res;
    % end
    % % Create gridded data in ST space
    % s_strip_edges = s_grid_edge_low:s_res:s_gird_edge_high;
    % s_strip_centers = s_strip_edges(1:end-1) + s_res/2;
    % [S_base, T_base] = meshgrid(s_strip_centers, t_strip_edges);
    % S_interp = S_base + s_low;
    % T_interp = T_base;
    % % Perform interpolation with gridded data
    % F = scatteredInterpolant(s_bin, t_bin, intensity_bin, 'nearest', 'nearest');
    % I_interp = F(S_interp, T_interp);
    % I_interp = fillmissing(I_interp, 'linear');
    % F.Values = z_bin;
    % Z_interp = F(S_interp, T_interp);
    % Z_interp = fillmissing(Z_interp, 'linear');
    [S_interp, T_interp, I_interp, Z_interp, s_strip_centers] = ...
    fcn_ExtractCL_interpolateSTBin(s_bin, t_bin, intensity_bin, z_bin, ...
                                    s_low, s_res, s_width, t_res);

    if flag_do_plot == 1
        figure(fig_num);
        clf;
        intensity_raw_norm = intensity_bin / max(intensity_bin);
        intensity_raw_norm = max(intensity_raw_norm, 0.3);
        scatter(t_bin, s_bin, 40, [intensity_raw_norm, zeros(size(intensity_raw_norm)), zeros(size(intensity_raw_norm))], 'filled');
        hold on
        intensity_interp_norm = I_interp / max(I_interp);
        intensity_interp_norm = max(intensity_interp_norm, 0.3);
        for ith_s_bin = 1:length(s_strip_centers)
            scatter(T_interp(:,ith_s_bin), S_interp(:,ith_s_bin), 40, [zeros(size(intensity_interp_norm)), intensity_interp_norm, zeros(size(intensity_interp_norm))], 'filled');
        end
        axis equal
        xlabel('T [m]')
        ylabel('S [m]')
        legend('Raw point cloud','Organized point cloud')
        title('Raw vs. Organized Point Cloud in ST Coordinate Frame')
        pause(0.01)
    end

    % Pattern Matching
    fig_num_inner = fig_num + 1;
    if flag_use_history_data == 1
        [center_line_mask, pattern_template_cell, extrema_filter_cell, best_pattern_template, best_fit_errors] = ...
            fcn_ExtractCL_matchCLByPattern_WhiteStrip(I_interp, pattern_template, t_res, single_strip_template, fig_num_inner,...
            HistoryData, pointcloud_in_bin, s_strip_edges, t_strip_edges);
    else
        [center_line_mask, pattern_template_cell, extrema_filter_cell, best_pattern_template, best_fit_errors] = ...
            fcn_ExtractCL_matchCLByPattern_WhiteStrip(I_interp, pattern_template, t_res, single_strip_template, fig_num_inner);
    end

    tf_pattern_is_valid = ~cellfun('isempty', pattern_template_cell);
    num_valid_pattern = sum(tf_pattern_is_valid);
    T_ref_cell_ith_bin = repmat({t_strip_edges}, num_valid_pattern,1);
    valid_pattern_cell = pattern_template_cell(tf_pattern_is_valid);
    valid_extrema_filter_cell = extrema_filter_cell(~cellfun('isempty', extrema_filter_cell));

    S_center_line = S_interp(center_line_mask);
    T_center_line = T_interp(center_line_mask);
    Z_center_line = Z_interp(center_line_mask);


    if ~isempty(S_center_line)
        XY_center_line = fcn_ExtractCL_projectPC_STToENU(S_center_line, T_center_line, Ref_Pose);
        XYZSTE_center_line = [XY_center_line, Z_center_line, S_center_line, T_center_line, best_fit_errors];
    end

    assert(length(valid_pattern_cell) == size([S_center_line, T_center_line, Z_center_line], 1), 'Pattern count mismatch with center points');
    XYZSTE_center_line_Cell{ith_bin} = XYZSTE_center_line;
    pattern_template = best_pattern_template;

    Pattern_Cell = [Pattern_Cell; valid_pattern_cell];
    Extrema_Filter_Cell = [Extrema_Filter_Cell; valid_extrema_filter_cell];
    T_Ref_Cell = [T_Ref_Cell; T_ref_cell_ith_bin];
end

XYZSTE_Center_Line_Array = cell2mat(XYZSTE_center_line_Cell);
N_pts = size(XYZSTE_Center_Line_Array,1);
CenterLine_Cell = mat2cell(XYZSTE_Center_Line_Array, ones(N_pts,1), 6);

newData = struct(...
    'CenterLine', CenterLine_Cell, ...
    'LanePattern', Pattern_Cell(:), ...
    'ExtremaFilter', Extrema_Filter_Cell(:), ...
    'T_Ref', T_Ref_Cell(:));
if isempty(HistoryData)
    
    % HistoryData.CenterLine = XYZ_Center_Line_Array;
    % HistoryData.LanePattern = Pattern_Cell;
    % HistoryData.ExtremaFilter = Extrema_Filter_Cell;
    % HistoryData.T_Ref = T_Ref_Cell;
    HistoryData = newData;
else
    HistoryData = [HistoryData; newData];
end

end
