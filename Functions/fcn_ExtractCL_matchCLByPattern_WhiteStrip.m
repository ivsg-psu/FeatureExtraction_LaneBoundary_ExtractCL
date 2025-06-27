function [center_line_mask, pattern_template_cell, extrema_filter_cell, best_pattern_template, best_fit_errors] = ...
    fcn_ExtractCL_matchCLByPattern_WhiteStrip(intensity_data, pattern_template, T_resolution, single_strip_template, varargin)
% fcn_CL_matchCenterlineByPattern_WhiteStrip
% Extracts lane centerline from intensity data using pattern matching and extrema filtering,
% optimized for white lane marker strips. Optionally uses history and previous templates.
%
% FORMAT:
%   [center_line_mask, pattern_template_cell, extrema_filter_cell, best_pattern_template, best_fit_errors] = ...
%       fcn_CL_matchCenterlineByPattern_WhiteStrip( ...
%           intensity_data, pattern_template, T_resolution, single_strip_template, ...
%           (fig_num), (HistoryData), (pointcloud_in_bin), (s_strip_edges), (t_strip_edges))
%
% INPUTS:
%   intensity_data: NxM matrix
%       Interpolated intensity image in ST grid
%
%   pattern_template: column vector
%       The expected pattern profile for lane marker
%
%   T_resolution: scalar
%       Resolution in T-direction (e.g., 0.01)
%
%   single_strip_template: column vector
%       Template for single white lane strip
%
%   (OPTIONAL) fig_num: scalar
%       Figure number to enable debug plotting
%
%   (OPTIONAL) HistoryData: struct
%       Previously extracted centerlines and patterns for reuse
%
%   (OPTIONAL) pointcloud_in_bin: Nx11 array
%       Original points in current S-bin
%
%   (OPTIONAL) s_strip_edges, t_strip_edges: vectors
%       ST grid boundaries for current bin
%
% OUTPUTS:
%   center_line_mask: logical NxM array
%       Mask marking centerline locations in ST image
%
%   pattern_template_cell: Mx1 cell
%       Detected pattern template per strip
%
%   extrema_filter_cell: Mx1 cell
%       Extrema filter used on each strip
%
%   best_pattern_template: column vector
%       Best matched pattern template (possibly updated)
%
%   best_fit_errors: column vector
%       Sum squared error of best fit per strip
%% Debugging and Input checks

% Check if flag_max_speed set. This occurs if the fig_num variable input
% argument (varargin) is given a number of -1, which is not a valid figure
% number.
flag_max_speed = 0;
if (nargin==5 && isequal(varargin{end},-1))
    flag_do_debug = 0; % % % % Flag to plot the results for debugging
    flag_check_inputs = 0; % Flag to perform input checking
    flag_max_speed = 1;
else
    % Check to see if we are externally setting debug mode to be "on"
    flag_do_debug = 0; % % % % Flag to plot the results for debugging
    flag_check_inputs = 1; % Flag to perform input checking
    MATLABFLAG_LAPS_FLAG_CHECK_INPUTS = getenv("MATLABFLAG_LAPS_FLAG_CHECK_INPUTS");
    MATLABFLAG_LAPS_FLAG_DO_DEBUG = getenv("MATLABFLAG_LAPS_FLAG_DO_DEBUG");
    if ~isempty(MATLABFLAG_LAPS_FLAG_CHECK_INPUTS) && ~isempty(MATLABFLAG_LAPS_FLAG_DO_DEBUG)
        flag_do_debug = str2double(MATLABFLAG_LAPS_FLAG_DO_DEBUG);
        flag_check_inputs  = str2double(MATLABFLAG_LAPS_FLAG_CHECK_INPUTS);
    end
end

% flag_do_debug = 1;

if flag_do_debug
    st = dbstack; %#ok<*UNRCH>
    fprintf(1,'STARTING function: %s, in file: %s\n',st(1).name,st(1).file);
    debug_fig_num = 999978; %#ok<NASGU>
else
    debug_fig_num = []; %#ok<NASGU>
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


%% Parse Optional Inputs
fig_num = -1;
flag_do_plot = 0;
if nargin >= 5
    fig_num = varargin{1};
    if fig_num >= 1
        flag_do_plot = 1;
    end
end

HistoryData = nan;
if nargin >= 6
    HistoryData = varargin{2};
end

pointcloud_in_bin = [];
if nargin >= 7
    pointcloud_in_bin = varargin{3};
end

s_strip_edges = nan;
if nargin >= 8
    s_strip_edges = varargin{4};
end

t_strip_edges = nan;
if nargin >= 9
    t_strip_edges = varargin{5};
end

flag_use_history_data = 0;
if ~isnan(HistoryData)
    OldCenterLine = HistoryData.CenterLine;
    OldCenterLineXY = OldCenterLine(:,1:2);
    CenterLineTree = KDTreeSearcher(OldCenterLineXY);
    OldPattern = HistoryData.LanePattern;
    T_Ref_Cell = HistoryData.T_Ref;
    s_in_bin = pointcloud_in_bin(:,9);
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

%% Parameters and Initialization
lane_marker_width = 0.1;
half_indices_width = round(lane_marker_width/T_resolution/2);
filter_factor = 1.0;
filter_lengthI = round(lane_marker_width/T_resolution*filter_factor);

extrema = Extrema_Vectorization();
[N_points, N_strips] = size(intensity_data);
center_line_mask = false(size(intensity_data));
pattern_template_cell = cell(N_strips,1);
extrema_filter_cell = cell(N_strips,1);
num_pattern_waves = sum(diff(pattern_template) == 1);
number_of_extremaI = num_pattern_waves;
best_pattern_template = pattern_template;
best_fit_errors = [];

%% Filter and Pattern Matching Per Strip
intensity_filter = extrema.fcn_createOptimalExtremaFilter(intensity_data, filter_lengthI);
for ith_strip = 1:N_strips
    current_pattern_template = best_pattern_template;
    intensity_ith_strip = intensity_data(:,ith_strip);
    intensity_filter_ith_strip = intensity_filter(:,ith_strip);
    [~, extrema_corr] = extrema.fcn_findExtrema(intensity_ith_strip, intensity_filter_ith_strip, number_of_extremaI);
    extrema_corr_norm = (extrema_corr - min(extrema_corr)) / max(extrema_corr);

    if flag_use_history_data
        s_low_edge = s_strip_edges(ith_strip);
        s_high_edge = s_strip_edges(ith_strip + 1);
        point_idx = (s_in_bin >= s_low_edge) & (s_in_bin < s_high_edge);
        points_in_strip = pointcloud_in_bin(point_idx,:);
        StripCenter_XY = mean(points_in_strip(:,1:2));
        idx_closest = knnsearch(CenterLineTree, StripCenter_XY);
        matched_pattern = interp1(T_Ref_Cell{idx_closest}, OldPattern{idx_closest}, t_strip_edges, 'nearest', 0);
        pattern_start_idx = find(matched_pattern,1,"first");
        differenceArray = extrema_corr_norm - matched_pattern;
        best_fit_error = sum(differenceArray.^2);
    else
        [matched_pattern, pattern_start_idx, best_fit_error] = fcn_LaneDetection_matchPattern(extrema_corr_norm, current_pattern_template);
    end

    middle_index = floor(N_points/2);
extrema_corr_clean = extrema_corr_norm;
extrema_corr_right = extrema_corr_clean(1:middle_index);
[~, single_pattern_start_idx, best_fit_error_single] = fcn_LaneDetection_matchPattern(extrema_corr_right, single_strip_template);

    best_fit_index = pattern_start_idx;
    if best_fit_error > best_fit_error_single
        best_fit_index = single_pattern_start_idx;
    end

    threshold = max([0.4, extrema_corr_norm(max([1, best_fit_index - 5]))]);
extrema_corr_clean(1:best_fit_index) = 0;
extrema_corr_right = extrema_corr_clean(1:middle_index);
extrema_corr_left = extrema_corr_clean(middle_index+1:end);

    [~, right_cands] = findpeaks(extrema_corr_right, "MinPeakHeight", threshold);
    [~, left_cands] = findpeaks(extrema_corr_left, "MinPeakHeight", threshold);
    left_cands = left_cands + middle_index;
    lane_marker_cands = sort([right_cands; left_cands],'ascend');
    detected_pattern = zeros(size(matched_pattern));
    idx_range = lane_marker_cands + (-half_indices_width:half_indices_width);
    idx_range = idx_range(idx_range >= 1 & idx_range <= N_points);
    detected_pattern(idx_range) = 1;

    best_pattern = detected_pattern;
    if isempty(lane_marker_cands)
        continue
    end
    center_line_index = lane_marker_cands(1);
    best_pattern_template = [0; detected_pattern(max([1,min(idx_range)]):min([end,max(idx_range)])); 0];

    if isempty(center_line_index)
        continue
    end
    center_line_mask(center_line_index, ith_strip) = true;
    best_fit_errors = [best_fit_errors; best_fit_error];
    pattern_template_cell{ith_strip,1} = best_pattern;
    extrema_filter_cell{ith_strip,1} = intensity_filter_ith_strip.';

    if flag_do_plot
        figure(fig_num); clf; hold on;
        plot(extrema_corr_norm, 'b');
        % plot(matched_pattern, '--m');
        plot(best_pattern, 'g');
        if ~isempty(center_line_index)
            xline(center_line_index, 'k--', 'LineWidth', 3);
        end
        legend('Normalized Extrema Corrleration', 'Best Matched Pattern', 'Center Line');
        
        set(findall(gca, 'Type', 'Line'), 'LineWidth', 3);

        xlim([1 N_points])
        title('Center line extraction via pattern matching')
        pause(0.1);
        
    end
end

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

% if flag_do_plot
%     figure(fig_num); clf; hold on;
%     plot(extrema_corr_norm, 'b');
%     plot(matched_pattern, '--m');
%     plot(best_pattern, 'r');
%     if ~isempty(center_line_index)
%         xline(center_line_index, 'k--', 'LineWidth', 3);
%     end
%     legend('Normalized Extrema', 'Matched Pattern', 'Detected Pattern', 'Center Line');
%     set(findall(gca, 'Type', 'Line'), 'LineWidth', 3);
%     pause(0.1);
% end

end
