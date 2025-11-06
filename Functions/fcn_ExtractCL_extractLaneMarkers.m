function [XYZST_LaneMarkers_Array, HistoryData] = fcn_ExtractCL_extractLaneMarkers(pointcloud_array, s_width, s_res, t_res, min_pts, Seg, varargin)
% fcn_ExtractCL_extractLaneMarkers
% Extracts lane marker points from an organized LiDAR intensity point cloud by
% (1) remapping data into the (s, t) grid along a reference path, (2) applying
% extrema filtering to obtain candidate lane-marker ridges in the lateral
% (t) direction, and (3) matching the resulting profiles to adaptive lane
% marker templates (e.g., double yellow + solid white). Detected marker
% points are projected back to ENU and returned as [X Y Z s t].
%
% FORMAT:
%
%      [XYZST_LaneMarkers_Array, HistoryData] = ...
%      fcn_ExtractCL_extractLaneMarkers(...
%           pointcloud_array,...
%           s_width,...
%           s_res,...
%           t_res,...
%           min_pts,...
%           Seg,...
%           (fig_num),...
%           (HistoryData));
%
% INPUTS:
%
%      pointcloud_array: Nx10 numeric array
%          Organized point cloud with ENU/attribute columns and appended
%          curvilinear coordinates:
%              col  1: X (ENU East)       [m]
%              col  2: Y (ENU North)      [m]
%              col  3: Z (elevation)      [m]
%              col  4: Intensity          [unitless]
%              col  5+: other attributes  (unused here)
%              col  9: s (station)        [m]  (curvilinear longitudinal)
%              col 10: t (lateral offset) [m]  (curvilinear lateral)
%
%      s_width: scalar (m)
%          Longitudinal window width for each s-bin (e.g., 0.5 m).
%          Larger values gather more points per strip (robustness ↑) but reduce
%          spatial responsiveness to quick changes (adaptivity ↓).
%
%      s_res: scalar (m)
%          Longitudinal sampling step to discretize each s-bin into
%          strips (columns). Typical values are ≤ s_width. Smaller values
%          produce more strips per bin and finer sampling.
%
%      t_res: scalar (m)
%          Lateral sampling step for constructing the t-axis grid for each
%          strip. Typical values: 0.02–0.05 m depending on sensor noise.
%
%      min_pts: scalar (count)
%          Minimum number of points required in an s-bin before we attempt
%          extrema filtering and pattern matching. Prevents unstable fits
%          when a bin is too sparse.
%
%      Seg: struct
%          Segment cache produced by fcn_ExtractCL_buildSegmentsFromRefPose,
%          used for projecting (s, t) back to ENU:
%              .traj_start, .traj_end, .seg_tangent, .seg_normal, ...
%
%      (OPTIONAL) fig_num: scalar
%          Figure number for debug visualization. If -1 or omitted, plotting
%          is disabled. Positive values enable per-bin visualization.
%
%      (OPTIONAL) HistoryData: struct
%          Carries adaptive state across bins:
%              .LanePattern   : cell of templates used/matched
%              .ExtremaFilter : cell of extrema-filter diagnostics
%              .T_Ref         : cell of t-grids used per bin
%              .MatchError    : cell/array of template match errors
%          If provided, we pass it to the pattern-matching function to bias
%          the template selection. If empty or omitted, a default template
%          is used and then adapted.
%
% OUTPUTS:
%
%      XYZST_LaneMarkers_Array: Kx5 (or Kx6) numeric array
%          Detected lane marker points:
%              [X Y Z s t] (and possibly an additional error metric column
%              if your pattern-matching function appends one upstream).
%
%      HistoryData: struct (updated)
%          Appends per-bin templates/filters/t-grids/match errors for
%          downstream analysis or for seeding subsequent runs.
%
% DEPENDENCIES:
%
%      fcn_ExtractCL_createLanePattern
%      fcn_ExtractCL_organizePointCloudST
%      fcn_ExtractCL_findLaneMarkersByPattern
%      fcn_ExtractCL_projectPC_STToENU
%
% EXAMPLES:
%
%      See script: script_test_fcn_ExtractCL_extractLaneMarkers
%
% This function was written on 2025_06_03 by X. Cao
% Questions or comments? xfc5113@psu.edu
%
% Revision history:
%      2025_06_03 - xfc5113@psu.edu
%     -- wrote the code originally
%      2025_10_28 - xfc5113@psu.edu
%      -- formatted per repository template, added input checks and thorough comments
%
% TO DO
%      - Add Name-Value pairs for template type and adaptive-update thresholds
%      - Complete the branch with history data

%% Debugging and Input checks
% Flags:
%   flag_max_speed     : skip checks/plots for maximum throughput
%   flag_do_debug      : print start/end markers
%   flag_check_inputs  : assert shapes/units are reasonable
flag_max_speed    = 0;
flag_do_debug     = 0;
flag_check_inputs = 1;

% Max-speed shortcut: if last varargin is -1 → disable checks and plotting.
if nargin >= 7 && isequal(varargin{end}, -1)
    flag_max_speed    = 1;
    flag_do_debug     = 0;
else
    % Environment overrides (optional)
    MATLABFLAG_EXTRACTCL_FLAG_CHECK_INPUTS = getenv("MATLABFLAG_EXTRACTCL_FLAG_CHECK_INPUTS");
    MATLABFLAG_EXTRACTCL_FLAG_DO_DEBUG     = getenv("MATLABFLAG_EXTRACTCL_FLAG_DO_DEBUG");
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
if isnumeric(fig_num) && isscalar(fig_num) && isfinite(fig_num) && fig_num >= 1
    flag_do_plot = 1;
end

% HistoryData handling: if provided, we allow adaptive matching to leverage it.
HistoryData = struct('S_Ref', {}, 'LanePattern', {}, 'ExtremaFilter', {}, ...
                    'T_Ref', {},  'Z_Ref', {}, 'MatchError', {});
flag_use_history_data = 0;
if nargin > 7
    HistoryData = varargin{2};
end
if ~isempty(HistoryData)
    flag_use_history_data = 1;
end

% Basic input validation (kept light for performance unless you raise strictness)
if flag_check_inputs
    if ~(isnumeric(pointcloud_array) && ismatrix(pointcloud_array) && size(pointcloud_array,2) >= 10)
        error('fcn_ExtractCL_extractLaneMarkers:BadInput',...
              'pointcloud_array must be numeric Nx10+ with columns [X Y Z I ... s t].');
        return

    end
    assert(isscalar(s_width) && s_width > 0,   's_width must be a positive scalar (meters).');
    assert(isscalar(s_res)   && s_res   > 0,   's_res must be a positive scalar (meters).');
    assert(isscalar(t_res)   && t_res   > 0,   't_res must be a positive scalar (meters).');
    assert(isscalar(min_pts) && min_pts >= 1,  'min_pts must be a positive count (>=1).');
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

% Return if the input is empty

if isempty(pointcloud_array)
    XYZST_LaneMarkers_Array = [];
    return

end


% --- Extract commonly used columns from the input array -------------------
% These are the only columns we need downstream. Keeping explicit indexing
% reduces coupling to upstream layout changes and clarifies semantics.
s         = pointcloud_array(:,9);   % longitudinal station [m]
t         = pointcloud_array(:,10);  % lateral offset     [m]
intensity = pointcloud_array(:,4);   % reflectivity proxy [unitless]
z         = pointcloud_array(:,3);   % elevation          [m]

% --- Define s-bin boundaries ---------------------------------------------
% We discretize the path-longitudinal axis into fixed-width windows
% [s_low, s_low + s_width), then discretize each window again into strips
% with step s_res to build a 2D (s, t) sampling grid. This creates a set of
% quasi-rectangular "intensity images" per bin for peak finding.
s_min       = floor(min(s) / s_width) * s_width;
s_max       = ceil(max(s)  / s_width) * s_width;
s_bin_edges = s_min:s_width:s_max;
N_bins      = max(0, numel(s_bin_edges) - 1);  % guard for edge cases

% --- Initialize outputs / history caches --------------------------------
XYZST_lane_markers_Cell = cell(N_bins, 1);  % per-bin detections (later concatenated)
Pattern_Cell            = {};               % per-bin pattern decisions (templates)
Extrema_Filter_Cell     = {};               % per-bin extrema diagnostics
T_Ref_Cell              = {};               % per-bin t-grid used for matching
MatchError_Array        = [];               % per-bin template errors (vectorized)
Station_Array           = [];               % per_bin station
Z_Ref_Cell              = {};               % per_bin Z
% Choose an initial template. The pattern-matching routine may replace it
% adaptively when repeated mismatches indicate a persistent layout change.
pattern_template    = fcn_ExtractCL_createLanePattern('left_double_yellow_right_white', t_res);
best_pattern_template = nan;    % placeholder; updated by matcher
templateUpdateCount   = 0;      % optional: track # of adaptive updates

% --- Per-bin processing loop ---------------------------------------------
for ith_bin = 1:N_bins

    % 1) Slice the global point set into the current s-window.
    s_low  = s_bin_edges(ith_bin);
    s_high = s_bin_edges(ith_bin + 1);
    point_idx         = (s >= s_low & s < s_high);
    pointcloud_in_bin = pointcloud_array(point_idx,:);

    % Skip bins with too few points (avoid unstable extrema/pattern fits).
    nPts_in_bin = size(pointcloud_in_bin, 1);  % use size(...) for numeric arrays
    if isempty(pointcloud_in_bin) || nPts_in_bin < min_pts
        continue;
    end

    % 2) Prepare arrays for grid organization within this bin.
    s_bin         = s(point_idx);
    t_bin         = t(point_idx);
    z_bin         = z(point_idx);
    intensity_bin = intensity(point_idx);

    % 3) Define t-axis sampling range for this bin.
    % Clamp to uniform bins so the lateral signal is comparable across strips.
    t_min         = floor(min(t_bin) / t_res) * t_res;
    t_max         = ceil(max(t_bin)  / t_res) * t_res;
    t_strip_edges = t_min:t_res:t_max;

    % 4) Organize points into a regular (s, t) grid:
    %    - S_organized, T_organized: coordinate of grid samples
    %    - I_organized: intensity at samples (interpolated/aggregated)
    %    - Z_organized: height per sample (optional usage)
    %    - s_strip_centers: centers of the s-strips within this bin
    [S_organized, T_organized, I_organized, Z_organized, s_strip_centers] = ...
        fcn_ExtractCL_organizePointCloudST(...
            s_bin, t_bin, intensity_bin, z_bin, ...
            s_low, s_res, s_width, t_res, t_strip_edges);

    % 5) Optional visualization of raw and organized data for this bin.
    % (For debug)
    if flag_do_plot
        figure(fig_num); 
        clf;

        % Raw points: quick sanity check (do we have a banded structure?)
        scatter(s_bin, t_bin, 40, intensity_bin, 'filled', 'DisplayName','Raw point cloud');
        hold on; grid on; grid minor; axis equal;
        xlabel('S [m]'); ylabel('T [m]');
        xlim([min(s_bin)-0.5, max(s_bin)+0.5]);

        % Overlay organized strips. We tone-map intensity for visibility
        % without saturating high-reflectivity paint.
        den = max(1, max(I_organized(isfinite(I_organized)), [], 'all'));
        I_norm = I_organized ./ den;
        I_norm = I_norm .^ 0.6;  % gentle gamma correction
        I_norm = min(1, 0.25 + 0.7 * (I_norm .* (1 + 0.7*I_norm.^3))); % compress highlights

        % Render each strip using the same color mapping to show lateral ridges.
        for ith_s = 1:length(s_strip_centers)
            I_col = I_norm(:, ith_s);     
            strip_color = [I_col, 0.25*I_col, zeros(size(I_col))];
            scatter(S_organized(:,ith_s), T_organized(:,ith_s), 36, strip_color, 'filled', ...
                    'HandleVisibility', iff(ith_s==1,'on','off'), ...
                    'DisplayName', iff(ith_s==1,'Organized point cloud',''));
        end
        legend('Location','best');
    end

    % 6) Extrema filtering and pattern matching on the organized grid.
    %    The matcher returns:
    %       - lane_marker_mask      : logical mask over grid samples
    %       - pattern_cell          : cell array with pattern decision(s)
    %       - extrema_filter_cell   : diagnostics for peak detection
    %       - best_pattern_template : possibly updated template to use next
    %       - best_match_errors     : numeric errors per matched strip
    if flag_do_plot
        fig_num_inner = fig_num + 10;  % avoid clobbering outer figure
    else
        fig_num_inner = -1;
    end

    if flag_use_history_data
        [lane_marker_mask, pattern_cell, extrema_filter_cell, best_pattern_template, best_match_errors] = ...
            fcn_ExtractCL_findLaneMarkersByPattern( ...
                I_organized, pattern_template, t_res, templateUpdateCount, ...
                T_organized, fig_num_inner, ...
                HistoryData, pointcloud_in_bin, s_strip_edges, t_strip_edges);
    else
        [lane_marker_mask, pattern_cell, extrema_filter_cell, best_pattern_template, best_match_errors] = ...
            fcn_ExtractCL_findLaneMarkersByPattern( ...
                I_organized, pattern_template, t_res, templateUpdateCount, ...
                T_organized, fig_num_inner);
    end


    T_ref_cell_ith_bin          = repmat({t_strip_edges}, numel(pattern_cell), 1);

    % 8) Extract the detected lane marker samples and back-project to ENU.
    S_lane_markers = S_organized(lane_marker_mask);
    T_lane_markers = T_organized(lane_marker_mask);
    Z_lane_markers = Z_organized(lane_marker_mask);

    if isempty(S_lane_markers)
        % Nothing detected in this bin → continue without updating template
        continue;
    end

    % Convert (s, t) → (X, Y) using the prebuilt segment cache. We do not
    % alter Z here; we carry Z from the organized grid.
    XY_lane_markers     = fcn_ExtractCL_projectPC_STToENU(S_lane_markers, T_lane_markers, Seg);
    XYZST_lane_markers  = [XY_lane_markers, Z_lane_markers, S_lane_markers, T_lane_markers];

    % 9) Persist results and update the running template for the next bin.
    XYZST_lane_markers_Cell{ith_bin} = XYZST_lane_markers;
    pattern_template                  = best_pattern_template;  % adaptive update
    % (Optionally increment templateUpdateCount here based on a policy,
    %  e.g., if the pattern changed and stayed stable for N strips.)

    % Append history for analysis and future seeding.
    Station_Array = [Station_Array; s_strip_centers.'];
    Pattern_Cell        = [Pattern_Cell;        pattern_cell];
    MatchError_Array    = [MatchError_Array;    best_match_errors];
    Extrema_Filter_Cell = [Extrema_Filter_Cell; extrema_filter_cell];
    T_Ref_Cell          = [T_Ref_Cell;          num2cell(T_organized.',2)];
    Z_Ref_Cell          = [Z_Ref_Cell;          num2cell(Z_organized.',2)];

end % for each s-bin

% --- Concatenate all per-bin detections into a single array ---------------
% If no bins produced detections, this returns an empty matrix (0x0).
XYZST_LaneMarkers_Array = cell2mat(XYZST_lane_markers_Cell);

% --- Update HistoryData ---------------------------------------------------
% We store one row per matched strip (not per bin). This enables temporal
% adaptation: downstream runs can look at recent layout changes (e.g., a
% transition from double-yellow to dashed-white).
MatchError_Cell = num2cell(MatchError_Array(:));
Station_Cell = num2cell(Station_Array(:));

newData = struct( ...
    'S_Ref', Station_Cell,...
    'LanePattern',   Pattern_Cell(:), ...
    'ExtremaFilter', Extrema_Filter_Cell(:), ...
    'T_Ref',         T_Ref_Cell(:), ...
    'Z_Ref',         Z_Ref_Cell(:), ...
    'MatchError',    MatchError_Cell);

if isempty(HistoryData)
    HistoryData = newData;
else
    HistoryData = [HistoryData; newData]; %#ok<AGROW>
end

% --- Optional end-of-function debug print --------------------------------
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
    fprintf('By setting fig_num > 1 to plot')
    fprintf(1, 'ENDING function: %s\n', st(1).name);
end

end % Ends main function


% === Local utility (inline) ===============================================
function out = iff(cond, a, b)
% iff: inline ternary helper for labeling plots without cluttering the legend.
if cond, out = a; else, out = b; end
end
