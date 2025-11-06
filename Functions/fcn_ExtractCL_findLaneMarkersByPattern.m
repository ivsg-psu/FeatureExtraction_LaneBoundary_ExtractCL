function [lane_marker_mask, pattern_cell, extrema_filter_cell, best_pattern_template, best_fit_errors] = ...
    fcn_ExtractCL_findLaneMarkersByPattern(intensity_data, pattern_template, T_resolution, templateUpdateCount, t_profile, varargin)
% fcn_ExtractCL_matchLaneMarkerWithPattern
% Extracts lane centerline from intensity data using pattern matching and extrema filtering,
% optimized for white lane marker strips. Optionally uses history and previous templates.
%
% FORMAT:
% [lane_marker_mask, pattern_cell, extrema_filter_cell, best_pattern_template, best_fit_errors] = ...
%     fcn_ExtractCL_findLaneMarkersByPattern(intensity_data, pattern_template, T_resolution, templateUpdateCount, t_profile, varargin)
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
%   templateUpdateCount: scalar
%       Number 
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
%
%   DEPENDENCIES:
%      fcn_ExtractCL_matchPattern
%
% EXAMPLES:
%
%      See script: script_test_fcn_ExtractCL_findLaneMarkersByPattern
% This function was written on 2025_06_03 by X. Cao
% Questions or comments? xfc5113@psu.edu
%
% Revision history:
%      2025_06_03 - xfc5113@psu.edu
%     -- wrote the code originally
%      2025_08_11 - xfc5113@psu.edu
%      -- added flip pattern matching feature
%      -- added intensity smooth
%      2025_10_28 - xfc5113@psu.edu
%      -- remove flip pattern matching feature
%      -- edited comments and headings
%
% TO DO
%      - Complete the history guiding feature
%      - Allow adaptive WindowWidthS based on curvature or density

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

% Fast-path flag (when last varargin is -1)
flag_max_speed = 0;
if (nargin==6 && isequal(varargin{end},-1))
    flag_do_debug = 0;           % disable plotting
    flag_check_inputs = 0;       % skip input checks
    flag_max_speed = 1;
else
    % Default flags (env can override)
    flag_do_debug = 0;
    flag_check_inputs = 1;
    MATLABFLAG_LAPS_FLAG_CHECK_INPUTS = getenv("MATLABFLAG_LAPS_FLAG_CHECK_INPUTS");
    MATLABFLAG_LAPS_FLAG_DO_DEBUG     = getenv("MATLABFLAG_LAPS_FLAG_DO_DEBUG");
    if ~isempty(MATLABFLAG_LAPS_FLAG_CHECK_INPUTS)
        flag_check_inputs  = str2double(MATLABFLAG_LAPS_FLAG_CHECK_INPUTS);
    end
    if ~isempty(MATLABFLAG_LAPS_FLAG_DO_DEBUG)
        flag_do_debug = str2double(MATLABFLAG_LAPS_FLAG_DO_DEBUG);
    end
end


if flag_do_debug
    st = dbstack;
    fprintf(1,'STARTING function: %s, in file: %s\n',st(1).name,st(1).file);
    debug_fig_num = 999978; %#ok<NASGU>
else
    debug_fig_num = []; %#ok<NASGU>
end

% ---- Parse Optional Inputs (do NOT alter names/logic) ----
fig_num = -1;
flag_do_plot = 0;
if nargin >= 6
    fig_num = varargin{1};
    if fig_num >= 1
        flag_do_plot = 1;
    end
end

HistoryData = nan;
if nargin >= 7
    HistoryData = varargin{2};
end

pointcloud_in_bin = [];
if nargin >= 9
    pointcloud_in_bin = varargin{3};
end

s_strip_edges = nan;
if nargin >= 10
    s_strip_edges = varargin{4};
end

t_strip_edges = nan;
if nargin >= 11
    t_strip_edges = varargin{5};
end

% ---- Reuse History (do NOT alter names/logic) ----
flag_use_history_data = 0;
if ~isnan(HistoryData)
    OldCenterLine   = HistoryData.CenterLine;
    OldCenterLineXY = OldCenterLine(:,1:2);
    CenterLineTree  = KDTreeSearcher(OldCenterLineXY);
    OldPattern      = HistoryData.LanePattern;
    T_Ref_Cell      = HistoryData.T_Ref;
    % NOTE: s_in_bin is used later to find points within current strip
    s_in_bin        = pointcloud_in_bin(:,9);
    flag_use_history_data = 1;
end

% ---- Input Validation (shapes/types), no logic change ----
if flag_check_inputs
    % intensity_data: numeric 2-D, not empty
    assert(isnumeric(intensity_data) && ismatrix(intensity_data) && ~isempty(intensity_data), ...
        'intensity_data must be a non-empty numeric 2-D matrix [N x M].');
    % pattern_template: numeric column vector
    assert(isnumeric(pattern_template) && isvector(pattern_template) && size(pattern_template,2)==1, ...
        'pattern_template must be a numeric column vector [K x 1].');
    % T_resolution: positive scalar
    assert(isnumeric(T_resolution) && isscalar(T_resolution) && isfinite(T_resolution) && (T_resolution>0), ...
        'T_resolution must be a positive finite scalar.');
    % templateUpdateCount: scalar numeric
    assert(isnumeric(templateUpdateCount) && isscalar(templateUpdateCount), ...
        'templateUpdateCount must be a numeric scalar.');
    % t_profile: numeric matrix same size as intensity_data
    assert(isnumeric(t_profile) && isequal(size(t_profile), size(intensity_data)), ...
        't_profile must be a numeric matrix the same size as intensity_data [N x M].');

    % If history is used, do a few sanity checks without changing logic
    if flag_use_history_data
        assert(isstruct(HistoryData) && isfield(HistoryData,'CenterLine') && ...
               isfield(HistoryData,'LanePattern') && isfield(HistoryData,'T_Ref'), ...
               'HistoryData must contain fields: CenterLine, LanePattern, T_Ref.');
        % pointcloud_in_bin used only when history is on
        assert(isnumeric(pointcloud_in_bin) && size(pointcloud_in_bin,2) >= 10, ...
               'pointcloud_in_bin must be numeric with at least 10 columns (expect ST columns at 9-10).');
        % s_strip_edges / t_strip_edges should be vectors (monotonic not enforced here)
        if ~isnan(s_strip_edges)
            assert(isvector(s_strip_edges), 's_strip_edges must be a vector when provided.');
        end
        if ~isnan(t_strip_edges)
            assert(isvector(t_strip_edges), 't_strip_edges must be a vector when provided.');
        end
    end
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
% Core pipeline (unchanged logic):
% 1) Smooth intensity along T, build per-strip extrema filter.
% 2) Extrema correlation -> detrend -> normalize.
% 3) Pattern match (history-guided if available) to find target window.
% 4) Adaptive peak picking inside window; optional recovery search.
% 5) Build detected pattern & (optionally) adapt template.
% 6) Accumulate outputs (mask, pattern, filters, errors).
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% ---- Parameters & preallocations (no logic change) ----
lane_marker_width   = 0.1;   % expected physical width of lane marker strip (m)
half_indices_width  = round(lane_marker_width/T_resolution/2);
filter_factor       = 1.0;
filter_lengthI      = round(lane_marker_width/T_resolution*filter_factor);

extrema                 = Extrema_Vectorization();
[N_points, N_strips]    = size(intensity_data);
lane_marker_mask        = false(size(intensity_data));
pattern_cell            = cell(N_strips,1);
extrema_filter_cell     = cell(N_strips,1);
num_pattern_waves       = sum(diff(pattern_template) == 1);
number_of_extremaI      = num_pattern_waves;
best_pattern_template   = pattern_template;
best_fit_errors         = [];

% % intensity_data_smoothed = movmean(intensity_data,filter_lengthI,1,'Endpoints','shrink');
w_l = max(11,2*floor(filter_lengthI/2)+1);
intensity_data_smoothed = sgolayfilt(intensity_data,3,w_l,[],1);

% ---- Extrema filter coefficients per strip ----
filter_coeff = extrema.fcn_createOptimalExtremaFilter(intensity_data_smoothed, filter_lengthI);


for ith_strip = 1:N_strips
    current_pattern_template   = best_pattern_template;
    intensity_ith_strip        = intensity_data(:,ith_strip);
    intensity_ith_strip_smoothed = intensity_data_smoothed(:,ith_strip);
    dataLength                 = size(intensity_ith_strip_smoothed,1);
    patternLength              = size(current_pattern_template,1);

    % per-strip filter coefficients
    filter_coeff_ith_strip = filter_coeff(:,ith_strip);

    % correlation from extrema-based filter
    lengthDifference = dataLength - patternLength + 1;
    [~, extrema_corr] = extrema.fcn_findExtrema(intensity_ith_strip_smoothed, filter_coeff_ith_strip, number_of_extremaI);

    % --- Optional visualization of raw/smoothed/extrema profiles ---
    if flag_do_plot
        figure(fig_num);
        clf; hold on; grid on; grid minor;
        plot(t_profile(:, ith_strip), intensity_ith_strip,          'Color',[0.5 0.5 0.5],'LineWidth',2,'DisplayName','Raw intensity profile');
        plot(t_profile(:, ith_strip), intensity_ith_strip_smoothed, 'Color',[0.5 0.5 0],  'LineWidth',2,'DisplayName','Smoothed intensity profile');
        plot(t_profile(:, ith_strip), extrema_corr,                 'Color',[0.0 0.45 0.75],'LineWidth',3,'DisplayName','Extrema-filtered profile');
        legend('Location','best');
        xlabel('T [m]'); 
        ylabel('Intensity');
        xlim([min(t_profile(:, ith_strip)) max(t_profile(:, ith_strip))]);
        pause(0.01);
    end

    % detrend & normalize correlation
    % baseline             = movmedian(extrema_corr, 5*filter_lengthI,'Endpoints','shrink');
    % w = 5*filter_lengthI;
    % medLoc = movmedian(extrema_corr, w, 'Endpoints','shrink');
    % madLoc = 1.4826*movmedian(abs(extrema_corr - medLoc), w, 'Endpoints','shrink');
    % clipped = min(extrema_corr, medLoc + 1.5*madLoc);          
    baseline = mean(extrema_corr);
    extrema_corr_detrended = max(0, extrema_corr - baseline);
    corr_min             = min(extrema_corr_detrended);
    corr_max             = max(extrema_corr_detrended);
    range                = corr_max - corr_min;
    if range <= eps
        extrema_corr_norm = zeros(size(extrema_corr_detrended));
    else
        extrema_corr_norm = (extrema_corr_detrended - corr_min) / range;
    end

    % guard: if template longer than data, skip
    if length(extrema_corr_norm) < length(current_pattern_template)
        continue;
    end

    % ---- history-guided matching (if enabled, update later) ----
    if flag_use_history_data
        s_low_edge   = s_strip_edges(ith_strip);
        s_high_edge  = s_strip_edges(ith_strip + 1);
        point_idx    = (s_in_bin >= s_low_edge) & (s_in_bin < s_high_edge);
        points_in_strip = pointcloud_in_bin(point_idx,:);
        StripCenter_XY  = mean(points_in_strip(:,1:2));
        idx_closest     = knnsearch(CenterLineTree, StripCenter_XY);
        matched_pattern = interp1(T_Ref_Cell{idx_closest}, OldPattern{idx_closest}, t_strip_edges, 'nearest', 0);
        pattern_start_idx = find(matched_pattern,1,"first");
        differenceArray = extrema_corr_norm - matched_pattern;
        best_fit_error  = sum(differenceArray.^2);
    else
        % template-based matching
        [matched_pattern_normal, pattern_start_idx_normal, best_fit_error_normal, MSE_normal] = ...
            fcn_ExtractCL_matchPattern(extrema_corr_norm, current_pattern_template);
        pattern_start_idx = pattern_start_idx_normal;
        pattern_end_idx   = pattern_start_idx_normal + patternLength  - 1;

        best_fit_error    = best_fit_error_normal;
        matched_pattern   = matched_pattern_normal;
        MSE               = MSE_normal;
    end

    % widen match window slightly (tolerance_ratio = 5%)
    tolerance_ratio   = 0.05;
    pattern_start_idx = max(1, ceil((1 - tolerance_ratio)*pattern_start_idx));
    pattern_end_idx   = min(floor((1 + tolerance_ratio)*pattern_end_idx), N_points);

    % zero-out correlation outside matched window
    extrema_corr_clean = extrema_corr_norm;
    extrema_corr_clean(1:pattern_start_idx) = 0;
    extrema_corr_clean(pattern_end_idx:end) = 0;

    % adaptive thresholds via MAD (core window)
    extrema_corr_clean_median = median(extrema_corr_clean,'omitnan');
    extrema_corr_clean_off    = extrema_corr_clean - extrema_corr_clean_median;
    extrema_corr_clean_MAD    = 1.4826 * median(abs(extrema_corr_clean_off), 'omitnan');
    MinPromCore        = min(max(3.0 * extrema_corr_clean_MAD, 0.05), 0.15);
    MinPeakHeightCore  = extrema_corr_clean_median + min(max(3.0 * extrema_corr_clean_MAD, 0.3), 0.4);

    % expected peak count from template
    num_peaks_expected = length(findpeaks(current_pattern_template));

    % candidate centers (within matched window)
    [~, lane_marker_center_cands] = findpeaks( ...
        extrema_corr_clean, 'MinPeakProminence', MinPromCore, "MinPeakHeight", MinPeakHeightCore, "SortStr", 'descend');
    num_peaks_detected = length(lane_marker_center_cands);
    outer_candidates   = [];

    % ---- recovery sweep if some peaks missing ----
    if num_peaks_detected < num_peaks_expected
        overlap_ratio   = 0.8;
        overlap_window  = round((1 - overlap_ratio)*patternLength);
        T_scan_min      = max(pattern_start_idx - overlap_window - half_indices_width, 1);
        T_scan_max      = min(pattern_end_idx + overlap_window + half_indices_width, length(extrema_corr_norm));
        left_zone       = T_scan_min:(pattern_start_idx - 1);
        right_zone      = (pattern_end_idx + 1):T_scan_max;
        scan_window     = [left_zone, right_zone];
        extrema_corr_outer = extrema_corr_norm(scan_window);

        % thresholds for the recovery band
        extrema_corr_outer_median = median(extrema_corr_outer,'omitnan');
        extrema_corr_outer_hat    = extrema_corr_outer - extrema_corr_outer_median;
        extrema_corr_outer_MAD    = 1.4826 * median(abs(extrema_corr_outer_hat), 'omitnan');
        MinProm       = min(max(3.0 * extrema_corr_outer_MAD, 0.05), 0.15);
        MinPeakHeight = extrema_corr_outer_median + min(max(3.0 * extrema_corr_outer_MAD, 0.3), 0.4);

        [~, peaks_indices] = findpeaks( ...
            extrema_corr_norm, "MinPeakProminence", MinProm, 'MinPeakHeight', MinPeakHeight, 'MinPeakDistance', 15);

        mse_candidate = []; % keep your variable; filled when outer_candidates exist

        if ~isempty(peaks_indices)
            mask             = ismember(peaks_indices, scan_window);
            outer_candidates = peaks_indices(mask);
            for candidate = outer_candidates(:)'
                if candidate <= lengthDifference
                    candidate_match_index = max(candidate - half_indices_width, 1);
                else
                    candidate_match_index = min(max(candidate + half_indices_width - patternLength,1), lengthDifference);
                end
                mse_candidate = [mse_candidate; MSE(candidate_match_index)]; %#ok<AGROW>
            end
        end
    end

    % merge outer candidates if any
    if ~isempty(outer_candidates)
        lane_marker_center_cands = [lane_marker_center_cands; outer_candidates]; %#ok<AGROW>
    end

    % build detected pattern mask around each center (± half_indices_width)
    lane_marker_center_cands = sort(lane_marker_center_cands,'ascend');
    detected_pattern         = zeros(size(matched_pattern));
    lane_marker_indices      = lane_marker_center_cands + (-half_indices_width:half_indices_width);
    lane_marker_indices      = lane_marker_indices(lane_marker_indices >= 1 & lane_marker_indices <= N_points);
    lane_marker_indices      = sort(lane_marker_indices,'ascend');
    detected_pattern(lane_marker_indices) = 1;

    % if still nothing, skip this strip
    if isempty(lane_marker_center_cands)
        continue
    end

    best_pattern = detected_pattern;

    % a compact template from first-to-last detected indices (padding zeros at ends)
    detected_pattern_template = [0; detected_pattern(max([1,min(lane_marker_indices)]): ...
                                   min([N_points,max(lane_marker_indices)])); 0];

    % adapt template if mismatch persists long enough
    if num_peaks_expected ~= num_peaks_detected
        templateUpdateCount = templateUpdateCount + 1;
    else
        templateUpdateCount = 0;
    end
    UpdateCount_threh = 30;
    if templateUpdateCount > UpdateCount_threh && num_peaks_detected > 1
        best_pattern_template = detected_pattern_template;
    else
        best_pattern_template = current_pattern_template;
    end


    % Save outputs for this strip
    lane_marker_mask(lane_marker_center_cands, ith_strip) = true;
    best_fit_errors = [best_fit_errors; best_fit_error]; %#ok<AGROW>
    pattern_cell{ith_strip,1} = best_pattern;
    extrema_filter_cell{ith_strip,1} = filter_coeff_ith_strip.';

    % --- Optional final overlay plot for this strip ---
    if flag_do_plot
        figure(fig_num + 5);
        clf; hold on;
        plot(t_profile(:, ith_strip), extrema_corr_norm, 'Color',[0.0 0.45 0.75],'LineWidth',3);
        plot(t_profile(:, ith_strip), best_pattern,      'Color',[0.0 0.6 0],   'LineWidth',3);
        if ~isempty(lane_marker_center_cands)
            xline(t_profile(lane_marker_center_cands, ith_strip), 'r--', 'LineWidth', 3);
        end
        legend('Extrema-filtered intensity', 'Best Matched Pattern', 'Lane marker candidates');
        set(findall(gca, 'Type', 'Line'), 'LineWidth', 3);
        grid on; grid minor;
        xlim([t_profile(1, ith_strip) t_profile(end, ith_strip)]);
        title('Center line extraction via pattern matching');
        xlabel('T [m]'); ylabel('Normalized correlation');
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
% (Kept commented for speed; nothing removed.)
%
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
