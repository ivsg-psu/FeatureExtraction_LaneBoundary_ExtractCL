function [LaneMarkers ,islands, outliers]= fcn_ExtractCL_separateLaneMarkers(XYZST_lane_markers_array, varargin)
% fcn_ExtratCL_separateLaneMarkers
% -------------------------------------------------------------------------
% Purpose:
%   Split a mixed set of lane-marker points into four groups purely by the
%   lateral coordinate T using robust (MAD-based) side-specific thresholds:
%     - LaneMarkerLeft  : points within [lower, upper] on the LEFT side
%     - LaneMarkerRight : points within [lower, upper] on the RIGHT side 
%     - islands         : points between the two side intervals
%     - outliers        : points outside both side intervals
%
% Inputs:
%   XYZST_lane_markers_array : N×5 numeric, columns are [X Y Z S T]
%
% Outputs:
%   LaneMarkers : struct with fields
%                   .LaneMarkerLeft  -> M×5 subset (left-side inliers)
%                   .LaneMarkerRight -> K×5 subset (right-side inliers)
%   islands     : Q×5 subset of points between the two side thresholds
%   outliers    : R×5 subset of points outside both side thresholds
%
% Notes:
%   - Thresholds are computed independently on each side using
%     median ± k*MAD (MAD scaled by 1.4826 for Gaussian consistency).
%   - k is fixed at 3 in this routine.
%   - No coordinates are modified; this only partitions rows.
%
% Revision history:
%    2025_08_11 - xfc5113@psu.edu
%    -- wrote the code originally
%    2025_09_23 - xfc5113@psu.edu
%    -- replace 0 with mean(T) as the separator of left/right zones
%    2025_10_28 - xfc5113@psu.edu
%    -- replace mean(T) with movmean(T,100) as the separator
% -------------------------------------------------------------------------

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
flag_do_debug   = 0;
flag_check_inputs = 1;
t1 = getenv("MATLABFLAG_CL_FLAG_DO_DEBUG");      
if ~isempty(t1) 
    flag_do_debug    = str2double(t1); 
end
t2 = getenv("MATLABFLAG_CL_FLAG_CHECK_INPUTS");   
if ~isempty(t2)
    flag_check_inputs = str2double(t2); 
end



if flag_check_inputs
    % Type and size
    assert(isnumeric(XYZST_lane_markers_array) && ~issparse(XYZST_lane_markers_array), ...
        'XYZST_lane_markers_array must be a numeric full matrix.');
    assert(size(XYZST_lane_markers_array,2) >= 5, ...
        'XYZST_lane_markers_array must have at least 5 columns: [X Y Z S T].');
    assert(~isempty(XYZST_lane_markers_array), ...
        'XYZST_lane_markers_array is empty.');

    % Finite check on the S/T columns used below
    S_col = XYZST_lane_markers_array(:,4);
    T_col = XYZST_lane_markers_array(:,5);
    assert(any(isfinite(S_col)) && any(isfinite(T_col)), ...
        'S/T columns contain no finite values.');

    % Warn (not assert) if very few points (movmean window may exceed length)
    if size(XYZST_lane_markers_array,1) < 20
        warning('fcn_ExtractCL_separateLaneMarkers:FewPoints', ...
            'Very few points (%d). Robust thresholds may be unstable.', size(XYZST_lane_markers_array,1));
    end
end

if nargin > 2
    error('fcn_ExtractCL_separateLaneMarkers requires 1 or 2 inputs');
end

flag_do_plot = 0;
if nargin == 2
    fig_num = varargin{1};
    flag_do_plot = 1;
end

if flag_do_debug
    st = dbstack; 
    fprintf(1,'STARTING function: %s\n', st(1).name);
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
% The following implements the requested logic:
% - Use movmean(T,100) to define a smooth separator between left/right zones
% - Compute robust thresholds per side via median ± 3*MAD
% - Classify into left/right inliers, islands, and outliers
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Extract lateral coordinate T
T = XYZST_lane_markers_array(:,5);

% Define left/right zones by the T_mean (need to be modified)
% (As per revision: use moving mean over a window of 100 samples.)
T_mean_sm = movmean(T,100);
left_zone = T >= T_mean_sm;
right_zone = T < T_mean_sm;

% Gather T on each side
T_left_zone = T(left_zone);
T_right_zone = T(right_zone);

% Compute robust thresholds per side using median ± k*MAD (k = 3)
[T_threshold_left_lower, T_threshold_left_upper] = fcn_Internal_calculateThreshold(T_left_zone, 3);
[T_threshold_right_lower, T_threshold_right_upper]= fcn_Internal_calculateThreshold(T_right_zone,3);

% Classify points:
%   - Left/Right lane markers: keep points within each side's [lower, upper]
%   - islands: points lying between the two side intervals
%   - outliers: points outside both side intervals
left_lane_markers_mask  = ((T <= T_threshold_left_upper)  & (T >= T_threshold_left_lower));
right_lane_markers_mask = ((T <= T_threshold_right_upper) & (T >= T_threshold_right_lower));
islands_mask            = ((T < T_threshold_left_lower)   & (T >  T_threshold_right_upper));
outlier_mask            = ((T > T_threshold_left_upper)   & (T <  T_threshold_right_lower));

% Pack outputs
LaneMarkers = struct();
LaneMarkers.LaneMarkerLeft  = XYZST_lane_markers_array(left_lane_markers_mask ,:);
LaneMarkers.LaneMarkerRight = XYZST_lane_markers_array(right_lane_markers_mask,:);
islands  = XYZST_lane_markers_array(islands_mask ,:);
outliers = XYZST_lane_markers_array(outlier_mask,:);

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
if flag_do_debug
    


    st = dbstack; 
    fprintf(1,'ENDING function: %s\n', st(1).name);

end

if flag_do_plot
       
    figure(fig_num)
    clf
    plot(XYZST_lane_markers_array(:,4), T)
    hold on
    plot(XYZST_lane_markers_array(:,4), T_mean_sm)
    xlabel('Station [m]')
    ylabel('Lateral coordinate T [m]')

end

end


function [T_threshold_lower, T_threshold_upper] = fcn_Internal_calculateThreshold(T, k)
% fcn_Internal_calculateThreshold
% -------------------------------------------------------------------------
% Purpose:
%   Given a vector T (one side), compute robust lower/upper bounds as
%   median ± k*MAD, where MAD is scaled by 1.4826.
%
% Inputs:
%   T : vector of lateral coordinates on one side
%   k : scalar multiplier for MAD (e.g., 3)
%
% Outputs:
%   T_threshold_lower : median(T) - k*1.4826*MAD(|T - median|)
%   T_threshold_upper : median(T) + k*1.4826*MAD(|T - median|)
% -------------------------------------------------------------------------

    % Robust center and scale
    T_median = median(T, 'omitnan');
    T_hat    = T - T_median;
    T_MAD    = 1.4826 * median(abs(T_hat), 'omitnan');

    % Symmetric bounds around the median
    T_threshold_upper = T_median + k*T_MAD;
    T_threshold_lower = T_median - k*T_MAD;
end
