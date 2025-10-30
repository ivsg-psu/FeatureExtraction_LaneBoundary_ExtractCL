function [RoadCenterLine, LaneMarkerCenterLine] = fcn_ExtractCL_computeCLwithLaneMarkers(LaneMarkers, mode, HistoryData, t_res, Seg)
% fcn_ExtractCL_computeCLwithLaneMarkers
% -------------------------------------------------------------------------
% Purpose:
%   Compute the road center line using per-station binary pattern profiles
%   stored in HistoryData.PatternCell. Each pattern indicates lateral lane
%   marker presence (1) or absence (0). Depending on the mode, the function
%   either directly computes the lane centerline (left mode) or combines
%   the solid right lane marker and the detected pattern structure to infer
%   the road centerline (right mode).
%
%   - mode 'left' : Directly use the left lane markers as reference.
%   - mode 'right': Use right lane markers (typically solid, clear) as the
%                   base, and use per-station pattern structure to infer
%                   offset toward the left side.
%
% FORMAT:
%   [RoadCenterLine, LaneMarkerCenterLine] = ...
%       fcn_ExtractCL_computeCLwithLaneMarkers(LaneMarkers, mode, HistoryData, t_res, Seg)
%
% INPUTS:
%   LaneMarkers : struct
%       .LaneMarkerLeft  -> N×5 [X Y Z S T]
%       .LaneMarkerRight -> M×5 [X Y Z S T]
%
%   mode        : 'left' or 'right'
%
%   HistoryData : struct containing per-station pattern info
%       .S_Ref       : [N×1] station vector
%       .LanePattern : {N×1} binary vector [0/1] per station
%       .T_Ref       : {N×1} lateral coordinate grid
%       .Z_Ref       : {N×1} corresponding Z values for interpolation
%
%   t_res       : scalar, lateral resolution [m] used to scale index offset
%   Seg         : segment structure for projecting (S,T) → (X,Y)
%
% OUTPUTS:
%   RoadCenterLine      : K×5 [X Y Z S T] road centerline points
%   LaneMarkerCenterLine: K×5 [X Y Z S T] reference lane-marker centerline
%
% -------------------------------------------------------------------------
% DETAILS:
%   For each station, the function finds peaks (continuous 1-ranges) within
%   the binary pattern and determines their relative positions:
%       - n_peaks = 3 → (single + double yellow)
%           → nearest pair = double yellow
%           → offset_idx = mean(double) - single
%       - n_peaks = 2 → (two single markers)
%           → offset_idx = mean(diff(peaks)) / 2
%       - n_peaks ≤ 1 → discarded
%
%   The offset is computed in *index units*; physical offset is:
%       offset_m = offset_idx * t_res
%
% -------------------------------------------------------------------------
% NOTE:
%   Although this pattern-guided approach is useful when the left markers
%   are ambiguous (e.g., faded double yellows), it is **recommended** to
%   directly use the selected lane marker centerline whenever possible.
%   Combining marker geometry with pattern-derived offsets introduces
%   dependency on pattern discretization and may increase noise if the
%   lateral resolution is coarse.
%
% This function was written on 2025_10_29 by X. Cao
% Questions or comments? xfc5113@psu.edu
%
% Revision history:
%      2025_10_29 - xfc5113@psu.edu
%     -- wrote the code originally
% -------------------------------------------------------------------------


flag_check_inputs = 1;
flag_do_debug = 0;

if flag_do_debug
    st = dbstack;
    fprintf(1,'STARTING function: %s\n', st(1).name);
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
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Input checking
if flag_check_inputs
    if nargin < 3
        error('Usage: fcn_ExtractCL_computeCLwithLaneMarkers(LaneMarkers, mode, HistoryData, t_res, Seg)');
    end
    if ~isstruct(LaneMarkers) || ~isstruct(HistoryData)
        error('Inputs must be structs.');
    end
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

%% Main computation
mode = lower(string(mode));
switch mode
    case "left"
        P = LaneMarkers.LaneMarkerLeft;
    case "right"
        P = LaneMarkers.LaneMarkerRight;
    otherwise
        error('mode must be ''left'' or ''right''.');
end

if isempty(P)
    warning('Selected side has no lane markers.');
    RoadCenterLine = [];
    LaneMarkerCenterLine = [];
    return;
end

% -------------------------------
% Parameter thresholds
% -------------------------------
thresh_outlier   = 0.20;  % 20 cm lateral deviation tolerance
max_span_single  = 0.20;  % maximum single-lane width
max_span_double  = 0.40;  % maximum double-lane width

% Reference station list
if isfield(HistoryData, "S_Ref")
    S_Ref = [HistoryData.S_Ref].';
else
    error('HistoryData must include field S_Ref.');
end

N_stations = length(S_Ref);
LaneMarkerCenterLine = nan(N_stations, 5);
RoadCenterLine       = nan(N_stations, 5);

% -------------------------------
% Determine lane-marker type by density
% -------------------------------
S_all = P(:,4);
N_total = length(S_all);
N_actual_stations = length(unique(S_all));
markerDensity = N_total / N_actual_stations;
densityThresh = 1; % >1 → double marker
laneMarkerType = "single";
if markerDensity > densityThresh
    laneMarkerType = "double";
end


% -------------------------------
% Per-station loop
% -------------------------------
for i_station = 1:N_stations
    s_val = S_Ref(i_station);
    P_s = P(S_all == s_val,:);
    if isempty(P_s)
        continue; 
    end

    % --- Remove outliers by T median ---
    T_s = P_s(:,5);
    T_med = median(T_s);
    P_s = P_s(abs(T_s - T_med) <= thresh_outlier,:);
    % P_s
    if isempty(P_s)
        continue; 
    end

    % --- Width sanity filter by marker type ---
    span_T = max(P_s(:,5)) - min(P_s(:,5));
    if strcmp(laneMarkerType,'single') && span_T > max_span_single
        continue;
    elseif strcmp(laneMarkerType,'double') && span_T > max_span_double
        continue;
    end

    % --- Compute per-station lane marker center (average of filtered points) ---
    LaneMarkerCenterLine(i_station,:) = mean(P_s,1);

    % Only perform pattern-guided computation in "right" mode
    if strcmp(mode, "right")

        % Extract pattern at this station
        lanePattern_i = (HistoryData(i_station).LanePattern).';
        if isempty(lanePattern_i) || all(lanePattern_i==0)
            continue;
        end
    
        % --- Detect peaks from binary pattern ---
        edges = diff([0 lanePattern_i 0]);
        rise_idx = find(edges==1);
        fall_idx = find(edges==-1);
        n_peaks = min(length(rise_idx), length(fall_idx));
        if n_peaks < 2
            continue; % discard sparse patterns
        end
    
        % --- Compute center indices of each peak (explicit loop) ---
        center_idx = zeros(n_peaks,1);
        for k = 1:n_peaks
            r = rise_idx(k);
            f = fall_idx(k);
            center_idx(k) = floor((r + f)/2);
        end
    
        % --- Compute offset index based on number of peaks ---
        if n_peaks == 2
            offset_idx = mean(diff(center_idx))/2;
        elseif n_peaks == 3
            % Identify nearest pair (double yellow)
            [~, minIdx] = min([abs(center_idx(1)-center_idx(2)), ...
                                abs(center_idx(1)-center_idx(3)), ...
                                abs(center_idx(2)-center_idx(3))]);
            switch minIdx
                case 1
                    double_pair = center_idx(1:2); single_idx = center_idx(3);
                case 2
                    double_pair = [center_idx(1) center_idx(3)]; single_idx = center_idx(2);
                case 3
                    double_pair = center_idx(2:3); single_idx = center_idx(1);
            end
            offset_idx = mean(double_pair) - single_idx;
        else
            continue;
        end
    
        % --- Compute (S,T,Z) for road centerline ---
        T_ref_i = HistoryData(i_station).T_Ref;
        Z_ref_i = HistoryData(i_station).Z_Ref;
        t_center = LaneMarkerCenterLine(i_station, 5) + offset_idx * t_res;
        z_center = interp1(T_ref_i, Z_ref_i, t_center,'linear','extrap');

        % Project back to ENU frame
        xy_center = fcn_ExtractCL_projectPC_STToENU(s_val, t_center, Seg);
        RoadCenterLine(i_station,:) = [xy_center z_center s_val t_center];
    end   
end


% -------------------------------
% Post-processing: clean up and sort
% -------------------------------
if strcmp(mode, "left")
    RoadCenterLine = LaneMarkerCenterLine;
end

LaneMarkerCenterLine = LaneMarkerCenterLine(all(isfinite(LaneMarkerCenterLine),2),:);
LaneMarkerCenterLine = sortrows(LaneMarkerCenterLine,4);

RoadCenterLine = RoadCenterLine(all(isfinite(RoadCenterLine),2),:);
RoadCenterLine = sortrows(RoadCenterLine,4);


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

if flag_do_debug
    fprintf(1,'ENDING function: %s\n', st(1).name);
end

end
