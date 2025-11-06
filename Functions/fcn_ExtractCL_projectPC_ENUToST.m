function [pointCloud_ST_cell, ref_station, ST_struct] = fcn_ExtractCL_projectPC_ENUToST(PointCloud_ENU_cell, Ref_Pose, varargin)
% fcn_ExtractCL_projectPC_ENUToST
% Projects ENU LiDAR point clouds into the curvilinear (s, t) frame with
% respect to a given reference trajectory. Each LiDAR point is associated
% to its closest reference-segment, producing longitudinal station s and
% lateral offset t for downstream lane-centerline extraction and mapping.
%
% FORMAT:
%
%      [pointCloud_ST_cell, ref_station, Seg] = ...
%      fcn_ExtractCL_projectPC_ENUToST(...
%           PointCloud_ENU_cell,...
%           Ref_Pose,...
%           (fig_num_or_minus1));
%
% INPUTS:
%
%      PointCloud_ENU_cell: Lx1 cell array
%          Each cell contains a LiDAR scan in ENU coordinates, formatted as:
%          [X Y Z Intensity (TimeOffset) (Ring) ...]. Only XYZI are required.
%
%      Ref_Pose: MxK numeric array (K >= 3 recommended)
%          Reference trajectory with rows like [X Y Z Roll Pitch Yaw].
%          Only [X, Y, Z, Yaw] are needed by downstream helpers.
%
%      (OPTIONAL) fig_num_or_minus1: scalar
%          If -1, disables debugging and input checking (max-speed mode).
%          Otherwise treated as a figure number for plotting (debug view).
%
% OUTPUTS:
%
%      pointCloud_ST_cell: Lx1 cell
%          Per-frame arrays with appended (s, t) columns:
%          [X, Y, Z, Intensity, ..., s, t].
%
%      ref_station: Mx1 vector
%          Cumulative station (arc length) along the reference trajectory.
%
%      Seg: struct
%          Precomputed segment cache returned by
%          fcn_ExtractCL_buildSegmentsFromRefPose (segment tree, tangents,
%          normals, segment starts/ends, etc.).
%
% DEPENDENCIES:
%
%      fcn_ExtractCL_convertRefTrajToST     % convert ref trajectory to ST
%      coordinate
%      Statistics and Machine Learning Toolbox   % knnsearch / KDTree
%
% EXAMPLES:
%
%      See script: script_test_fcn_ExtractCL_projectPC_ENUToST
%
% This function was written on 2025_06_23 by X. Cao
% Questions or comments? xinyucao@psu.edu
%
% Revision history:
%
%      2025_08_15 - xfc5113@psu.edu
%      -- added fcn_ExtractCL_buildSegmentsFromRefPose
%      2025_10_28 - xfc5113@psu.edu
%      -- aligned headings/comments to project template
%      -- added strict input checks and debug/plot gating
%      2025_11_05 - xfc5113@psu.edu
%      -- renamed fcn_ExtractCL_buildSegmentsFromRefPose to 
%         fcn_ExtractCL_convertRefTrajToST
% TO DO
%      - Add optional robust fallbacks if knnsearch is unavailable

%% Debug and input-check flags
flag_max_speed = 0;
if nargin >= 3 && isequal(varargin{end}, -1)
    flag_do_debug = 0;
    flag_check_inputs = 0;
    flag_max_speed = 1;
    fig_num = -1;
else
    % Defaults
    flag_do_debug = 0;
    flag_check_inputs = 1;
    fig_num = -1;
    % Environment gating (optional)
    t1 = getenv("MATLABFLAG_CL_FLAG_DO_DEBUG");        if ~isempty(t1), flag_do_debug    = str2double(t1); end
    t2 = getenv("MATLABFLAG_CL_FLAG_CHECK_INPUTS");    if ~isempty(t2), flag_check_inputs = str2double(t2); end
    % If a normal figure number was provided
    if ~isempty(varargin)
        fig_num = varargin{1};
    end
end
flag_do_plot = isnumeric(fig_num) && isscalar(fig_num) && isfinite(fig_num) && (fig_num > 1);

% Tell user where we are
if flag_do_debug
    st = dbstack; fprintf(1,'STARTING function: %s, in file: %s\n',st(1).name,st(1).file);
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
if flag_check_inputs
    % Basic type/shape checks
    if ~iscell(PointCloud_ENU_cell) || isempty(PointCloud_ENU_cell)
        error('fcn_ExtractCL_projectPC_ENUToST:BadPointCloudCell',...
              'PointCloud_ENU_cell must be a non-empty cell array of frames.');
    end
    if ~(isnumeric(Ref_Pose) && ismatrix(Ref_Pose) && ~isempty(Ref_Pose))
        error('fcn_ExtractCL_projectPC_ENUToST:BadRefPose',...
              'Ref_Pose must be a non-empty numeric 2-D array (MxK).');
    end
    if size(Ref_Pose,2) < 4
        error('fcn_ExtractCL_projectPC_ENUToST:RefPoseTooFewCols',...
              'Ref_Pose must have at least 4 columns (X Y Z Yaw ...).');
    end

    % Per-frame checks (only light checks to keep runtime low)
    for iFrame = 1:numel(PointCloud_ENU_cell)
        frameArr = PointCloud_ENU_cell{iFrame};
        if isempty(frameArr)
            continue; 
        end
        if ~(isnumeric(frameArr) && ismatrix(frameArr) && size(frameArr,2) >= 4)
            error('fcn_ExtractCL_projectPC_ENUToST:BadFrameFormat',...
                  'Each frame must be numeric Nx4+ with [X Y Z Intensity ...].');
        end
    end

    % Toolbox / function availability
    if exist('knnsearch','file') ~= 2
        error('fcn_ExtractCL_projectPC_ENUToST:MissingKNN',...
              'knnsearch not found. Statistics and Machine Learning Toolbox is required.');
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

% Build/reference ST structure from the reference pose
ST_struct = fcn_ExtractCL_convertRefTrajToST(Ref_Pose, varargin{:});
ref_station          = ST_struct.ref_station;
traj_start           = ST_struct.traj_start;
traj_end             = ST_struct.traj_end;
segment_length       = ST_struct.segment_length;
seg_tangent          = ST_struct.seg_tangent;
seg_normal           = ST_struct.seg_normal;
seg_start_station    = ST_struct.seg_start_station;
d_seg                = ST_struct.d_seg;
seg_tree             = ST_struct.seg_tree;

% Quick guard for empty/degenerate path
if isempty(ref_station) || ref_station(end) <= 0
    pointCloud_ST_cell = cell(size(PointCloud_ENU_cell));
    if flag_do_debug, fprintf(1,'Empty/degenerate station profile. Returning empties.\n'); end
    return
end

% Parameters
Kcand = 5;                                   % number of candidate segments to evaluate per point
N_LiDAR_frames = length(PointCloud_ENU_cell);
s_total_length = ref_station(end);
pointCloud_ST_cell = cell(N_LiDAR_frames,1);

% Optional debug-plot accumulators
if flag_do_plot
    all_s = []; all_t = []; all_intensity = [];
end

% Per-frame projection
for ith_frame = 1:N_LiDAR_frames
    PointCloud_ENU_array = PointCloud_ENU_cell{ith_frame};
    if isempty(PointCloud_ENU_array)
        pointCloud_ST_cell{ith_frame} = [];
        continue;
    end

    points_xyz        = PointCloud_ENU_array(:,1:3);
    points_xy         = points_xyz(:,1:2);
    points_intensity  = PointCloud_ENU_array(:,4);

    % Candidate segment indices per point
    idx_nearest = knnsearch(seg_tree, points_xy,'K',Kcand);

    % Preallocate best results
    Npts            = size(points_xy,1);
    best_dist2      = inf(Npts,1);
    best_station    = zeros(Npts,1);
    best_T          = zeros(Npts,1);

    % Evaluate each candidate segment
    for ith_candidate = 1:Kcand
        candidate_indices        = idx_nearest(:,ith_candidate);

        seg_start_candidate      = traj_start(candidate_indices,:);        % [Nx2]
        seg_end_candidate        = traj_end(candidate_indices,:);          % [Nx2]
        seg_tangent_candidate    = seg_tangent(candidate_indices,:);       % [Nx2] unit
        seg_normal_candidate     = seg_normal(candidate_indices,:);        % [Nx2] unit
        segment_length_candidate = segment_length(candidate_indices);       % [Nx1]
        segment_start_candidate  = seg_start_station(candidate_indices);    % [Nx1]
        d_seg_candidate          = d_seg(candidate_indices);                % [Nx1]

        % Normalized projection factor along the segment, clipped to [0,1]
        denom = max(segment_length_candidate, eps);
        alpha = sum((points_xy - seg_start_candidate).*seg_tangent_candidate,2)./denom;
        alpha = max(0, min(1, alpha));

        % Closest point on segment and residual
        points_proj_on_seg = seg_start_candidate + alpha.*(seg_end_candidate - seg_start_candidate);
        relative_offsets   = points_xy - points_proj_on_seg;
        dist2              = sum(relative_offsets.^2,2);

        % Keep closest candidate
        tf_is_closer                   = dist2 < best_dist2;
        best_dist2(tf_is_closer)       = dist2(tf_is_closer);
        best_station(tf_is_closer)     = segment_start_candidate(tf_is_closer) + alpha(tf_is_closer).*d_seg_candidate(tf_is_closer);
        best_T(tf_is_closer)           = sum(seg_normal_candidate(tf_is_closer,:).*relative_offsets(tf_is_closer,:),2);
    end

    % Assemble per-frame output: append s, t
    s_array = best_station;
    t_array = best_T;
    pointCloud_ST_cell{ith_frame} = [PointCloud_ENU_array, s_array, t_array];

    % Aggregate for debug plotting
    if flag_do_plot
        all_s         = [all_s; mod(s_array, s_total_length)]; %#ok<AGROW>
        all_t         = [all_t; t_array];                      %#ok<AGROW>
        all_intensity = [all_intensity; points_intensity];     %#ok<AGROW>
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
if flag_do_plot
    figure(fig_num); clf;
    scatter(all_s, all_t, 16, all_intensity, 'filled');
    axis equal; grid on;
    xlabel('s [m]');
    ylabel('t [m]');
    title('LiDAR point cloud projected into (s, t) frame');
    cb = colorbar; %#ok<NASGU>
end

if flag_do_debug
    fprintf(1,'ENDING function: %s\n', st(1).name);
end

end % Ends main function
