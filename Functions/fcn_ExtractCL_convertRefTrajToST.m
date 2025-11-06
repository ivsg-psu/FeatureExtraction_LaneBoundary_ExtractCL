function ST_struct = fcn_ExtractCL_convertRefTrajToST(Ref_Pose, varargin)
% fcn_ExtractCL_convertRefTrajToST
% Builds segment-wise geometry from a reference trajectory for reuse.
% Each adjacent pair of reference points forms a segment with associated
% station, tangent, and left-normal vectors. This struct is used for
% efficient LiDAR point projection in (s, t) coordinates.
%
% FORMAT:
%
%      ST_struct = fcn_ExtractCL_convertRefTrajToST(...
%                  Ref_Pose,...
%                  (flag_do_debug));
%
% INPUTS:
%
%      Ref_Pose: MxK numeric array (K ≥ 2)
%          Columns: [X Y Z ... Yaw Station]
%          Only X,Y and optionally Yaw/Station are used. A pre-defined
%          station (column 7) is used if monotonic and finite; otherwise,
%          geometric arc length is computed.
%
%      (OPTIONAL) flag_do_debug: scalar
%          If provided as the last argument:
%             -1 disables all debugging and input checks.
%              0 disables debug prints but performs checks.
%              1 enables verbose mode with printouts.
%
% OUTPUTS:
%
%      ST_struct: struct containing segment-level geometry
%          .traj_XY           (M x 2)  original XY points
%          .ref_station       (M x 1)  cumulative station
%          .traj_start        (K x 2)  segment start points
%          .traj_end          (K x 2)  segment end points
%          .segment           (K x 2)  segment vectors
%          .segment_length    (K x 1)  Euclidean length per segment
%          .seg_tangent       (K x 2)  unit tangents
%          .seg_normal        (K x 2)  unit left normals ([-t_y, t_x])
%          .seg_start_station (K x 1)  station at segment start
%          .seg_mid           (K x 2)  segment midpoints
%          .edges             (K+1 x 1) station edges for discretization
%          .d_seg             (K x 1)  station spacing
%          .seg_tree          KDTreeSearcher of segment midpoints
%
% DEPENDENCIES:
%
%      Statistics and Machine Learning Toolbox (KDTreeSearcher)
%
% EXAMPLES:
%
%      See script: script_test_fcn_ExtractCL_convertRefTrajToST
%
% This function was written on 2025_08_15 by X. Cao
% Questions or comments? xfc5113@psu.edu
%
% Revision history:
%
%      2025_10_28 - xfc5113@psu.edu
%      -- reformatted comments and structure per repository template
%      -- added input validation and debug flag handling
%      
%      2025_11_15 - xfc5113@psu.edu
%      -- renamed the function from fcn_ExtractCL_buildSegmentsFromRefPose 
%           to fcn_ExtractCL_convertRefTrajToST
%      -- renamed the output namd from Seg to ST_struct
%
% TO DO
%      - Add option to compute normals using smoothed heading yaw
%      - Add visualization hooks for debugging

%% Debug and input-check flags
flag_do_debug = 0;
if nargin >= 2
    flag_do_debug = varargin{end};
end

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
% See: http://patorjk.com/software/taag/#p=display&f=Big&t=Inputs
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
assert(isnumeric(Ref_Pose) && size(Ref_Pose,2) >= 2, ...
    'fcn_ExtractCL_convertRefTrajToST:BadInput',...
    'Ref_Pose must be a numeric Mx2+ array.');
assert(size(Ref_Pose,1) >= 2, ...
    'fcn_ExtractCL_convertRefTrajToST:TooFewSamples',...
    'Ref_Pose must contain at least 2 points.');

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

% Extract XY trajectory
traj_XY = Ref_Pose(:,1:2);

% Use predefined station if available and monotonic, otherwise compute
use_pre_defined_station = size(Ref_Pose,2) >= 7 ...
    && all(isfinite(Ref_Pose(:,7))) ...
    && all(diff(Ref_Pose(:,7)) >= -1e-9);

if use_pre_defined_station
    ref_station = Ref_Pose(:,7);
else
    ds = [0; sqrt(sum(diff(traj_XY).^2, 2))];
    ref_station = cumsum(ds);
end

% Compute raw segments
traj_start       = traj_XY(1:end-1,:);
traj_end         = traj_XY(2:end,:);
segment          = traj_end - traj_start;
segment_length   = sqrt(sum(segment.^2,2));

% Remove zero-length segments
tf_valid         = segment_length > 1e-6;
traj_start       = traj_start(tf_valid,:);
traj_end         = traj_end(tf_valid,:);
segment          = segment(tf_valid,:);
segment_length   = segment_length(tf_valid);

% Tangent and left-normal vectors
seg_tangent      = segment ./ segment_length;
seg_normal       = [-seg_tangent(:,2), seg_tangent(:,1)];

% Segment start stations (aligned with valid indices)
seg_start_station = ref_station(1:end-1);
seg_start_station = seg_start_station(tf_valid);

% Midpoints, station edges, and spacing
seg_mid = 0.5*(traj_start + traj_end);
edges   = [seg_start_station; ref_station(end)];
d_seg   = diff(edges);

% Build KD-tree on midpoints (optional dependency)
try
    seg_tree = KDTreeSearcher(seg_mid);
catch
    seg_tree = [];
end

%% Pack results
ST_struct = struct( ...
    'traj_XY',          traj_XY, ...
    'ref_station',      ref_station, ...
    'traj_start',       traj_start, ...
    'traj_end',         traj_end, ...
    'segment',          segment, ...
    'segment_length',   segment_length, ...
    'seg_tangent',      seg_tangent, ...
    'seg_normal',       seg_normal, ...
    'seg_start_station',seg_start_station, ...
    'seg_mid',          seg_mid, ...
    'edges',            edges, ...
    'd_seg',            d_seg, ...
    'seg_tree',         seg_tree );

if flag_do_debug
    fprintf(1,'ENDING function: %s, in file: %s\n',st(1).name,st(1).file);
end

end % Ends main function
