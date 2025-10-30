function XY_extrema = fcn_ExtractCL_projectPC_STToENU(S_extrema, T_extrema, Seg)
% fcn_ExtractCL_projectPC_STToENU
% Projects lane-peak points from (S,T) back to ENU coordinates using segment-wise geometry.
%
% FORMAT:
%   XY_extrema = fcn_ExtractCL_projectPC_STToENU(S_extrema, T_extrema, Seg)
%
% INPUTS:
%   S_extrema : (N x 1) array
%       Station positions of detected peaks in ST frame.
%
%   T_extrema : (N x 1) array
%       Lateral offsets of detected peaks in ST frame.
%
%   Seg : struct
%       Segment-wise geometry data including:
%           - ref_station       : [M x 1] full reference station sequence
%           - traj_start        : [M x 2] segment start points [x y]
%           - traj_end          : [M x 2] segment end points [x y]
%           - segment           : [M x 2] segment vector (end - start)
%           - segment_length    : [M x 1] segment lengths
%           - seg_tangent       : [M x 2] unit tangent vectors
%           - seg_normal        : [M x 2] unit left normals
%           - seg_start_station : [M x 1] station value at each segment start
%           - d_seg             : [M x 1] delta S (segment spacing)
%           - seg_tree          : KD-tree object (optional)
%
% OUTPUT:
%   XY_extrema : (N x 2) array
%       Projected ENU coordinates [x y] corresponding to (S_extrema, T_extrema)
%
% METHOD:
%   For each S_i, find segment j where S_ref(j) <= S_i <= S_ref(j+1),
%   interpolate along segment via alpha = (S_i - S_ref(j)) / (S_ref(j+1)-S_ref(j)),
%   then offset laterally by T_i * n_j (unit left normal).
%
% This function was written on 2025_08_11 by X. Cao
% Questions or comments? xfc5113@psu.edu
% Revision history:
%      2025_08_11 - xfc5113@psu.edu
%     -- wrote the code originally
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
% Validate inputs and structure fields
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if nargin < 3
    error('fcn_ExtractCL_projectPC_STToENU requires 3 inputs: S_extrema, T_extrema, Seg.');
end

% Check vector dimensions
assert(isnumeric(S_extrema) && iscolumn(S_extrema), 'S_extrema must be a numeric column vector.');
assert(isnumeric(T_extrema) && iscolumn(T_extrema), 'T_extrema must be a numeric column vector.');
assert(isstruct(Seg), 'Seg must be a struct containing geometric segment fields.');

% Basic field existence validation (do not alter contents)
required_fields = {'ref_station','traj_start','traj_end','segment','segment_length', ...
                   'seg_tangent','seg_normal','seg_start_station','d_seg'};
for k = 1:length(required_fields)
    assert(isfield(Seg, required_fields{k}), ...
        'Seg struct missing required field: %s', required_fields{k});
end

% Sanity check for consistent sizes
assert(size(Seg.traj_start,2)==2 && size(Seg.traj_end,2)==2, ...
    'traj_start and traj_end must be N×2 arrays.');
assert(length(Seg.seg_start_station)==length(Seg.d_seg), ...
    'seg_start_station and d_seg must have same length.');

% Handle NaN or empty cases gracefully
if isempty(S_extrema) || isempty(T_extrema)
    XY_extrema = [];
    return;
end

%% Main Code
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   __  __       _
%  |  \/  |     (_)
%  | \  / | __ _ _ _ __
%  | |\/| |/ _` | | '_ \
%  | |  | | (_| | | | | |
%  |_|  |_|\__,_|_|_| |_|
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 1. Determine which segment each S_extrema belongs to.
% 2. Interpolate along the segment (beta = normalized offset).
% 3. Project back to ENU via T offset along unit normal.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

ref_station          = Seg.ref_station;
traj_start           = Seg.traj_start;
segment              = Seg.segment;
seg_normal           = Seg.seg_normal;
seg_start_station    = Seg.seg_start_station;
d_seg                = Seg.d_seg;
station_edges        = [seg_start_station; ref_station(end)];
num_segments         = length(d_seg);

% Map each S_extrema to its segment index
j_station = discretize(S_extrema, station_edges);  % find which segment S belongs to

% Handle edge cases (beyond first/last segment)
j_station(isnan(j_station) & S_extrema >= station_edges(end)) = num_segments;
j_station(isnan(j_station) & S_extrema <= station_edges(1))   = 1;

% Clamp indices to valid range
j_station = max(1, min(num_segments, j_station));

% Compute normalized longitudinal interpolation within each segment
beta = (S_extrema - seg_start_station(j_station)) ./ max(d_seg(j_station), eps);
beta = max(0, min(1, beta));  % clamp to [0,1]

% Compute projection points along segment centerline
%   P_on_seg = start + beta * segment_vector
XY_projection_on_segs = traj_start(j_station,:) + beta .* (segment(j_station,:));

% Add lateral offset using segment normal
XY_extrema = XY_projection_on_segs + T_extrema .* seg_normal(j_station,:);

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
% To visualize the projection, enable this section in debugging mode.
% Example:
%   figure(1001); hold on; axis equal;
%   plot(Seg.traj_start(:,1), Seg.traj_start(:,2), 'k-', 'LineWidth',2);
%   scatter(XY_extrema(:,1), XY_extrema(:,2), 20, 'r', 'filled');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

end
