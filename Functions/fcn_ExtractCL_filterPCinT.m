function pointCloud_ST_filtered_cell = fcn_ExtractCL_filterPCinT(pointCloud_ST_cell, T_range)
% fcn_ExtractCL_filterPCinT
% Filters per-frame LiDAR point clouds in (s, t) space by a lateral-offset
% window T_range. Keeps only rows whose t-value (column 10) lies within
% the inclusive bounds [T_range(1), T_range(2)].
%
% FORMAT:
%
%      pointCloud_ST_filtered_cell = fcn_ExtractCL_filterPCinT(...
%           pointCloud_ST_cell,...
%           T_range);
%
% INPUTS:
%
%      pointCloud_ST_cell: Lx1 cell
%          Each cell holds a frame array with at least 10 columns where
%          column 10 is t (lateral offset). Typical layout:
%          [X Y Z Intensity ... s t] with t at column 10.
%
%      T_range: 1x2 numeric vector
%          [T_min, T_max] inclusive filter bounds (units of meters).
%          If T_min > T_max, the pair is auto-sorted.
%
% OUTPUTS:
%
%      pointCloud_ST_filtered_cell: Lx1 cell
%          Same structure as input, with each frame filtered to retain only
%          points whose t lies in the given range.
%
% DEPENDENCIES:
%
%      None
%
% EXAMPLES:
%
%      See script: script_test_fcn_ExtractCL_filterPCinT
%
% This function was written on 2025_10_28 by X. Cao
% Questions or comments? xfc5113@psu.edu
%
% Revision history:
%
%      2025_10_28 - xfc5113@psu.edu
%      -- initial documentation pass, added input checks and edge handling
%
% TO DO
%      - Expose t-column index as a Name-Value option if layout changes
%      - Add optional NaN-removal prior to filtering

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
if nargin ~= 2
    error('fcn_ExtractCL_filterPCinT:IncorrectNargin',...
        'Expected 2 inputs: pointCloud_ST_cell, T_range.');
end

% Check cell container
if ~iscell(pointCloud_ST_cell)
    error('fcn_ExtractCL_filterPCinT:BadCell',...
        'pointCloud_ST_cell must be a cell array of per-frame arrays.');
end

% Check T_range
if ~(isnumeric(T_range) && isvector(T_range) && numel(T_range) == 2 && all(isfinite(T_range)))
    error('fcn_ExtractCL_filterPCinT:BadTRange',...
        'T_range must be a finite numeric 1x2 vector [T_min T_max].');
end

% Ensure increasing order
if T_range(1) > T_range(2)
    T_range = sort(T_range);
end

% Light per-frame validation (layout requires at least 10 columns)
for k = 1:numel(pointCloud_ST_cell)
    A = pointCloud_ST_cell{k};
    if isempty(A), continue; end
    if ~(isnumeric(A) && ismatrix(A) && size(A,2) >= 10)
        error('fcn_ExtractCL_filterPCinT:BadFrameLayout',...
            'Each non-empty frame must be numeric Nx10+ with t at column 10.');
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
N_frames = numel(pointCloud_ST_cell);
pointCloud_ST_filtered_cell = cell(N_frames,1);

for k_frame = 1:N_frames
    pointCloud_ST_array = pointCloud_ST_cell{k_frame};

    if isempty(pointCloud_ST_array)
        pointCloud_ST_filtered_cell{k_frame} = [];
        continue;
    end

    % Column 10 is T (lateral offset)
    T_array  = pointCloud_ST_array(:,10);

    % Inclusive bounds; keep NaNs out
    inTRange = isfinite(T_array) & (T_array >= T_range(1)) & (T_array <= T_range(2));

    % Filter and store
    pointCloud_ST_filtered_cell{k_frame} = pointCloud_ST_array(inTRange,:);
end

end % Ends main function
