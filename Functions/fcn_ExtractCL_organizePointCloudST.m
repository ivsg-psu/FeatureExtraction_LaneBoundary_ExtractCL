function [S_interp, T_interp, I_interp, Z_interp, s_strip_centers] = ...
    fcn_ExtractCL_organizePointCloudST(s_bin, t_bin, intensity_bin, z_bin, ...
                                       s_low, s_res, s_width, t_res, t_edges, varargin)
% fcn_ExtractCL_organizePointCloudST
% Converts unstructured LiDAR points in the (s, t) frame into regularized
% grid strips ("artificial rings") via fast boxcar aggregation in the S direction.
%
% For each S-strip centered at sc, this function selects all points whose
% station s lies in [sc - wS/2, sc + wS/2], bins them along the T direction,
% and computes the per-bin mean intensity and height. No interpolation,
% convolution, or masking is used—this is a purely geometric aggregation,
% optimized for speed and simplicity.
%
% FORMAT:
%
%      [S_interp, T_interp, I_interp, Z_interp, s_strip_centers] = ...
%           fcn_ExtractCL_organizePointCloudST(...
%               s_bin, t_bin, intensity_bin, z_bin, ...
%               s_low, s_res, s_width, t_res, t_edges, ...
%               ('TMargin', val, ...));
%
% INPUTS:
%
%      s_bin, t_bin, intensity_bin, z_bin : Nx1 vectors
%          Points for the current S-section, expressed in ST coordinates.
%          - s_bin: longitudinal station values [m]
%          - t_bin: lateral offset values [m]
%          - intensity_bin: LiDAR reflectivity [unitless]
%          - z_bin: elevation [m]
%
%      s_low: scalar
%          Starting S coordinate of the current section [m].
%
%      s_res: scalar
%          Sampling interval (ring spacing) along S [m].
%
%      s_width: scalar
%          Total longitudinal span of the current section [m].
%
%      t_res: scalar
%          Lateral bin size for histogram aggregation [m].
%
%      t_edges: 1xM vector
%          Bin edges along the T-axis. (T bins are [t_edges(i), t_edges(i+1)]).
%
% NAME-VALUE PAIRS (optional):
%
%      'TMargin'          : lateral margin factor (default = 0.20)
%      'MinTRangeWidth'   : minimum T-range width (default = 1.50)
%      'WindowWidthS'     : window width in S (default = s_res)
%      'MinPointsPerStrip': minimum total points required per S-strip (default = 50)
%      'MinCoverageRatio' : minimum fraction of nonempty T bins per strip (default = 0.15)
%
% OUTPUTS:
%
%      S_interp (MxK): S coordinates for the output intensity grid
%      T_interp (MxK): T coordinates for the output intensity grid
%      I_interp (MxK): mean intensity per (S, T) cell
%      Z_interp (MxK): mean elevation per (S, T) cell
%      s_strip_centers (1xK): centers of each S-strip [m]
%
% DEPENDENCIES:
%
%      None (self-contained)
%
% EXAMPLES:
%
%      See script: script_test_fcn_ExtractCL_organizePointCloudST
%
% This function was written on 2025_08_11 by X. Cao
% Questions or comments? xfc5113@psu.edu
%
% Revision history:
%
%      2025_08_11 - xfc5113@psu.edu
%      -- initial implementation with direct boxcar aggregation
%      -- standardized comments and I/O structure
%
% TO DO
%      - Add visualization of per-strip coverage
%      - Allow adaptive WindowWidthS based on curvature or density

%% Input argument parsing
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
ip = inputParser;
ip.addParameter('TMargin',0.20);
ip.addParameter('MinTRangeWidth',1.50);
ip.addParameter('WindowWidthS', s_res);
ip.addParameter('MinPointsPerStrip', 50);
ip.addParameter('MinCoverageRatio', 0.15);
ip.parse(varargin{:});
opt = ip.Results;

%% Setup S and T grids
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   __  __       _
%  |  \/  |     (_)
%  | \  / | __ _ _ _ __
%  | |\/| |/ _` | | '_ \
%  | |  | | (_| | | | | |
%  |_|  |_|\__,_|_|_| |_|
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Number of S-strips (K)
K = max(1, floor(s_width / s_res));
s_strip_centers = (s_low + s_res/2) + (0:K-1)*s_res;
wS = opt.WindowWidthS;  % S-direction boxcar width

% Discretize T axis
M = numel(t_edges) - 1;
t_idx_all = discretize(t_bin, t_edges);  % bin index per point (1..M)

% Sort by S to enable efficient sliding window accumulation
[s_sorted, ord] = sort(s_bin, 'ascend');
t_idx_sorted = t_idx_all(ord);
ival_sorted  = intensity_bin(ord);
zval_sorted  = z_bin(ord);

% Allocate intensity/elevation outputs
I_interp = zeros(M, K);
Z_interp = zeros(M, K);



% Initialize pointers and accumulators for incremental updates
Lptr = 1; Rptr = 0; N = numel(s_sorted);
count_T = zeros(M,1);
sumI_T  = zeros(M,1);
sumZ_T  = zeros(M,1);

for k = 1:K
    % Define current window boundaries
    sc = s_strip_centers(k);
    sL = sc - wS/2;
    sR = sc + wS/2;

    % --- Expand right: include new points entering window ---
    while (Rptr < N) && (s_sorted(Rptr+1) <= sR)
        Rptr = Rptr + 1;
        b = t_idx_sorted(Rptr);
        if ~isnan(b)
            count_T(b) = count_T(b) + 1;
            sumI_T(b)  = sumI_T(b)  + ival_sorted(Rptr);
            sumZ_T(b)  = sumZ_T(b)  + zval_sorted(Rptr);
        end
    end

    % --- Shrink left: remove points leaving window ---
    while (Lptr <= N) && (s_sorted(Lptr) < sL)
        b = t_idx_sorted(Lptr);
        if ~isnan(b)
            count_T(b) = count_T(b) - 1;
            sumI_T(b)  = sumI_T(b)  - ival_sorted(Lptr);
            sumZ_T(b)  = sumZ_T(b)  - zval_sorted(Lptr);
        end
        Lptr = Lptr + 1;
    end

    % --- Skip this strip if too few points or too narrow coverage ---
    total_pts = sum(count_T);
    coverage  = nnz(count_T > 0) / M;
    if (total_pts < opt.MinPointsPerStrip) || (coverage < opt.MinCoverageRatio)
        I_interp(:,k) = NaN;
        Z_interp(:,k) = NaN;
        continue;
    end

    % --- Compute mean intensity and height per T-bin ---
    nonempty_bin = (count_T > 0);
    col_I = nan(M,1);
    col_Z = nan(M,1);
    col_I(nonempty_bin) = sumI_T(nonempty_bin) ./ count_T(nonempty_bin);
    col_Z(nonempty_bin) = sumZ_T(nonempty_bin) ./ count_T(nonempty_bin);

    I_interp(:,k) = col_I;
    Z_interp(:,k) = col_Z;
end


% Remove NaN columns (invalid strips)
valid_cols = any(~isnan(I_interp), 1);
I_interp = I_interp(:, valid_cols);
Z_interp = Z_interp(:, valid_cols);
s_strip_centers = s_strip_centers(valid_cols);

% Generate coordinate grids (MxK), same orientation as intensity
t_centers = t_edges(1:end-1) + t_res/2;
[S_interp, T_interp] = ndgrid(s_strip_centers, t_centers);
S_interp = S_interp.';  % MxK (lateral-major)
T_interp = T_interp.';

% Fill small gaps by linear interpolation along S
I_interp = fillmissing(I_interp, 'linear', 1, 'EndValues', 'nearest');
Z_interp = fillmissing(Z_interp, 'linear', 1, 'EndValues', 'nearest');

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

end
