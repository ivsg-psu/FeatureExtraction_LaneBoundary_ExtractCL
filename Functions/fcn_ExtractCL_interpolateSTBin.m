function [S_interp, T_interp, I_interp, Z_interp] = ...
    fcn_ExtractCL_interpolateSTBin(s_bin, t_bin, intensity_bin, z_bin, ...
                                    s_low, s_res, s_width, t_res)
% fcn_ExtractCL_interpolateSTBin
%
% Interpolates LiDAR intensity and height values in a local S-bin onto a
% uniform (S, T) grid for pattern-based lane detection. This is a core step
% for organizing raw point cloud data into structured form for downstream
% processing (e.g., extrema filtering, pattern matching).
%
% FORMAT:
%   [S_interp, T_interp, I_interp, Z_interp] = ...
%       fcn_ExtractCL_interpolateSTBin(s_bin, t_bin, intensity_bin, z_bin, ...
%                                      s_low, s_res, s_width, t_res)
%
% INPUTS:
%   s_bin: Nx1 vector
%       Longitudinal coordinates (S) of LiDAR points in current bin
%
%   t_bin: Nx1 vector
%       Lateral coordinates (T) of LiDAR points in current bin
%
%   intensity_bin: Nx1 vector
%       Intensity values corresponding to (s_bin, t_bin)
%
%   z_bin: Nx1 vector
%       Elevation values corresponding to (s_bin, t_bin)
%
%   s_low: scalar
%       Starting S-position of the current bin
%
%   s_res: scalar
%       Resolution in longitudinal S-direction (e.g., 0.05 m)
%
%   s_width: scalar
%       Width of the current S-bin (e.g., 0.5 m)
%
%   t_res: scalar
%       Resolution in lateral T-direction (e.g., 0.01 m)
%
% OUTPUTS:
%   S_interp: MxK matrix
%       Interpolated S-coordinate grid (in meters)
%
%   T_interp: MxK matrix
%       Interpolated T-coordinate grid (in meters)
%
%   I_interp: MxK matrix
%       Interpolated intensity values on the (S, T) grid
%
%   Z_interp: MxK matrix
%       Interpolated elevation values on the (S, T) grid

%% Define T grid
t_min = floor(min(t_bin) / t_res) * t_res;
t_max = ceil(max(t_bin) / t_res) * t_res;
t_strip_edges = t_min:t_res:(t_max - t_res);

%% Define S grid based on density and adaptive range
s_range = max(s_bin) - min(s_bin);
s_grid_edge_low = 0;
s_grid_edge_high = s_width;

% Adaptive shrinking of bin if data doesn't fill it
if s_range < 0.8 * s_width
    s_low_new = min(s_bin);
    s_high_new = max(s_bin);
    s_grid_edge_low = floor((s_low_new - s_low) / s_res) * s_res;
    s_grid_edge_high = ceil((s_high_new - s_low) / s_res) * s_res;
end

s_strip_edges = s_grid_edge_low:s_res:s_grid_edge_high;
s_strip_centers = s_strip_edges(1:end-1) + s_res/2;

%% Create (S, T) grid
[S_base, T_base] = meshgrid(s_strip_centers, t_strip_edges);
S_interp = S_base + s_low;  % Recover absolute S
T_interp = T_base;

%% Interpolate intensity
F_I = scatteredInterpolant(s_bin, t_bin, intensity_bin, 'nearest', 'nearest');
I_interp = F_I(S_interp, T_interp);
I_interp = fillmissing(I_interp, 'linear');

%% Interpolate elevation
F_Z = scatteredInterpolant(s_bin, t_bin, z_bin, 'nearest', 'nearest');
Z_interp = F_Z(S_interp, T_interp);
Z_interp = fillmissing(Z_interp, 'linear');

end
