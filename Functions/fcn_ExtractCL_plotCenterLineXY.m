function fcn_ExtractCL_plotCenterLineXY(center_line_points, pointcloud_array, plotFormat, fig_num)
% fcn_ExtractCL_plotCenterLineXY
% Plots extracted center line and raw point cloud in the ENU XY plane.
%
% FORMAT:
%   fcn_ExtractCL_plotCenterLineXY(center_line_points, pointcloud_array, plotFormat, fig_num)
%
% INPUTS:
%   center_line_points : Nx3 or Nx5 array
%       [X Y Z ...] center line points in ENU coordinates
%
%   pointcloud_array : Mx4 array
%       [X Y Z Intensity] raw point cloud in ENU coordinates
%
%   plotFormat : string or struct
%       Plot format passed to fcn_plotRoad_plotXY
%
%   fig_num : integer
%       Figure number for plotting
%
% OUTPUT:
%   None (generates XY plot)
%
% Author: Xinyu Cao, 2025-06-26

% Prepare center line data
XY_center_line = center_line_points(:, 1:2);

% Prepare point cloud data
XY_pointcloud = pointcloud_array(:, 1:2);
intensity_pointcloud = pointcloud_array(:, 4);

% Plot
figure(fig_num); clf;
scatter(XY_pointcloud(:,1), XY_pointcloud(:,2), 20, intensity_pointcloud, 'filled'); hold on;

% Plot center line on top
h_plot = fcn_plotRoad_plotXY(XY_center_line, plotFormat, fig_num);

% Formatting
title('Extracted Lane Centerline in ENU XY Plane', 'FontSize', 20);
xlabel('X-East [m]'); ylabel('Y-North [m]');
legend('Point Cloud', 'Center Line');
axis equal
end
