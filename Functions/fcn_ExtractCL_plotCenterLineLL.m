function fcn_ExtractCL_plotCenterLineLL(center_line_points, ref_base_station, plotFormat, fig_num)
% fcn_ExtractCL_plotCenterLineLL
% Projects extracted ENU center line points into LLA coordinates and plots them on a satellite map.
%
% FORMAT:
%   fcn_ExtractCL_plotCenterLineLL(center_line_points, ref_base_station, plotFormat, fig_num)
%
% INPUTS:
%   center_line_points : Nx3 or Nx5 array
%       [X, Y, Z, ...] coordinates in ENU frame
%
%   ref_base_station : 1x3 vector
%       Reference LLA position [lat, lon, alt] used for ENU to LLA conversion
%
%   plotFormat : struct or string
%       Plotting style passed to fcn_plotRoad_plotLL
%
%   fig_num : integer
%       Figure number to use for plotting
%
% OUTPUTS:
%   None (generates a geospatial plot with geoplot and geoscatter)
%
% Author: Xinyu Cao, 2025-06-26

% Extract ENU XYZ points
XYZ_center_line = center_line_points(:, 1:3);

% Convert to LLA (latitude, longitude, altitude)
LLA_center_line = enu2lla(XYZ_center_line, ref_base_station, "ellipsoid");

% Extract only latitude and longitude
LL_center_line = LLA_center_line(:, 1:2);

% Plot center line
figure(fig_num); 
clf;
h_geoplot = fcn_plotRoad_plotLL(LL_center_line, plotFormat, fig_num);
hold on;

% Plot reference base station
geoscatter(ref_base_station(1), ref_base_station(2), 100, 'green', 'filled');

% Add title and legend
% title('Extracted Lane Centerline (LLA Coordinates)', 'FontSize', 22);
legend('Extracted line', 'Reference base station', 'FontSize', 18);

end
