function [XYZSTE_Center_Line_Array_clean, XYZSTE_Center_Line_Array_outliers] = fcn_ExtractCL_cleanCLPoints(XYZSTE_Center_Line_Array)
%
% fcn_ExtractCL_cleanCLPoints
% Cleans center line points by removing outliers based on lateral T values
% and ensures sorting along longitudinal station S.
%
% FORMAT:
%   [XYZSTE_Center_Line_Array_clean, XYZSTE_Center_Line_Array_outliers] = fcn_ExtractCL_cleanCLPoints(...
%       XYZSTE_Center_Line_Array)
%
% INPUTS:
%   XYZSTE_Center_Line_Array: Nx6 array
%       Center line points formatted as [X Y Z S T E], where:
%         - S: longitudinal station
%         - T: lateral offset
%         - E: pattern match error or confidence
%
% OUTPUTS:
%   XYZSTE_Center_Line_Array_clean: Mx6 array
%       Cleaned center line points with outliers removed.
%
%   XYZSTE_Center_Line_Array_outliers: Kx6 array
%       Points removed as outliers.
%
% Author:
%   Xinyu Cao, 2025-06-25

% Sort by station value (S - 4th column)
[~, idx_sorted] = sort(XYZSTE_Center_Line_Array(:,4));
XYZSTE_Center_Line_Array_sorted = XYZSTE_Center_Line_Array(idx_sorted,:);

% Extract T (5th column)
T_val = XYZSTE_Center_Line_Array_sorted(:,5);

% Apply median filtering to estimate expected T trend
window_size = 10;
T_median = medfilt1(T_val, window_size, 'omitnan', 'truncate');

% Outlier condition 1: T should be negative (i.e., lane marker is on left)
tf_T_is_negative = T_val <= 0;

% Outlier condition 2: deviation from trend must be small
T_deviation = abs(T_val - T_median);
MAD = median(T_deviation, 'omitnan');
k = 3;
outlier_thresh = k * MAD;
tf_T_deviation_is_small = T_deviation < outlier_thresh;

% Combine conditions
valid_idx = tf_T_is_negative & tf_T_deviation_is_small;

% Separate valid and outlier points
XYZSTE_Center_Line_Array_clean = XYZSTE_Center_Line_Array_sorted(valid_idx,:);
XYZSTE_Center_Line_Array_outliers = XYZSTE_Center_Line_Array_sorted(~valid_idx,:);

end
