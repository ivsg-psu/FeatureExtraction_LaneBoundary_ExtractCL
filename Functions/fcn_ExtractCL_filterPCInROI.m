function LiDAR_Data_Output = fcn_ExtractCL_filterPointCloudInROI(LiDAR_Data, ROI, varargin)
% fcn_ExtractCL_filterPointCloudInROI
% Filters LiDAR point clouds based on a specified Region of Interest (ROI)
% in either Cartesian (X/Y/Z limits) or Polar (R/theta/Z limits) coordinate.
%
% FORMAT:
%   LiDAR_Data_Output = fcn_ExtractCL_filterPointCloudInROI(LiDAR_Data, ROI, (fig_num))
%
% INPUTS:
%   LiDAR_Data - struct with field 'PointCloud' (Nx1 cell array) or a raw cell array
%   ROI - struct with fields depending on ROI.mode:
%       Cartesian Mode (default):
%           ROI.mode = 'cartesian';
%           ROI.X_lim = [xmin xmax] or Nx2 per-frame array
%           ROI.Y_lim = [ymin ymax] or Nx2 per-frame array
%           ROI.Z_lim = [zmin zmax] or Nx2 per-frame array (optional)
%       Polar Mode:
%           ROI.mode = 'polar';
%           ROI.R_lim = [rmin rmax];
%           ROI.Theta_lim = [theta_min theta_max]; % in degrees
%           ROI.Z_lim = [zmin zmax]; % optional
%
% OPTIONAL INPUTS:
%   fig_num - figure number for debugging. If set to -1, disables checks and plotting.
%
% OUTPUT:
%   LiDAR_Data_Output - same type as input but with filtered point clouds
%
% DEPENDENCIES:
%   None
%
% Author: Xinyu Cao, 2025-06-23

%% Debugging and Input checks
flag_max_speed = 0;
if (nargin == 3 && isequal(varargin{end}, -1))
    flag_do_debug = 0;
    flag_check_inputs = 0;
    flag_max_speed = 1;
else
    % Check to see if we are externally setting debug mode to be "on"
    flag_do_debug = 0;
    flag_check_inputs = 1;
    MATLABFLAG_EXTRACTCL_FLAG_DO_DEBUG = getenv("MATLABFLAG_EXTRACTCL_FLAG_DO_DEBUG");
    MATLABFLAG_EXTRACTCL_FLAG_CHECK_INPUTS = getenv("MATLABFLAG_EXTRACTCL_FLAG_CHECK_INPUTS");
    if ~isempty(MATLABFLAG_EXTRACTCL_FLAG_DO_DEBUG)
        flag_do_debug = str2double(MATLABFLAG_EXTRACTCL_FLAG_DO_DEBUG);
    end
    if ~isempty(MATLABFLAG_EXTRACTCL_FLAG_CHECK_INPUTS)
        flag_check_inputs = str2double(MATLABFLAG_EXTRACTCL_FLAG_CHECK_INPUTS);
    end
end

if flag_do_debug
    st = dbstack;
    fprintf(1,'STARTING function: %s, in file: %s\n',st(1).name,st(1).file);
    debug_fig_num = 999978; %#ok<NASGU>
else
    debug_fig_num = []; %#ok<NASGU>
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


%% Input check
if flag_check_inputs
    narginchk(2, 3);
    assert(isstruct(ROI), 'ROI must be a struct.');
    if ~isfield(ROI, 'mode')
        ROI.mode = 'cartesian';
    end
    assert(isstruct(LiDAR_Data) || iscell(LiDAR_Data), 'LiDAR_Data must be a struct or cell array.');
end

%% Parse point cloud
if isstruct(LiDAR_Data)
    LiDAR_PointCloud_Cell = LiDAR_Data.PointCloud;
elseif iscell(LiDAR_Data)
    LiDAR_PointCloud_Cell = LiDAR_Data;
end

mode_type = lower(ROI.mode);
N_scans = length(LiDAR_PointCloud_Cell);
filtered_PointCloud_cell = cell(N_scans,1);

%% Filter each scan
for ith_scan = 1:N_scans
    scan = LiDAR_PointCloud_Cell{ith_scan};
    if isempty(scan)
        filtered_PointCloud_cell{ith_scan} = [];
        continue;
    end

    xyz = scan(:,1:3);
    keep_idx = true(size(scan,1),1);

    switch mode_type
        case 'cartesian'
            X_lim = ROI.X_lim;
            Y_lim = ROI.Y_lim;
            Z_lim = [-inf inf];
            if isfield(ROI,'Z_lim')
                Z_lim = ROI.Z_lim(min(end,ith_scan),:);
            end

            keep_idx = keep_idx & xyz(:,1) >= X_lim(1) & xyz(:,1) <= X_lim(2);
            keep_idx = keep_idx & xyz(:,2) >= Y_lim(1) & xyz(:,2) <= Y_lim(2);
            keep_idx = keep_idx & xyz(:,3) >= Z_lim(1) & xyz(:,3) <= Z_lim(2);

        case 'polar'
            R = hypot(xyz(:,1), xyz(:,2));
            Theta = atan2d(xyz(:,2), xyz(:,1));
            R_lim = ROI.R_lim;
            Theta_lim = ROI.Theta_lim;
            Z_lim = [-inf inf];
            if isfield(ROI,'Z_lim')
                Z_lim = ROI.Z_lim(min(end,ith_scan),:);
            end

            keep_idx = keep_idx & R >= R_lim(1) & R <= R_lim(2);
            keep_idx = keep_idx & Theta >= Theta_lim(1) & Theta <= Theta_lim(2);
            keep_idx = keep_idx & xyz(:,3) >= Z_lim(1) & xyz(:,3) <= Z_lim(2);

        otherwise
            error('Unsupported ROI.mode: %s', ROI.mode);
    end

    filtered_PointCloud_cell{ith_scan} = scan(keep_idx,:);
end

%% Return in original format
if isstruct(LiDAR_Data)
    LiDAR_Data_Output = LiDAR_Data;
    LiDAR_Data_Output.PointCloud = filtered_PointCloud_cell;
else
    LiDAR_Data_Output = filtered_PointCloud_cell;
end



if flag_do_debug
    fprintf(1,'ENDING function: %s\n', st(1).name);
end

end
