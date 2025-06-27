function [pointCloud_ST_cell, ref_station] = fcn_ExtractCL_projectPC_ENUToST(PointCloud_ENU_cell, Ref_Pose, varargin)
% fcn_ExtractCL_projectPointCloud2ST
% Projects ENU LiDAR point clouds into (s, t) space with respect to a given reference trajectory.
%
% FORMAT:
%   [pointCloud_ST_cell, ref_station] = fcn_ExtractCL_projectPointCloud2ST(...
%       PointCloud_ENU_cell, Ref_Pose, (fig_num))
%
% INPUTS:
%   PointCloud_ENU_cell: Nx1 cell array
%     Each cell contains a LiDAR scan in ENU coordinates, formatted as [X Y Z Intensity TimeOffset Ring ...].
%
%   Ref_Pose: Mx7 array
%     Reference trajectory, with each row as [X, Y, Z, ..., Yaw, Station].
%     Only [X, Y, Z, Yaw] are used in this function.
%
%   (OPTIONAL) fig_num: scalar
%     If -1, disables debugging and input checking.
%
% OUTPUTS:
%   pointCloud_ST_cell: Lx1 cell
%     Each cell contains an array of Mx1 cells, one per frame, where each frame is a Nx10 array:
%     [X, Y, Z, ..., s, t]
%
%   ref_station: Mx1 vector
%     Cumulative arc-length station values along the reference trajectory.
%
% DEPENDENCIES:
%   Requires Statistics and Machine Learning Toolbox (KDTreeSearcher).
%
% Author:
%   Xinyu Cao, 2025-06-23

%% Debugging and Input checks
flag_max_speed = 0;
if nargin == 4 && isequal(varargin{end}, -1)
    flag_do_debug = 0;
    flag_check_inputs = 0;
    flag_max_speed = 1;
else
    flag_do_debug = 0;
    flag_check_inputs = 1;
    MATLABFLAG_CL_FLAG_DO_DEBUG = getenv("MATLABFLAG_CL_FLAG_DO_DEBUG");
    MATLABFLAG_CL_FLAG_CHECK_INPUTS = getenv("MATLABFLAG_CL_FLAG_CHECK_INPUTS");
    if ~isempty(MATLABFLAG_CL_FLAG_DO_DEBUG)
        flag_do_debug = str2double(MATLABFLAG_CL_FLAG_DO_DEBUG);
    end
    if ~isempty(MATLABFLAG_CL_FLAG_CHECK_INPUTS)
        flag_check_inputs = str2double(MATLABFLAG_CL_FLAG_CHECK_INPUTS);
    end
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

%% Input check
if flag_check_inputs
    assert(iscell(PointCloud_ENU_cell), 'PointCloud_ENU_cell must be a cell array');
    assert(isnumeric(Ref_Pose) && size(Ref_Pose,2) >= 6, 'Ref_Pose must be Mx6 or Mx7');
end
fig_num = -1;
if nargin >= 3
    fig_num = varargin{1};
end
flag_do_plot = 0;
if fig_num > 1
    flag_do_plot = 1;
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

traj_XY = Ref_Pose(:,1:2);
yaw_array = Ref_Pose(:,6);
ds = sqrt(sum(diff(traj_XY).^2, 2));
base_ref_station = [0; cumsum(ds)];
ref_tree = KDTreeSearcher(traj_XY);
S_offset = 0;
N_LiDAR_frames = length(PointCloud_ENU_cell);
ref_station = base_ref_station + S_offset;
s_total_length = max(ref_station);
pointCloud_ST_cell = cell(N_LiDAR_frames,1);

all_s = [];
all_t = [];
all_intensity = [];
for ith_frame = 1:N_LiDAR_frames
    PointCloud_ENU_array = PointCloud_ENU_cell{ith_frame};
    if isempty(PointCloud_ENU_array)
        pointCloud_ST_cell{ith_frame} = [];
        continue;
    end

    points_xyz = PointCloud_ENU_array(:,1:3);
    points_xy = points_xyz(:,1:2);
    points_intensity = PointCloud_ENU_array(:,4);
    idx_nearest = knnsearch(ref_tree, points_xy);
    % ref_pose_index = mode(idx_nearest);

    ref_xy = traj_XY(idx_nearest,:);
    ref_yaw = yaw_array(idx_nearest);
    ref_station_point = ref_station(idx_nearest);

    relative_x = points_xy(:,1) - ref_xy(:,1);
    relative_y = points_xy(:,2) - ref_xy(:,2);

    s_array = ref_station_point + relative_x .* cos(ref_yaw) + relative_y .* sin(ref_yaw);
    t_array = -relative_x .* sin(ref_yaw) + relative_y .* cos(ref_yaw);
    
 
    
    % station_index_array = ref_pose_index * ones(size(s_array,1),1);

    pointCloud_St_array = [PointCloud_ENU_array, s_array, t_array];
    pointCloud_ST_cell{ith_frame} = pointCloud_St_array;

    if flag_do_plot
      
        all_s = [all_s; mod(s_array,s_total_length)];
        all_t = [all_t; t_array];
        all_intensity = [all_intensity; points_intensity];
    end

end
% S_offset = S_offset + ref_station(end); % Optional if cumulative stationing is needed


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

if flag_do_debug
    fprintf(1,'ENDING function: %s\n', st(1).name);
end

if flag_do_plot
    figure(fig_num); 
    clf;
    scatter(all_s, all_t, 20, all_intensity);
    axis equal;
    grid on;
    xlabel('S [m]');
    ylabel('T [m]');
    title('LiDAR point cloud projected into ST frame');
end

end
