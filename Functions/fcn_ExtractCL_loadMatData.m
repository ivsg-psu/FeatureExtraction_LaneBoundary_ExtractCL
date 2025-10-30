function [VehiclePose_Example, PointCloud_ENU_Example] = fcn_ExtractCL_loadMatData(varargin)
% fcn_ExtractCL_loadMatData
% -------------------------------------------------------------------------
% Purpose:
%   Load example data for lane-centerline extraction:
%     - VehiclePose_Example           (from VehiclePoseExample.mat)
%     - PointCloud_ENU_Example        (from PointCloudENUExample.mat)
%
% FORMAT:
%   [VehiclePose_Example, PointCloud_ENU_Example] = fcn_ExtractCL_loadMatData()
%   [VehiclePose_Example, PointCloud_ENU_Example] = fcn_ExtractCL_loadMatData(data_dir)
%   [VehiclePose_Example, PointCloud_ENU_Example] = fcn_ExtractCL_loadMatData(data_dir, vp_file, pc_file)
%
% INPUTS (OPTIONAL):
%   data_dir : folder containing mat files           (default: 'Data')
%   vp_file  : vehicle pose mat filename             (default: 'VehiclePoseExample.mat')
%   pc_file  : point cloud mat filename              (default: 'PointCloudENUExample.mat')
%
% OUTPUTS:
%   VehiclePose_Example    : example vehicle pose structure/array loaded from MAT
%   PointCloud_ENU_Example : example ENU point cloud array loaded from MAT
%
% DEPENDENCIES:
%   - None (uses MATLAB built-ins only)
%
% EXAMPLES:
%   % Default locations:
%   [VehiclePose_Example, PointCloud_ENU_Example] = fcn_ExtractCL_loadMatData();
%
%   % Custom folder:
%   [VehiclePose_Example, PointCloud_ENU_Example] = fcn_ExtractCL_loadMatData('D:\MappingVanData');
%
%   % Custom files:
%   [VehiclePose_Example, PointCloud_ENU_Example] = fcn_ExtractCL_loadMatData('Data', ...
%       'VehiclePoseExample.mat', 'PointCloudENUExample.mat');
%
% This function was written on 2025_10_29 by X. Cao
% Questions or comments? xfc5113@psu.edu
%
% Revision history:
%   2025_10_29 - xfc5113@psu.edu
%   -- write the code originally
%
% -------------------------------------------------------------------------

flag_check_inputs = 1;
flag_do_debug     = 0;

if flag_do_debug
    st = dbstack;
    fprintf(1,'STARTING function: %s, in file: %s\n', st(1).name, st(1).file);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   _____                   _
%  |_   _|                 | |
%    | |  _ __  _ __  _   _| |_ ___
%    | | | '_ \| '_ \| | | | __/ __|
%   _| |_| | | | |_) | |_| | |_\__ \
%  |_____|_| |_| .__/ \__,_|\__|___/
%              | |
%              |_|
% See: http://patorjk.com/software/taag/#p=display&f=Big&t=Inputs
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Input parsing / checks
% Defaults
data_dir_default = 'Data';
vp_file_default  = 'VehiclePoseExample.mat';
pc_file_default  = 'PointCloudENUExample.mat';

% Parse varargin
switch nargin
    case 0
        data_dir = data_dir_default;
        vp_file  = vp_file_default;
        pc_file  = pc_file_default;
    case 1
        data_dir = varargin{1};
        vp_file  = vp_file_default;
        pc_file  = pc_file_default;
    case 2
        data_dir = varargin{1};
        vp_file  = varargin{2};
        pc_file  = pc_file_default;
    otherwise
        data_dir = varargin{1};
        vp_file  = varargin{2};
        pc_file  = varargin{3};
end

if flag_check_inputs
    if ~(ischar(data_dir) || isstring(data_dir))
        error('data_dir must be a char or string.');
    end
    if ~(ischar(vp_file) || isstring(vp_file))
        error('vp_file must be a char or string.');
    end
    if ~(ischar(pc_file) || isstring(pc_file))
        error('pc_file must be a char or string.');
    end
end

% Build full paths
vp_path = fullfile(char(data_dir), char(vp_file));
pc_path = fullfile(char(data_dir), char(pc_file));

% Existence checks
if ~exist(vp_path, 'file')
    error('Vehicle pose MAT file not found: %s', vp_path);
end
if ~exist(pc_path, 'file')
    error('Point cloud MAT file not found: %s', pc_path);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   __  __       _
%  |  \/  |     (_)
%  | \  / | __ _ _ _ __
%  | |\/| |/ _` | | '_ \
%  | |  | | (_| | | | | |
%  |_|  |_|\__,_|_|_| |_|
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Main code
% Load vehicle pose
S_vp = load(vp_path);
if ~isfield(S_vp, 'VehiclePose_Example')
    error('Variable ''VehiclePose_Example'' not found inside %s', vp_path);
end
VehiclePose_Example = S_vp.VehiclePose_Example;

% Load point cloud
S_pc = load(pc_path);
if ~isfield(S_pc, 'PointCloud_ENU_Example')
    error('Variable ''PointCloud_ENU_Example'' not found inside %s', pc_path);
end
PointCloud_ENU_Example = S_pc.PointCloud_ENU_Example;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   _____       _
%  |  __ \     | |
%  | |  | | ___| |__  _   _  __ _
%  | |  | |/ _ \ '_ \| | | |/ _` |
%  | |__| |  __/ |_) | |_| | (_| |
%  |_____/ \___|_.__/ \__,_|\__, |
%                            __/ |
%                           |___/
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Debug (optional prints)
if flag_do_debug
    fprintf(1,'Loaded VehiclePose_Example from: %s\n', vp_path);
    fprintf(1,'Loaded PointCloud_ENU_Example from: %s\n', pc_path);
    fprintf(1,'ENDING function: %s\n', st(1).name);
end

end
