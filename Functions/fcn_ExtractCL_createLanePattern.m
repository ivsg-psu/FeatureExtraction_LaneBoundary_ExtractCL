function pattern_template = fcn_ExtractCL_createLanePattern(lane_type, t_res, varargin)
% fcn_ExtractCL_createLanePattern
% Generates a 1D binary lane marker pattern template based on the specified
% lane configuration. This template can be used for pattern matching in
% LiDAR intensity strips.
%
% FORMAT:
%   pattern_template = fcn_ExtractCL_createLanePattern(lane_type, t_res, (lane_width), (marker_width), (double_marker_gap), (fig_num))
%
% INPUTS:
%   lane_type: string
%     Type of lane pattern to create. Supported types:
%       - 'left_double_yellow_right_white'
%       - 'single_strip'
%
%   t_res: scalar
%     Resolution of T-axis in meters.
%
% OPTIONAL INPUTS:
%   lane_width: scalar (default = 3.6)
%     Width of the lane.
%
%   marker_width: scalar (default = 0.10)
%     Width of each lane marker.
%
%   double_marker_gap: scalar (default = 0.12)
%     Gap between two yellow markers in a double yellow lane.
%
%   fig_num: scalar (default = -1)
%     Figure number for debugging plot. If >= 1, plots the template.
%
% OUTPUTS:
%   pattern_template: binary column vector (0/1)
%     The pattern representing lane marker configuration.
%
% DEPENDENCIES:
%   None.
%
% EXAMPLES:
%   pattern = fcn_ExtractCL_createLanePattern('left_double_yellow_right_white', 0.01);
%   pattern = fcn_ExtractCL_createLanePattern('single_strip', 0.01, 3.6, 0.1);
%
% Author:
% This function was written on 2025_06_03 by X. Cao
% Questions or comments? xfc5113@psu.edu
%
% Revision history:

% 2025_06_23 - xfc5113@psu.edu
% -- Formatted according to fcn_Laps_breakDataIntoLaps template
% -- Updated documentation, debug controls, and default parameter handling
% 2025_11_05 - xfc5113@psu.edu
% -- added new layout 'single_both_sides'



%% Debugging and Input checks
flag_max_speed = 0;
if (nargin >= 6 && isequal(varargin{end}, -1))
    flag_do_debug = 0; % Disable debug if max speed mode is requested
    flag_check_inputs = 0;
    flag_max_speed = 1;
else
    flag_do_debug = 0;
    flag_check_inputs = 1;
    MATLABFLAG_LANEDETECTION_FLAG_CHECK_INPUTS = getenv("MATLABFLAG_LANEDETECTION_FLAG_CHECK_INPUTS");
    MATLABFLAG_LANEDETECTION_FLAG_DO_DEBUG = getenv("MATLABFLAG_LANEDETECTION_FLAG_DO_DEBUG");
    if ~isempty(MATLABFLAG_LANEDETECTION_FLAG_CHECK_INPUTS) && ~isempty(MATLABFLAG_LANEDETECTION_FLAG_DO_DEBUG)
        flag_do_debug = str2double(MATLABFLAG_LANEDETECTION_FLAG_DO_DEBUG);
        flag_check_inputs  = str2double(MATLABFLAG_LANEDETECTION_FLAG_CHECK_INPUTS);
    end
end

if flag_do_debug
    st = dbstack;
    fprintf(1,'STARTING function: %s, in file: %s',st(1).name,st(1).file);
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


if (0==flag_max_speed)
    if flag_check_inputs
        % Are there the right number of inputs?
        narginchk(2, 6);

        % Check the reference_traversal variables
        % fcn_DebugTools_checkInputsToFunctions(input_traversal, 'traversal');

        % NOTE: the start_definition required input is checked below!

    end
end


lane_width = 3.6;   % road width in meters
if nargin >= 3
    if ~isempty(varargin{1})
        lane_width = varargin{1};
    end
end

marker_width = 0.10; % each lane marker ~10 cm
if nargin >= 4
    if ~isempty(varargin{2})
        marker_width = varargin{2};
    end
end

double_marker_gap = 0.12;   % gap between double yellow lines
if nargin >= 5
    if ~isempty(varargin{3})
        double_marker_gap = varargin{3};
    end
end


% Does user want to show the plots?
fig_num = -1;
if nargin >= 6
    fig_num = varargin{4};
end
flag_do_plot = 0;
if fig_num >= 1
    flag_do_plot = 1;
end




%% Main Code Starts Here
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   __  __       _
%  |  \/  |     (_)
%  | \  / | __ _ _ _ __
%  | |\/| |/ _` | | '_ \
%  | |  | | (_| | | | | |
%  |_|  |_|\__,_|_|_| |_|
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

switch lane_type
    case 'left_double_yellow_right_white'
        dist_yellow_to_white_edge = lane_width - double_marker_gap/2 - marker_width;

        white_start_idx = 2;
        white_end_idx = white_start_idx + round(marker_width/t_res);

        first_yellow_start_idx = white_end_idx + round(dist_yellow_to_white_edge/t_res);
        first_yellow_end_idx = first_yellow_start_idx + round(marker_width/t_res);

        second_yellow_start_idx = first_yellow_end_idx + round(double_marker_gap/t_res);
        second_yellow_end_idx = second_yellow_start_idx + round(marker_width/t_res);

        pattern_length = second_yellow_end_idx + 1;
        pattern_template = zeros(pattern_length, 1);

        pattern_template(white_start_idx:white_end_idx) = 1;
        pattern_template(first_yellow_start_idx:first_yellow_end_idx) = 1;
        pattern_template(second_yellow_start_idx:second_yellow_end_idx) = 1;
    case 'single_both_sides'
        start_idx_1 = 2;
        end_idx_1 = start_idx_1 + round(marker_width/t_res);
        start_idx_2 = end_idx_1 + round((lane_width - marker_width)/t_res);
        end_idx_2 = start_idx_2 + round(marker_width/t_res);
        pattern_length = end_idx_2 + 1;
        pattern_template = zeros(pattern_length, 1);
        pattern_template(start_idx_1:end_idx_1) = 1;
        pattern_template(start_idx_2:end_idx_2) = 1;
    case 'single_strip'
        start_idx = 2;
        end_idx = start_idx + round(marker_width/t_res);
        pattern_length = end_idx + 1;
        pattern_template = zeros(pattern_length, 1);
        pattern_template(start_idx:end_idx) = 1;

    otherwise
        error('Unsupported lane_type: %s', lane_type);
end

%% Optional Plotting
if flag_do_plot == 1
    figure(fig_num)
    clf
    plot(pattern_template, 'k-', 'LineWidth', 2);
    % title(sprintf('Lane Pattern Template: %s', lane_type), 'Interpreter', 'none');
    xlabel('Index'); 
    ylabel('Lane marker presence (0/1)');
    grid on;
    grid minor;
end

if flag_do_debug
    fprintf(1,'ENDING function: %s, in file: %s\n',st(1).name,st(1).file);
end

end
