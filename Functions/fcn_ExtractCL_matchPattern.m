function [best_fit_pattern, best_fit_idx, best_fit_error] = fcn_ExtractCL_matchPattern(intensity_data, pattern_template, shift_range)
% fcn_CL_matchPattern
% Matches a given pattern to intensity data by sliding window method.
% Returns the best-matching index and error.
%
% FORMAT:
%   [best_fit_pattern, best_fit_idx, best_fit_error] = fcn_CL_matchPattern( ...
%       intensity_data, pattern_template, (shift_range))
%
% INPUTS:
%   intensity_data: Nx1 vector
%       Input signal (e.g., filtered intensity or extrema correlation)
%
%   pattern_template: Mx1 vector
%       Reference pattern to match (must be M <= N)
%
%   shift_range: optional, scalar or vector (unused in current version)
%       Reserved for future use (e.g., limited search range)
%
% OUTPUTS:
%   best_fit_pattern: Nx1 vector
%       Best aligned pattern padded to length of input
%
%   best_fit_idx: scalar
%       Index in intensity_data where pattern starts (1-based)
%
%   best_fit_error: scalar
%       Sum of squared errors of best match

%% Debugging and Input checks

% Check if flag_max_speed set. This occurs if the fig_num variable input
% argument (varargin) is given a number of -1, which is not a valid figure
% number.
flag_max_speed = 0;
if (nargin==5 && isequal(varargin{end},-1))
    flag_do_debug = 0; % % % % Flag to plot the results for debugging
    flag_check_inputs = 0; % Flag to perform input checking
    flag_max_speed = 1;
else
    % Check to see if we are externally setting debug mode to be "on"
    flag_do_debug = 0; % % % % Flag to plot the results for debugging
    flag_check_inputs = 1; % Flag to perform input checking
    MATLABFLAG_LAPS_FLAG_CHECK_INPUTS = getenv("MATLABFLAG_LAPS_FLAG_CHECK_INPUTS");
    MATLABFLAG_LAPS_FLAG_DO_DEBUG = getenv("MATLABFLAG_LAPS_FLAG_DO_DEBUG");
    if ~isempty(MATLABFLAG_LAPS_FLAG_CHECK_INPUTS) && ~isempty(MATLABFLAG_LAPS_FLAG_DO_DEBUG)
        flag_do_debug = str2double(MATLABFLAG_LAPS_FLAG_DO_DEBUG);
        flag_check_inputs  = str2double(MATLABFLAG_LAPS_FLAG_CHECK_INPUTS);
    end
end

% flag_do_debug = 1;

if flag_do_debug
    st = dbstack; %#ok<*UNRCH>
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

% Ensure input is column
if isrow(intensity_data)
    intensity_data = intensity_data';
end
if isrow(pattern_template)
    pattern_template = pattern_template';
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

NpointsData = size(intensity_data,1);
NpointsPattern = length(pattern_template);
lengthDifference = NpointsData - NpointsPattern;

if lengthDifference < 0
    error('Pattern length exceeds input data length. Matching not possible.');
end

sumSquaredError =  inf(lengthDifference+1,1);

% Slide pattern through input data
for ith_shift = 0:lengthDifference
    temp_pattern = zeros(NpointsData,1);
    temp_pattern((ith_shift+1):(ith_shift+NpointsPattern)) = pattern_template;
    diff_array = intensity_data - temp_pattern;
    sumSquaredError(ith_shift + 1) = sum(diff_array.^2);
end

meanSquaredError = sumSquaredError / NpointsData;
[best_fit_error, min_idx] = min(meanSquaredError);
best_fit_idx = min_idx + 1; % Adjust to match indexing convention

% Reconstruct aligned pattern
best_fit_pattern = zeros(NpointsData,1);
best_fit_pattern(min_idx:(min_idx+NpointsPattern-1)) = pattern_template;


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
