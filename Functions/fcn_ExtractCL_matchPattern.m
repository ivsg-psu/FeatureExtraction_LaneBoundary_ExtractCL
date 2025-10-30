function [best_fit_pattern, best_fit_idx, best_fit_error, meanSquaredError_Combined] = fcn_ExtractCL_matchPattern(intensity_data, pattern_template, varargin)
% fcn_ExtractCL_matchPattern
% Matches a given pattern to intensity data by sliding-window method.
% Supports optional restriction to neighborhoods around candidate indices.
%
% FORMAT:
%   [best_fit_pattern, best_fit_idx, best_fit_error, MSE] = fcn_ExtractCL_matchPattern( ...
%       intensity_data, pattern_template, 'CandidateIdx', cand_idx, 'HalfWindowIdx', 5)
%
% INPUTS:
%   intensity_data: Nx1 vector
%       Input signal (e.g., filtered intensity or extrema correlation)
%
%   pattern_template: Mx1 vector
%       Reference pattern to match (must be M <= N)
%
% OPTIONAL NAME-VALUE:
%   'CandidateIdx'  : [] (default). If provided, only evaluate start indices
%                     within +/- HalfWindowIdx around each candidate index.
%   'HalfWindowIdx' : 5 (default). Neighborhood half-size in samples.
%
% OUTPUTS:
%   best_fit_pattern: Nx1 vector
%       Best aligned pattern padded to length of input
%
%   best_fit_idx: scalar
%       Index in intensity_data where pattern starts (1-based)
%
%   best_fit_error: scalar
%       Mean squared error (MSE) of best match
%
%   MSE: Kx1 vector
%       Per-start MSE for all valid starts (K = N-M+1). If CandidateIdx is
%       used, disallowed starts are set to +Inf.

%% Debugging and Input checks

% Check if flag_max_speed set. This occurs if the fig_num variable input
% argument (varargin) is given a number of -1, which is not a valid figure
% number.
%% Debugging and Input checks (kept as in your style)
flag_max_speed = 0;
if (nargin>=3 && isequal(varargin{end},-1))
    flag_do_debug = 0;  %#ok<NASGU>
    flag_check_inputs = 0; %#ok<NASGU>
    flag_max_speed = 1;
else
    flag_do_debug = 0;  %#ok<NASGU>
    flag_check_inputs = 1; %#ok<NASGU>
    MATLABFLAG_LAPS_FLAG_CHECK_INPUTS = getenv("MATLABFLAG_LAPS_FLAG_CHECK_INPUTS");
    MATLABFLAG_LAPS_FLAG_DO_DEBUG = getenv("MATLABFLAG_LAPS_FLAG_DO_DEBUG");
    if ~isempty(MATLABFLAG_LAPS_FLAG_CHECK_INPUTS) && ~isempty(MATLABFLAG_LAPS_FLAG_DO_DEBUG)
        flag_do_debug = str2double(MATLABFLAG_LAPS_FLAG_DO_DEBUG); %#ok<NASGU>
        flag_check_inputs  = str2double(MATLABFLAG_LAPS_FLAG_CHECK_INPUTS); %#ok<NASGU>
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

DataLength = size(intensity_data,1);
PatternLength = length(pattern_template);
DifferenceLength = DataLength - PatternLength;
K = DifferenceLength + 1;

if DifferenceLength < 0
    error('Pattern length exceeds input data length. Matching not possible.');
end

% Precompute constants (global SSD terms)
sum_intensity2 = sum(intensity_data.^2);   % constant over shifts
sum_pattern2   = sum(pattern_template.^2); % constant over shifts
sumSquaredError = zeros(DifferenceLength+1,1);

sum_pattern = sum(pattern_template);
pattern_ave = sum_pattern / PatternLength;
pattern_var = max(sum_pattern2 - PatternLength * pattern_ave^2, eps);

% Precompute constants (zNCC term)
csum_intensity = [0; cumsum(intensity_data)];
csum_intensity2 = [0; cumsum(intensity_data.^2)];
zNCC_score = zeros(DifferenceLength+1,1);
dtw_score = zeros(DifferenceLength+1,1);
corr_score = zeros(DifferenceLength+1,1);
eps_zncc = 1e-12;
% Compute SSD per shift using only the window dot product

for ith_shift = 0:DifferenceLength
    pattern_zero_padding = zeros(DataLength,1);
    
    r1 = ith_shift + 1;
    r2 = ith_shift + PatternLength;
    pattern_zero_padding(r1:r2) = pattern_template;
    sum_IP = intensity_data(r1:r2).' * pattern_template;  % dot(I_window, P)
    sumSquaredError(r1) = sum_intensity2 - 2*sum_IP + sum_pattern2;  % global SSD
    intensity_win = intensity_data(r1:r2);
    % corr_score(r1) = sqrt(mean((intensity_win - pattern_template).^2));
    % corr_score(r1) = corr(intensity_data(:), pattern_zero_padding(:));
    % corr_score(r1) = vecnorm(intensity_win - pattern_template,2,2);
    sum_intensity_win  = csum_intensity(r2+1)  - csum_intensity(r1);
    sum_intensity2_win = csum_intensity2(r2+1) - csum_intensity2(r1);

    intensity_win_ave = sum_intensity_win / PatternLength;
    corr_score(r1) = sum_intensity2_win - 2*sum_IP + sum_pattern2;
    zNCC_num = sum_IP - PatternLength*intensity_win_ave*pattern_ave;
    zNCC_den = sqrt(max(sum_intensity2_win - PatternLength*intensity_win_ave^2,0) * pattern_var) + eps_zncc;
    zNCC_score(r1) = zNCC_num / zNCC_den;
end
zNCC_error = -zNCC_score;
MSE = sumSquaredError / DataLength; 
MSEWin = corr_score / PatternLength;
% scale_MSE   = max(mad(MSE,1),        1e-9);
% scale_zNCC  = max(mad(zNCC_error,1), 1e-9);
% 
% normMSE     = MSE        / scale_MSE;
% normZNCCerr = zNCC_error / scale_zNCC;
weight_MSE  = 0.1;         
meanSquaredError_Combined = weight_MSE*MSE + (1-weight_MSE)*zNCC_error;
[best_fit_error, min_idx] = min(MSE);
best_fit_idx = min_idx;  % keep your indexing convention

% Reconstruct aligned pattern
best_fit_pattern = zeros(DataLength,1);
best_fit_pattern(min_idx:(min_idx+PatternLength-1)) = pattern_template;


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


function cur_corr = fcn_Internal_corr(I, P)
    I = I(:); 
    P = P(:);
    I = (I - mean(I)) / max(std(I), eps);
    P = (P - mean(P)) / max(std(P), eps);
    cur_corr = (I' * P) / numel(I);   % ∈ [-1,1]
end
