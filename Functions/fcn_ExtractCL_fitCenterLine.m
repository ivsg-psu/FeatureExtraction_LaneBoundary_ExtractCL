function [SegTable, info] = fcn_ExtractCL_fitCenterLine(XYZST, varargin)
% fcn_ExtractCL_fitCenterLine
% ------------------------------------------------------------------------------
% Generate a C0+C1 continuous center line model from XYZST_array for HD Map.
% Pipeline:
%   A) Global robust smoothing on (x,y) vs arc-length s to get a continuous
%      reference curve r_ref(s) with derivatives (C1/C2-like).
%   B) Segment by curvature magnitude/trend (line/arc/spiral) with hysteresis
%      and minimal length; merge and clean segments.
%   C) On each segment, fit {line|arc|spiral|poly3} under hard C0/C1 constraint
%      at the start boundary; choose by robust mean orthogonal error plus
%      a complexity penalty; perform MAD outlier removal and refit; if not
%      acceptable, split the segment and recurse (binary refinement).
%   D) Output SegTable with unified XYModel.EvalXY(s) for downstream use.
%
% INPUT:
%   XYZST : Nx5 double [X Y Z S T], S must be non-decreasing (will be sorted).
%
% PARAMS (Name-Value):
%   'ErrorTol'      : double, mean orthogonal error threshold [m] (default 0.10).
%   'MinLen'        : double, minimal segment length [m] (default 40).
%   'LambdaC'       : double, complexity penalty (line=0, arc=1, spiral=2, poly3=3) (0.06).
%   'RobustK'       : double, MAD multiplier for outlier rejection (default 2.5).
%   'SmoothP'       : double in (0,1), smoothing (csaps) strength (default 0.999).
%   'KLine'         : double, |kappa| <= KLine => line candidate (default 2e-4 [1/m]).
%   'KPrimeSmall'   : double, |dk/ds| small threshold (default 1e-5 [1/m^2]).
%   'KPrimeArc'     : double, |dk/ds| <= KPrimeArc => arc candidate (default 7e-6).
%   'Hyst'          : double, hysteresis factor on thresholds (default 1.3).
%   'Verbose'       : logical, print progress (default false).
%
% OUTPUT:
%   SegTable : table with columns
%       Label, S0, S1, I0, I1, Length_m, SegmentData, XYModel, XYErrorMean, Outliers
%   info : struct with diagnostics and references (reference curve etc.).
%
% DEPENDENCIES:
%   Uses csaps if available; falls back to smoothdata moving-average otherwise.
% ------------------------------------------------------------------------------

% ---------- Parse inputs ----------
p = inputParser;
p.addParameter('ErrorTol', 0.10);
p.addParameter('MinLen', 40);
p.addParameter('LambdaC', 0.06);
p.addParameter('RobustK', 2.5);
p.addParameter('SmoothP', 0.999);
p.addParameter('KLine', 2e-4);
p.addParameter('KPrimeSmall', 1e-5);
p.addParameter('KPrimeArc', 7e-6);
p.addParameter('Hyst', 1.30);
p.addParameter('Verbose', false);
p.addParameter('WinMeters', 0.8*40);   % default 0.8*MinLen, will be evaluated after MinLen is parsed
p.addParameter('ThrK_line',    3.0e-3);
p.addParameter('ThrKstd_line', 5.0e-4);
p.addParameter('ThrArc_stdK',  3.0e-4);
p.addParameter('ThrSp_minKp',  1.0e-5);
p.addParameter('ThrSp_stdK',   1.0e-5);

p.parse(varargin{:});
opt = p.Results;

% ---------- Basic checks and sort by S ----------
assert(size(XYZST,2)==5, 'XYZST must be Nx5 = [X Y Z S T].');
[~, idx_sort] = sort(XYZST(:,4), 'ascend');
XYZST = XYZST(idx_sort, :);
X = XYZST(:,1);
Y = XYZST(:,2);
S = XYZST(:,4);
N = numel(S);
assert(N >= 5, 'Too few points.');

% ---------- Stage A: global reference smoothing ----------
[x_ref, y_ref] = local_smooth_reference(S, X, Y, opt.SmoothP);
dx = local_derivative(S, x_ref);
dy = local_derivative(S, y_ref);
spd = hypot(dx, dy);
t_ref = [dx ./ max(spd, 1e-12), dy ./ max(spd, 1e-12)];
ddx = local_second_derivative(S, x_ref);
ddy = local_second_derivative(S, y_ref);
kappa_ref = (dx .* ddy - dy .* ddx) ./ max(spd.^3, 1e-12);
dkds_ref = local_derivative(S, kappa_ref);

% ---------- Stage B: curvature-based coarse segmentation ----------
segments = local_segment_by_stats(S, X, Y, opt);
segments = local_merge_short_segments(segments, opt.MinLen);

% ---------- Stage C: constrained fitting per segment with refinement ----------
rows = {};
StartPose = struct();
StartPose.r0 = [x_ref(1) y_ref(1)];
StartPose.t0 = t_ref(1, :);
StartPose.gamma = local_estimate_gamma(S, x_ref, y_ref);
StartPose.k0 = kappa_ref(1);
StartPose.C1_slack = 0;

for si = 1:2
    s0 = segments(si,1);
    s1 = segments(si,2);
    rows = local_fit_segment_recursive(rows, s0, s1, StartPose, ...
        S, X, Y, kappa_ref, dkds_ref, opt);
    % last_row = rows{end, :};
    % XYModel = last_row{8};
    XYModel = rows{end, 8};

    [r_end, dr_end] = XYModel.EvalXY(s1);
    t_end = dr_end ./ max(norm(dr_end), 1e-12);
    k_end = local_estimate_kappa_from_model(XYModel, s1);
    StartPose.r0 = r_end;
    StartPose.t0 = t_end;
    StartPose.k0 = k_end;
    StartPose.gamma = local_estimate_gamma(S, x_ref, y_ref);
    StartPose.C1_slack = 0;
end

SegTable = cell2table(rows, 'VariableNames', ...
    {'Label','S0','S1','I0','I1','Length_m','SegmentData','XYModel','XYErrorMean','Outliers'});

% ---------- Stage D: pack info ----------
info = struct();
info.Reference.S = S;
info.Reference.X = x_ref;
info.Reference.Y = y_ref;
info.Reference.Kappa = kappa_ref;

end


% =========================================================================
% ========================== Helper functions =============================
% =========================================================================

function [x_ref, y_ref] = local_smooth_reference(S, X, Y, p)
% Continuous reference curve from robust smoothing.
if exist('csaps','file')
    spx = csaps(S, X, p);
    spy = csaps(S, Y, p);
    x_ref = fnval(spx, S);
    y_ref = fnval(spy, S);
else
    x_ref = smoothdata(X, 'movmean', max(5, floor(numel(X)/100)));
    y_ref = smoothdata(Y, 'movmean', max(5, floor(numel(Y)/100)));
end
end

function d = local_derivative(S, v)
% First derivative w.r.t S using central differences.
d = gradient(v, S);
end

function d2 = local_second_derivative(S, v)
% Second derivative w.r.t S using central differences.
d1 = gradient(v, S);
d2 = gradient(d1, S);
end

function gamma = local_estimate_gamma(S, X, Y)
% Robust XY-per-S scale used by line/arc parameterization.
dS = diff(S);
dXY = hypot(diff(X), diff(Y));
gamma = median(dXY ./ max(dS, 1e-9), 'omitnan');
if ~isfinite(gamma) || gamma <= 0
    gamma = 1;
end
end

function segments = local_segment_by_stats(S, X, Y, opt)
% Segment by windowed statistics of kappa and kappa' (your proposed method).
% Output: [s0, s1, label_code], label_code: 1=line, 2=arc, 3=spiral, 4=poly3

N = numel(S);

% --- window length in samples, based on meters ---
ds = diff(S);
ds_med = max(median(ds), 1e-3);
winMetersEff = max(opt.WinMeters, 0.8*opt.MinLen);
winN = max(11, 2*round(0.5*winMetersEff/ds_med) + 1);  % odd window

% --- smooth first, then differentiate ---
X_sm = movmean(X, winN, 'Endpoints','shrink');
Y_sm = movmean(Y, winN, 'Endpoints','shrink');

dXdS   = gradient(X_sm, S);
dYdS   = gradient(Y_sm, S);
d2XdS2 = gradient(dXdS, S);
d2YdS2 = gradient(dYdS, S);

meters_per_station = sqrt(abs(dXdS).^2 + abs(dYdS).^2);
meters_per_station = max(meters_per_station, 1e-9);

Yaw   = unwrap(atan2(dYdS, dXdS)); %#ok<NASGU> % not used downstream but kept for debugging
Kappa = (dXdS.*d2YdS2 - dYdS.*d2XdS2) ./ (meters_per_station.^3);
Kappasm = movmean(Kappa, winN, 'Endpoints','shrink');

KappaPrime_S  = gradient(Kappasm, S);
KappaPrime_ds = KappaPrime_S ./ meters_per_station;

% --- windowed stats ---
mK  = movmean(Kappasm, winN, 'Endpoints','shrink');
sK  = movstd (Kappasm, winN, 'Endpoints','shrink');
mKp = movmean(KappaPrime_ds, winN, 'Endpoints','shrink');
sKp = movstd (KappaPrime_ds, winN, 'Endpoints','shrink');

% --- thresholds (from opt) ---
ThrK_line    = opt.ThrK_line;
ThrKstd_line = opt.ThrKstd_line;
ThrArc_stdK  = opt.ThrArc_stdK;
ThrSp_minKp  = opt.ThrSp_minKp;
ThrSp_stdK   = opt.ThrSp_stdK;

% --- pointwise coarse labels ---
label = strings(N,1);
isLine   = (abs(mK) < ThrK_line) & (sK < ThrKstd_line);
isArc    = ~isLine & (sK < ThrArc_stdK) & (abs(mK) >= ThrK_line);
isSpiral = ~isLine & ~isArc & (abs(mKp) > ThrSp_minKp) & (sKp < ThrSp_stdK);

label(isLine)    = "line";
label(isArc)     = "arc";
label(isSpiral)  = "spiral";
label(label=="") = "poly3";

% --- optional small hysteresis / morphological smoothing ---
% to reduce rapid toggling; you can uncomment if needed
% label = local_label_smooth(label, 5);

% --- compress to segments ---
edges = [1; find(label(2:end) ~= label(1:end-1)) + 1; N+1];
segments = zeros(numel(edges)-1, 3);
for k = 1:numel(edges)-1
    i0 = edges(k);
    i1 = edges(k+1) - 1;
    s0 = S(i0);
    s1 = S(i1);
    if s1 <= s0
        s1 = s0 + 1e-6;
    end
    segments(k,1) = s0;
    segments(k,2) = s1;
    switch label(i0)
        case "line"
            segments(k,3) = 1;
        case "arc"
            segments(k,3) = 2;
        case "spiral"
            segments(k,3) = 3;
        otherwise
            segments(k,3) = 4; % poly3
    end
end
end



function segments = local_segment_by_curvature(S, kappa, dkds, opt)
% Build coarse segments by curvature regimes with hysteresis and minimal run.
Kline_on = opt.KLine;
Kline_off = opt.KLine * opt.Hyst;
Kp_arc_on = opt.KPrimeArc;
Kp_arc_off = opt.KPrimeArc * opt.Hyst;

labels = strings(numel(S),1);
for i = 1:numel(S)
    if abs(kappa(i)) <= Kline_on && abs(dkds(i)) <= opt.KPrimeSmall
        labels(i,1) = "line";
    elseif abs(dkds(i)) <= Kp_arc_on
        labels(i,1) = "arc";
    else
        labels(i,1) = "spiral";
    end
end

% Hysteresis pass
for i = 2:numel(S)
    if labels(i) == "line" && labels(i-1) ~= "line"
        if abs(kappa(i)) <= Kline_off && abs(dkds(i)) <= opt.KPrimeSmall * opt.Hyst
            labels(i) = "line";
        end
    elseif labels(i) == "arc" && labels(i-1) == "line"
        if abs(kappa(i)) <= Kline_off
            labels(i) = "line";
        end
    elseif labels(i) == "arc" && labels(i-1) == "spiral"
        if abs(dkds(i)) <= Kp_arc_off
            labels(i) = "arc";
        end
    end
end

% Compress to [s0,s1] intervals
edges = [1; find(labels(2:end) ~= labels(1:end-1)) + 1; numel(S)+1];
segments = zeros(numel(edges)-1, 3);
for k = 1:numel(edges)-1
    i0 = edges(k);
    i1 = edges(k+1) - 1;
    segments(k,1) = S(i0);
    segments(k,2) = S(i1);
    if segments(k,2) <= segments(k,1)
        segments(k,2) = segments(k,1) + 1e-6;
    end
    if labels(i0) == "line"
        segments(k,3) = 1;
    elseif labels(i0) == "arc"
        segments(k,3) = 2;
    else
        segments(k,3) = 3;
    end
end
end

function segments = local_merge_short_segments(segments, MinLen)
% Merge segments shorter than MinLen to neighbors.
if isempty(segments)
    return;
end
k = 1;
while k <= size(segments,1)
    L = segments(k,2) - segments(k,1);
    if L < MinLen && size(segments,1) > 1
        if k == 1
            segments(k+1,1) = segments(k,1);
            segments(k,:) = [];
            continue;
        elseif k == size(segments,1)
            segments(k-1,2) = segments(k,2);
            segments(k,:) = [];
            continue;
        else
            % Merge to the neighbor with lower complexity (prefer line<arc<spiral)
            compL = segments(k-1,3);
            compR = segments(k+1,3);
            if compL <= compR
                segments(k-1,2) = segments(k,2);
                segments(k,:) = [];
                continue;
            else
                segments(k+1,1) = segments(k,1);
                segments(k,:) = [];
                continue;
            end
        end
    end
    k = k + 1;
end
end

function rows = local_fit_segment_recursive(rows, s0, s1, StartPose, ...
    S, X, Y, kappa_ref, dkds_ref, opt)
% Fit one segment; if error too high, split and recurse.
idx = find(S >= s0 & S <= s1);
if numel(idx) < 3
    return;
end
S_seg = S(idx);
XY_seg = [X(idx) Y(idx)];
% Try four models with hard C0/C1 at s0
cands = cell(4,1);
cands{1} = local_tryLine(S_seg, XY_seg, StartPose);
cands{2} = local_tryArc(S_seg, XY_seg, StartPose, kappa_ref, s0, s1);
cands{3} = local_trySpiral(S_seg, XY_seg, StartPose, kappa_ref, dkds_ref, s0, s1);
cands{4} = local_tryPoly3(S_seg, XY_seg, StartPose);
% Score = mean_err + lambdaC * complexity
scores = zeros(4,1);
for i = 1:4
    scores(i) = cands{i}.mean_err + opt.LambdaC .* cands{i}.complexity;
end
[~, ibest] = min(scores);
best = cands{ibest};
% Prefer line if it is close enough and within tolerance
eta = 0.05;
if (cands{1}.mean_err <= opt.ErrorTol) && (cands{1}.mean_err <= best.mean_err .* (1 + eta))
    best = cands{1};
end
% MAD outlier removal and refit on inliers
[mask_in, ~] = local_mad_filter(best.dperp, opt.RobustK);
inliers = idx(mask_in);
outliers = idx(~mask_in);
S_in = S(inliers);
XY_in = [X(inliers) Y(inliers)];
best_refit = local_dispatch_try(best.label, S_in, XY_in, StartPose, kappa_ref, dkds_ref, s0, s1);
% Accept or split
if best_refit.mean_err <= opt.ErrorTol || (s1 - s0) <= opt.MinLen
    % Pack XYModel
    XYModel = struct();
    XYModel.Type = best_refit.label;
    XYModel.S0 = s0;
    XYModel.SRange = [s0, s1];
    XYModel.Params = best_refit.params;
    XYModel.EvalXY = best_refit.EvalXY;
    % Append row
    row = cell(1,10);
    row{1} = string(best_refit.label);
    row{2} = s0;
    row{3} = s1;
    row{4} = inliers(1);
    row{5} = inliers(end);
    row{6} = s1 - s0;
    row{7} = [X(inliers) Y(inliers) zeros(numel(inliers),1) S(inliers) zeros(numel(inliers),1)];
    row{8} = XYModel;
    row{9} = best_refit.mean_err;
    row{10} = outliers(:);
    rows(end+1, :) = row;
else
    % Split at max residual location
    [~, imaxLocal] = max(best.dperp);
    isplit = idx(imaxLocal);
    s_mid = S(isplit);
    s_mid = max(min(s_mid, s1 - 1e-3), s0 + 1e-3);
    rows = local_fit_segment_recursive(rows, s0, s_mid, StartPose, S, X, Y, kappa_ref, dkds_ref, opt);
    % last_row = rows{end, :};
    % XYModelL = last_row{8};
    XYModelL = rows{end, 8};

    [r_m, dr_m] = XYModelL.EvalXY(s_mid);
    t_m = dr_m ./ max(norm(dr_m), 1e-12);
    StartPose2 = StartPose;
    StartPose2.r0 = r_m;
    StartPose2.t0 = t_m;
    StartPose2.k0 = local_estimate_kappa_from_model(XYModelL, s_mid);
    rows = local_fit_segment_recursive(rows, s_mid, s1, StartPose2, S, X, Y, kappa_ref, dkds_ref, opt);
end
end

function [mask_in, mask_out] = local_mad_filter(d, K)
% MAD-based robust inlier mask on non-negative distances.
med = median(d);
madv = 1.4826 .* median(abs(d - med));
thr = med + K .* madv;
mask_in = (d <= thr);
mask_out = ~mask_in;
end

function out = local_dispatch_try(label, S_seg, XY_seg, StartPose, kappa_ref, dkds_ref, s0, s1)
% Call the correct model fitter by label.
switch string(label)
    case "line"
        out = local_tryLine(S_seg, XY_seg, StartPose);
    case "arc"
        out = local_tryArc(S_seg, XY_seg, StartPose, kappa_ref, s0, s1);
    case "spiral"
        out = local_trySpiral(S_seg, XY_seg, StartPose, kappa_ref, dkds_ref, s0, s1);
    otherwise
        out = local_tryPoly3(S_seg, XY_seg, StartPose);
end
end

% ========================= Model fitters (C0/C1 clamped) =======================

function c = local_tryLine(S_seg, XY_seg, StartPose)
% Line with C0/C1 clamped at start; LS for gamma.
label = "line";
complexity = 0;
S0 = S_seg(1);
r0 = StartPose.r0;
t0 = StartPose.t0;
s_hat = S_seg - S0;
proj = (XY_seg - r0) * t0';
den = sum(s_hat.^2);
if den < 1e-12
    gamma = StartPose.gamma;
else
    gamma = sum(s_hat .* proj) / den;
end
if ~isfinite(gamma) || gamma <= 0
    gamma = StartPose.gamma;
end
EvalXY = @(Sq) local_eval_line(Sq, S0, r0, t0, gamma);
[~, dperp] = local_eval_distances(EvalXY, S_seg, XY_seg);
mean_err = mean(dperp, 'omitnan');
params = struct();
params.gamma = gamma;
c = struct();
c.label = label;
c.params = params;
c.EvalXY = EvalXY;
c.dperp = dperp;
c.mean_err = mean_err;
c.complexity = complexity;
end

function c = local_tryArc(S_seg, XY_seg, StartPose, kappa_ref, s0, s1)
% Constant-curvature arc with C0/C1 clamped at start; grid search on kappa.
label = "arc";
complexity = 1;
S0 = S_seg(1);
r0 = StartPose.r0;
t0 = StartPose.t0;
gamma = StartPose.gamma;
% Center search around reference curvature
k0 = local_interp_scalar(kappa_ref, S0, s0, s1);
span = max(5e-4, abs(k0)) * 6;
grid = linspace(k0 - span, k0 + span, 81);
best_err = inf;
best_k = 0;
best_eval = @(s) local_eval_arc(s, S0, r0, t0, gamma, 0);
for kappa = grid
    EvalXY_try = @(Sq) local_eval_arc(Sq, S0, r0, t0, gamma, kappa);
    [~, dperp_try] = local_eval_distances(EvalXY_try, S_seg, XY_seg);
    e = mean(dperp_try, 'omitnan');
    if e < best_err
        best_err = e;
        best_k = kappa;
        best_eval = EvalXY_try;
    end
end
EvalXY = best_eval;
[~, dperp] = local_eval_distances(EvalXY, S_seg, XY_seg);
mean_err = mean(dperp, 'omitnan');
params = struct();
params.kappa = best_k;
params.gamma = gamma;
c = struct();
c.label = label;
c.params = params;
c.EvalXY = EvalXY;
c.dperp = dperp;
c.mean_err = mean_err;
c.complexity = complexity;
end

function c = local_trySpiral(S_seg, XY_seg, StartPose, kappa_ref, dkds_ref, s0, s1)
% Clothoid with k(s) = k0 + a*(s - S0); search on slope a around dk/ds.
label = "spiral";
complexity = 2;
S0 = S_seg(1);
r0 = StartPose.r0;
t0 = StartPose.t0;
gamma = StartPose.gamma;
k0 = local_interp_scalar(kappa_ref, S0, s0, s1);
ap = local_interp_scalar(dkds_ref, S0, s0, s1);
span = max(2e-5, abs(ap)) * 6;
agrid = linspace(ap - span, ap + span, 61);
best_err = inf;
best_a = 0;
best_eval = @(s) local_eval_spiral(s, S0, r0, t0, gamma, k0, 0);
for a = agrid
    EvalXY_try = @(Sq) local_eval_spiral(Sq, S0, r0, t0, gamma, k0, a);
    [~, dperp_try] = local_eval_distances(EvalXY_try, S_seg, XY_seg);
    e = mean(dperp_try, 'omitnan');
    if e < best_err
        best_err = e;
        best_a = a;
        best_eval = EvalXY_try;
    end
end
EvalXY = best_eval;
[~, dperp] = local_eval_distances(EvalXY, S_seg, XY_seg);
mean_err = mean(dperp, 'omitnan');
params = struct();
params.k0 = k0;
params.a = best_a;
params.gamma = gamma;
c = struct();
c.label = label;
c.params = params;
c.EvalXY = EvalXY;
c.dperp = dperp;
c.mean_err = mean_err;
c.complexity = complexity;
end

function c = local_tryPoly3(S_seg, XY_seg, StartPose)
% Cubic per component with C0/C1 clamped at start.
label = "poly3";
complexity = 3;
S0 = S_seg(1);
r0 = StartPose.r0;
t0 = StartPose.t0;
gamma = StartPose.gamma;
s_hat = S_seg - S0;
c1 = t0 .* gamma;
A = [s_hat.^2, s_hat.^3];
bx = XY_seg(:,1) - (r0(1) + c1(1) .* s_hat);
by = XY_seg(:,2) - (r0(2) + c1(2) .* s_hat);
theta_x = A \ bx;
theta_y = A \ by;
c2x = theta_x(1);
c3x = theta_x(2);
c2y = theta_y(1);
c3y = theta_y(2);
EvalXY = @(Sq) local_eval_poly3(Sq, S0, r0, c1, [c2x c3x], [c2y c3y]);
[~, dperp] = local_eval_distances(EvalXY, S_seg, XY_seg);
mean_err = mean(dperp, 'omitnan');
params = struct();
params.c1 = c1;
params.c2x = c2x;
params.c3x = c3x;
params.c2y = c2y;
params.c3y = c3y;
c = struct();
c.label = label;
c.params = params;
c.EvalXY = EvalXY;
c.dperp = dperp;
c.mean_err = mean_err;
c.complexity = complexity;
end

% ============================ Evaluators =================================

function [xy, dxy] = local_eval_line(Sq, S0, r0, t0, gamma)
s_hat = Sq - S0;
xy = r0 + (s_hat .* gamma) .* t0;
dxy = repmat(gamma .* t0, size(Sq,1), 1);
end

function [xy, dxy] = local_eval_arc(Sq, S0, r0, t0, gamma, kappa)
s_hat = Sq - S0;
psi0 = atan2(t0(2), t0(1));
psi = psi0 + (kappa .* gamma) .* s_hat;
xy = zeros(numel(Sq), 2);
dxy = zeros(numel(Sq), 2);
if abs(kappa) < 1e-12
    xy = r0 + (s_hat .* gamma) .* t0;
    dxy = repmat(gamma .* t0, size(Sq,1), 1);
    return;
end
R = 1 ./ kappa;
cx = r0(1) - R .* sin(psi0);
cy = r0(2) + R .* cos(psi0);
xy(:,1) = cx + R .* sin(psi);
xy(:,2) = cy - R .* cos(psi);
dxy(:,1) = gamma .* cos(psi);
dxy(:,2) = gamma .* sin(psi);
end

function [xy, dxy] = local_eval_spiral(Sq, S0, r0, t0, gamma, k0, a)
% Simple Euler integration for clothoid; sufficient for HD map segments.
s_hat = Sq - S0;
xy = zeros(numel(Sq), 2);
dxy = zeros(numel(Sq), 2);
psi = atan2(t0(2), t0(1));
xy(1,:) = r0;
dxy(1,:) = gamma .* t0;
for i = 2:numel(Sq)
    ds = s_hat(i) - s_hat(i-1);
    k_mid = k0 + a .* (s_hat(i-1) + s_hat(i)) ./ 2.0;
    dpsi = k_mid .* gamma .* ds;
    psi = psi + dpsi;
    dx = gamma .* cos(psi) .* ds;
    dy = gamma .* sin(psi) .* ds;
    xy(i,1) = xy(i-1,1) + dx;
    xy(i,2) = xy(i-1,2) + dy;
    dxy(i,1) = gamma .* cos(psi);
    dxy(i,2) = gamma .* sin(psi);
end
end

function [xy, dxy] = local_eval_poly3(Sq, S0, r0, c1, cx, cy)
s_hat = Sq - S0;
x = r0(1) + c1(1) .* s_hat + cx(1) .* s_hat.^2 + cx(2) .* s_hat.^3;
y = r0(2) + c1(2) .* s_hat + cy(1) .* s_hat.^2 + cy(2) .* s_hat.^3;
dx = c1(1) + 2 .* cx(1) .* s_hat + 3 .* cx(2) .* s_hat.^2;
dy = c1(2) + 2 .* cy(1) .* s_hat + 3 .* cy(2) .* s_hat.^2;
xy = [x y];
dxy = [dx dy];
end

function [S_star, dperp] = local_eval_distances(EvalXY, S_seg, XY_seg)
% Newton projection for orthogonal distance between points and a param curve.
n = numel(S_seg);
S0 = S_seg(1);
S1 = S_seg(end);
S_star = zeros(n,1);
dperp = zeros(n,1);
MAX_IT = 12;
for j = 1:n
    s = S_seg(j);
    for it = 1:MAX_IT
        [r, rp] = EvalXY(s);
        v = r - XY_seg(j,1:2);
        h = dot(v, rp);
        ds = 1e-3 .* max(1, S1 - S0);
        [r2, rp2] = EvalXY(s + ds);
        v2 = r2 - XY_seg(j,1:2);
        h2 = dot(v2, rp2);
        dh = (h2 - h) ./ ds;
        if abs(dh) < 1e-12
            break;
        end
        s = s - h ./ dh;
        s = max(min(s, S1), S0);
        if abs(h) < 1e-6
            break;
        end
    end
    S_star(j) = s;
    [r_star, ~] = EvalXY(s);
    dperp(j,1) = hypot(r_star(1) - XY_seg(j,1), r_star(2) - XY_seg(j,2));
end
end

function v = local_interp_scalar(vs, s0, smin, smax)
% Interpolate scalar value vs known S grid for an s0 in [smin,smax].
% For simplicity we assume vs is sampled on the same S grid used upstream.
% Here we just clamp to bounds.
idx = round( (s0 - smin) / max(smax - smin, 1e-9) * (numel(vs)-1) ) + 1;
idx = max(1, min(numel(vs), idx));
v = vs(idx);
end

function k = local_estimate_kappa_from_model(XYModel, s)
% Estimate curvature from EvalXY via finite differences around s.
ds = 0.5;
[~, d0] = XYModel.EvalXY(s);
[~, d1] = XYModel.EvalXY(s + ds);
[~, d2] = XYModel.EvalXY(s - ds);
acc = (d1 - 2 .* d0 + d2) ./ (ds.^2);
t = d0 ./ max(norm(d0), 1e-12);
nrm = [-t(2), t(1)];
k = dot(acc, nrm) ./ max(norm(d0), 1e-12)^2;
end
