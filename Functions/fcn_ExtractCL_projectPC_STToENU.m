function XY_extrema = fcn_ExtractCL_projectPC_STToENU(S_extrema,T_extrema, Ref_pose)







X_ref_vec = Ref_pose(:,1);
Y_ref_vec = Ref_pose(:,2);
heading_ref_vec = Ref_pose(:,6);
S_ref_vec = Ref_pose(:,7);

% For each (S_extrema), find closest reference pose index
[~, nearest_idx] = min(abs(S_extrema - S_ref_vec'), [], 2);

% Gather corresponding ref values
X_ref = X_ref_vec(nearest_idx);
Y_ref = Y_ref_vec(nearest_idx);
S_ref = S_ref_vec(nearest_idx);
heading_ref = heading_ref_vec(nearest_idx);

% Apply ST → ENU transformation
x_extrema = X_ref + (S_extrema - S_ref) .* cos(heading_ref) - T_extrema .* sin(heading_ref);
y_extrema = Y_ref + (S_extrema - S_ref) .* sin(heading_ref) + T_extrema .* cos(heading_ref);
XY_extrema = [x_extrema y_extrema];
