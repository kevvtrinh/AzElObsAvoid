function plane = verifySeparatingLine(plane, controlPoint_units, vertices_units, reserve_units, target_units)
%% Section 0: Header & Readme
% SYNTAX
%   plane = bmtpEngine.verifySeparatingLine( ...
%       plane, controlPoint_units, vertices_units, reserve_units, target_units)
%
% PURPOSE
%   - Verify obstacle, trajectory, gap, and normal inequalities for one
%     degree-one separating line using direct Bernstein product bounds.
%
% INPUTS
%   - plane (scalar separating-line struct)
%       Candidate normals and offsets.
%   - controlPoint_units, vertices_units (N-by-2 numeric arrays)
%       Bezier control hull and convex obstacle region.
%   - reserve_units, target_units (nonnegative numeric scalars)
%       Trajectory-side reserve and obstacle-side target.
%
% OUTPUTS
%   - plane (scalar struct)
%       Corrected offsets, certified gap, and Verified state.
%
% UNITS
%   - Positions, offsets, targets, reserves, and gaps are coordinate units.
%

%% Section 1: Verify Direct Separation Inequalities

minimumObstacleSide_units = min(vertices_units * plane.Normal.' + plane.Offset_units, [], "all");
degree = size(controlPoint_units, 1) - 1;
% Exact degree-N by degree-one Bernstein product weights.
beta   = (0:degree + 1).' / (degree + 1);
alpha  = 1 - beta;
product_units = alpha .* [sum(controlPoint_units .* plane.Normal(1, :), 2); 0] + beta .* [0; sum(controlPoint_units .* plane.Normal(2, :), 2)] + alpha * plane.Offset_units(1) + beta * plane.Offset_units(2);
[maximumTrajectorySide_units, maximumNormalNorm] = deal(max(product_units), max(vecnorm(plane.Normal, 2, 2)));
[minimumCorrection_units, maximumCorrection_units] = deal(target_units - minimumObstacleSide_units, -reserve_units - maximumTrajectorySide_units);
if minimumCorrection_units <= maximumCorrection_units
    scale_units    = bmtpEngine.createCoordinateTolerances(plane.Offset_units, vertices_units, controlPoint_units);
    roundoff_units = 16 * eps(scale_units);
    [robustMinimum_units, robustMaximum_units] = deal(minimumCorrection_units + roundoff_units, maximumCorrection_units - roundoff_units);
    if robustMinimum_units <= robustMaximum_units
        correction_units = min(max(0, robustMinimum_units), robustMaximum_units);
    else
        correction_units = 0.5 * (minimumCorrection_units + maximumCorrection_units);
    end
    plane.Offset_units = plane.Offset_units + correction_units;
    [minimumObstacleSide_units, maximumTrajectorySide_units] = deal(minimumObstacleSide_units + correction_units, maximumTrajectorySide_units + correction_units);
end
signedGap_units = minimumObstacleSide_units - maximumTrajectorySide_units;
plane.SignedGap_units = signedGap_units;
normalNormLimit        = 1 + 2 ^ 20 * eps;
clearanceTarget_units    = (target_units - reserve_units) / normalNormLimit;
certifiedClearance_units = (signedGap_units - 2 * reserve_units) / max(maximumNormalNorm, realmin);
plane.Verified = minimumObstacleSide_units >= target_units && maximumTrajectorySide_units <= -reserve_units && signedGap_units >= target_units + reserve_units && certifiedClearance_units >= clearanceTarget_units && maximumNormalNorm <= normalNormLimit;
end
