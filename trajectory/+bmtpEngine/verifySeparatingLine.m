function plane = verifySeparatingLine( ...
        plane, controlPoint_deg, vertices_deg, reserve_deg, target_deg)
%% Section 0: Header & Readme
% SYNTAX
%   plane = bmtpEngine.verifySeparatingLine( ...
%       plane, controlPoint_deg, vertices_deg, reserve_deg, target_deg)
%**************************************************************************
% PURPOSE
%   - Verify obstacle, trajectory, gap, and normal inequalities for one
%     degree-one separating line using direct Bernstein product bounds.
%**************************************************************************
% INPUTS
%   - plane (scalar separating-line struct)
%       Candidate normals and offsets.
%   - controlPoint_deg, vertices_deg (N-by-2 numeric arrays)
%       Bezier control hull and convex obstacle region.
%   - reserve_deg, target_deg (nonnegative numeric scalars)
%       Trajectory-side reserve and obstacle-side target.
%**************************************************************************
% OUTPUTS
%   - plane (scalar struct)
%       Corrected offsets, certified gap, and Verified state.
%**************************************************************************
% UNITS
%   - Positions, offsets, targets, reserves, and gaps are degrees.
%**************************************************************************

%% Section 1: Verify Direct Separation Inequalities

minimumObstacleSide_deg = min( ...
    vertices_deg * plane.Normal.' + plane.Offset_deg, [], "all");
degree = size(controlPoint_deg, 1) - 1;
[alpha, beta] = productWeights(degree);
product_deg = alpha .* ...
    [sum(controlPoint_deg .* plane.Normal(1, :), 2); 0] + ...
    beta .* [0; sum(controlPoint_deg .* plane.Normal(2, :), 2)] + ...
    alpha * plane.Offset_deg(1) + beta * plane.Offset_deg(2);
[maximumTrajectorySide_deg, maximumNormalNorm] = ...
    deal(max(product_deg), max(vecnorm(plane.Normal, 2, 2)));
[minimumCorrection_deg, maximumCorrection_deg] = deal( ...
    target_deg - minimumObstacleSide_deg, ...
    -reserve_deg - maximumTrajectorySide_deg);
if minimumCorrection_deg <= maximumCorrection_deg
    scale_deg = bmtpEngine.createCoordinateTolerances( ...
        plane.Offset_deg, vertices_deg, controlPoint_deg);
    roundoff_deg = 16 * eps(scale_deg);
    [robustMinimum_deg, robustMaximum_deg] = deal( ...
        minimumCorrection_deg + roundoff_deg, ...
        maximumCorrection_deg - roundoff_deg);
    if robustMinimum_deg <= robustMaximum_deg
        correction_deg = min(max(0, robustMinimum_deg), robustMaximum_deg);
    else
        correction_deg = ...
            0.5 * (minimumCorrection_deg + maximumCorrection_deg);
    end
    plane.Offset_deg = plane.Offset_deg + correction_deg;
    [minimumObstacleSide_deg, maximumTrajectorySide_deg] = deal( ...
        minimumObstacleSide_deg + correction_deg, ...
        maximumTrajectorySide_deg + correction_deg);
end
signedGap_deg = minimumObstacleSide_deg - maximumTrajectorySide_deg;
plane.SignedGap_deg = signedGap_deg;
normalNormLimit = 1 + 2 ^ 20 * eps;
clearanceTarget_deg = (target_deg - reserve_deg) / normalNormLimit;
certifiedClearance_deg = (signedGap_deg - 2 * reserve_deg) / ...
    max(maximumNormalNorm, realmin);
plane.Verified = minimumObstacleSide_deg >= target_deg && ...
    maximumTrajectorySide_deg <= -reserve_deg && ...
    signedGap_deg >= target_deg + reserve_deg && ...
    certifiedClearance_deg >= clearanceTarget_deg && ...
    maximumNormalNorm <= normalNormLimit;
end

%% Section 2: Local Functions

function [alpha, beta] = productWeights(degree)
% Return exact degree-N by degree-one Bernstein product weights.
beta = (0:degree + 1).' / (degree + 1);
alpha = 1 - beta;
end
