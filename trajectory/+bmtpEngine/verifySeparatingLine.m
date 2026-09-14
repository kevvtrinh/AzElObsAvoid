function plane = verifySeparatingLine(plane, controlPoint_units, vertices_units, reserve_units, target_units)
%% Section 0: Header & Readme
% SYNTAX: plane = bmtpEngine.verifySeparatingLine( plane, controlPoint_units, vertices_units,
%   reserve_units, target_units)
% PURPOSE: Bound one degree-one separating line with direct Bernstein products, then accept it
%   through the shared bmtpEngine.certifySeparation decision.
% INPUTS: plane (scalar separating-line struct) Candidate normals and offsets. controlPoint_units
%   (N-by-2), vertices_units (M-by-2 or M-by-2-by-2) Bezier control hull and static or affine
%   obstacle endpoint vertices. reserve_units, target_units (nonnegative numeric scalars)
%   Trajectory-side reserve and obstacle-side target.
% OUTPUTS: plane (scalar struct) Corrected offsets, certified gap, and Verified state.
% UNITS: Positions, offsets, targets, reserves, and gaps are coordinate units.

%% Section 1: Bound The Obstacle And Trajectory Sides Exactly
if ismatrix(vertices_units)
    % Static vertices make the obstacle-side polynomial linear.
    minimumObstacleSide_units = min(vertices_units*plane.Normal.'+plane.Offset_units,[],"all");
else
    first_units = vertices_units(:,:,1);
    last_units = vertices_units(:,:,end);
    % Affine vertex motion times an affine normal is quadratic. Its three
    % Bernstein coefficients bound the obstacle side throughout the interval.
    obstacleSide_units = [first_units*plane.Normal(1,:).'+plane.Offset_units(1), ...
        (first_units*plane.Normal(2,:).'+last_units*plane.Normal(1,:).'+sum(plane.Offset_units))/2, ...
        last_units*plane.Normal(2,:).'+plane.Offset_units(2)];
    minimumObstacleSide_units = min(obstacleSide_units,[],"all");
end
degree = size(controlPoint_units, 1) - 1;
% Exact degree-N by degree-one Bernstein product weights.
beta   = (0:degree + 1).' / (degree + 1);
alpha  = 1 - beta;
product_units = alpha .* [sum(controlPoint_units .* plane.Normal(1, :), 2); 0] + beta .* [0; sum(controlPoint_units .* plane.Normal(2, :), 2)] + alpha * plane.Offset_units(1) + beta * plane.Offset_units(2);
[maximumTrajectorySide_units, maximumNormalNorm] = deal(max(product_units), max(vecnorm(plane.Normal, 2, 2)));

%% Section 2: Apply The Shared Correction And Acceptance Decision
% Measuring the coordinate scale costs a pass over the obstacle and the hull,
% so measure it only where an offset correction can actually be applied.
roundoff_units = 0;
if target_units - minimumObstacleSide_units <= -reserve_units - maximumTrajectorySide_units
    scale_units    = bmtpEngine.createCoordinateTolerances(plane.Offset_units, vertices_units, controlPoint_units);
    roundoff_units = 16 * eps(scale_units);
end
[plane.Offset_units, plane.SignedGap_units, plane.Verified] = bmtpEngine.certifySeparation( ...
    minimumObstacleSide_units, maximumTrajectorySide_units, maximumNormalNorm, ...
    plane.Offset_units, roundoff_units, reserve_units, target_units);
end
