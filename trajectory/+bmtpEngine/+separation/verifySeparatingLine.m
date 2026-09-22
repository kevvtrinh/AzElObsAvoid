function plane = verifySeparatingLine( ...
    plane, controlPoint_units, vertices_units, roundoffReserve_units, separationTarget_units)
%% Section 0: Header & Readme
% SYNTAX
%   plane = bmtpEngine.separation.verifySeparatingLine(plane, controlPoint_units, ...
%       vertices_units, roundoffReserve_units, separationTarget_units)
%**************************************************************************
% PURPOSE
%   - Bound the obstacle and curve sides of a line that changes linearly.
%     Check the entire interval, then apply the shared clearance requirements.
%**************************************************************************
% INPUTS
%   - plane (scalar struct)
%       Candidate normal vectors and offsets at the interval start/end.
%   - controlPoint_units (N-by-2 numeric array)
%       Bezier control points for one trajectory segment.
%   - vertices_units (M-by-2 or M-by-2-by-2 numeric array)
%       Static vertices, or matching vertices at the start/end of linear motion.
%   - roundoffReserve_units (nonnegative numeric scalar)
%       Numerical reserve applied on the trajectory side.
%   - separationTarget_units (nonnegative numeric scalar)
%       Required obstacle-side separation target.
%**************************************************************************
% OUTPUTS
%   - plane (scalar struct)
%       Updated offsets and a bound on the signed gap. Verified is true only
%       if every separation condition passes for the entire interval.
%**************************************************************************
% UNITS
%   - Position, offsets, target, reserve, and gap are coordinate units;
%     normals are dimensionless.
%**************************************************************************

%% Section 1: Bound Both Sides Of The Line Throughout The Interval

% Line-side value = normal x position + offset. The obstacle must stay
% above its target and the curve below its negative roundoff reserve.

if ismatrix(vertices_units)
    % With static vertices, each line-side value changes linearly.
    % Its minimum occurs at the start or end.
    minimumObstacleSide_units = min( ...
        vertices_units * plane.Normal.' + plane.Offset_units, [], "all");
else
    startVertices_units = vertices_units(:, :, 1);
    endVertices_units   = vertices_units(:, :, end);

    % A linearly moving vertex x a linearly changing normal gives a quadratic.
    % Its three Bernstein coefficients bound its value between the endpoints;
    % endpoint values alone could miss a smaller value during the interval.
    obstacleSideCoefficients_units = [startVertices_units * plane.Normal(1, :).' + plane.Offset_units(1), ...
        (startVertices_units * plane.Normal(2, :).' + endVertices_units * plane.Normal(1, :).' + ...
        sum(plane.Offset_units)) / 2, ...
        endVertices_units * plane.Normal(2, :).' + plane.Offset_units(2)];
    minimumObstacleSide_units = min(obstacleSideCoefficients_units, [], "all");
end

degree = size(controlPoint_units, 1) - 1;

% The degree-D curve x a degree-1 line normal gives degree D + 1.
% Its largest Bernstein coefficient bounds the curve-side value everywhere.
% Cache the product weights because they depend only on curve degree.
persistent cachedDegree cachedEndNormalWeights cachedStartNormalWeights
if isempty(cachedDegree) || cachedDegree ~= degree
    cachedDegree             = degree;
    cachedEndNormalWeights   = (0:degree + 1).' / (degree + 1);
    cachedStartNormalWeights = 1 - cachedEndNormalWeights;
end
endNormalWeights   = cachedEndNormalWeights;
startNormalWeights = cachedStartNormalWeights;

curveSideCoefficients_units = startNormalWeights .* [sum(controlPoint_units .* plane.Normal(1, :), 2); 0] + ...
    endNormalWeights .* [0; sum(controlPoint_units .* plane.Normal(2, :), 2)] + ...
    startNormalWeights * plane.Offset_units(1) + endNormalWeights * plane.Offset_units(2);
[maximumTrajectorySide_units, maximumNormalLength] = deal( ...
    max(curveSideCoefficients_units), max(vecnorm(plane.Normal, 2, 2)));

%% Section 2: Shift The Line When Possible And Apply All Gap Checks

% A common offset shift needs room for both required gaps:
% target - obstacleSide <= shift <= -reserve - curveSide. Calculate the
% rounding allowance only when that allowed shift interval is nonempty.
offsetRoundoffAllowance_units = 0;
if separationTarget_units - minimumObstacleSide_units <= ...
    -roundoffReserve_units - maximumTrajectorySide_units
    finiteCoordinates_units      = [plane.Offset_units(:); vertices_units(:); controlPoint_units(:)];
    finiteCoordinates_units      = abs(finiteCoordinates_units(isfinite(finiteCoordinates_units)));
    geometryScale_units          = max([1; finiteCoordinates_units]);
    offsetRoundoffAllowance_units = 16 * eps(geometryScale_units);
end
[plane.Offset_units, plane.SignedGap_units, plane.Verified] = bmtpEngine.separation.proveSeparation( ...
    minimumObstacleSide_units, maximumTrajectorySide_units, maximumNormalLength, ...
    plane.Offset_units, offsetRoundoffAllowance_units, roundoffReserve_units, separationTarget_units);
end
