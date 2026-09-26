function plane = verifySeparatingLine( ...
    plane, controlPoint_units, vertices_units, roundoffReserve_units, separationTarget_units)
%% Section 0: Header & Readme
% SYNTAX
%   plane = bmtpEngine.separation.verifySeparatingLine(plane, controlPoint_units, ...
%       vertices_units, roundoffReserve_units, separationTarget_units)
%**************************************************************************
% PURPOSE
%   - Check a proposed separating line over one full motion interval. The
%     line normal and offset may change linearly. Bound both the curve and
%     obstacle between endpoints, then apply the required margins.
%**************************************************************************
% INPUTS
%   - plane (scalar struct)
%       Candidate Normal and Offset_units at the interval start and end.
%   - controlPoint_units (N-by-2 numeric array)
%       Bezier controls of one motion curve in [x, y] rows.
%   - vertices_units (M-by-2 or M-by-2-by-2 numeric array)
%       Static polygon vertices, or matching vertices at the start and end
%       of linearly moving obstacle geometry in the third dimension.
%   - roundoffReserve_units (nonnegative numeric scalar)
%       Extra curve-side margin for numerical rounding.
%   - separationTarget_units (nonnegative numeric scalar)
%       Minimum required obstacle-side value at the line.
%**************************************************************************
% OUTPUTS
%   - plane (scalar struct)
%       Updated offsets and conservative SignedGap_units bound. Verified
%       is true only if all line-separation checks pass for the interval;
%       the complete motion still needs public independent validation.
%**************************************************************************
% UNITS
%   - Position, offsets, target, reserve, and gap are coordinate units;
%     normals are dimensionless.
%**************************************************************************

%% Section 1: Bound Both Sides Of The Line Throughout The Interval

% Line-side value = dot(normal, position) + offset. Keep the obstacle at
% or above its target and the curve at or below the negative reserve.

if ismatrix(vertices_units)
    % For a fixed vertex, the changing line gives a value linear in time.
    % Its minimum is at one end, so both endpoint lines suffice.
    minimumObstacleSide_units = min( ...
        vertices_units * plane.Normal.' + plane.Offset_units, [], "all");
else
    startVertices_units = vertices_units(:, :, 1);
    endVertices_units   = vertices_units(:, :, end);

    % A moving vertex dotted with a changing normal gives a quadratic.
    % Its three Bezier coefficients bound the whole interval; checking only
    % the endpoints could miss a lower obstacle-side value in the middle.
    obstacleSideCoefficients_units = [startVertices_units * plane.Normal(1, :).' + plane.Offset_units(1), ...
        (startVertices_units * plane.Normal(2, :).' + endVertices_units * plane.Normal(1, :).' + ...
        sum(plane.Offset_units)) / 2, ...
        endVertices_units * plane.Normal(2, :).' + plane.Offset_units(2)];
    minimumObstacleSide_units = min(obstacleSideCoefficients_units, [], "all");
end

degree = size(controlPoint_units, 1) - 1;

% A degree-D curve dotted with a linear normal gives degree D+1. Its
% largest Bezier coefficient bounds the curve-side value everywhere.
% Reuse weights for another curve with the same degree.
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

% Shifting both endpoint offsets equally can place the line between the
% bounds only if target - obstacleSide <= shift <= -reserve - curveSide.
% Seek extra rounding room only when that allowed range exists.
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
