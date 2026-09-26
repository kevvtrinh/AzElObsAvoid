function [plane, exitFlag, lineDiagnostics] = solveSeparatingLine(controlPoint_units, vertices_units, ...
    separationTarget_units, roundoffReserve_units, obstacleGeometry)
%% Section 0: Header & Readme
% SYNTAX
%   [plane, exitFlag, lineDiagnostics] = bmtpEngine.separation.solveSeparatingLine( ...
%       controlPoint_units, vertices_units, separationTarget_units, roundoffReserve_units)
%   [plane, exitFlag, lineDiagnostics] = bmtpEngine.separation.solveSeparatingLine( ...
%       controlPoint_units, vertices_units, separationTarget_units, ...
%       roundoffReserve_units, obstacleGeometry)
%**************************************************************************
% PURPOSE
%   - Try line directions perpendicular to obstacle and curve-control
%     edges. Keep one normal direction through the segment, allow its
%     offset to move with the obstacle, and check the whole curve.
%   - If no direction proves enough gap, return the best available
%     direction as a proposal. A proposal is not verified clearance.
%**************************************************************************
% INPUTS
%   - controlPoint_units (N-by-2 numeric array)
%       Bezier controls for one motion segment, in [x, y] rows.
%   - vertices_units (M-by-2 or M-by-2-by-2 numeric array)
%       Static polygon vertices, or matching vertices at the start and end
%       of linear region motion in the third dimension.
%   - separationTarget_units (nonnegative numeric scalar)
%       Minimum required obstacle-side value at the line.
%   - roundoffReserve_units (nonnegative numeric scalar)
%       Extra curve-side margin checked for numerical rounding.
%   - obstacleGeometry (scalar struct, optional)
%       Saved obstacle edge normals and vertex projections for these exact
%       vertices. Omit to calculate them for this call.
%**************************************************************************
% OUTPUTS
%   - plane (scalar struct)
%       Selected normal, endpoint offsets, and checked gap. Active means a
%       direction was constructed; Verified means the full separation
%       check passed.
%   - exitFlag (numeric scalar)
%       1 means a direction was constructed; -2 means none was available.
%       A value of 1 does not mean separation passed verification.
%   - lineDiagnostics (scalar struct)
%       IsAnalytic is true and TotalTime_s is zero; this direction search
%       does not call a numerical optimization solver.
%**************************************************************************
% UNITS
%   - Position, offsets, target, and reserve are coordinate units; normals
%     and normalized time are dimensionless.
%**************************************************************************

%% Section 1: Build Candidate Directions From Edges

startVertices_units = vertices_units(:, :, 1);
endVertices_units   = vertices_units(:, :, end);
controlPointCount   = size(controlPoint_units, 1);

% Fraction weights and every pair of curve controls depend only on the
% curve degree. Cache them for later calls with the same control count.
persistent cachedControlPointCount cachedControlFractions cachedProductFractions ...
    cachedSecondControlIndices cachedFirstControlIndices cachedEmptyPlane cachedLineDiagnostics
if isempty(cachedControlPointCount) || cachedControlPointCount ~= controlPointCount
    cachedControlPointCount = controlPointCount;
    cachedControlFractions  = (0:controlPointCount - 1)' / (controlPointCount - 1);
    cachedProductFractions  = (0:controlPointCount)' / controlPointCount;
    [cachedSecondControlIndices, cachedFirstControlIndices] = ...
        find(tril(true(controlPointCount), -1));
end
if isempty(cachedEmptyPlane)
    cachedEmptyPlane      = bmtpEngine.separation.createEmptyPlane();
    cachedLineDiagnostics = struct( ...
        'TotalTime_s', 0, ...
        'IsAnalytic',  true, ...
        'message',     'Convex supporting-axis subproblem.');
end
controlFractions     = cachedControlFractions;
productFractions     = cachedProductFractions;
secondControlIndices = cachedSecondControlIndices;
firstControlIndices  = cachedFirstControlIndices;

% Subtract the obstacle centroid's start-to-end displacement before
% forming control-to-control edges. Their directions then describe curve
% movement relative to that overall obstacle translation.
movingFrameControls_units = controlPoint_units - ...
    controlFractions .* (mean(endVertices_units, 1) - mean(startVertices_units, 1));

% A line normal is perpendicular to an edge. Try both signs because the
% curve could be on either side of the obstacle. For each normal, project
% obstacle vertices onto it and keep the nearest obstacle-side value.
if nargin < 5 || isempty(obstacleGeometry)
    edges_units = diff([startVertices_units; startVertices_units(1, :)], 1, 1);
    if size(vertices_units, 3) > 1
        edges_units = [edges_units; diff([endVertices_units; endVertices_units(1, :)], 1, 1)];
    end
    edges_units = [edges_units; ...
        movingFrameControls_units(secondControlIndices, :) - movingFrameControls_units(firstControlIndices, :)];
    edgeLength_units         = vecnorm(edges_units, 2, 2);
    edges_units              = edges_units(edgeLength_units > 0, :);
    edgeLength_units         = edgeLength_units(edgeLength_units > 0);
    candidateNormals         = [-edges_units(:, 2), edges_units(:, 1)] ./ edgeLength_units;
    candidateNormals         = [candidateNormals; -candidateNormals];
    startObstacleBound_units = min(startVertices_units * candidateNormals.', [], 1);
    endObstacleBound_units   = min(endVertices_units * candidateNormals.', [], 1);
else
    % Prepared obstacle edge directions and their vertex projections are
    % reusable when only the candidate curve changes between calls.
    positiveObstacleNormals     = obstacleGeometry.PositiveNormals;
    startObstaclePositive_units = obstacleGeometry.FirstPositiveSupport_units;
    endObstaclePositive_units   = obstacleGeometry.LastPositiveSupport_units;
    startObstacleNegative_units = obstacleGeometry.FirstNegativeSupport_units;
    endObstacleNegative_units   = obstacleGeometry.LastNegativeSupport_units;
    controlEdges_units          = movingFrameControls_units(secondControlIndices, :) - ...
        movingFrameControls_units(firstControlIndices, :);
    controlEdgeLength_units = vecnorm(controlEdges_units, 2, 2);
    controlEdges_units      = controlEdges_units(controlEdgeLength_units > 0, :);
    controlEdgeLength_units = controlEdgeLength_units(controlEdgeLength_units > 0);
    positiveControlNormals  = [-controlEdges_units(:, 2), controlEdges_units(:, 1)] ./ controlEdgeLength_units;
    positiveNormals         = [positiveObstacleNormals; positiveControlNormals];
    candidateNormals        = [positiveNormals; -positiveNormals];

    startProjectionOnControlNormals_units = startVertices_units * positiveControlNormals.';
    endProjectionOnControlNormals_units   = endVertices_units * positiveControlNormals.';

    startControlPositive_units = min(startProjectionOnControlNormals_units, [], 1);
    endControlPositive_units   = min(endProjectionOnControlNormals_units, [], 1);
    startControlNegative_units = -max(startProjectionOnControlNormals_units, [], 1);
    endControlNegative_units   = -max(endProjectionOnControlNormals_units, [], 1);

    startObstacleBound_units = [startObstaclePositive_units, startControlPositive_units, ...
        startObstacleNegative_units, startControlNegative_units];
    endObstacleBound_units = [endObstaclePositive_units, endControlPositive_units, ...
        endObstacleNegative_units, endControlNegative_units];
end

%% Section 2: Select The Direction With The Largest Usable Gap

% For each direction, compare each curve control with the obstacle bound
% at the same fraction. A positive gap puts all controls on the curve side.
% This control gap ranks candidate directions; it is not the final proof.
curveMinusObstacleBound_units = controlPoint_units * candidateNormals.' - ...
    (1 - controlFractions) .* startObstacleBound_units - controlFractions .* endObstacleBound_units;
controlGaps_units = -max(curveMinusObstacleBound_units, [], 1);

% The final line-side polynomial has degree D+1. Bound its D+2 Bezier
% coefficients, just as the verifier will. If any direction already has
% enough gap, choose only among those; otherwise keep the best control-gap
% direction as an unverified proposal for the trajectory solver.
productGaps_units = -max((1 - productFractions) .* ...
    [curveMinusObstacleBound_units; zeros(1, size(candidateNormals, 1))] + ...
    productFractions .* [zeros(1, size(candidateNormals, 1)); curveMinusObstacleBound_units], [], 1);
directionIsProvable = productGaps_units >= separationTarget_units + roundoffReserve_units;
if any(directionIsProvable)
    controlGaps_units(~directionIsProvable) = -Inf;
end
[selectedGap_units, directionIndex] = max(controlGaps_units);

plane           = cachedEmptyPlane;
plane.ExitFlag  = -2;
exitFlag        = -2;
lineDiagnostics = cachedLineDiagnostics;
if isempty(selectedGap_units)
    return
end

%% Section 3: Place The Line And Verify The Whole Segment

% Place the line so the nearest obstacle vertex meets the target at each
% endpoint. The offsets interpolate between them. Verification still
% checks the curve-side reserve, full obstacle motion, and normal length.

plane.Active       = true;
plane.ExitFlag     = 1;
exitFlag           = 1;
plane.Normal       = repmat(candidateNormals(directionIndex, :), 2, 1);
plane.Offset_units = separationTarget_units - ...
    [startObstacleBound_units(directionIndex), endObstacleBound_units(directionIndex)];
plane = bmtpEngine.separation.verifySeparatingLine( ...
    plane, controlPoint_units, vertices_units, roundoffReserve_units, separationTarget_units);
end
