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
%   - Choose a line direction from obstacle edges and curve control points,
%     then check separation throughout the trajectory segment.
%   - When no direction has enough gap, return the best available direction
%     for the trajectory solver to try. It must still pass verification.
%**************************************************************************
% INPUTS
%   - controlPoint_units (N-by-2 numeric array)
%       Bezier control points for one trajectory segment.
%   - vertices_units (M-by-2 or M-by-2-by-2 numeric array)
%       Static vertices, or matching vertices at the start/end of linear motion.
%   - separationTarget_units (nonnegative numeric scalar)
%       Required obstacle-side separation target.
%   - roundoffReserve_units (nonnegative numeric scalar)
%       Numerical reserve applied on the trajectory side.
%   - obstacleGeometry (scalar struct, optional)
%       Saved edge normals and minimum vertex projections for these exact vertices.
%**************************************************************************
% OUTPUTS
%   - plane (scalar struct)
%       Line direction, offsets and checked gap. Verified reports whether the
%       curve and obstacle remain on the required sides throughout the segment.
%   - exitFlag (numeric scalar)
%       1 means a direction was constructed; -2 means none was available.
%       A value of 1 does not mean separation passed verification.
%   - lineDiagnostics (scalar struct)
%       Records that the line was constructed directly, without a solver.
%**************************************************************************
% UNITS
%   - Position, offsets, target, and reserve are coordinate units; normals
%     and normalized time are dimensionless.
%**************************************************************************

%% Section 1: Build Candidate Directions From Edges

startVertices_units = vertices_units(:, :, 1);
endVertices_units   = vertices_units(:, :, end);
controlPointCount   = size(controlPoint_units, 1);

% These weights and control-point pairs depend only on the curve degree.
% Reuse them for later calls with the same number of control points.
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

% Remove the obstacle centroid displacement when forming control-point
% edges. This expresses those edges relative to the obstacle motion.
movingFrameControls_units = controlPoint_units - ...
    controlFractions .* (mean(endVertices_units, 1) - mean(startVertices_units, 1));

% A normal is perpendicular to an edge. Try both signs because the curve
% can lie on either side. Projecting vertices onto a normal reduces each
% region to bounds along that direction.
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
    % Reuse the obstacle edge calculations; only the curve changes per call.
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

% For each direction, compare curve controls with the nearest obstacle bound.
% A positive gap means the curve controls lie on the other side of that bound.
curveMinusObstacleBound_units = controlPoint_units * candidateNormals.' - ...
    (1 - controlFractions) .* startObstacleBound_units - controlFractions .* endObstacleBound_units;
controlGaps_units = -max(curveMinusObstacleBound_units, [], 1);

% The verifier uses degree D + 1 product coefficients. Check those same
% coefficients before selecting a direction. If any direction has enough
% gap, exclude the others; otherwise keep the best available proposal.
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

% Offset the line so the nearest obstacle vertex meets the separation target.
% The final check also requires the curve-side reserve and a nonzero normal.

plane.Active       = true;
plane.ExitFlag     = 1;
exitFlag           = 1;
plane.Normal       = repmat(candidateNormals(directionIndex, :), 2, 1);
plane.Offset_units = separationTarget_units - ...
    [startObstacleBound_units(directionIndex), endObstacleBound_units(directionIndex)];
plane = bmtpEngine.separation.verifySeparatingLine( ...
    plane, controlPoint_units, vertices_units, roundoffReserve_units, separationTarget_units);
end
