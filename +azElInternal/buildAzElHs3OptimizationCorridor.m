function corridor = buildAzElHs3OptimizationCorridor( ...
        solution, meshTau, obstacleField, initialTime_s, ...
        referencePosition_deg, requestedPointTau)
%% Section 0: Header & Readme
% SYNTAX
%   corridor = azElInternal.buildAzElHs3OptimizationCorridor( ...
%       solution, meshTau, obstacleField, initialTime_s)
%   corridor = azElInternal.buildAzElHs3OptimizationCorridor( ...
%       solution, meshTau, obstacleField, initialTime_s, ...
%       referencePosition_deg)
%   corridor = azElInternal.buildAzElHs3OptimizationCorridor( ...
%       solution, meshTau, obstacleField, initialTime_s, ...
%       referencePosition_deg, requestedPointTau)
%**************************************************************************
% PURPOSE
%   - Freeze one nearby edge and one free-side separator for each active
%     obstacle and HS-3 corridor point.
%**************************************************************************
% INPUTS
%   - solution (scalar struct)
%       HS-3 state and control data with FinalTime_s.
%   - meshTau (N-by-1 numeric vector)
%       Strictly increasing normalized knot times.
%   - obstacleField (scalar packed-obstacle struct)
%       Canonical original or safety-adjusted obstacle geometry.
%   - initialTime_s (finite numeric scalar)
%       Absolute initial motion time.
%   - referencePosition_deg (M-by-2 numeric matrix, optional; default [])
%       Positions used to select separators. Empty input samples solution.
%   - requestedPointTau (M-by-1 numeric vector, optional; default [])
%       Exact corridor times. Empty input uses five points per segment.
%**************************************************************************
% OUTPUTS
%   - corridor (scalar struct)
%       Frozen edge indices, side signs, active mask, points, clearances,
%       selected static geometry, and activation distance.
%**************************************************************************
% UNITS
%   - Position and clearance are degrees. Time is seconds.
%**************************************************************************

%% Section 1: Resolve Corridor Points

if nargin >= 6 && ~isempty(requestedPointTau)
    pointTau = requestedPointTau(:);
    associationTau = pointTau;
else
    [pointTau, associationTau] = ...
        azElInternal.azElHs3CorridorTau(meshTau);
end
if nargin >= 5 && ~isempty(referencePosition_deg)
    pointPosition_deg = referencePosition_deg;
else
    [pointState, ~] = azElInternal.sampleAzElHs3Solution( ...
        solution, meshTau, associationTau);
    pointPosition_deg = pointState(:, 1:2);
end
if size(pointPosition_deg, 1) ~= numel(pointTau)
    error("buildAzElHs3OptimizationCorridor:CorridorPointSizeMismatch", ...
        "referencePosition_deg must have one row for each corridor time.");
end
pointTime_s = initialTime_s + ...
    (solution.FinalTime_s - initialTime_s) * pointTau;
pointCount = numel(pointTau);
obstacleCount = numel(obstacleField.Obstacles);

%% Section 2: Set The Local Activation Distance

orderedPoint = sortrows([pointTau, pointPosition_deg], 1);
orderedStep_deg = vecnorm(diff(orderedPoint(:, 2:3), 1, 1), 2, 2);
orderedStep_deg = orderedStep_deg(orderedStep_deg > 1e-10);
if isempty(orderedStep_deg)
    corridorActivationDistance_deg = Inf;
else
    % One local sample spacing keeps the route boundary and excludes
    % unrelated half-spaces from remote sections of the same obstacle.
    corridorActivationDistance_deg = 1.25 * median(orderedStep_deg);
end

%% Section 3: Select Edge Separators

edgeSets_deg = azElInternal.interpolateAzElObstacleEdges( ...
    obstacleField, pointTime_s);
edgeIndex = zeros(pointCount, obstacleCount);
sideSign = ones(pointCount, obstacleCount);
active = false(pointCount, obstacleCount);
selectedEdgeStart_deg = zeros(pointCount, obstacleCount, 2);
selectedNormal = zeros(pointCount, obstacleCount, 2);
for pointIndex = 1:pointCount
    queryPoint_deg = pointPosition_deg(pointIndex, :);
    for obstacleIndex = 1:obstacleCount
        edges_deg = edgeSets_deg{pointIndex, obstacleIndex};
        if isempty(edges_deg)
            continue;
        end
        [nearestEdgeIndex, nearestPoint_deg] = ...
            nearestEdge(queryPoint_deg, edges_deg);
        edge_deg = edges_deg(nearestEdgeIndex, :);
        tangent_deg = edge_deg(3:4) - edge_deg(1:2);
        leftNormal = [-tangent_deg(2), tangent_deg(1)];
        normalLength = norm(leftNormal);
        if normalLength <= eps
            continue;
        end
        leftNormal = leftNormal / normalLength;
        signedSide = dot(queryPoint_deg - nearestPoint_deg, leftNormal);
        if abs(signedSide) <= 1e-12
            signedSide = dot(queryPoint_deg - edge_deg(1:2), leftNormal);
        end
        selectedSideSign = sign(signedSide);
        if selectedSideSign == 0
            selectedSideSign = 1;
        end
        pointIsInside = pointInsideEdges(queryPoint_deg, edges_deg);
        if pointIsInside
            selectedSideSign = -selectedSideSign;
        end
        edgeIndex(pointIndex, obstacleIndex) = nearestEdgeIndex;
        sideSign(pointIndex, obstacleIndex) = selectedSideSign;
        selectedEdgeStart_deg(pointIndex, obstacleIndex, :) = ...
            edge_deg(1:2);
        selectedNormal(pointIndex, obstacleIndex, :) = ...
            selectedSideSign * leftNormal / normalLength;
        pointDistance_deg = norm(queryPoint_deg - nearestPoint_deg);
        active(pointIndex, obstacleIndex) = pointIsInside || ...
            pointDistance_deg <= corridorActivationDistance_deg;
    end
end

%% Section 4: Assemble The Corridor

corridor = struct( ...
    "EdgeIndex", edgeIndex, ...
    "SideSign", sideSign, ...
    "Active", active, ...
    "PointTau", pointTau, ...
    "PointClearance_deg", zeros(pointCount, 1), ...
    "SelectedEdgeStart_deg", selectedEdgeStart_deg, ...
    "SelectedNormal", selectedNormal, ...
    "ActivationDistance_deg", corridorActivationDistance_deg);
end

%% Section 5: Local Functions

function [edgeIndex, nearestPoint_deg] = nearestEdge(point_deg, edges_deg)
% PURPOSE
%   - Select the deterministic closest edge and its projection.
edgeStart_deg = edges_deg(:, 1:2);
edgeDelta_deg = edges_deg(:, 3:4) - edgeStart_deg;
edgeLengthSquared_deg2 = sum(edgeDelta_deg .^ 2, 2);
projectionFraction = sum((point_deg - edgeStart_deg) .* ...
    edgeDelta_deg, 2) ./ max(edgeLengthSquared_deg2, eps);
projectionFraction = min(1, max(0, projectionFraction));
projection_deg = edgeStart_deg + projectionFraction .* edgeDelta_deg;
distanceSquared_deg2 = sum((projection_deg - point_deg) .^ 2, 2);
[~, edgeIndex] = min(distanceSquared_deg2);
nearestPoint_deg = projection_deg(edgeIndex, :);
end

function inside = pointInsideEdges(point_deg, edges_deg)
% PURPOSE
%   - Classify one point for separator orientation before optimization.
y0 = edges_deg(:, 2);
y1 = edges_deg(:, 4);
crossesElevation = (y0 > point_deg(2)) ~= (y1 > point_deg(2));
crossingAzimuth_deg = edges_deg(:, 1) + ...
    (point_deg(2) - y0) .* (edges_deg(:, 3) - edges_deg(:, 1)) ./ ...
    max(abs(y1 - y0), eps) .* sign(y1 - y0 + eps);
inside = mod(nnz(crossesElevation & ...
    crossingAzimuth_deg > point_deg(1)), 2) == 1;
end
