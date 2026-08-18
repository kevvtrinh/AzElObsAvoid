function inequality = evaluateAzElHs3CorridorConstraints( ...
        solution, meshTau, corridor, obstacleField, initialTime_s, ...
        clearance_deg)
%% Section 0: Header & Readme
% SYNTAX
%   inequality = azElInternal.evaluateAzElHs3CorridorConstraints( ...
%       solution, meshTau, corridor, obstacleField, initialTime_s, ...
%       clearance_deg)
%**************************************************************************
% PURPOSE
%   - Evaluate point and complete-segment HS-3 obstacle separators.
%**************************************************************************
% INPUTS
%   - solution (scalar struct)
%       HS-3 state, control, and time data.
%   - meshTau (N-by-1 numeric vector)
%       Strictly increasing normalized knot times.
%   - corridor (scalar struct)
%       Frozen obstacle edges, side signs, active mask, and clearances.
%   - obstacleField (scalar packed-obstacle struct)
%       Canonical original or safety-adjusted obstacle geometry.
%   - initialTime_s (finite numeric scalar)
%       Absolute initial motion time.
%   - clearance_deg (nonnegative numeric scalar)
%       Base obstacle clearance required by each separator.
%**************************************************************************
% OUTPUTS
%   - inequality (M-by-1 numeric vector)
%       Feasible constraints have values less than or equal to zero.
%**************************************************************************
% UNITS
%   - Position and clearance are degrees. Time is seconds.
%**************************************************************************

%% Section 1: Evaluate Point Separators

[pointState, ~] = azElInternal.sampleAzElHs3Solution( ...
    solution, meshTau, corridor.PointTau);
pointPosition_deg = pointState(:, 1:2);
pointTime_s = initialTime_s + ...
    (solution.FinalTime_s - initialTime_s) * corridor.PointTau;
activeCount = nnz(corridor.Active);
inequality = -ones(activeCount, 1);
pointClearance_deg = clearance_deg * ...
    ones(numel(corridor.PointTau), 1);
if isfield(corridor, "PointClearance_deg") && ...
        numel(corridor.PointClearance_deg) == numel(corridor.PointTau)
    pointClearance_deg = max( ...
        pointClearance_deg, corridor.PointClearance_deg(:));
end
useSelectedStaticEdge = isfield(corridor, "GeometryTimeInvariant") && ...
    corridor.GeometryTimeInvariant && ...
    isfield(corridor, "SelectedEdgeStart_deg");
if useSelectedStaticEdge
    edgeSets_deg = cell(0, 0);
else
    edgeSets_deg = azElInternal.interpolateAzElObstacleEdges( ...
        obstacleField, pointTime_s);
end
constraintIndex = 0;
for pointIndex = 1:numel(corridor.PointTau)
    for obstacleIndex = 1:numel(obstacleField.Obstacles)
        if ~corridor.Active(pointIndex, obstacleIndex)
            continue;
        end
        constraintIndex = constraintIndex + 1;
        if useSelectedStaticEdge
            edgeStart_deg = reshape(corridor.SelectedEdgeStart_deg( ...
                pointIndex, obstacleIndex, :), 1, 2);
            normal = reshape(corridor.SelectedNormal( ...
                pointIndex, obstacleIndex, :), 1, 2);
        else
            edges_deg = edgeSets_deg{pointIndex, obstacleIndex};
            if isempty(edges_deg)
                inequality(constraintIndex) = -1;
                continue;
            end
            selectedEdgeIndex = min(size(edges_deg, 1), max(1, ...
                corridor.EdgeIndex(pointIndex, obstacleIndex)));
            edge_deg = edges_deg(selectedEdgeIndex, :);
            tangent_deg = edge_deg(3:4) - edge_deg(1:2);
            leftNormal = [-tangent_deg(2), tangent_deg(1)];
            normalLength = norm(leftNormal);
            if normalLength <= eps
                inequality(constraintIndex) = clearance_deg;
                continue;
            end
            normal = corridor.SideSign(pointIndex, obstacleIndex) * ...
                leftNormal / normalLength;
            edgeStart_deg = edge_deg(1:2);
        end
        separation_deg = dot( ...
            pointPosition_deg(pointIndex, :) - edgeStart_deg, normal);
        inequality(constraintIndex) = ...
            pointClearance_deg(pointIndex) - separation_deg;
    end
end

%% Section 2: Add Complete Static-Segment Separators

inequality = [inequality; staticSegmentCorridorConstraints( ...
    solution, meshTau, corridor, obstacleField, clearance_deg)];
end

%% Section 3: Local Functions

function inequality = staticSegmentCorridorConstraints( ...
        solution, meshTau, corridor, obstacleField, clearance_deg)
% PURPOSE
%   - Keep each static HS-3 segment inside its visibility half-space.
segmentCount = numel(meshTau) - 1;
obstacleCount = numel(obstacleField.Obstacles);
isStaticCorridor = isfield(corridor, "GeometryTimeInvariant") && ...
    corridor.GeometryTimeInvariant && ...
    isfield(corridor, "SelectedEdgeStart_deg");
if ~isStaticCorridor || obstacleCount == 0
    inequality = zeros(0, 1);
    return;
end
tauTolerance = 64 * eps(max(1, max(abs(meshTau))));
polynomialConstraintRow = false(numel(corridor.PointTau), 1);
for segmentIndex = 1:segmentCount
    midpointTau = 0.5 * ...
        (meshTau(segmentIndex) + meshTau(segmentIndex + 1));
    [midpointDistance, midpointRow] = min(abs( ...
        corridor.PointTau - midpointTau));
    if midpointDistance <= tauTolerance
        polynomialConstraintRow(midpointRow) = true;
    end
end
inequality = zeros(6 * nnz(polynomialConstraintRow) * ...
    obstacleCount, 1);
constraintIndex = 0;
duration_s = solution.FinalTime_s - solution.InitialTime_s;
for segmentIndex = 1:segmentCount
    segmentDuration_s = duration_s * ...
        (meshTau(segmentIndex + 1) - meshTau(segmentIndex));
    statePower = azElInternal.buildAzElHs3SegmentPolynomials( ...
        solution.KnotState(segmentIndex, :), ...
        solution.KnotControl(segmentIndex, :), ...
        solution.MidpointControl(segmentIndex, :), ...
        solution.KnotControl(segmentIndex + 1, :), ...
        segmentDuration_s);
    positionBernstein_deg = ...
        azElInternal.powerToBernstein(statePower(:, 1:2));
    if segmentIndex < segmentCount
        rowIsInSegment = corridor.PointTau >= ...
            meshTau(segmentIndex) - tauTolerance & ...
            corridor.PointTau < ...
            meshTau(segmentIndex + 1) - tauTolerance;
    else
        rowIsInSegment = corridor.PointTau >= ...
            meshTau(segmentIndex) - tauTolerance & ...
            corridor.PointTau <= ...
            meshTau(segmentIndex + 1) + tauTolerance;
    end
    constraintRows = find(polynomialConstraintRow & rowIsInSegment);
    for corridorRow = reshape(constraintRows, 1, [])
        for obstacleIndex = 1:obstacleCount
            if ~corridor.Active(corridorRow, obstacleIndex)
                continue;
            end
            edgeStart_deg = reshape( ...
                corridor.SelectedEdgeStart_deg( ...
                corridorRow, obstacleIndex, :), 1, 2);
            normal = reshape(corridor.SelectedNormal( ...
                corridorRow, obstacleIndex, :), 1, 2);
            separation_deg = ...
                (positionBernstein_deg - edgeStart_deg) * normal.';
            polynomialClearance_deg = clearance_deg;
            if isfield(corridor, "PointClearance_deg") && ...
                    numel(corridor.PointClearance_deg) >= corridorRow
                polynomialClearance_deg = max( ...
                    polynomialClearance_deg, ...
                    corridor.PointClearance_deg(corridorRow));
            end
            row = constraintIndex + (1:6);
            inequality(row) = ...
                polynomialClearance_deg - separation_deg;
            constraintIndex = constraintIndex + 6;
        end
    end
end
inequality = inequality(1:constraintIndex);
end
