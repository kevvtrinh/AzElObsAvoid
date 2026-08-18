function isTimeInvariant = packedAzElGeometryIsTimeInvariant(obstacleField)
%% Section 0: Header & Readme
% SYNTAX
%   isTimeInvariant = ...
%       azElInternal.packedAzElGeometryIsTimeInvariant(obstacleField)
%**************************************************************************
% PURPOSE
%   - Prove exact equality of protected packed geometry at every slice.
%**************************************************************************
% INPUTS
%   - obstacleField (scalar packed-obstacle struct)
%       Canonical AzElTimeObstacleField data.
%**************************************************************************
% OUTPUTS
%   - isTimeInvariant (logical scalar)
%       True only when every packed slice has identical geometry.
%**************************************************************************
% UNITS
%   - Packed positions and bounds use degrees.
%**************************************************************************

%% Section 1: Compare Every Packed Geometry Slice

isTimeInvariant = true;
for obstacleIndex = 1:numel(obstacleField.Obstacles)
    obstacle = obstacleField.Obstacles(obstacleIndex);
    if obstacle.SampleCount <= 1
        continue;
    end
    referenceVertexRows = packedRows(obstacle.SliceOffsets, 1);
    referenceEdgeRows = packedRows(obstacle.EdgeOffsets, 1);
    referenceAzimuth_deg = obstacle.AzimuthDeg(referenceVertexRows);
    referenceElevation_deg = obstacle.ElevationDeg(referenceVertexRows);
    referenceEdges_deg = [ ...
        obstacle.EdgeStartAzimuthDeg(referenceEdgeRows), ...
        obstacle.EdgeStartElevationDeg(referenceEdgeRows), ...
        obstacle.EdgeEndAzimuthDeg(referenceEdgeRows), ...
        obstacle.EdgeEndElevationDeg(referenceEdgeRows)];
    referenceBounds_deg = obstacle.BoundsDeg(1, :);
    for sampleIndex = 2:obstacle.SampleCount
        vertexRows = packedRows(obstacle.SliceOffsets, sampleIndex);
        edgeRows = packedRows(obstacle.EdgeOffsets, sampleIndex);
        edges_deg = [ ...
            obstacle.EdgeStartAzimuthDeg(edgeRows), ...
            obstacle.EdgeStartElevationDeg(edgeRows), ...
            obstacle.EdgeEndAzimuthDeg(edgeRows), ...
            obstacle.EdgeEndElevationDeg(edgeRows)];
        verticesMatch = isequaln( ...
            obstacle.AzimuthDeg(vertexRows), referenceAzimuth_deg) && ...
            isequaln(obstacle.ElevationDeg(vertexRows), ...
            referenceElevation_deg);
        edgesMatch = isequaln(edges_deg, referenceEdges_deg);
        boundsMatch = isequaln( ...
            obstacle.BoundsDeg(sampleIndex, :), referenceBounds_deg);
        if ~(verticesMatch && edgesMatch && boundsMatch)
            isTimeInvariant = false;
            return;
        end
    end
end
end

%% Section 2: Local Functions

function rows = packedRows(offsets, sampleIndex)
% PURPOSE
%   - Convert one packed CSR slice to a valid row-index vector.
firstRow = double(offsets(sampleIndex));
finalRow = double(offsets(sampleIndex + 1)) - 1;
rows = firstRow:finalRow;
end
