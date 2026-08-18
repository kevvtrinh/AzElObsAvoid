function edgeSets_deg = interpolateAzElObstacleEdges( ...
        obstacleField, queryTime_s)
%% Section 0: Header & Readme
% SYNTAX
%   edgeSets_deg = azElInternal.interpolateAzElObstacleEdges( ...
%       obstacleField, queryTime_s)
%**************************************************************************
% PURPOSE
%   - Interpolate all packed obstacle edges at one or more query times.
%   - Keep packed-array access inside one batch call for low overhead.
%**************************************************************************
% INPUTS
%   - obstacleField (scalar packed-obstacle struct)
%       Contains the canonical Obstacles structure array.
%   - queryTime_s (N-by-1 finite numeric vector)
%       Times at which the obstacle geometry is required.
%**************************************************************************
% OUTPUTS
%   - edgeSets_deg (N-by-M cell array)
%       Each cell contains K-by-4 rows [start azimuth, start elevation,
%       end azimuth, end elevation] for one time and one obstacle.
%**************************************************************************
% UNITS
%   - Position is degrees. Time is seconds.
%**************************************************************************

%% Section 1: Interpolate Each Obstacle In One Batch

queryTime_s = double(queryTime_s(:));
pointCount = numel(queryTime_s);
obstacleCount = numel(obstacleField.Obstacles);
edgeSets_deg = cell(pointCount, obstacleCount);
for obstacleIndex = 1:obstacleCount
    packedObstacle = obstacleField.Obstacles(obstacleIndex);
    obstacleTime_s = double(packedObstacle.TimeSeconds(:));
    sourceEdgeSets_deg = cell(numel(obstacleTime_s), 1);
    for sliceIndex = 1:numel(obstacleTime_s)
        sourceEdgeSets_deg{sliceIndex} = packedSliceEdges( ...
            packedObstacle, sliceIndex);
    end
    for pointIndex = 1:pointCount
        edgeSets_deg{pointIndex, obstacleIndex} = edgeSetAtTime( ...
            packedObstacle, sourceEdgeSets_deg, obstacleTime_s, ...
            queryTime_s(pointIndex));
    end
end
end

%% Section 2: Local Functions

function edges_deg = edgeSetAtTime( ...
        packedObstacle, sourceEdgeSets_deg, obstacleTime_s, queryTime_s)
% PURPOSE
%   - Select or interpolate one edge set with the canonical topology rule.
if isempty(obstacleTime_s)
    edges_deg = zeros(0, 4);
    return;
end
upperSliceIndex = find(obstacleTime_s >= queryTime_s, 1, "first");
if isempty(upperSliceIndex)
    upperSliceIndex = numel(obstacleTime_s);
end
lowerSliceIndex = find(obstacleTime_s <= queryTime_s, 1, "last");
if isempty(lowerSliceIndex)
    lowerSliceIndex = 1;
end
if lowerSliceIndex == upperSliceIndex
    edges_deg = sourceEdgeSets_deg{lowerSliceIndex};
    return;
end
fraction = (queryTime_s - obstacleTime_s(lowerSliceIndex)) / ...
    (obstacleTime_s(upperSliceIndex) - obstacleTime_s(lowerSliceIndex));
fraction = min(1, max(0, fraction));
firstEdges_deg = sourceEdgeSets_deg{lowerSliceIndex};
lastEdges_deg = sourceEdgeSets_deg{upperSliceIndex};
topologyMatches = size(firstEdges_deg, 1) == size(lastEdges_deg, 1);
if isfield(packedObstacle, "TopologyMatchesNext") && ...
        numel(packedObstacle.TopologyMatchesNext) >= lowerSliceIndex
    topologyMatches = topologyMatches && ...
        packedObstacle.TopologyMatchesNext(lowerSliceIndex);
end
if topologyMatches
    edges_deg = (1 - fraction) * firstEdges_deg + fraction * lastEdges_deg;
elseif fraction < 0.5
    edges_deg = firstEdges_deg;
else
    edges_deg = lastEdges_deg;
end
end

function edges_deg = packedSliceEdges(packedObstacle, sliceIndex)
% PURPOSE
%   - Read one packed edge slice without repeated package calls.
firstEdgeIndex = double(packedObstacle.EdgeOffsets(sliceIndex));
lastEdgeIndex = double(packedObstacle.EdgeOffsets(sliceIndex + 1)) - 1;
if lastEdgeIndex < firstEdgeIndex
    edges_deg = zeros(0, 4);
    return;
end
edgeRows = firstEdgeIndex:lastEdgeIndex;
edges_deg = [ ...
    double(packedObstacle.EdgeStartAzimuthDeg(edgeRows)), ...
    double(packedObstacle.EdgeStartElevationDeg(edgeRows)), ...
    double(packedObstacle.EdgeEndAzimuthDeg(edgeRows)), ...
    double(packedObstacle.EdgeEndElevationDeg(edgeRows))];
end
