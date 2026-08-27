function [active, hasEdge, edgeStart_deg, edgeEnd_deg] = ...
        queryBoundaryEdgeAtTime(obstacle, queryTime_s, edgeIndex)
%% Section 0: Header & Readme
% SYNTAX
%   [active, hasEdge, edgeStart_deg, edgeEnd_deg] = ...
%       obstacleAvoidance.obstacles.queryBoundaryEdgeAtTime( ...
%       obstacle, queryTime_s, edgeIndex)
%**************************************************************************
% PURPOSE
%   - Query one canonical boundary edge without reconstructing every edge.
%**************************************************************************
% INPUTS
%   - obstacle (scalar prepared obstacle struct)
%       InternalPreparation must come from prepareDynamic.
%   - queryTime_s (finite numeric scalar)
%       Absolute obstacle query time.
%   - edgeIndex (positive integer scalar)
%       Canonical edge row selected by the spatial corridor.
%**************************************************************************
% OUTPUTS
%   - active (logical scalar)
%       True when the obstacle has active query-time geometry.
%   - hasEdge (logical scalar)
%       True when edgeIndex exists in the canonical boundary ordering.
%   - edgeStart_deg, edgeEnd_deg (one-by-two numeric rows or 0-by-2)
%       Selected directed edge endpoints. Empty rows report no edge.
%**************************************************************************
% UNITS
%   - Geometry is degrees and time is seconds.
%**************************************************************************

%% Section 1: Select The Authoritative Boundary Source

% This hot internal query mirrors shapeAtTime's exact source-time and
% interval rules. Finite ring bounds were prepared once, while query-time
% closure remains exact because interpolation can open or close a ring.
preparation = obstacle.InternalPreparation;
time_s = double(obstacle.time_s(:));
active = false;
hasEdge = false;
edgeStart_deg = zeros(0, 2);
edgeEnd_deg = zeros(0, 2);
if ~isnumeric(queryTime_s) || ~isscalar(queryTime_s) || ...
        ~isreal(queryTime_s) || ~isfinite(queryTime_s)
    error("queryBoundaryEdgeAtTime:InvalidQueryTime", ...
        "queryTime_s must be one finite real numeric scalar.");
end
queryTime_s = double(queryTime_s);
if ~isnumeric(edgeIndex) || ~isscalar(edgeIndex) || ...
        ~isreal(edgeIndex) || ~isfinite(edgeIndex) || ...
        edgeIndex < 1 || edgeIndex ~= fix(edgeIndex)
    error("queryBoundaryEdgeAtTime:InvalidEdgeIndex", ...
        "edgeIndex must be one finite positive integer numeric scalar.");
end
edgeIndex = double(edgeIndex);
if isempty(time_s)
    return;
end
hasCompleteEdgeCache = ...
    isfield(preparation, "SampleBoundaryRunBounds") && ...
    isfield(preparation, "IntervalUnionBoundaryRunBounds") && ...
    isfield(preparation, "SelectedEdgeQueryIsExact");
if ~hasCompleteEdgeCache || ~preparation.SelectedEdgeQueryIsExact
    % Results saved by an older release can retain a valid preparation cache
    % without ring bounds. Preserve their exact full-boundary query behavior.
    [~, geometry] = obstacleAvoidance.obstacles.shapeAtTime( ...
        obstacle, queryTime_s, true);
    active = geometry.Active;
    if ~active
        return;
    end
    [edgeStartRows_deg, edgeEndRows_deg] = ...
        obstacleAvoidance.geometry.canonicalBoundaryToEdges(geometry);
    if edgeIndex <= size(edgeStartRows_deg, 1)
        edgeStart_deg = edgeStartRows_deg(edgeIndex, :);
        edgeEnd_deg = edgeEndRows_deg(edgeIndex, :);
        hasEdge = true;
    end
    return;
end
if isscalar(time_s)
    lowerIndex = 1;
    upperIndex = 1;
    fraction = 0;
elseif queryTime_s < time_s(1) || queryTime_s > time_s(end)
    return;
else
    upperIndex = find(time_s >= queryTime_s, 1, "first");
    if time_s(upperIndex) == queryTime_s || upperIndex == 1
        lowerIndex = upperIndex;
        fraction = 0;
    else
        lowerIndex = upperIndex - 1;
        intervalDuration_s = time_s(upperIndex) - time_s(lowerIndex);
        fraction = (queryTime_s - time_s(lowerIndex)) / ...
            intervalDuration_s;
    end
end

isInterpolated = false;
if lowerIndex == upperIndex
    azimuth_deg = double(obstacle.az_deg{lowerIndex}(:));
    elevation_deg = double(obstacle.el_deg{lowerIndex}(:));
    runBounds = preparation.SampleBoundaryRunBounds{lowerIndex};
elseif preparation.MatchingTopology(lowerIndex)
    azimuth_deg = double(obstacle.az_deg{lowerIndex}(:));
    elevation_deg = double(obstacle.el_deg{lowerIndex}(:));
    azimuthDelta_deg = preparation.DeltaAzimuth_deg{lowerIndex};
    elevationDelta_deg = preparation.DeltaElevation_deg{lowerIndex};
    runBounds = preparation.SampleBoundaryRunBounds{lowerIndex};
    isInterpolated = true;
else
    azimuth_deg = ...
        preparation.IntervalUnionAzimuth_deg{lowerIndex};
    elevation_deg = ...
        preparation.IntervalUnionElevation_deg{lowerIndex};
    runBounds = ...
        preparation.IntervalUnionBoundaryRunBounds{lowerIndex};
end

finiteVertexCount = sum(runBounds(:, 2) - runBounds(:, 1) + 1);
active = finiteVertexCount >= 3;
if ~active
    return;
end

%% Section 2: Resolve The Canonical Edge Row

% canonicalBoundaryToEdges removes an exactly repeated closing vertex and
% closes every remaining ring. Resolve that ordering from ring bounds, then
% evaluate only the two coordinates owned by the requested edge.
remainingEdgeIndex = edgeIndex;
for regionIndex = 1:size(runBounds, 1)
    regionStart = runBounds(regionIndex, 1);
    regionStop = runBounds(regionIndex, 2);
    regionVertexCount = regionStop - regionStart + 1;
    if regionVertexCount < 2
        continue;
    end
    firstAzimuth_deg = azimuth_deg(regionStart);
    firstElevation_deg = elevation_deg(regionStart);
    lastAzimuth_deg = azimuth_deg(regionStop);
    lastElevation_deg = elevation_deg(regionStop);
    if isInterpolated
        firstAzimuth_deg = firstAzimuth_deg + ...
            fraction * azimuthDelta_deg(regionStart);
        firstElevation_deg = firstElevation_deg + ...
            fraction * elevationDelta_deg(regionStart);
        lastAzimuth_deg = lastAzimuth_deg + ...
            fraction * azimuthDelta_deg(regionStop);
        lastElevation_deg = lastElevation_deg + ...
            fraction * elevationDelta_deg(regionStop);
    end
    isExplicitlyClosed = firstAzimuth_deg == lastAzimuth_deg && ...
        firstElevation_deg == lastElevation_deg;
    regionEdgeCount = regionVertexCount - isExplicitlyClosed;
    if remainingEdgeIndex > regionEdgeCount
        remainingEdgeIndex = remainingEdgeIndex - regionEdgeCount;
        continue;
    end
    startRow = regionStart + remainingEdgeIndex - 1;
    if remainingEdgeIndex < regionEdgeCount
        endRow = startRow + 1;
    else
        endRow = regionStart;
    end
    edgeStart_deg = [azimuth_deg(startRow), elevation_deg(startRow)];
    edgeEnd_deg = [azimuth_deg(endRow), elevation_deg(endRow)];
    if isInterpolated
        edgeStart_deg = edgeStart_deg + fraction * [ ...
            azimuthDelta_deg(startRow), elevationDelta_deg(startRow)];
        edgeEnd_deg = edgeEnd_deg + fraction * [ ...
            azimuthDelta_deg(endRow), elevationDelta_deg(endRow)];
    end
    hasEdge = true;
    return;
end
end
