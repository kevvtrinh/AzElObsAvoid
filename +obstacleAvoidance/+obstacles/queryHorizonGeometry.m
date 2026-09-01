function geometry = queryHorizonGeometry( ...
        obstacle, startTime_s, endTime_s)
%% Section 0: Header & Readme
% SYNTAX
%   geometry = obstacleAvoidance.obstacles.queryHorizonGeometry( ...
%       obstacle, startTime_s, endTime_s)
%**************************************************************************
% PURPOSE
%   - Create one conservative source-derived occupied set for a closed
%     request horizon without including unrelated history samples.
%**************************************************************************
% INPUTS
%   - obstacle (scalar canonical or prepared obstacle struct)
%       Protected sample and interval geometry remains authoritative.
%   - startTime_s, endTime_s (finite numeric scalars)
%       Closed request interval with endTime_s not before startTime_s.
%**************************************************************************
% OUTPUTS
%   - geometry (scalar struct)
%       Active flag, conservative sweep shape and bounds, exact query times,
%       source sample indices, and intersecting source interval indices.
%**************************************************************************
% UNITS
%   - Boundary coordinates are degrees and times are seconds.
%**************************************************************************

%% Section 1: Select Applicable Source Geometry

validateattributes(startTime_s, {'numeric'}, ...
    {'real', 'finite', 'scalar'});
validateattributes(endTime_s, {'numeric'}, ...
    {'real', 'finite', 'scalar', '>=', startTime_s});
if ~isstruct(obstacle) || ~isscalar(obstacle)
    error("queryHorizonGeometry:InvalidObstacle", ...
        "obstacle must be one canonical obstacle record.");
end
obstacle = obstacleAvoidance.obstacles.prepareDynamic(obstacle);
preparation = obstacle.InternalPreparation;
sourceTime_s = double(obstacle.time_s(:));
sampleIndices = find(sourceTime_s >= startTime_s & ...
    sourceTime_s <= endTime_s);
queryTime_s = unique([startTime_s; sourceTime_s(sampleIndices); endTime_s]);
if numel(sourceTime_s) > 1
    queryTime_s = queryTime_s(queryTime_s >= sourceTime_s(1) & ...
        queryTime_s <= sourceTime_s(end));
end
intervalIndices = zeros(0, 1);
if numel(sourceTime_s) > 1 && endTime_s > startTime_s
    intervalIndices = find(sourceTime_s(1:end - 1) < endTime_s & ...
        sourceTime_s(2:end) > startTime_s);
end
componentShapes = cell(numel(queryTime_s) + numel(intervalIndices), 1);
componentCount = 0;
for queryIndex = 1:numel(queryTime_s)
    shape = obstacleAvoidance.obstacles.shapeAtTime( ...
        obstacle, queryTime_s(queryIndex));
    if ~isempty(shape.Vertices)
        componentCount = componentCount + 1;
        componentShapes{componentCount} = shape;
    end
end
for intervalIndex = reshape(intervalIndices, 1, [])
    if preparation.MatchingTopology(intervalIndex)
        continue;
    end
    intervalShape = preparation.IntervalUnionShapes{intervalIndex};
    if ~isempty(intervalShape.Vertices)
        componentCount = componentCount + 1;
        componentShapes{componentCount} = intervalShape;
    end
end
componentShapes = componentShapes(1:componentCount);

%% Section 2: Create The Conservative Request Sweep

sweepShape = polyshape();
bounds_deg = [Inf -Inf Inf -Inf];
if componentCount > 0
    vertices_deg = zeros(0, 2);
    for componentIndex = 1:componentCount
        componentVertices_deg = componentShapes{componentIndex}.Vertices;
        vertices_deg = [vertices_deg; componentVertices_deg( ...
            all(isfinite(componentVertices_deg), 2), :)]; %#ok<AGROW>
    end
    vertices_deg = unique(vertices_deg, "rows", "stable");
    if startTime_s == endTime_s && componentCount == 1
        sweepShape = componentShapes{1};
    elseif size(vertices_deg, 1) >= 3
        hullIndex = convhull(vertices_deg(:, 1), vertices_deg(:, 2));
        sweepShape = polyshape(vertices_deg(hullIndex(1:end - 1), :), ...
            "Simplify", false, "KeepCollinearPoints", true);
    end
    finiteVertices_deg = sweepShape.Vertices;
    finiteVertices_deg = finiteVertices_deg( ...
        all(isfinite(finiteVertices_deg), 2), :);
    if ~isempty(finiteVertices_deg)
        bounds_deg = [min(finiteVertices_deg(:, 1)), ...
            max(finiteVertices_deg(:, 1)), ...
            min(finiteVertices_deg(:, 2)), ...
            max(finiteVertices_deg(:, 2))];
    end
end
geometry = struct( ...
    "Active", ~isempty(sweepShape.Vertices), ...
    "StartTime_s", double(startTime_s), ...
    "EndTime_s", double(endTime_s), ...
    "QueryTime_s", queryTime_s, ...
    "SourceSampleIndices", sampleIndices, ...
    "SourceIntervalIndices", intervalIndices, ...
    "SweepShape", sweepShape, ...
    "Bounds_deg", bounds_deg, ...
    "ComponentCount", componentCount, ...
    "ContainmentBasis", ...
    "exact endpoints, in-horizon samples, and intersecting interval enclosures");
end
