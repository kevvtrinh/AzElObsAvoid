function obstaclePlanningData = addRouteSearchGeometry(obstaclePlanningData, initialState, goalState, options)
%% Section 0: Header & Readme
% SYNTAX
%   obstaclePlanningData = obstacleAvoidance.search.addRouteSearchGeometry( ...
%       obstaclePlanningData, initialState, goalState, options)
%
% PURPOSE
%   - Build a 2-D obstacle outline for finding possible paths.
%   - Expose sample times, work estimate, geometry choice, shape, and edges.
%
% INPUTS
%   - initialState, goalState: route endpoints.
%   - options: coordinate wrapping policy.
%   - obstaclePlanningData (scalar struct)
%       Prepared obstacles and the physical request horizon. Its empty
%       routeSearchGeometry field receives the derived search geometry.
%
% OUTPUTS
%   - obstaclePlanningData (scalar struct)
%       The same request-wide record with routeSearchGeometry populated.
%       Route-search geometry can suggest paths but cannot approve motion.
%
% UNITS
%   - Geometry is degrees, time is seconds, and work is a vertex count.
%

%% Section 1: Resolve Endpoints And Sample Times

% Use the planning horizon and resolve the wrapped endpoint.

obstacles = obstaclePlanningData.preparedObstacles;
start_deg = initialState.position_deg;
goal_deg  = obstacleAvoidance.input.goalPositionAtTime(goalState, obstaclePlanningData.endTime_s);
if options.AllowAzimuthWrapping
    goal_deg(1) = goal_deg(1) + 360 * round((start_deg(1) - goal_deg(1)) / 360);
end
sampleTimes_s = createObstacleSampleTimes(obstacles, obstaclePlanningData.startTime_s, obstaclePlanningData.endTime_s);

%% Section 2: Select The Route-Search Representation

% For dense histories, try a conservative envelope first.
% Use the sampled union if the envelope covers an endpoint.

vertexWorkBudget = 10e3;
[routeSearchShape, usedDenseEnvelope, estimatedVertexWork] = obstacleAvoidance.search.denseSweptEnvelope(obstacles, sampleTimes_s, [start_deg; goal_deg], vertexWorkBudget);
if usedDenseEnvelope
    sampledShapeCount = numel(sampleTimes_s) * numel(obstacles);
    representation    = "denseHistoryEnvelope";
else
    parts             = cell(numel(sampleTimes_s) * numel(obstacles), 1);
    sampledShapeCount = 0;
    % Process each time in temporal order and accumulate its result.
    for timeIndex = 1:numel(sampleTimes_s)
        % Evaluate each obstacle against the current geometry or motion.
        for obstacleIndex = 1:numel(obstacles)
            part = obstacleAvoidance.obstacles.preparedShapeAtTime(obstacles(obstacleIndex), sampleTimes_s(timeIndex));
            if ~isempty(part.Vertices)
                sampledShapeCount = sampledShapeCount + 1;
                parts{sampledShapeCount} = part;
            end
        end
    end
    routeSearchShape = polyshape();
    if sampledShapeCount > 0
        routeSearchShape = union([parts{1:sampledShapeCount}]);
    end
    representation = "sampledObstacleUnion";
end

%% Section 3: Create Reusable Boundary Edges

% Cache boundary edges for visibility checks and route shortening.

[edgeStart_deg, edgeEnd_deg] = obstacleAvoidance.geometry.boundaryToEdges(routeSearchShape, 1e-12);

%% Section 4: Add The Route-Search Geometry

% Save geometry choices for diagnostics and plots.

obstaclePlanningData.routeSearchGeometry = struct("start_deg", start_deg, ...
    "goal_deg", goal_deg, ...
    "sampleTimes_s", sampleTimes_s, ...
    "vertexWorkBudget", vertexWorkBudget, ...
    "estimatedVertexWork", estimatedVertexWork, ...
    "representation", representation, ...
    "usedDenseEnvelope", usedDenseEnvelope, ...
    "sampledShapeCount", sampledShapeCount, ...
    "shape", routeSearchShape, ...
    "edgeStart_deg", edgeStart_deg, ...
    "edgeEnd_deg", edgeEnd_deg);
end

%% Section 5: Local Functions

function sampleTimes_s = createObstacleSampleTimes(obstacles, startTime_s, endTime_s)
    % Retain all source, midpoint, endpoint, and uniform request times.
    sampleTimes_s = [startTime_s; ...
        linspace(startTime_s, endTime_s, 9).'; endTime_s];
    % Evaluate each obstacle against the current geometry or motion.
    for obstacleIndex = 1:numel(obstacles)
        sourceTime_s      = obstacles(obstacleIndex).time_s(:);
        intervalMidTime_s = (sourceTime_s(1:end - 1) + sourceTime_s(2:end)) / 2;
        sampleTimes_s     = [sampleTimes_s; sourceTime_s; ...
            intervalMidTime_s]; %#ok<AGROW>
    end
    sampleTimes_s = unique(sampleTimes_s(sampleTimes_s >= startTime_s & sampleTimes_s <= endTime_s));
end
