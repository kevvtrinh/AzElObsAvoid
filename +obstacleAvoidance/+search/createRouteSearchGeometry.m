function proposal = createRouteSearchGeometry(initialState, goalState, scene)
%% Section 0: Header & Readme
% SYNTAX
%   proposal = obstacleAvoidance.search.createRouteSearchGeometry( ...
%       initialState, goalState, scene)
%
% PURPOSE
%   - Build a 2-D obstacle outline for finding possible paths.
%   - Expose sample times, work estimate, geometry choice, shape, and edges.
%
% INPUTS
%   - initialState, goalState: route endpoints.
%   - scene (scalar prepared-scene struct)
%       Prepared obstacles and physical request horizon.
%
% OUTPUTS
%   - proposal (scalar struct)
%       Start, goal, times, work data, selected polyshape, and boundary edges.
%       This route-search input cannot approve a completed trajectory.
%
% UNITS
%   - Geometry is coordinate units, time is seconds, and work is a vertex count.
%

%% Section 1: Resolve Endpoints And Sample Times

% Use the planning horizon and resolve the wrapped endpoint.

obstacles = scene.preparedObstacles;
start_units = initialState.position_units;
goal_units  = obstacleAvoidance.input.goalPositionAtTime(goalState, scene.endTime_s);
sampleTimes_s = obstacleAvoidance.search.createTimeLayers(obstacles, scene.startTime_s, scene.endTime_s);

%% Section 2: Select The Proposal Representation

% For dense histories, try a conservative envelope first.
% Use the sampled union if the envelope covers an endpoint.

vertexWorkBudget = 10e3;
[proposalShape, usedDenseEnvelope, estimatedVertexWork] = obstacleAvoidance.search.denseSweptEnvelope(obstacles, sampleTimes_s, [start_units; goal_units], vertexWorkBudget);
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
    proposalShape = polyshape();
    if sampledShapeCount > 0
        proposalShape = union([parts{1:sampledShapeCount}]);
    end
    representation = "sampledObstacleUnion";
end

%% Section 3: Create Reusable Proposal Edges

% Cache proposal edges for visibility checks and route shortening.

[edgeStart_units, edgeEnd_units] = obstacleAvoidance.geometry.boundaryToEdges(proposalShape, 1e-12);

%% Section 4: Assemble The Proposal

% Save geometry choices for diagnostics and plots.

proposal = struct("start_units", start_units, ...
    "goal_units", goal_units, ...
    "sampleTimes_s", sampleTimes_s, ...
    "vertexWorkBudget", vertexWorkBudget, ...
    "estimatedVertexWork", estimatedVertexWork, ...
    "representation", representation, ...
    "usedDenseEnvelope", usedDenseEnvelope, ...
    "sampledShapeCount", sampledShapeCount, ...
    "shape", proposalShape, ...
    "edgeStart_units", edgeStart_units, ...
    "edgeEnd_units", edgeEnd_units);
end
