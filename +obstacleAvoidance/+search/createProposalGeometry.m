function proposal = createProposalGeometry(scene, request)
%% Section 0: Header & Readme
% SYNTAX
%   proposal = obstacleAvoidance.search.createProposalGeometry( ...
%       scene, request)
%**************************************************************************
% PURPOSE
%   - Create one spatial obstacle representation for route proposals.
%   - Expose sample times, work estimate, geometry choice, shape, and edges.
%**************************************************************************
% INPUTS
%   - scene (scalar prepared-scene struct)
%       Prepared obstacles and physical request horizon.
%   - request (scalar planning-request struct)
%       Normalized endpoint states, limits, and resolved options.
%**************************************************************************
% OUTPUTS
%   - proposal (scalar struct)
%       Start, goal, times, work data, selected polyshape, and boundary edges.
%       This route-search input cannot approve a completed trajectory.
%**************************************************************************
% UNITS
%   - Geometry is degrees, time is seconds, and work is a vertex count.
%**************************************************************************

%% Section 1: Resolve Endpoints And Sample Times

% Proposal geometry spans the same physical horizon as later timed search.
% Resolve azimuth wrapping here so its edges and all routes share one endpoint.

requiredSceneFields = {'preparedObstacles', 'startTime_s', 'endTime_s'};
requiredRequestFields = {'initialState', 'goalState', 'options'};
if ~isstruct(scene) || ~isscalar(scene) || ...
        ~all(isfield(scene, requiredSceneFields)) || ...
        ~isstruct(request) || ~isscalar(request) || ...
        ~all(isfield(request, requiredRequestFields))
    error("createProposalGeometry:InvalidInput", ...
        "scene and request must contain prepared obstacles, horizon, endpoints, and options.");
end
obstacles = scene.preparedObstacles;
start_deg = request.initialState.position_deg;
goal_deg = obstacleAvoidance.input.goalPositionAtTime( ...
    request.goalState, scene.endTime_s);
if request.options.AllowAzimuthWrapping
    goal_deg(1) = goal_deg(1) + 360 * round( ...
        (start_deg(1) - goal_deg(1)) / 360);
end
sampleTimes_s = createObstacleSampleTimes( ...
    obstacles, scene.startTime_s, scene.endTime_s);

%% Section 2: Estimate Sampled-Union Work

% Very dense histories can make the ordinary sampled union disproportionately
% expensive. Estimate its vertex work before selecting the representation.

vertexWorkBudget = 10e3;
verticesPerTime = 0;
for obstacleIndex = 1:numel(obstacles)
    maximumVertexCount = 0;
    for sampleIndex = 1:numel(obstacles(obstacleIndex).az_deg)
        maximumVertexCount = max(maximumVertexCount, ...
            numel(obstacles(obstacleIndex).az_deg{sampleIndex}));
    end
    verticesPerTime = verticesPerTime + maximumVertexCount;
end
estimatedVertexWork = numel(sampleTimes_s) * verticesPerTime;

%% Section 3: Select The Proposal Representation

% Try the dense-history envelope only when sampled-union work exceeds its
% limit. If that envelope covers an endpoint, use the exact sampled union so
% conservative proposal geometry does not erase the planning request.

obstacleAvoidance.input.throwIfCancellationRequested(request.options);
[proposalShape, usedDenseEnvelope] = ...
    obstacleAvoidance.search.denseSweptEnvelope( ...
    obstacles, sampleTimes_s, [start_deg; goal_deg], vertexWorkBudget);
if usedDenseEnvelope
    sampledShapeCount = numel(sampleTimes_s) * numel(obstacles);
    representation = "denseHistoryEnvelope";
else
    [proposalShape, sampledShapeCount] = ...
        obstacleAvoidance.search.createSampledObstacleUnion( ...
        obstacles, sampleTimes_s);
    representation = "sampledObstacleUnion";
end

%% Section 4: Create Reusable Proposal Edges

% Visibility attempts and spatial route cleanup query the same selected shape.
% Create its ordered edges once so those later stages cannot diverge.

[edgeStart_deg, edgeEnd_deg] = ...
    obstacleAvoidance.geometry.boundaryToEdges(proposalShape, 1e-12);

%% Section 5: Assemble The Proposal

% Retain the decision inputs and chosen geometry. Plotting and diagnosis can
% inspect this stage without rerunning obstacle queries or route planning.

proposal = struct( ...
    "start_deg", start_deg, ...
    "goal_deg", goal_deg, ...
    "sampleTimes_s", sampleTimes_s, ...
    "vertexWorkBudget", vertexWorkBudget, ...
    "estimatedVertexWork", estimatedVertexWork, ...
    "representation", representation, ...
    "usedDenseEnvelope", usedDenseEnvelope, ...
    "sampledShapeCount", sampledShapeCount, ...
    "shape", proposalShape, ...
    "edgeStart_deg", edgeStart_deg, ...
    "edgeEnd_deg", edgeEnd_deg);
end

%% Section 6: Local Functions

function sampleTimes_s = createObstacleSampleTimes( ...
        obstacles, startTime_s, endTime_s)
% Retain all source, midpoint, endpoint, and uniform request times.
sampleTimes_s = [startTime_s; ...
    linspace(startTime_s, endTime_s, 9).'; endTime_s];
for obstacleIndex = 1:numel(obstacles)
    sourceTime_s = obstacles(obstacleIndex).time_s(:);
    intervalMidTime_s = ...
        (sourceTime_s(1:end - 1) + sourceTime_s(2:end)) / 2;
    sampleTimes_s = [sampleTimes_s; sourceTime_s; ...
        intervalMidTime_s]; %#ok<AGROW>
end
sampleTimes_s = unique(sampleTimes_s( ...
    sampleTimes_s >= startTime_s & sampleTimes_s <= endTime_s));
end
