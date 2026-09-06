function seedSet = createSeeds( ...
        obstacles, initialState, goalState, limits, options, routeSet, proposal)
%% Section 0: Header & Readme
% SYNTAX
%   seedSet = obstacleAvoidance.search.createSeeds( ...
%       obstacles, initialState, goalState, limits, options, routeSet, proposal)
%**************************************************************************
% PURPOSE
%   - Convert direct, timed, and spatial routes into deterministic seeds.
%   - Preserve spatial routes while keeping duration estimates advisory.
%**************************************************************************
% INPUTS
%   - obstacles, initialState, goalState, limits, options
%       Normalized planning inputs in public planner order. Unused inputs
%       are accepted to keep stage signatures consistent.
%   - routeSet (scalar struct or empty)
%       Timed and spatial route suggestions returned by searchRoutes.
%   - proposal (scalar struct or empty)
%       Proposal geometry that supplies spatial seed corridor provenance.
%**************************************************************************
% OUTPUTS
%   - seedSet (struct array)
%       Direct seed first, followed by a timed seed and distinct spatial
%       seeds in search order. Estimates never reject a route.
%**************************************************************************
% UNITS
%   - Positions, boundaries, and lengths are degrees; duration is seconds.
%**************************************************************************

%% Section 1: Create The Required Direct Seed

% Always propose the direct route first. Its velocity-based duration
% is a lower bound, not a deadline for solving the seed.

start_deg = initialState.position_deg;
goal_deg = obstacleAvoidance.input.goalPositionAtTime( ...
    goalState, goalState.time_s);
if options.AllowAzimuthWrapping
    goal_deg(1) = goal_deg(1) + 360 * round( ...
        (start_deg(1) - goal_deg(1)) / 360);
end
available_s = goalState.time_s - initialState.time_s;
directRoute_deg = [start_deg; goal_deg];
directLength_deg = norm(goal_deg - start_deg);
directDuration_s = min(available_s, max(1e-3, ...
    max(abs(goal_deg - start_deg) ./ limits.maxVelocity_deg_s)));
template = obstacleAvoidance.search.createSeed();
seedSet = template;
seedSet.Index = 1;
seedSet.Source = "directVisibilityEdge";
seedSet.position_deg = directRoute_deg;
seedSet.tau = [0; 1];
seedSet.EstimatedDuration_s = directDuration_s;
seedSet.Length_deg = directLength_deg;
if isempty(routeSet)
    return;
end

%% Section 2: Append The Timed Seed

% Keep timed routes before spatial routes so their waits are preserved.

if ~isempty(routeSet.TimedRoute_deg) && ...
        routeSet.TimedRouteTime_s(end) > routeSet.TimedRouteTime_s(1)
    seed = template;
    seed.Index = numel(seedSet) + 1;
    seed.Source = "timeExpandedVisibilityGraph";
    positionChanges = [true; vecnorm(diff( ...
        routeSet.TimedRoute_deg, 1, 1), 2, 2) > 1e-12];
    if any(~positionChanges(2:end)) && nnz(positionChanges) == 2
        seed.Source = "directWait";
    end
    seed.position_deg = routeSet.TimedRoute_deg;
    seed.tau = (routeSet.TimedRouteTime_s - ...
        routeSet.TimedRouteTime_s(1)) / ...
        (routeSet.TimedRouteTime_s(end) - ...
        routeSet.TimedRouteTime_s(1));
    seed.EstimatedDuration_s = routeSet.TimedRouteTime_s(end) - ...
        routeSet.TimedRouteTime_s(1);
    seed.Length_deg = obstacleAvoidance.geometry.routeLength( ...
        routeSet.TimedRoute_deg);
    seedSet(end + 1, 1) = seed;
end

%% Section 3: Append Distinct Spatial Seeds

% A smooth motion can be faster than its guide polyline. Use only the
% endpoint velocity lower bound; do not reject seeds by estimated duration.

spatialTemplate = template;
spatialTemplate.CorridorBoundary_deg = proposal.shape.Vertices;
spatialTemplate.UsesReducedGeometry = routeSet.UsesReducedGeometry;
distinctLengthTolerance_deg = 1e-9 * max(1, directLength_deg);
for routeIndex = 1:numel(routeSet.SpatialRoutes_deg)
    route_deg = routeSet.SpatialRoutes_deg{routeIndex};
    seed = createSpatialSeed(spatialTemplate, numel(seedSet) + 1, ...
        route_deg, directDuration_s);
    if seed.Length_deg > ...
            directLength_deg + distinctLengthTolerance_deg
        seedSet(end + 1, 1) = seed; %#ok<AGROW>
    end
end
end

%% Section 4: Local Functions

function seed = createSpatialSeed(template, index, route_deg, directDuration_s)
% Create a spatial seed with a duration estimate, not a feasibility test.
seed = template;
seed.Index = index;
seed.Source = "visibilityGraph";
seed.position_deg = route_deg;
[seed.tau, seed.Length_deg] = routeTau(route_deg);
seed.EstimatedDuration_s = directDuration_s;
end

function [tau, length_deg] = routeTau(route_deg)
% Parameterize a polyline by normalized cumulative Euclidean length.
cumulative_deg = [0; cumsum(vecnorm(diff(route_deg, 1, 1), 2, 2))];
length_deg = cumulative_deg(end);
if length_deg <= 0
    tau = linspace(0, 1, size(route_deg, 1)).';
else
    tau = cumulative_deg / length_deg;
end
end
