function seedSet = createRoutePathGuesses(routeSet, obstacleEnvelope_deg, directDuration_s, directLength_deg)
%% Section 0: Header & Readme
% SYNTAX
%   guesses = createRoutePathGuesses(routes, boundary_deg, duration_s, length_deg)
% PURPOSE
%   Convert searched routes to path guesses without constructing a direct guess.
% INPUTS
%   routeSet: searched timed/spatial routes; obstacleEnvelope_deg: search outline.
%   directDuration_s and directLength_deg: advisory direct-path reference values.
% OUTPUTS
%   seedSet: timed guesses followed by distinct spatial guesses.
% UNITS
%   Positions and lengths are degrees; duration is seconds; tau is normalized.

template = obstacleAvoidance.search.createEmptyPathGuess();
seedSet  = repmat(template, 0, 1);
%% Section 1: Append The Timed Seed

% Keep timed routes before spatial routes so their waits are preserved.

if ~isempty(routeSet.TimedRoute_deg) && routeSet.TimedRouteTime_s(end) > routeSet.TimedRouteTime_s(1)
    seed = template;
    seed.Index  = numel(seedSet) + 1;
    seed.Source = "timeExpandedVisibilityGraph";
    positionChanges = [true; vecnorm(diff(routeSet.TimedRoute_deg, 1, 1), 2, 2) > 1e-12];
    % Collapse a timed wait-then-move route to its spatial endpoints so duplicate wait positions do not define a false spatial bend.
    if any(~positionChanges(2:end)) && nnz(positionChanges) == 2
        seed.Source = "directWait";
    end
    seed.position_deg        = routeSet.TimedRoute_deg;
    seed.ParameterBasis      = "normalizedTime";
    seed.tau                 = (routeSet.TimedRouteTime_s - routeSet.TimedRouteTime_s(1)) / (routeSet.TimedRouteTime_s(end) - routeSet.TimedRouteTime_s(1));
    seed.EstimatedDuration_s = routeSet.TimedRouteTime_s(end) - routeSet.TimedRouteTime_s(1);
    seed.Length_deg          = obstacleAvoidance.geometry.routeLength(routeSet.TimedRoute_deg);
    seedSet(end + 1, 1) = seed;
end

%% Section 2: Append Distinct Spatial Seeds

% A smooth motion can be faster than its guide polyline. Use only the
% endpoint velocity lower bound; do not reject seeds by estimated duration.

spatialTemplate = template;
spatialTemplate.ObstacleEnvelope_deg     = obstacleEnvelope_deg;
spatialTemplate.UsesConservativeEnvelope = routeSet.UsesConservativeEnvelope;
distinctLengthTolerance_deg = 1e-9 * max(1, directLength_deg);
% Evaluate each route before retaining the best admissible candidate.
for routeIndex = 1:numel(routeSet.SpatialRoutes_deg)
    route_deg = routeSet.SpatialRoutes_deg{routeIndex};
    seed      = createSpatialSeed(spatialTemplate, numel(seedSet) + 1, route_deg, directDuration_s);
    % Keep only route seeds that are meaningfully distinct in length from the direct path.
    if seed.Length_deg > directLength_deg + distinctLengthTolerance_deg
        seedSet(end + 1, 1) = seed; %#ok<AGROW>
    end
end
end

%% Section 3: Local Functions

function seed = createSpatialSeed(template, index, route_deg, directDuration_s)
    % Create a spatial seed with a duration estimate, not a feasibility test.
    seed = template;
    seed.Index        = index;
    seed.Source       = "visibilityGraph";
    seed.position_deg = route_deg;
    [seed.tau, seed.Length_deg] = routeTau(route_deg);
    seed.EstimatedDuration_s = directDuration_s;
end

function [tau, length_deg] = routeTau(route_deg)
    % Parameterize a polyline by normalized cumulative Euclidean length.
    cumulative_deg = [0; cumsum(vecnorm(diff(route_deg, 1, 1), 2, 2))];
    length_deg     = cumulative_deg(end);
    if length_deg <= 0
        tau = linspace(0, 1, size(route_deg, 1)).';
    else
        tau = cumulative_deg / length_deg;
    end
end
