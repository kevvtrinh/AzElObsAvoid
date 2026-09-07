function seedSet = createRoutePathGuesses(routeSet, obstacleEnvelope_units, directDuration_s, directLength_units)
%% Section 0: Header & Readme
% SYNTAX
%   guesses = createRoutePathGuesses(routes, boundary_units, duration_s, length_units)
% PURPOSE
%   Convert searched routes to path guesses without constructing a direct guess.
% INPUTS
%   routeSet: searched timed/spatial routes; obstacleEnvelope_units: search outline.
%   directDuration_s and directLength_units: advisory direct-path reference values.
% OUTPUTS
%   seedSet: timed guesses followed by distinct spatial guesses.
% UNITS
%   Positions and lengths are coordinate units; duration is seconds; tau is normalized.

template = obstacleAvoidance.search.createEmptyPathGuess();
seedSet  = repmat(template, 0, 1);
%% Section 1: Append The Timed Seed

% Keep timed routes before spatial routes so their waits are preserved.

if ~isempty(routeSet.TimedRoute_units) && routeSet.TimedRouteTime_s(end) > routeSet.TimedRouteTime_s(1)
    seed = template;
    seed.Index  = numel(seedSet) + 1;
    seed.Source = "timeExpandedVisibilityGraph";
    positionChanges = [true; vecnorm(diff(routeSet.TimedRoute_units, 1, 1), 2, 2) > 1e-12];
    % Collapse a timed wait-then-move route to its spatial endpoints so duplicate wait positions do not define a false spatial bend.
    if any(~positionChanges(2:end)) && nnz(positionChanges) == 2
        seed.Source = "directWait";
    end
    seed.position_units        = routeSet.TimedRoute_units;
    seed.ParameterBasis      = "normalizedTime";
    seed.tau                 = (routeSet.TimedRouteTime_s - routeSet.TimedRouteTime_s(1)) / (routeSet.TimedRouteTime_s(end) - routeSet.TimedRouteTime_s(1));
    seed.EstimatedDuration_s = routeSet.TimedRouteTime_s(end) - routeSet.TimedRouteTime_s(1);
    seed.Length_units          = obstacleAvoidance.geometry.routeLength(routeSet.TimedRoute_units);
    seedSet(end + 1, 1) = seed;
end

%% Section 2: Append Distinct Spatial Seeds

% A smooth motion can be faster than its guide polyline. Use only the
% endpoint velocity lower bound; do not reject seeds by estimated duration.

spatialTemplate = template;
spatialTemplate.ObstacleEnvelope_units     = obstacleEnvelope_units;
spatialTemplate.UsesConservativeEnvelope = routeSet.UsesConservativeEnvelope;
distinctLengthTolerance_units = 1e-9 * max(1, directLength_units);
% Evaluate each route before retaining the best admissible candidate.
for routeIndex = 1:numel(routeSet.SpatialRoutes_units)
    route_units = routeSet.SpatialRoutes_units{routeIndex};
    seed      = createSpatialSeed(spatialTemplate, numel(seedSet) + 1, route_units, directDuration_s);
    % Keep only route seeds that are meaningfully distinct in length from the direct path.
    if seed.Length_units > directLength_units + distinctLengthTolerance_units
        seedSet(end + 1, 1) = seed; %#ok<AGROW>
    end
end
end

%% Section 3: Local Functions

function seed = createSpatialSeed(template, index, route_units, directDuration_s)
    % Create a spatial seed with a duration estimate, not a feasibility test.
    seed = template;
    seed.Index        = index;
    seed.Source       = "visibilityGraph";
    seed.position_units = route_units;
    [seed.tau, seed.Length_units] = routeTau(route_units);
    seed.EstimatedDuration_s = directDuration_s;
end

function [tau, length_units] = routeTau(route_units)
    % Parameterize a polyline by normalized cumulative Euclidean length.
    cumulative_units = [0; cumsum(vecnorm(diff(route_units, 1, 1), 2, 2))];
    length_units     = cumulative_units(end);
    if length_units <= 0
        tau = linspace(0, 1, size(route_units, 1)).';
    else
        tau = cumulative_units / length_units;
    end
end
