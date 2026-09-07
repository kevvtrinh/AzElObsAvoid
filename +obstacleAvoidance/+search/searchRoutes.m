function routeSet = searchRoutes(initialState, goalState, limits, options, scene, proposal, visibilityGraph, priorRouteSet)
%% Section 0: Header & Readme
% SYNTAX
%   routeSet = obstacleAvoidance.search.searchRoutes( ...
%       initialState, goalState, limits, options, ...
%       scene, proposal, visibilityGraph, priorRouteSet)
%
% PURPOSE
%   - Coordinate timed route search and distinct spatial route search.
%   - Retain multi-winding routes for failure-only motion recovery.
%   - Return route suggestions and complete search records before seeding.
%
% INPUTS
%   - initialState, goalState, limits, options: route-search constraints.
%   - scene (scalar prepared-scene struct)
%       Prepared obstacle histories and request horizon.
%   - proposal (scalar proposal-geometry struct)
%       Spatial route-guidance geometry and sample times.
%   - visibilityGraph (scalar visibility-graph struct)
%       Final nodes, edge costs, and obstacle reference points.
%   - priorRouteSet (scalar route-set struct, optional)
%       Initial deferred result to resume with exact timed search. Its
%       spatial routes and search record are reused without recomputation.
%
% OUTPUTS
%   - routeSet (scalar struct)
%       Timed, ordinary spatial, and deferred multi-winding routes plus
%       route-class patterns, search records, selected search modes, and
%       coverage details. Routes are suggestions and cannot approve a
%       completed obstacle-avoidance motion.
%
% UNITS
%   - Positions and route lengths are coordinate units; physical times are seconds.
%

%% Section 1: Search Complete Input-Derived Time Layers

% Defer costly timed search for dense histories until cheap attempts fail.
% Supplying priorRouteSet resumes timed search without repeating spatial search.

isTimedRecovery = nargin >= 8 && ~isempty(priorRouteSet);
% Reject timed-recovery calls unless the primary search explicitly deferred a timed stage.
if isTimedRecovery && (~isstruct(priorRouteSet) || ~isscalar(priorRouteSet) || ~isfield(priorRouteSet, "TimedSearchDeferred") || ~priorRouteSet.TimedSearchDeferred)
    error("searchRoutes:InvalidRecoveryState", "priorRouteSet must be a deferred scalar route-set record.");
end

obstacles                    = scene.preparedObstacles;
nodePosition_units             = visibilityGraph.NodePosition_units;
timedRoute_units               = zeros(0, 2);
timedRouteTime_s             = zeros(0, 1);
timedRecord                  = struct();
timedSearchOptions           = options;
timedSearchAttempted         = false;
timedSearchDeferred          = false;
timedSearchSuppressionReason = "staticObstacleHistory";
requiresTimedSearch          = ~scene.obstaclesRemainStatic;
% Defer timed search after dense-envelope planning so recovery can reuse the spatial evidence without duplicating work.
if requiresTimedSearch && proposal.usedDenseEnvelope && ~isTimedRecovery
    timedSearchDeferred          = true;
    timedSearchSuppressionReason = "deferredDenseTimedSearch";
% Run the timed visibility search when dynamic geometry requires it and no deferred-recovery shortcut applies.
elseif requiresTimedSearch
    timedSearchAttempted         = true;
    timedSearchSuppressionReason = "";
    timedCost_units                = hypot(nodePosition_units(:, 1) - nodePosition_units(:, 1).', nodePosition_units(:, 2) - nodePosition_units(:, 2).');
    [timedRoute_units, timedRouteTime_s, timedRecord] = obstacleAvoidance.search.timeExpandedVisibilitySearch(nodePosition_units, timedCost_units, obstacles, initialState, goalState, limits, proposal.sampleTimes_s, timedSearchOptions);
end
% Reuse the prior spatial search evidence during timed recovery instead of rebuilding it.
if isTimedRecovery
    routeSet = priorRouteSet;
    routeSet.TimedRoute_units               = timedRoute_units;
    routeSet.TimedRouteTime_s             = timedRouteTime_s;
    routeSet.TimedSearchRecord            = timedRecord;
    routeSet.TimedSearchOptions           = timedSearchOptions;
    routeSet.TimedSearchAttempted         = true;
    routeSet.TimedSearchRecoveryAttempted = true;
    routeSet.TimedSearchSuppressionReason = "";
    return;
end

%% Section 2: Search Distinct Spatial Route Classes

% Reserve a seed slot for a timed route, then find distinct spatial routes.

hasTimedRoute      = ~isempty(timedRoute_units) && timedRouteTime_s(end) > timedRouteTime_s(1);
reservesTimedRoute = hasTimedRoute || timedSearchDeferred;
maximumClassCount  = max(0, options.MaximumSeedCount - 1 - double(reservesTimedRoute));
visibilityFunction = @(first_units, second_units) obstacleAvoidance.search.checkVisibilitySegments(first_units, second_units, proposal.shape, proposal.edgeStart_units, proposal.edgeEnd_units);
[spatialRoutes_units, routeClassPattern, spatialSearchRecord] = obstacleAvoidance.search.searchDistinctSpatialRoutes(visibilityGraph.EdgeCost_units, nodePosition_units, visibilityGraph.ObstacleReferencePoints_units, maximumClassCount, visibilityFunction);

% Defer multi-winding motion solves until ordinary routes fail.
% Keep the routes so recovery does not repeat spatial search.
isDeferredSpatialRoute    = any(abs(routeClassPattern) > 1, 2);
deferredSpatialRoutes_units = spatialRoutes_units(isDeferredSpatialRoute);
spatialRoutes_units         = spatialRoutes_units(~isDeferredSpatialRoute);

%% Section 3: Assemble The Route Set

% Keep routes with their search diagnostics.

routeSet = struct("TimedRoute_units", timedRoute_units, ...
    "TimedRouteTime_s", timedRouteTime_s, ...
    "TimedSearchRecord", timedRecord, ...
    "TimedSearchOptions", timedSearchOptions, ...
    "TimedSearchAttempted", timedSearchAttempted, ...
    "TimedSearchDeferred", timedSearchDeferred, ...
    "TimedSearchRecoveryAttempted", false, ...
    "TimedSearchSuppressionReason", timedSearchSuppressionReason, ...
    "SpatialRoutes_units", {spatialRoutes_units}, ...
    "DeferredSpatialRoutes_units", {deferredSpatialRoutes_units}, ...
    "DeferredSpatialSolveAttempted", false, ...
    "RouteClassPattern", routeClassPattern, ...
    "SpatialSearchRecord", spatialSearchRecord, ...
    "MaximumSpatialClassCount", maximumClassCount, ...
    "ObstacleReferencePoints_units", ...
    visibilityGraph.ObstacleReferencePoints_units, ...
    "UsesConservativeEnvelope", proposal.usedDenseEnvelope);
end
