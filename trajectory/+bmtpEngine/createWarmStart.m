function warmStart = createWarmStart(request)
%% Section 0: Header & Readme
% SYNTAX
%   warmStart = bmtpEngine.createWarmStart(request)
%**************************************************************************
% PURPOSE
%   - Convert a topology seed into the established feasible Bezier warm curve.
%   - Return route resampling, active obstacle pairs, controls, and duration.
%**************************************************************************
% INPUTS
%   - request (scalar BMTP solve-request struct)
%       Validated seed, regions, coverage, representation, limits, and horizon.
%**************************************************************************
% OUTPUTS
%   - warmStart (scalar struct)
%       Route, controls, uniform segment time, active pairs, counts, and
%       resampling evidence.
%**************************************************************************
% UNITS
%   - Position is degrees and segment time is seconds.
%**************************************************************************

%% Section 1: Create The Timed Or Spatial Warm Route

% A usable starting curve gives the alternating solver a topology-consistent
% homotopy before separating lines are introduced. Timed coverage preserves
% its cells. Spatial routes preserve every genuine turn while redistributing
% their existing optimizer spans by distance, so input vertex spacing cannot
% give a long edge the same representation as a nearly zero-length edge.

seed = request.Seed;
if request.UsesTimedCells
    route_deg = createTimedWarmRoute( ...
        seed, request.Coverage.TimedSegmentCount, ...
        request.MaximumWarmSegmentCount);
    warmRouteResampled = size(seed.position_deg, 1) - 1 > ...
        request.MaximumWarmSegmentCount;
else
    [route_deg, warmRouteResampled] = createSpatialWarmRoute( ...
        double(seed.position_deg), request.SplitCount, ...
        request.MaximumWarmSegmentCount);
end
route_deg([1 end], :) = [request.InitialState.position_deg; ...
    request.GoalState.position_deg];
segmentCount = size(route_deg, 1) - 1;
regionActiveBySegment = createRegionActiveMask( ...
    segmentCount, numel(request.Regions_deg), request.Coverage);

%% Section 2: Create Feasible Initial Controls And Timing

% Repeated endpoint controls impose the required rest boundary state. The
% derivative-based segment time supplies a kinematically feasible initial
% scale before any trajectory or maximum-margin optimization.

controlPoint_deg = createWarmControl(route_deg, request.Degree);
segmentTime_s = bmtpEngine.findRequiredSegmentTime( ...
    controlPoint_deg, request.Limits);
originalSeedSegmentCount = size(seed.position_deg, 1) - 1;
warmStart = struct( ...
    "Route_deg", route_deg, ...
    "ControlPoint_deg", controlPoint_deg, ...
    "SegmentTime_s", segmentTime_s, ...
    "Duration_s", segmentCount * segmentTime_s, ...
    "SegmentCount", segmentCount, ...
    "RegionActiveBySegment", regionActiveBySegment, ...
    "OriginalSeedSegmentCount", originalSeedSegmentCount, ...
    "WarmRouteResampled", warmRouteResampled);
end
%% Section 3: Local Functions

function activePairs = createRegionActiveMask( ...
        segmentCount, regionCount, coverage)
% Map equal-duration spans to caller-owned cells with positive-time overlap.
activePairs = true(segmentCount, regionCount);
if ~isfield(coverage, "RegionActiveTauInterval")
    return;
end
activeInterval = double(coverage.RegionActiveTauInterval);
segmentStartTau = (0:segmentCount - 1).' / segmentCount;
segmentFinishTau = (1:segmentCount).' / segmentCount;
activePairs = segmentStartTau < activeInterval(:, 2).' & ...
    segmentFinishTau > activeInterval(:, 1).';
end

function route_deg = createTimedWarmRoute( ...
        seed, requestedSegmentCount, maximumSegmentCount)
% Sample the timed seed on the equal-duration grid used by the optimizer.
segmentCount = min(round(double(requestedSegmentCount)), maximumSegmentCount);
queryTau = linspace(0, 1, segmentCount + 1).';
route_deg = interp1(double(seed.tau(:)), double(seed.position_deg), ...
    queryTau, "linear");
end

function [route_deg, wasCanonicalized] = createSpatialWarmRoute( ...
        seedRoute_deg, splitCount, maximumSegmentCount)
% Preserve true corners while making the optimizer independent of vertex density.
[canonicalRoute_deg, wasCanonicalized] = ...
    removeRedundantRoutePoints(seedRoute_deg);
edgeCount = size(canonicalRoute_deg, 1) - 1;

% The existing split count remains the representation budget per genuine
% edge. More than the nominal cap is allowed only when required to retain
% genuine corners; silently deleting a corner could change the route class.
subdivisionEdgeCount = min(edgeCount, maximumSegmentCount);
targetSegmentCount = max(edgeCount, subdivisionEdgeCount * splitCount);
segmentCountByEdge = allocateSegmentsByLength( ...
    canonicalRoute_deg, targetSegmentCount);
route_deg = splitRouteByCount(canonicalRoute_deg, segmentCountByEdge);
wasCanonicalized = wasCanonicalized || edgeCount > maximumSegmentCount;
end

function [route_deg, wasReduced] = removeRedundantRoutePoints(route_deg)
% Remove only roundoff-scale duplicates and points lying on a straight edge.
[~, geometryTolerance_deg] = ...
    bmtpEngine.createCoordinateTolerances(route_deg);
originalPointCount = size(route_deg, 1);

distinctRoute_deg = zeros(size(route_deg));
distinctPointCount = 1;
distinctRoute_deg(1, :) = route_deg(1, :);
for pointIndex = 2:originalPointCount
    if norm(route_deg(pointIndex, :) - ...
            distinctRoute_deg(distinctPointCount, :)) > ...
            geometryTolerance_deg
        distinctPointCount = distinctPointCount + 1;
        distinctRoute_deg(distinctPointCount, :) = route_deg(pointIndex, :);
    end
end
distinctRoute_deg = distinctRoute_deg(1:distinctPointCount, :);
if distinctPointCount == 1
    route_deg = [distinctRoute_deg; distinctRoute_deg];
    wasReduced = originalPointCount > 2;
    return;
end

route_deg = zeros(size(distinctRoute_deg));
retainedPointCount = 0;
for pointIndex = 1:distinctPointCount
    retainedPointCount = retainedPointCount + 1;
    route_deg(retainedPointCount, :) = distinctRoute_deg(pointIndex, :);
    while retainedPointCount >= 3 && pointLiesOnSegment( ...
            route_deg(retainedPointCount - 1, :), ...
            route_deg(retainedPointCount - 2, :), ...
            route_deg(retainedPointCount, :), geometryTolerance_deg)
        route_deg(retainedPointCount - 1, :) = ...
            route_deg(retainedPointCount, :);
        retainedPointCount = retainedPointCount - 1;
    end
end
route_deg = route_deg(1:retainedPointCount, :);
wasReduced = retainedPointCount < originalPointCount;
end

function isOnSegment = pointLiesOnSegment( ...
        point_deg, start_deg, finish_deg, tolerance_deg)
% Recognize subdivision points without erasing reversals or genuine turns.
chord_deg = finish_deg - start_deg;
chordLengthSquared_deg2 = dot(chord_deg, chord_deg);
if chordLengthSquared_deg2 <= tolerance_deg ^ 2
    isOnSegment = false;
    return;
end
progress = dot(point_deg - start_deg, chord_deg) / ...
    chordLengthSquared_deg2;
projection_deg = start_deg + progress * chord_deg;
isOnSegment = progress >= 0 && progress <= 1 && ...
    norm(point_deg - projection_deg) <= tolerance_deg;
end

function segmentCountByEdge = allocateSegmentsByLength( ...
        route_deg, targetSegmentCount)
% Give each true edge one span, then allocate the remainder by arc length.
edgeLength_deg = vecnorm(diff(route_deg, 1, 1), 2, 2);
edgeCount = numel(edgeLength_deg);
segmentCountByEdge = ones(edgeCount, 1);
remainingSegmentCount = targetSegmentCount - edgeCount;
if remainingSegmentCount <= 0 || sum(edgeLength_deg) <= 0
    segmentCountByEdge(1) = ...
        segmentCountByEdge(1) + remainingSegmentCount;
    return;
end

exactAdditionalCount = remainingSegmentCount * edgeLength_deg / ...
    sum(edgeLength_deg);
additionalCount = floor(exactAdditionalCount);
segmentCountByEdge = segmentCountByEdge + additionalCount;
unassignedCount = remainingSegmentCount - sum(additionalCount);
fractionalCount = exactAdditionalCount - additionalCount;
[~, allocationOrder] = sortrows( ...
    [-fractionalCount, (1:edgeCount).'], [1 2]);
segmentCountByEdge(allocationOrder(1:unassignedCount)) = ...
    segmentCountByEdge(allocationOrder(1:unassignedCount)) + 1;
end

function route_deg = splitRouteByCount(seedRoute_deg, segmentCountByEdge)
% Subdivide each straight edge without moving any original corner.
edgeCount = size(seedRoute_deg, 1) - 1;
route_deg = zeros(sum(segmentCountByEdge) + 1, 2);
routePointIndex = 1;
for edgeIndex = 1:edgeCount
    segmentCount = segmentCountByEdge(edgeIndex);
    fractions = (0:segmentCount - 1).' / segmentCount;
    routePointIndices = routePointIndex: ...
        routePointIndex + segmentCount - 1;
    route_deg(routePointIndices, :) = seedRoute_deg(edgeIndex, :) + ...
        fractions .* (seedRoute_deg(edgeIndex + 1, :) - ...
        seedRoute_deg(edgeIndex, :));
    routePointIndex = routePointIndex + segmentCount;
end
route_deg(end, :) = seedRoute_deg(end, :);
end


function controlPoint_deg = createWarmControl(route_deg, degree)
% Create the route-shaped C3 rest-through-jerk warm control net.
segmentCount = size(route_deg, 1) - 1;
fraction = reshape(min(1, max(0, ((0:degree) - 2) / (degree - 4))), 1, [], 1);
start_deg = reshape(route_deg(1:end - 1, :), segmentCount, 1, 2);
finish_deg = reshape(route_deg(2:end, :), segmentCount, 1, 2);
controlPoint_deg = (1 - fraction) .* start_deg + fraction .* finish_deg;
end
