function warmStart = createWarmStart(request)
%% Section 0: Header & Readme
% SYNTAX
%   warmStart = bmtpEngine.createWarmStart(request)
%
% PURPOSE
%   - Convert the proposed path into an initial smooth curve for optimization.
%   - Return route resampling, active obstacle pairs, controls, and duration.
%
% INPUTS
%   - request (scalar BMTP solve-request struct)
%       Validated seed, regions, coverage, representation, limits, and horizon.
%
% OUTPUTS
%   - warmStart (scalar struct)
%       Route, controls, uniform segment time, active pairs, counts, and
%       resampling evidence.
%
% UNITS
%   - Position is coordinate units and segment time is seconds.
%

%% Section 1: Create The Timed Or Spatial Warm Route

% Start from the seed's route. Preserve timed cells and spatial corners;
% allocate spatial spans by length rather than input vertex density.

seed = request.Seed;
% Interpolate the seed on timed cell boundaries when topology changes with time; static seeds use their spatial parameterization.
if request.UsesTimedCells
    route_units          = createTimedWarmRoute(seed, request.Coverage.TimedSegmentCount, request.MaximumWarmSegmentCount);
    warmRouteResampled = size(seed.position_units, 1) - 1 > request.MaximumWarmSegmentCount;
else
    [route_units, warmRouteResampled] = createSpatialWarmRoute(double(seed.position_units), request.SplitCount, request.MaximumWarmSegmentCount);
end
route_units([1 end], :) = [request.InitialState.position_units; ...
    request.GoalState.position_units];
segmentCount          = size(route_units, 1) - 1;
regionActiveBySegment = createRegionActiveMask(segmentCount, numel(request.Regions_units), request.Coverage);

%% Section 2: Create Feasible Initial Controls And Timing

% Repeat endpoint controls to enforce rest. Choose initial segment time
% from derivative bounds.

controlPoint_units         = createWarmControl(route_units, request.Degree);
segmentTime_s            = bmtpEngine.findRequiredSegmentTime(controlPoint_units, request.Limits);
originalSeedSegmentCount = size(seed.position_units, 1) - 1;
warmStart                = struct("Route_units", route_units, ...
    "ControlPoint_units", controlPoint_units, ...
    "SegmentTime_s", segmentTime_s, ...
    "Duration_s", segmentCount * segmentTime_s, ...
    "SegmentCount", segmentCount, ...
    "RegionActiveBySegment", regionActiveBySegment, ...
    "OriginalSeedSegmentCount", originalSeedSegmentCount, ...
    "WarmRouteResampled", warmRouteResampled);
end
%% Section 3: Local Functions

function activePairs = createRegionActiveMask(segmentCount, regionCount, coverage)
    % Find caller-supplied cells overlapping each equal-duration span.
    activePairs = true(segmentCount, regionCount);
    if ~isfield(coverage, "RegionActiveTauInterval")
        return;
    end
    activeInterval   = double(coverage.RegionActiveTauInterval);
    segmentStartTau  = (0:segmentCount - 1).' / segmentCount;
    segmentFinishTau = (1:segmentCount).' / segmentCount;
    activePairs      = segmentStartTau < activeInterval(:, 2).' & segmentFinishTau > activeInterval(:, 1).';
end

function route_units = createTimedWarmRoute(seed, requestedSegmentCount, maximumSegmentCount)
    % Sample the timed seed on the equal-duration grid used by the optimizer.
    segmentCount = min(round(double(requestedSegmentCount)), maximumSegmentCount);
    queryTau     = linspace(0, 1, segmentCount + 1).';
    route_units    = interp1(double(seed.tau(:)), double(seed.position_units), queryTau, "linear");
end

function [route_units, wasCanonicalized] = createSpatialWarmRoute(seedRoute_units, splitCount, maximumSegmentCount)
    % Preserve true corners while making the optimizer independent of vertex density.
    [canonicalRoute_units, wasCanonicalized] = removeRedundantRoutePoints(seedRoute_units);
    edgeCount = size(canonicalRoute_units, 1) - 1;

    % Allow the span count to exceed the cap when needed to preserve corners.
    subdivisionEdgeCount = min(edgeCount, maximumSegmentCount);
    targetSegmentCount   = max(edgeCount, subdivisionEdgeCount * splitCount);
    segmentCountByEdge   = allocateSegmentsByLength(canonicalRoute_units, targetSegmentCount);
    route_units            = splitRouteByCount(canonicalRoute_units, segmentCountByEdge);
    wasCanonicalized     = wasCanonicalized || edgeCount > maximumSegmentCount;
end

function [route_units, wasReduced] = removeRedundantRoutePoints(route_units)
    % Remove only roundoff-scale duplicates and points lying on a straight edge.
    [~, geometryTolerance_units] = bmtpEngine.createCoordinateTolerances(route_units);
    originalPointCount = size(route_units, 1);

    distinctRoute_units  = zeros(size(route_units));
    distinctPointCount = 1;
    distinctRoute_units(1, :) = route_units(1, :);
    % Process each point needed to complete remove redundant route points.
    for pointIndex = 2:originalPointCount
        % Retain points separated beyond the geometry tolerance and drop near-duplicate consecutive points.
        if norm(route_units(pointIndex, :) - distinctRoute_units(distinctPointCount, :)) > geometryTolerance_units
            distinctPointCount = distinctPointCount + 1;
            distinctRoute_units(distinctPointCount, :) = route_units(pointIndex, :);
        end
    end
    distinctRoute_units = distinctRoute_units(1:distinctPointCount, :);
    % Duplicate the lone surviving waypoint so the motion engine still receives a valid two-endpoint route.
    if distinctPointCount == 1
        route_units  = [distinctRoute_units; distinctRoute_units];
        wasReduced = originalPointCount > 2;
        return;
    end

    route_units          = zeros(size(distinctRoute_units));
    retainedPointCount = 0;
    % Process each point needed to complete remove redundant route points.
    for pointIndex = 1:distinctPointCount
        retainedPointCount = retainedPointCount + 1;
        route_units(retainedPointCount, :) = distinctRoute_units(pointIndex, :);
        % Continue iterating until the stopping condition for complete remove redundant route points is satisfied.
        while retainedPointCount >= 3 && pointLiesOnSegment(route_units(retainedPointCount - 1, :), route_units(retainedPointCount - 2, :), route_units(retainedPointCount, :), geometryTolerance_units)
            route_units(retainedPointCount - 1, :) = route_units(retainedPointCount, :);
            retainedPointCount = retainedPointCount - 1;
        end
    end
    route_units  = route_units(1:retainedPointCount, :);
    wasReduced = retainedPointCount < originalPointCount;
end

function isOnSegment = pointLiesOnSegment(point_units, start_units, finish_units, tolerance_units)
    % Recognize subdivision points without erasing reversals or genuine turns.
    chord_units               = finish_units - start_units;
    chordLengthSquared_units2 = dot(chord_units, chord_units);
    if chordLengthSquared_units2 <= tolerance_units ^ 2
        isOnSegment = false;
        return;
    end
    progress       = dot(point_units - start_units, chord_units) / chordLengthSquared_units2;
    projection_units = start_units + progress * chord_units;
    isOnSegment    = progress >= 0 && progress <= 1 && norm(point_units - projection_units) <= tolerance_units;
end

function segmentCountByEdge = allocateSegmentsByLength(route_units, targetSegmentCount)
    % Give each true edge one span, then allocate the remainder by arc length.
    edgeLength_units        = vecnorm(diff(route_units, 1, 1), 2, 2);
    edgeCount             = numel(edgeLength_units);
    segmentCountByEdge    = ones(edgeCount, 1);
    remainingSegmentCount = targetSegmentCount - edgeCount;
    if remainingSegmentCount <= 0 || sum(edgeLength_units) <= 0
        segmentCountByEdge(1) = segmentCountByEdge(1) + remainingSegmentCount;
        return;
    end

    exactAdditionalCount = remainingSegmentCount * edgeLength_units / sum(edgeLength_units);
    additionalCount      = floor(exactAdditionalCount);
    segmentCountByEdge   = segmentCountByEdge + additionalCount;
    unassignedCount      = remainingSegmentCount - sum(additionalCount);
    fractionalCount      = exactAdditionalCount - additionalCount;
    [~, allocationOrder] = sortrows([-fractionalCount, (1:edgeCount).'], [1 2]);
    segmentCountByEdge(allocationOrder(1:unassignedCount)) = segmentCountByEdge(allocationOrder(1:unassignedCount)) + 1;
end

function route_units = splitRouteByCount(seedRoute_units, segmentCountByEdge)
    % Subdivide each straight edge without moving any original corner.
    edgeCount       = size(seedRoute_units, 1) - 1;
    route_units       = zeros(sum(segmentCountByEdge) + 1, 2);
    routePointIndex = 1;
    % Process each geometric edge while constructing or checking the region topology.
    for edgeIndex = 1:edgeCount
        segmentCount      = segmentCountByEdge(edgeIndex);
        fractions         = (0:segmentCount - 1).' / segmentCount;
        routePointIndices = routePointIndex: routePointIndex + segmentCount - 1;
        route_units(routePointIndices, :) = seedRoute_units(edgeIndex, :) + fractions .* (seedRoute_units(edgeIndex + 1, :) - seedRoute_units(edgeIndex, :));
        routePointIndex = routePointIndex + segmentCount;
    end
    route_units(end, :) = seedRoute_units(end, :);
end


function controlPoint_units = createWarmControl(route_units, degree)
    % Build initial controls with continuous position through jerk and resting endpoints.
    segmentCount     = size(route_units, 1) - 1;
    fraction         = reshape(min(1, max(0, ((0:degree) - 2) / (degree - 4))), 1, [], 1);
    start_units        = reshape(route_units(1:end - 1, :), segmentCount, 1, 2);
    finish_units       = reshape(route_units(2:end, :), segmentCount, 1, 2);
    controlPoint_units = (1 - fraction) .* start_units + fraction .* finish_units;
end
