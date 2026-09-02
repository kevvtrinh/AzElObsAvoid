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
% its cells; ordinary routes retain the historical segment cap and split order.

seed = request.Seed;
if request.UsesTimedCells
    route_deg = createTimedWarmRoute( ...
        seed, request.Coverage.TimedSegmentCount, ...
        request.MaximumWarmSegmentCount);
else
    route_deg = limitWarmRouteSegments( ...
        double(seed.position_deg), request.MaximumWarmSegmentCount);
    route_deg = splitRoute(route_deg, request.SplitCount);
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
    "WarmRouteResampled", ...
    originalSeedSegmentCount > request.MaximumWarmSegmentCount);
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

function route_deg = splitRoute(seedRoute_deg, splitCount)
% Split each authored edge uniformly without introducing route preference.
edgeCount = size(seedRoute_deg, 1) - 1;
route_deg = zeros(edgeCount * splitCount + 1, 2);
fractions = repmat((0:splitCount - 1).' / splitCount, edgeCount, 1);
edgeStart_deg = repelem(seedRoute_deg(1:end - 1, :), splitCount, 1);
edgeDelta_deg = repelem(diff(seedRoute_deg, 1, 1), splitCount, 1);
route_deg(1:end - 1, :) = edgeStart_deg + fractions .* edgeDelta_deg;
route_deg(end, :) = seedRoute_deg(end, :);
end

function route_deg = limitWarmRouteSegments(route_deg, maximumSegmentCount)
% Resample only oversized warm routes; final feasibility is independently checked.
segmentCount = size(route_deg, 1) - 1;
if segmentCount <= maximumSegmentCount
    return;
end
distance_deg = [0; cumsum(vecnorm(diff(route_deg, 1, 1), 2, 2))];
keepPoint = [true; diff(distance_deg) > 0];
distance_deg = distance_deg(keepPoint);
route_deg = route_deg(keepPoint, :);
if distance_deg(end) <= 0
    route_deg = repmat(route_deg(1, :), maximumSegmentCount + 1, 1);
    return;
end
queryDistance_deg = linspace(0, distance_deg(end), ...
    maximumSegmentCount + 1).';
route_deg = interp1(distance_deg, route_deg, queryDistance_deg, "linear");
end


function controlPoint_deg = createWarmControl(route_deg, degree)
% Create the route-shaped C3 rest-through-jerk warm control net.
segmentCount = size(route_deg, 1) - 1;
fraction = reshape(min(1, max(0, ((0:degree) - 2) / (degree - 4))), 1, [], 1);
start_deg = reshape(route_deg(1:end - 1, :), segmentCount, 1, 2);
finish_deg = reshape(route_deg(2:end, :), segmentCount, 1, 2);
controlPoint_deg = (1 - fraction) .* start_deg + fraction .* finish_deg;
end
