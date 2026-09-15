function warmStart = createWarmStart(request)
%% Section 0: Header & Readme
% SYNTAX
%   warmStart = bmtpEngine.createWarmStart(request)
%**************************************************************************
% PURPOSE
%   - Convert an exact visibility route into BMTP Bezier controls.
%**************************************************************************
% INPUTS
%   - request (scalar struct)
%       Validated BMTP request with static or timed exclusion cells.
%**************************************************************************
% OUTPUTS
%   - warmStart (scalar struct)
%       Route, controls, timing, and active region pairs.
%**************************************************************************
% UNITS
%   - Position is coordinate units and time is seconds.
%**************************************************************************

%% Section 1: Use The Exact Visibility Route
route_units = double(request.Seed.position_units);
route_units([1 end], :) = [request.InitialState.position_units; request.GoalState.position_units];
usesLengthBalancedMesh = request.Options.GoalTimeMode == "earliestArrival" && ...
    ~isfield(request.Coverage, 'ActiveTimeInterval_s') && request.SplitCount > 1;
if usesLengthBalancedMesh
    route_units = removeRedundantRouteVertices(route_units);
end
originalSegmentCount = size(route_units, 1) - 1;
solverRoute_units     = route_units;
if usesLengthBalancedMesh
    minimumSteeringSegmentCount = 2 * (request.Degree - 2);
    targetSegmentCount          = max(minimumSteeringSegmentCount, ...
        originalSegmentCount * request.SplitCount);
    segmentCountByEdge          = allocateSegmentsByMeasure( ...
        vecnorm(diff(route_units), 2, 2), targetSegmentCount);
    solverRoute_units           = splitByCount(route_units, segmentCountByEdge);
end
segmentCount          = size(solverRoute_units, 1) - 1;
regionActiveBySegment = true(segmentCount, numel(request.Regions_units));

%% Section 2: Create Linear Rest-To-Rest Controls
degree             = request.Degree;
fraction           = reshape(min(1, max(0, ((0:degree) - 2) / (degree - 4))), 1, [], 1);
start_units        = reshape(solverRoute_units(1:end - 1, :), segmentCount, 1, 2);
finish_units       = reshape(solverRoute_units(2:end, :), segmentCount, 1, 2);
controlPoint_units = (1 - fraction) .* start_units + fraction .* finish_units;
segmentTime_s      = bmtpEngine.findRequiredSegmentTime(controlPoint_units, request.Limits);
if request.SplitCount > 1 && ~usesLengthBalancedMesh
    subdivisionCount = request.SplitCount;
    refined_units    = zeros(segmentCount * subdivisionCount, degree + 1, 2);
    for segmentIndex = 1:segmentCount
        for subdivisionIndex = 1:subdivisionCount
            targetIndex    = (segmentIndex - 1) * subdivisionCount + subdivisionIndex;
            sourceInterval = [subdivisionIndex - 1, subdivisionIndex] / subdivisionCount;
            refined_units(targetIndex, :, :) = bmtpEngine.restrictBezier( ...
                squeeze(controlPoint_units(segmentIndex, :, :)), sourceInterval);
        end
    end
    controlPoint_units    = refined_units;
    segmentTime_s         = repelem(segmentTime_s, subdivisionCount) / subdivisionCount;
    regionActiveBySegment = repelem(regionActiveBySegment, subdivisionCount, 1);
    segmentCount          = segmentCount * subdivisionCount;
end

%% Section 3: Return The Solver Initialization
warmStart                          = struct();
warmStart.Route_units              = route_units;
warmStart.ControlPoint_units       = controlPoint_units;
warmStart.SegmentTime_s            = segmentTime_s(:);
warmStart.Duration_s               = sum(segmentTime_s);
warmStart.SegmentRatio             = segmentTime_s(:) / mean(segmentTime_s);
warmStart.SegmentCount             = segmentCount;
warmStart.RegionActiveBySegment    = regionActiveBySegment;
warmStart.OriginalSeedSegmentCount = originalSegmentCount;
warmStart.WarmRouteResampled       = false;
if request.UsesVariableClock && isfield(request.Coverage, 'BreakTime_s')
    sourceBreaks_s = request.Coverage.BreakTime_s;
    % The timed guide's knots are physical events. Preserve every knot and
    % subdivide its normalized intervals so changing the arrival clock scales
    % the complete guide instead of deleting its waits.
    routeTau            = double(request.Seed.tau(:));
    minimumSegmentCount = max([20, numel(sourceBreaks_s) - 1, ...
        originalSegmentCount * request.SplitCount]);
    segmentCountByEdge  = allocateSegmentsByMeasure(diff(routeTau), ...
        minimumSegmentCount);
    meshTau             = splitByCount(routeTau, segmentCountByEdge);
    segmentRatio       = diff(meshTau) / mean(diff(meshTau));
    segmentTime_s      = request.MotionHorizon_s * diff(meshTau);
    segmentCount       = numel(segmentTime_s);
    tau                = meshTau(1:end - 1) + diff(meshTau) .* ((0:degree) / degree);
    breakTime_s        = request.InitialState.time_s + [0; cumsum(segmentTime_s)];
    controls_units     = interp1(request.Seed.tau, route_units, tau(:), 'linear');
    controlPoint_units = reshape(controls_units, segmentCount, degree + 1, 2);
    controlPoint_units(1, 1:3, :) = reshape( ...
        repmat(request.InitialState.position_units, 3, 1), 1, 3, 2);
    controlPoint_units(end, end - 2:end, :) = reshape( ...
        repmat(request.GoalState.position_units, 3, 1), 1, 3, 2);
    intervals_s                        = request.Coverage.ActiveTimeInterval_s;
    warmStart.ControlPoint_units       = controlPoint_units;
    warmStart.SegmentTime_s            = segmentTime_s(:);
    warmStart.SegmentRatio             = segmentRatio;
    warmStart.Duration_s               = sum(segmentTime_s);
    warmStart.SegmentCount             = segmentCount;
    warmStart.RegionActiveBySegment    = breakTime_s(1:end - 1) < intervals_s(:, 2).' & ...
        breakTime_s(2:end) > intervals_s(:, 1).';
    warmStart.WarmRouteResampled = true;
end

if request.Options.GoalTimeMode == "fixedArrival"
    % The motion mesh follows the guide, not the obstacle sampling frequency.
    % Every source interval still constrains its exact overlap with these spans.
    seedUsesTimedSolver = request.UsesTimeScopedSolver;
    minimumSegmentCount = 8;
    if seedUsesTimedSolver
        minimumSegmentCount = 16;
    end
    segmentCount = max(minimumSegmentCount, originalSegmentCount);
    if seedUsesTimedSolver
        segmentCount       = max(segmentCount, originalSegmentCount * request.SplitCount);
        segmentCountByEdge = allocateSegmentsByMeasure(diff(request.Seed.tau), ...
            segmentCount);
        routeTau           = splitByCount(request.Seed.tau(:), segmentCountByEdge);
        segmentCount       = numel(routeTau) - 1;
        timedRoute_units   = interp1(request.Seed.tau, route_units, routeTau, 'linear');
        start_units        = reshape(timedRoute_units(1:end - 1, :), segmentCount, 1, 2);
        finish_units       = reshape(timedRoute_units(2:end, :), segmentCount, 1, 2);
        warmStart.ControlPoint_units = ...
            (1 - fraction) .* start_units + fraction .* finish_units;
        warmStart.Route_units        = timedRoute_units;
        warmStart.SegmentTime_s      = diff(routeTau) * request.MotionHorizon_s;
        warmStart.SegmentRatio       = warmStart.SegmentTime_s / ...
            mean(warmStart.SegmentTime_s);
    else
        tau                          = ((0:segmentCount - 1).' + (0:degree) / degree) / segmentCount;
        controls_units               = interp1(request.Seed.tau, route_units, tau(:), 'linear');
        warmStart.ControlPoint_units = reshape(controls_units, segmentCount, degree + 1, 2);
        warmStart.SegmentTime_s      = repmat( ...
            request.MotionHorizon_s / segmentCount, segmentCount, 1);
        warmStart.SegmentRatio       = ones(segmentCount, 1);
    end
    warmStart.SegmentCount          = segmentCount;
    warmStart.RegionActiveBySegment = true(segmentCount, numel(request.Regions_units));
    if isfield(request.Coverage, 'ActiveTimeInterval_s')
        breaks_s    = request.InitialState.time_s + [0; cumsum(warmStart.SegmentTime_s)];
        intervals_s = request.Coverage.ActiveTimeInterval_s;
        warmStart.RegionActiveBySegment = breaks_s(1:end - 1) < intervals_s(:, 2).' & ...
            breaks_s(2:end) > intervals_s(:, 1).';
    end
    warmStart.WarmRouteResampled = true;
end
if request.Options.GoalTimeMode == "fixedArrival"
    warmStart.SegmentTime_s = warmStart.SegmentTime_s * request.MotionHorizon_s / ...
        sum(warmStart.SegmentTime_s);
    warmStart.Duration_s    = request.MotionHorizon_s;
end
warmStart.ControlPoint_units = bmtpEngine.imposeEndpointControls(warmStart.ControlPoint_units, ...
    warmStart.SegmentTime_s, request.InitialState, request.GoalState);
if request.UsesVariableClock
    % A timed visibility route is a geometric corridor proposal at one
    % supplied physical clock, not a complete feasible motion. Initialize
    % its exact obstacle planes on that clock. Stretching the unsolved guide
    % to satisfy dynamics first changes which moving geometry it encounters
    % and therefore corrupts the proposal before BMTP sees it.
    commonSegmentTime_s = request.SeedMotionDuration_s / ...
        sum(warmStart.SegmentRatio);
    warmStart.SegmentTime_s = commonSegmentTime_s * warmStart.SegmentRatio;
    warmStart.Duration_s    = sum(warmStart.SegmentTime_s);
    if isfield(request.Coverage, 'ActiveTimeInterval_s')
        breaks_s    = request.InitialState.time_s + [0; cumsum(warmStart.SegmentTime_s)];
        intervals_s = request.Coverage.ActiveTimeInterval_s;
        warmStart.RegionActiveBySegment = breaks_s(1:end - 1) < intervals_s(:, 2).' & ...
            breaks_s(2:end) > intervals_s(:, 1).';
    end
end
end

%% Section 4: Local Functions

function route_units = removeRedundantRouteVertices(route_units)
    % Remove zero-length and collinear interior route vertices.
    keep            = true(size(route_units, 1), 1);
    scale_units     = max(1, max(abs(route_units), [], 'all'));
    tolerance_units = 128 * eps(scale_units);
    routeChanged = true;
    while routeChanged
        routeChanged = false;
        keptIndices  = find(keep);
        for localIndex = 2:numel(keptIndices) - 1
            previous_units  = route_units(keptIndices(localIndex - 1), :);
            current_units   = route_units(keptIndices(localIndex), :);
            following_units = route_units(keptIndices(localIndex + 1), :);
            firstEdge_units   = current_units - previous_units;
            secondEdge_units  = following_units - current_units;
            firstLength_units  = norm(firstEdge_units);
            secondLength_units = norm(secondEdge_units);
            edgesHaveSameDirection = dot(firstEdge_units, secondEdge_units) >= -tolerance_units ^ 2;
            twiceArea_units2       = abs(firstEdge_units(1) * secondEdge_units(2) - ...
                firstEdge_units(2) * secondEdge_units(1));
            edgesAreCollinear      = twiceArea_units2 <= ...
                tolerance_units * (firstLength_units + secondLength_units);
            vertexIsRedundant      = firstLength_units <= tolerance_units || ...
                secondLength_units <= tolerance_units || ...
                (edgesHaveSameDirection && edgesAreCollinear);
            if vertexIsRedundant
                keep(keptIndices(localIndex)) = false;
                routeChanged                  = true;
                break
            end
        end
    end
    route_units = route_units(keep, :);
end

function segmentCountByEdge = allocateSegmentsByMeasure(edgeMeasures, targetSegmentCount)
    % Apportion a fixed segment count by edge measure and stable edge order.
    edgeMeasures       = edgeMeasures(:);
    edgeCount          = numel(edgeMeasures);
    segmentCountByEdge = ones(edgeCount, 1);
    remainingCount     = targetSegmentCount - edgeCount;
    if remainingCount <= 0 || sum(edgeMeasures) <= 0
        return
    end
    exactCounts        = remainingCount * edgeMeasures / sum(edgeMeasures);
    additionalCounts   = floor(exactCounts);
    segmentCountByEdge = segmentCountByEdge + additionalCounts;
    unassignedCount    = remainingCount - sum(additionalCounts);
    [~, allocationOrder] = sortrows([-mod(exactCounts, 1), (1:edgeCount).'], [1 2]);
    selectedEdgeIndices  = allocationOrder(1:unassignedCount);
    segmentCountByEdge(selectedEdgeIndices) = segmentCountByEdge(selectedEdgeIndices) + 1;
end

function refinedValues = splitByCount(values, segmentCountByEdge)
    % Interpolate one knot column or a two-column route at the identical
    % fractions of every supplied edge.
    refinedValues = zeros(sum(segmentCountByEdge) + 1, size(values, 2));
    nextRowIndex  = 1;
    for edgeIndex = 1:numel(segmentCountByEdge)
        edgeSegmentCount = segmentCountByEdge(edgeIndex);
        fraction         = (0:edgeSegmentCount - 1).' / edgeSegmentCount;
        targetRowIndices = nextRowIndex:nextRowIndex + edgeSegmentCount - 1;
        refinedValues(targetRowIndices, :) = values(edgeIndex, :) + ...
            fraction .* (values(edgeIndex + 1, :) - values(edgeIndex, :));
        nextRowIndex = nextRowIndex + edgeSegmentCount;
    end
    refinedValues(end, :) = values(end, :);
end
