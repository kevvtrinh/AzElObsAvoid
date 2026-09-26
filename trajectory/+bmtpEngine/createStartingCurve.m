function startingCurve = createStartingCurve(solverRequest)
%% Section 0: Header & Readme
% SYNTAX
%   startingCurve = bmtpEngine.createStartingCurve(solverRequest)
%**************************************************************************
% PURPOSE
%   - Turn the visibility route into a starting curve for BMTP.
%   - Assign initial segment durations and identify regions active during them.
%**************************************************************************
% INPUTS
%   - solverRequest (scalar struct)
%       Checked BMTP inputs with static or timed obstacle regions.
%**************************************************************************
% OUTPUTS
%   - startingCurve (scalar struct)
%       Initial curve and timing, called a warm start because BMTP begins
%       its calculations from these values. This is not yet a validated motion.
%**************************************************************************
% UNITS
%   - Position is coordinate units and time is seconds.
%**************************************************************************

%% Section 1: Prepare The Starting Route
% Keep the requested start and goal positions. Redundant points can be
% removed from an untimed route; repeated positions on timed routes may be waits.
route_units = double(solverRequest.Seed.position_units);
route_units([1 end], :) = [solverRequest.InitialState.position_units; solverRequest.GoalState.position_units];

divideRouteByLength = solverRequest.Options.GoalTimeMode == "earliestArrival" && ...
    ~isfield(solverRequest.Coverage, 'ActiveTimeInterval_s') && solverRequest.SplitCount > 1;
if divideRouteByLength
    route_units = removeRedundantRouteVertices(route_units);
end
originalSegmentCount = size(route_units, 1) - 1;

%% Section 2: Build The Starting Curve And Assign Segment Times
% Control points define the curve's shape; they are not all points the vehicle
% visits. Store them as [segment, control point, x/y], with degree + 1 per segment.
% When placing controls along one route edge, repeat each endpoint three times
% to give the curve zero endpoint velocity and acceleration initially.
curveDegree            = solverRequest.Degree;
interpolationFractions = reshape(min(1, max(0, ((0:curveDegree) - 2) / (curveDegree - 4))), 1, [], 1);

if solverRequest.Options.GoalTimeMode == "fixedArrival"
    % Divide the route into curve segments. Obstacle sample spacing does
    % not set the segment count. Fixed arrival divides exactly the available
    % trip time among segments.
    useTimedSolver = solverRequest.UsesTimeScopedSolver;
    if useTimedSolver
        minimumSegmentCount = 16;
    else
        minimumSegmentCount = 8;
    end
    segmentCount = max(minimumSegmentCount, originalSegmentCount);
    if useTimedSolver
        segmentCount       = max(segmentCount, originalSegmentCount * solverRequest.SplitCount);
        segmentCountByEdge = allocateSegmentsByWeight(diff(solverRequest.Seed.tau), ...
            segmentCount);
        routeProgress           = subdivideEdges(solverRequest.Seed.tau(:), segmentCountByEdge);
        segmentCount            = numel(routeProgress) - 1;
        solverRoute_units       = interp1(solverRequest.Seed.tau, route_units, routeProgress, 'linear');
        segmentStart_units      = reshape(solverRoute_units(1:end - 1, :), segmentCount, 1, 2);
        segmentEnd_units        = reshape(solverRoute_units(2:end, :), segmentCount, 1, 2);
        controlPoint_units      = (1 - interpolationFractions) .* segmentStart_units + ...
            interpolationFractions .* segmentEnd_units;

        segmentTime_s           = diff(routeProgress) * solverRequest.MotionHorizon_s;
        relativeSegmentDuration = segmentTime_s / mean(segmentTime_s);
        outputRoute_units       = solverRoute_units;
    else
        % Without obstacle timing, sample the route at evenly spaced progress
        % values and give every curve segment the same duration.
        controlPointProgress        = ((0:segmentCount - 1).' + (0:curveDegree) / curveDegree) / segmentCount;
        interpolatedPositions_units = interp1( ...
            solverRequest.Seed.tau, route_units, controlPointProgress(:), 'linear');
        controlPoint_units          = reshape(interpolatedPositions_units, segmentCount, curveDegree + 1, 2);

        segmentTime_s = repmat( ...
            solverRequest.MotionHorizon_s / segmentCount, segmentCount, 1);
        relativeSegmentDuration = ones(segmentCount, 1);
        outputRoute_units       = route_units;
    end

    % Remove accumulated rounding error so the durations sum to the fixed trip time up to rounding.
    segmentTime_s = segmentTime_s * solverRequest.MotionHorizon_s / sum(segmentTime_s);

    % The active-pair mask must use the durations returned to later stages.
    % Row = curve segment, column = obstacle region. Mark a pair when their
    % time intervals overlap, even if the curve is far from that obstacle.
    segmentBoundaryTime_s = solverRequest.InitialState.time_s + [0; cumsum(segmentTime_s)];
    regionActiveBySegment = bmtpEngine.separation.activePairsOnClock( ...
        segmentBoundaryTime_s, solverRequest.Coverage, numel(solverRequest.Regions_units));

    duration_s            = solverRequest.MotionHorizon_s;
    endpointControlTime_s = segmentTime_s;
    routeWasResampled     = true;
elseif solverRequest.UsesVariableClock
    % For earliest arrival, the solver may adjust individual segment durations.
    if isfield(solverRequest.Coverage, 'BreakTime_s')
        % Preserve every route point when adding curve segments. Two equal
        % positions at different progress values describe a wait, so both stay.
        routeProgress       = double(solverRequest.Seed.tau(:));
        minimumSegmentCount = max(20, originalSegmentCount * solverRequest.SplitCount);
        segmentCountByEdge  = allocateSegmentsByWeight(diff(routeProgress), ...
            minimumSegmentCount);
        segmentProgress             = subdivideEdges(routeProgress, segmentCountByEdge);
        relativeSegmentDuration     = diff(segmentProgress) / mean(diff(segmentProgress));
        endpointControlTime_s        = solverRequest.MotionHorizon_s * diff(segmentProgress);
        segmentCount                = numel(endpointControlTime_s);
        controlPointProgress        = segmentProgress(1:end - 1) + ...
            diff(segmentProgress) .* ((0:curveDegree) / curveDegree);
        interpolatedPositions_units = interp1( ...
            solverRequest.Seed.tau, route_units, controlPointProgress(:), 'linear');
        controlPoint_units          = reshape(interpolatedPositions_units, segmentCount, curveDegree + 1, 2);
        routeWasResampled            = true;
    else
        [controlPoint_units, endpointControlTime_s, segmentCount] = ...
            createCurveWithoutAssignedTimes( ...
            route_units, solverRequest, divideRouteByLength, interpolationFractions);
        relativeSegmentDuration = endpointControlTime_s(:) / mean(endpointControlTime_s);
        routeWasResampled       = false;
    end

    % Start with the route's assigned travel times. A route alone has not
    % yet passed the motion limits. Stretching it now would change which
    % moving obstacles it encounters before BMTP solves the motion.
    % A ratio of 0.5 assigns half the average segment duration; 2 assigns twice it.
    commonSegmentTime_s = solverRequest.SeedMotionDuration_s / sum(relativeSegmentDuration);
    segmentTime_s       = commonSegmentTime_s * relativeSegmentDuration;
    duration_s          = sum(segmentTime_s);

    % Row = curve segment, column = obstacle region. Mark a pair when their
    % time intervals overlap, even if the curve is far from that obstacle.
    segmentBoundaryTime_s = solverRequest.InitialState.time_s + [0; cumsum(segmentTime_s)];
    regionActiveBySegment = bmtpEngine.separation.activePairsOnClock( ...
        segmentBoundaryTime_s, solverRequest.Coverage, numel(solverRequest.Regions_units));
    outputRoute_units = route_units;
else
    % With an untimed route, estimate durations from the motion limits.
    [controlPoint_units, segmentTime_s, segmentCount] = ...
        createCurveWithoutAssignedTimes( ...
        route_units, solverRequest, divideRouteByLength, interpolationFractions);
    regionActiveBySegment   = true(segmentCount, numel(solverRequest.Regions_units));
    segmentTime_s           = segmentTime_s(:);
    duration_s              = sum(segmentTime_s);
    relativeSegmentDuration = segmentTime_s / mean(segmentTime_s);
    endpointControlTime_s   = segmentTime_s;
    outputRoute_units       = route_units;
    routeWasResampled       = false;
end

% Set the first and last curve controls from the requested position, velocity,
% and acceleration. BMTP still has to solve the whole motion and its joins.
controlPoint_units = bmtpEngine.motion.imposeEndpointControls(controlPoint_units, ...
    endpointControlTime_s, solverRequest.InitialState, solverRequest.GoalState);

%% Section 3: Return The Starting Curve And Timing
startingCurve = struct();
startingCurve.Route_units              = outputRoute_units;
startingCurve.ControlPoint_units       = controlPoint_units;
startingCurve.SegmentTime_s            = segmentTime_s(:);
startingCurve.Duration_s               = duration_s;
startingCurve.SegmentRatio             = relativeSegmentDuration;
startingCurve.SegmentCount             = segmentCount;
startingCurve.RegionActiveBySegment    = regionActiveBySegment;
startingCurve.OriginalSeedSegmentCount = originalSegmentCount;
startingCurve.WarmRouteResampled       = routeWasResampled;
end

%% Section 4: Local Functions

function [controlPoint_units, segmentTime_s, segmentCount] = createCurveWithoutAssignedTimes( ...
    route_units, solverRequest, divideRouteByLength, interpolationFractions)
    % Divide a route without assigned times into curve segments, then
    % calculate initial durations from the speed, acceleration, and jerk limits.
    if divideRouteByLength
        % Longer route edges get more curve segments, independent of how many
        % redundant points originally described each straight edge.
        originalSegmentCount        = size(route_units, 1) - 1;
        minimumSteeringSegmentCount = 2 * (solverRequest.Degree - 2);
        targetSegmentCount          = max(minimumSteeringSegmentCount, ...
            originalSegmentCount * solverRequest.SplitCount);
        segmentCountByEdge = allocateSegmentsByWeight( ...
            vecnorm(diff(route_units), 2, 2), targetSegmentCount);
        solverRoute_units = subdivideEdges(route_units, segmentCountByEdge);
    else
        solverRoute_units = route_units;
    end

    curveDegree        = solverRequest.Degree;
    segmentCount       = size(solverRoute_units, 1) - 1;
    segmentStart_units = reshape(solverRoute_units(1:end - 1, :), segmentCount, 1, 2);
    segmentEnd_units   = reshape(solverRoute_units(2:end, :), segmentCount, 1, 2);
    controlPoint_units = (1 - interpolationFractions) .* segmentStart_units + ...
        interpolationFractions .* segmentEnd_units;
    segmentTime_s = bmtpEngine.motion.findRequiredSegmentTime(controlPoint_units, solverRequest.Limits);
    if solverRequest.SplitCount > 1 && ~divideRouteByLength
        % Split each Bezier curve exactly: the path stays the same, while the
        % solver gains more segments it can adjust during optimization.
        subdivisionCount         = solverRequest.SplitCount;
        subdividedControls_units = zeros(segmentCount * subdivisionCount, curveDegree + 1, 2);
        for segmentIndex = 1:segmentCount
            for subdivisionIndex = 1:subdivisionCount
                refinedSegmentIndex     = (segmentIndex - 1) * subdivisionCount + subdivisionIndex;
                segmentProgressInterval = [subdivisionIndex - 1, subdivisionIndex] / subdivisionCount;
                subdividedControls_units(refinedSegmentIndex, :, :) = bmtpEngine.motion.restrictBezier( ...
                    squeeze(controlPoint_units(segmentIndex, :, :)), segmentProgressInterval);
            end
        end
        controlPoint_units = subdividedControls_units;
        segmentTime_s      = repelem(segmentTime_s, subdivisionCount) / subdivisionCount;
        segmentCount       = segmentCount * subdivisionCount;
    end
end

function route_units = removeRedundantRouteVertices(route_units)
    % Remove duplicate points and points on a straight continuation of the
    % route. Keep a point where the route reverses direction.
    % For example, [0 0; 1 0; 2 0] becomes [0 0; 2 0].
    keepVertex              = true(size(route_units, 1), 1);
    coordinateScale_units   = max(1, max(abs(route_units), [], 'all'));
    roundoffTolerance_units = 128 * eps(coordinateScale_units);
    routeChanged            = true;
    while routeChanged
        routeChanged      = false;
        keptVertexIndices = find(keepVertex);
        for keptListIndex = 2:numel(keptVertexIndices) - 1
            previousVertex_units     = route_units(keptVertexIndices(keptListIndex - 1), :);
            currentVertex_units      = route_units(keptVertexIndices(keptListIndex), :);
            nextVertex_units         = route_units(keptVertexIndices(keptListIndex + 1), :);
            incomingEdge_units       = currentVertex_units - previousVertex_units;
            outgoingEdge_units       = nextVertex_units - currentVertex_units;
            incomingEdgeLength_units = norm(incomingEdge_units);
            outgoingEdgeLength_units = norm(outgoingEdge_units);
            edgesHaveSameDirection   = dot(incomingEdge_units, outgoingEdge_units) >= -roundoffTolerance_units ^ 2;

            % The cross product is zero for exactly aligned edges. Scale the
            % allowed rounding error by edge length to compare distance error.
            twiceArea_units2 = abs(incomingEdge_units(1) * outgoingEdge_units(2) - ...
                incomingEdge_units(2) * outgoingEdge_units(1));
            edgesAreCollinear = twiceArea_units2 <= ...
                roundoffTolerance_units * (incomingEdgeLength_units + outgoingEdgeLength_units);
            vertexIsRedundant = incomingEdgeLength_units <= roundoffTolerance_units || ...
                outgoingEdgeLength_units <= roundoffTolerance_units || ...
                (edgesHaveSameDirection && edgesAreCollinear);
            if vertexIsRedundant
                keepVertex(keptVertexIndices(keptListIndex)) = false;
                routeChanged = true;
                % Restart because deleting this point changes its neighbors.
                break
            end
        end
    end

    route_units = route_units(keepVertex, :);
end

function segmentCountByEdge = allocateSegmentsByWeight(edgeWeights, targetSegmentCount)
    % Give every edge one segment, then distribute the rest in proportion to
    % edge length or elapsed time, whichever the caller supplies as the weight.
    edgeWeights           = edgeWeights(:);
    edgeCount             = numel(edgeWeights);
    segmentCountByEdge    = ones(edgeCount, 1);
    remainingSegmentCount = targetSegmentCount - edgeCount;
    if remainingSegmentCount <= 0 || sum(edgeWeights) <= 0
        return
    end
    % Round down first. Give leftover segments to the largest fractional
    % shares; when shares tie, the earlier route edge gets the segment.
    idealExtraSegmentCounts  = remainingSegmentCount * edgeWeights / sum(edgeWeights);
    extraSegmentCounts       = floor(idealExtraSegmentCounts);
    segmentCountByEdge       = segmentCountByEdge + extraSegmentCounts;
    unassignedSegmentCount   = remainingSegmentCount - sum(extraSegmentCounts);
    [~, edgeAllocationOrder] = sortrows([-mod(idealExtraSegmentCounts, 1), (1:edgeCount).'], [1 2]);
    selectedEdgeIndices      = edgeAllocationOrder(1:unassignedSegmentCount);
    segmentCountByEdge(selectedEdgeIndices) = segmentCountByEdge(selectedEdgeIndices) + 1;
end

function refinedValues = subdivideEdges(values, segmentCountByEdge)
    % Divide each edge into the requested number of equal pieces. This works
    % for a column of route progress values or rows of [x y] positions.
    % Shared endpoints appear once, and the final endpoint is kept below.
    refinedValues = zeros(sum(segmentCountByEdge) + 1, size(values, 2));
    nextRowIndex  = 1;
    for edgeIndex = 1:numel(segmentCountByEdge)
        edgeSegmentCount       = segmentCountByEdge(edgeIndex);
        interpolationFractions = (0:edgeSegmentCount - 1).' / edgeSegmentCount;
        targetRowIndices       = nextRowIndex:nextRowIndex + edgeSegmentCount - 1;
        refinedValues(targetRowIndices, :) = values(edgeIndex, :) + ...
            interpolationFractions .* (values(edgeIndex + 1, :) - values(edgeIndex, :));
        nextRowIndex = nextRowIndex + edgeSegmentCount;
    end
    refinedValues(end, :) = values(end, :);
end
