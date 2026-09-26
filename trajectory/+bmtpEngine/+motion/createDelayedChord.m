function [controlPoint_units, segmentTime_s, powerCoefficients_units, departureTiming] = ...
    createDelayedChord(solverRequest)
%% Section 0: Header & Readme
% SYNTAX
%   [controlPoint_units, segmentTime_s, powerCoefficients_units, departureTiming] = ...
%       bmtpEngine.motion.createDelayedChord(solverRequest)
%**************************************************************************
% PURPOSE
%   - Try a smooth, straight-line trip after waiting at the start. Convert
%     each moving obstacle's path crossing into departure delays that would
%     collide, including contact during the wait. Use the first remaining
%     delay that fits the horizon. The public validator must still check
%     the complete returned motion.
%**************************************************************************
% INPUTS
%   - solverRequest (scalar struct)
%       Checked rest-to-rest start and goal, motion limits, horizon, and
%       protected convex regions. Coverage supplies each moving region's
%       active times and end shape; Options supplies collision clearance.
%**************************************************************************
% OUTPUTS
%   - controlPoint_units (S-by-6-by-2 numeric array)
%       Bezier controls for S segments, including a wait when needed;
%       the last dimension selects x or y. Empty if no departure fits.
%   - segmentTime_s (numeric column)
%       Duration of each wait or move segment, or empty if unavailable.
%   - powerCoefficients_units (S-by-2-by-6 numeric array)
%       The same positions as c0 + c1 x u + ... + c5 x u^5, with local
%       segment fraction u from 0 to 1. Empty if no departure fits.
%   - departureTiming (scalar struct)
%       DepartureDelay_s is the earliest delay outside the calculated
%       blocked intervals. Available says whether it fits the horizon.
%**************************************************************************
% UNITS
%   - Positions and coefficients: coordinate units; times: seconds;
%     progress and segment fractions: dimensionless.
%**************************************************************************

%% Section 1: Build The Motion And Its Progress Along The Straight Path

[controlPoint_units, segmentTime_s, powerCoefficients_units] = bmtpEngine.motion.createC3Chord( ...
    solverRequest.InitialState.position_units, solverRequest.GoalState.position_units, solverRequest.Limits);
segmentStartTime_s = [0; cumsum(segmentTime_s(1:end - 1))];
segmentTiming     = struct('StartTime_s', segmentStartTime_s, 'SegmentTime_s', segmentTime_s);

motionDuration_s = sum(segmentTime_s);
maximumWait_s    = solverRequest.MotionHorizon_s - motionDuration_s;

% Project each x/y position onto the start-to-goal line. The squared
% displacement appears in that projection; the perpendicular unit vector
% measures how far a region sits to either side of the path.
start_units        = solverRequest.InitialState.position_units;
displacement_units = solverRequest.GoalState.position_units - start_units;
displacementLengthSquared_units2 = sum(displacement_units .^ 2);
pathNormalDirection = [-displacement_units(2), displacement_units(1)] / ...
    sqrt(displacementLengthSquared_units2);

% Convert the position polynomial into progress: 0 at start, 1 at goal.
% Subtract start only from its constant coefficient; higher powers already
% describe changes from the start. This links each path position to its
% travel time after departure.
positionOffsetCoefficients_units = powerCoefficients_units;
positionOffsetCoefficients_units(:, :, 1) = positionOffsetCoefficients_units(:, :, 1) - start_units;
progressCoefficients = reshape(sum( ...
    positionOffsetCoefficients_units .* reshape(displacement_units, 1, 2, 1), 2), [], 6) / ...
    displacementLengthSquared_units2;

% The smallest and largest Bezier controls bound the progress slope over
% each segment. This slope is per unit of local fraction, not per second.
progressSlopeCoefficients = progressCoefficients(:, 2:end) .* (1:5);
progressSlopeControls     = bmtpEngine.motion.powerToBernstein(progressSlopeCoefficients.');
minimumProgressSlope      = min(progressSlopeControls, [], 1).';
maximumProgressSlope      = max(progressSlopeControls, [], 1).';

endRegions_units = {};
if isfield(solverRequest.Coverage, 'EndRegions_units')
    endRegions_units = solverRequest.Coverage.EndRegions_units;
end
[~, roundoffReserve_units] = bmtpEngine.validation.createCoordinateTolerances(start_units, ...
    solverRequest.GoalState.position_units, solverRequest.Limits.xInterval_units, ...
    solverRequest.Limits.yInterval_units, solverRequest.Regions_units, endRegions_units);

% Prepared regions already contain the obstacle safety margin. Add only
% the validator's clearance and a rounding reserve for this calculation.
requiredClearance_units = (1 + 2 ^ 20 * eps) * solverRequest.Options.CollisionClearanceTolerance_units + ...
    3 * roundoffReserve_units;
blockedDelayIntervals_s   = zeros(0, 2);
pathMinimum_units         = min(start_units, solverRequest.GoalState.position_units);
pathMaximum_units         = max(start_units, solverRequest.GoalState.position_units);
clearanceAcrossPath_units = requiredClearance_units * sum(abs(pathNormalDirection));
clearanceProgressFraction        = requiredClearance_units * sum(abs(displacement_units)) / displacementLengthSquared_units2;

%% Section 2: Find Departure Delays Blocked By Each Moving Region

for regionIndex = 1:numel(solverRequest.Regions_units)
    % A timed region matters only during its active interval. Otherwise
    % inspect the full requested motion horizon.
    activeInterval_s = solverRequest.InitialState.time_s + [0, solverRequest.MotionHorizon_s];
    if isfield(solverRequest.Coverage, 'ActiveTimeInterval_s')
        activeInterval_s = solverRequest.Coverage.ActiveTimeInterval_s(regionIndex, :);
    end
    regionVertices_units = bmtpEngine.separation.regionOnInterval( ...
        solverRequest.Regions_units{regionIndex}, solverRequest.Coverage, regionIndex, activeInterval_s);

    % Skip a region only when its entire movement misses the path. Check
    % x/y bounds, distance across the path, and distance along the path.
    endpointVertices_units = [regionVertices_units(:, :, 1); regionVertices_units(:, :, end)];
    coordinateScale_units  = max([1; abs(endpointVertices_units(:)); ...
        abs(start_units(:)); abs(solverRequest.GoalState.position_units(:)); requiredClearance_units]);
    coordinateRoundingAllowance_units = 4096 * eps(coordinateScale_units);
    regionMinimum_units = min(endpointVertices_units, [], 1) - ...
        requiredClearance_units - coordinateRoundingAllowance_units;
    regionMaximum_units = max(endpointVertices_units, [], 1) + ...
        requiredClearance_units + coordinateRoundingAllowance_units;
    boxesAreDisjoint = any( ...
        regionMaximum_units < pathMinimum_units - coordinateRoundingAllowance_units | ...
        regionMinimum_units > pathMaximum_units + coordinateRoundingAllowance_units);
    distanceFromPathLine_units    = (endpointVertices_units - start_units) * pathNormalDirection.';
    normalRoundingAllowance_units = 4096 * eps(max([ ...
        1; abs(distanceFromPathLine_units); clearanceAcrossPath_units]));
    missesPathLine = min(distanceFromPathLine_units) > ...
        clearanceAcrossPath_units + normalRoundingAllowance_units || ...
        max(distanceFromPathLine_units) < -clearanceAcrossPath_units - normalRoundingAllowance_units;
    vertexProgress = (endpointVertices_units - start_units) * displacement_units.' / displacementLengthSquared_units2;
    progressRoundingAllowance = 4096 * eps(max([1; abs(vertexProgress); clearanceProgressFraction]));
    missesPathExtent = max(vertexProgress) < -clearanceProgressFraction - progressRoundingAllowance || ...
        min(vertexProgress) > 1 + clearanceProgressFraction + progressRoundingAllowance;
    if boxesAreDisjoint || missesPathLine || missesPathExtent
        continue
    end

    % Add clearance to both endpoint polygons. Their [x, y, time] points
    % enclose the moving region between those times. Intersect that volume
    % with the plane through the straight path. A line between points on
    % opposite sides crosses the plane; keep every such crossing.
    expandedStartVertices_units = expandRegionForClearance(regionVertices_units(:, :, 1), requiredClearance_units);
    expandedEndVertices_units   = expandRegionForClearance(regionVertices_units(:, :, end), requiredClearance_units);
    regionSpaceTimePoints       = [expandedStartVertices_units, ...
        repmat(activeInterval_s(1), size(expandedStartVertices_units, 1), 1); ...
        expandedEndVertices_units, repmat(activeInterval_s(2), size(expandedEndVertices_units, 1), 1)];
    distanceFromPathLine_units = (regionSpaceTimePoints(:, 1:2) - start_units) * pathNormalDirection.';
    [positivePointIndices, negativePointIndices] = ndgrid( ...
        find(distanceFromPathLine_units > 0), find(distanceFromPathLine_units < 0));
    positivePointIndices  = positivePointIndices(:);
    negativePointIndices  = negativePointIndices(:);
    pathCrossingFractions = distanceFromPathLine_units(positivePointIndices) ./ ...
        (distanceFromPathLine_units(positivePointIndices) - distanceFromPathLine_units(negativePointIndices));
    pathIntersectionPoints = [regionSpaceTimePoints(distanceFromPathLine_units == 0, :); ...
        regionSpaceTimePoints(positivePointIndices, :) + pathCrossingFractions .* ...
        (regionSpaceTimePoints(negativePointIndices, :) - regionSpaceTimePoints(positivePointIndices, :))];
    if isempty(pathIntersectionPoints)
        continue;
    end

    % Express each crossing as [path progress, absolute time]. The outer
    % boundary covers all possible path occupancy by this region. When
    % every point is on one line, its two ends describe that boundary.
    progressTimeBoundary = unique([ ...
        (pathIntersectionPoints(:, 1:2) - start_units) * displacement_units.' / displacementLengthSquared_units2, ...
        pathIntersectionPoints(:, 3)], 'rows');
    if size(progressTimeBoundary, 1) > 2 && rank(progressTimeBoundary - progressTimeBoundary(1, :)) == 2
        hullIndices          = convhull(progressTimeBoundary(:, 1), progressTimeBoundary(:, 2));
        progressTimeBoundary = progressTimeBoundary(hullIndices(1:end - 1), :);
    elseif size(progressTimeBoundary, 1) > 2
        sectionDirection = progressTimeBoundary(end, :) - progressTimeBoundary(1, :);
        [~, sortOrder]    = sort(progressTimeBoundary * sectionDirection.');
        progressTimeBoundary = progressTimeBoundary(sortOrder([1, end]), :);
    end

    % Blocked delay = obstacle time - request start time - travel time to
    % that progress. For a request at 0 s, an obstacle halfway along the
    % path at 8 s, and 3 s of travel to halfway, departure at 5 s collides.
    % Find the full range of blocked delays along each boundary edge.
    minimumBlockedDelay_s   = Inf;
    maximumBlockedDelay_s   = -Inf;
    firstStartBlockedTime_s = Inf;
    for edgeIndex = 1:size(progressTimeBoundary, 1)
        edgeStartProgressTime = progressTimeBoundary(edgeIndex, :);
        edgeEndProgressTime   = progressTimeBoundary(mod(edgeIndex, size(progressTimeBoundary, 1)) + 1, :);
        edgeProgressInterval  = [max(0, min(edgeStartProgressTime(1), edgeEndProgressTime(1))), ...
            min(1, max(edgeStartProgressTime(1), edgeEndProgressTime(1)))];
        if edgeProgressInterval(1) > edgeProgressInterval(2)
            continue;
        end

        % A vertical edge in [progress, time] keeps progress fixed while
        % obstacle time changes. Its two times give the delay range there.
        if edgeStartProgressTime(1) == edgeEndProgressTime(1)
            timeFromDeparture_s   = findTimeAtProgress(progressCoefficients, segmentTiming, edgeProgressInterval(1));
            minimumBlockedDelay_s = min(minimumBlockedDelay_s, ...
                min(edgeStartProgressTime(2), edgeEndProgressTime(2)) - ...
                solverRequest.InitialState.time_s - timeFromDeparture_s);
            maximumBlockedDelay_s = max(maximumBlockedDelay_s, ...
                max(edgeStartProgressTime(2), edgeEndProgressTime(2)) - ...
                solverRequest.InitialState.time_s - timeFromDeparture_s);
            if edgeStartProgressTime(1) == 0
                firstStartBlockedTime_s = min(firstStartBlockedTime_s, ...
                    min(edgeStartProgressTime(2), edgeEndProgressTime(2)));
            end
            continue;
        end

        % On every other straight boundary edge, obstacle time is
        % slope x path progress + intercept.
        boundaryTimeSlope_s = (edgeEndProgressTime(2) - edgeStartProgressTime(2)) / ...
            (edgeEndProgressTime(1) - edgeStartProgressTime(1));
        boundaryTimeIntercept_s = edgeStartProgressTime(2) - boundaryTimeSlope_s * edgeStartProgressTime(1);
        if edgeProgressInterval(1) == 0
            firstStartBlockedTime_s = min(firstStartBlockedTime_s, boundaryTimeIntercept_s);
        end

        % Only compare an edge's progress range with segments that pass
        % through that range.
        for segmentIndex = 1:numel(segmentTime_s)
            overlappingProgressInterval = [max(edgeProgressInterval(1), progressCoefficients(segmentIndex, 1)), ...
                min(edgeProgressInterval(2), sum(progressCoefficients(segmentIndex, :)))];
            if overlappingProgressInterval(1) > overlappingProgressInterval(2)
                continue;
            end
            evaluationFractions = [findSegmentFractionAtProgress( ...
                progressCoefficients(segmentIndex, :), overlappingProgressInterval(1)), ...
                findSegmentFractionAtProgress( ...
                progressCoefficients(segmentIndex, :), overlappingProgressInterval(2))];

            % Substitute this segment's progress curve into the edge's time
            % line, then subtract travel time. The result is blocked delay
            % as a polynomial of the segment fraction.
            delayCoefficients_s = boundaryTimeSlope_s * progressCoefficients(segmentIndex, :);
            delayCoefficients_s(1) = delayCoefficients_s(1) + boundaryTimeIntercept_s - ...
                solverRequest.InitialState.time_s - segmentTiming.StartTime_s(segmentIndex);
            delayCoefficients_s(2) = delayCoefficients_s(2) - segmentTime_s(segmentIndex);
            delaySlopeCoefficients_s = delayCoefficients_s(2:end) .* (1:5);

            % A polynomial reaches its largest or smallest delay at an
            % overlap endpoint or where its slope is zero. Bezier slope
            % bounds let us skip root finding when the slope keeps one sign.
            lastNonzeroSlopeIndex = find(delaySlopeCoefficients_s ~= 0, 1, 'last');
            turningPointFractions = [];
            if ~isempty(lastNonzeroSlopeIndex)
                if boundaryTimeSlope_s >= 0
                    minimumDelaySlope_s = boundaryTimeSlope_s * minimumProgressSlope(segmentIndex) - ...
                        segmentTime_s(segmentIndex);
                    maximumDelaySlope_s = boundaryTimeSlope_s * maximumProgressSlope(segmentIndex) - ...
                        segmentTime_s(segmentIndex);
                else
                    minimumDelaySlope_s = boundaryTimeSlope_s * maximumProgressSlope(segmentIndex) - ...
                        segmentTime_s(segmentIndex);
                    maximumDelaySlope_s = boundaryTimeSlope_s * minimumProgressSlope(segmentIndex) - ...
                        segmentTime_s(segmentIndex);
                end
                slopeRoundingAllowance_s = 4096 * eps(max([ ...
                    1; abs(minimumDelaySlope_s); abs(maximumDelaySlope_s); ...
                    abs(segmentTime_s(segmentIndex))]));
                delayChangesInOneDirection = minimumDelaySlope_s > slopeRoundingAllowance_s || ...
                    maximumDelaySlope_s < -slopeRoundingAllowance_s;
                if ~delayChangesInOneDirection
                    turningPointFractions = roots( ...
                        delaySlopeCoefficients_s(lastNonzeroSlopeIndex:-1:1));
                end
            end

            % Only real slope-zero fractions inside this overlap can set
            % an additional extreme delay.
            turningPointFractions = real(turningPointFractions(abs(imag(turningPointFractions)) <= ...
                64 * eps(max(1, abs(turningPointFractions)))));
            turningPointIsInRange = turningPointFractions >= evaluationFractions(1) & ...
                turningPointFractions <= evaluationFractions(2);
            evaluationFractions = [evaluationFractions, ...
                reshape(turningPointFractions(turningPointIsInRange), 1, [])]; %#ok<AGROW>
            evaluatedDelays_s = delayCoefficients_s(1) + evaluationFractions .* (delayCoefficients_s(2) + ...
                evaluationFractions .* (delayCoefficients_s(3) + evaluationFractions .* (delayCoefficients_s(4) + ...
                evaluationFractions .* (delayCoefficients_s(5) + evaluationFractions .* delayCoefficients_s(6)))));
            minimumBlockedDelay_s = min(minimumBlockedDelay_s, min(evaluatedDelays_s));
            maximumBlockedDelay_s = max(maximumBlockedDelay_s, max(evaluatedDelays_s));
        end
    end
    if minimumBlockedDelay_s <= maximumBlockedDelay_s
        blockedDelayIntervals_s(end + 1, :) = [minimumBlockedDelay_s, maximumBlockedDelay_s]; %#ok<AGROW>
    end

    % Waiting occupies the start point for the entire delay. Once a region
    % reaches that point, every later departure waits through that contact.
    if isfinite(firstStartBlockedTime_s)
        blockedDelayIntervals_s(end + 1, :) = ...
            [firstStartBlockedTime_s - solverRequest.InitialState.time_s, maximumWait_s]; %#ok<AGROW>
    end

    % Every 128 regions, stop early if the current blocked intervals leave
    % no delay within the horizon. More regions can only block more delays.
    if mod(regionIndex, 128) == 0
        firstAllowedDelay_s = findFirstAllowedDelay(blockedDelayIntervals_s);
        if firstAllowedDelay_s > maximumWait_s
            departureTiming         = struct('DepartureDelay_s', firstAllowedDelay_s, 'Available', false);
            controlPoint_units      = zeros(0, solverRequest.Degree + 1, 2);
            segmentTime_s           = zeros(0, 1);
            powerCoefficients_units = [];
            return
        end
    end
end

%% Section 3: Select The First Allowed Delay And Add Any Waiting Segment

departureDelay_s = findFirstAllowedDelay(blockedDelayIntervals_s);
departureTiming  = struct('DepartureDelay_s', departureDelay_s, 'Available', departureDelay_s <= maximumWait_s);
if ~departureTiming.Available
    controlPoint_units      = zeros(0, solverRequest.Degree + 1, 2);
    segmentTime_s           = zeros(0, 1);
    powerCoefficients_units = [];
    return
end

% Add the wait as a constant-position segment. Identical controls make
% its velocity, acceleration, and jerk zero throughout the delay.
if departureDelay_s > 0
    controlPoint_units = cat(1, reshape(repmat(start_units, solverRequest.Degree + 1, 1), ...
        1, solverRequest.Degree + 1, 2), controlPoint_units);
    segmentTime_s = [departureDelay_s; segmentTime_s];
    waitingCoefficients_units = zeros(1, 2, solverRequest.Degree + 1);
    waitingCoefficients_units(:, :, 1) = start_units;
    powerCoefficients_units = cat(1, waitingCoefficients_units, powerCoefficients_units);
end
end

%% Section 4: Local Functions

function departureDelay_s = findFirstAllowedDelay(blockedDelayIntervals_s)
    % Start with no wait. Scan blocked delays in increasing order; when an
    % interval contains the current delay, move just beyond its end. The
    % first gap is the earliest remaining departure delay.
    blockedDelayIntervals_s = sortrows(blockedDelayIntervals_s, 1);
    departureDelay_s        = 0;
    for blockedIntervalIndex = 1:size(blockedDelayIntervals_s, 1)
        if blockedDelayIntervals_s(blockedIntervalIndex, 1) > departureDelay_s
            break;
        end
        if blockedDelayIntervals_s(blockedIntervalIndex, 2) >= departureDelay_s
            % Contact at the interval end is still blocked; step just beyond it.
            departureDelay_s = blockedDelayIntervals_s(blockedIntervalIndex, 2) + ...
                64 * eps(max(1, abs(blockedDelayIntervals_s(blockedIntervalIndex, 2))));
        end
    end
end

function regionVertices_units = expandRegionForClearance(regionVertices_units, requiredClearance_units)
    % Put a clearance square around every polygon vertex, then keep the
    % outer boundary. This includes every point within the requested
    % Euclidean clearance, including around corners.
    cornerOffsets_units  = requiredClearance_units * [-1, -1; 1, -1; 1, 1; -1, 1];
    regionVertices_units = reshape(permute(regionVertices_units + ...
        reshape(cornerOffsets_units.', 1, 2, 4), [1, 3, 2]), [], 2);
    hullIndices          = convhull(regionVertices_units(:, 1), regionVertices_units(:, 2));
    regionVertices_units = regionVertices_units(hullIndices(1:end - 1), :);
end

function timeFromDeparture_s = findTimeAtProgress(progressCoefficients, segmentTiming, requestedProgress)
    % Find the segment containing the requested path progress. Convert
    % that segment's 0-to-1 fraction into time since departure.
    segmentIndex = find(sum(progressCoefficients, 2) >= requestedProgress, 1);
    if isempty(segmentIndex)
        segmentIndex = size(progressCoefficients, 1);
    end
    timeFromDeparture_s = segmentTiming.StartTime_s(segmentIndex) + ...
        segmentTiming.SegmentTime_s(segmentIndex) * ...
        findSegmentFractionAtProgress(progressCoefficients(segmentIndex, :), requestedProgress);
end

function segmentFraction = findSegmentFractionAtProgress(segmentProgressCoefficients, requestedProgress)
    % Progress increases along this chord. Segment start and end have
    % known fractions 0 and 1. For an interior value, halve the possible
    % fraction interval 48 times until it locates the requested progress.
    if requestedProgress <= segmentProgressCoefficients(1)
        segmentFraction = 0;
        return
    end
    if requestedProgress >= sum(segmentProgressCoefficients)
        segmentFraction = 1;
        return
    end
    lowerFraction = 0;
    upperFraction = 1;
    for iterationIndex = 1:48
        midpointFraction = (lowerFraction + upperFraction) / 2;
        midpointProgress = segmentProgressCoefficients(1) + midpointFraction * (segmentProgressCoefficients(2) + ...
            midpointFraction * (segmentProgressCoefficients(3) + midpointFraction * (segmentProgressCoefficients(4) + ...
            midpointFraction * (segmentProgressCoefficients(5) + midpointFraction * segmentProgressCoefficients(6)))));
        if midpointProgress < requestedProgress
            lowerFraction = midpointFraction;
        else
            upperFraction = midpointFraction;
        end
    end
    segmentFraction = (lowerFraction + upperFraction) / 2;
end
