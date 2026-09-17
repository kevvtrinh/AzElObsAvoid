function [controls_units, durations_s, prescribedPower_units, diagnostics] = createDelayedChord(request)
%% Section 0: Header & Readme
% SYNTAX
%   [controls_units, durations_s, prescribedPower_units, diagnostics] = ...
%       bmtpEngine.motion.createDelayedChord(request)
%**************************************************************************
% PURPOSE
%   - Find the earliest safe departure of the C3 quintic chord.
%**************************************************************************
% INPUTS
%   - request (scalar struct)
%       Validated rest-to-rest request and authoritative convex time cells.
%**************************************************************************
% OUTPUTS
%   - controls_units (S-by-6-by-2 numeric array)
%       Waiting-plus-chord Bezier controls, or empty when unavailable.
%   - durations_s (numeric column)
%       Waiting-plus-chord span durations, or empty when unavailable.
%   - prescribedPower_units (S-by-2-by-6 numeric array)
%       Matching analytic powers, or empty when unavailable.
%   - diagnostics (scalar struct)
%       Selected departure delay and availability state.
%**************************************************************************
% UNITS
%   - Position is coordinate units and time is seconds.
%**************************************************************************

%% Section 1: Construct The Scalar Progress Clock
[controls_units, durations_s, prescribedPower_units] = bmtpEngine.motion.createC3Chord( ...
    request.InitialState.position_units, request.GoalState.position_units, request.Limits);
phaseStartTime_s      = [0; cumsum(durations_s(1:end - 1))];
phaseTiming           = struct('StartTime_s', phaseStartTime_s, 'SegmentTime_s', durations_s);
duration_s            = sum(durations_s);
maximumWait_s         = request.MotionHorizon_s - duration_s;
initial_units         = request.InitialState.position_units;
direction_units       = request.GoalState.position_units - initial_units;
directionNorm2_units2 = sum(direction_units .^ 2);
pathNormal             = [-direction_units(2), direction_units(1)] / sqrt(directionNorm2_units2);
relativePower_units    = prescribedPower_units;
relativePower_units(:, :, 1) = relativePower_units(:, :, 1) - initial_units;
progressPower = reshape(sum(relativePower_units .* reshape(direction_units, 1, 2, 1), 2), [], 6) / ...
    directionNorm2_units2;
progressDerivativePower = progressPower(:, 2:end) .* (1:5);
progressDerivativeControl = bmtpEngine.motion.powerToBernstein(progressDerivativePower.');
minimumProgressDerivative = min(progressDerivativeControl, [], 1).';
maximumProgressDerivative = max(progressDerivativeControl, [], 1).';
endRegions_units = {};
if isfield(request.Coverage, 'EndRegions_units')
    endRegions_units = request.Coverage.EndRegions_units;
end
[~, reserve_units] = bmtpEngine.validation.createCoordinateTolerances(initial_units, ...
    request.GoalState.position_units, request.Limits.xInterval_units, ...
    request.Limits.yInterval_units, request.Regions_units, endRegions_units);
clearance_units = (1 + 2 ^ 20 * eps) * request.Options.CollisionClearanceTolerance_units + ...
    3 * reserve_units;
forbidden_s = zeros(0, 2);
pathMinimum_units = min(initial_units, request.GoalState.position_units);
pathMaximum_units = max(initial_units, request.GoalState.position_units);
squareNormalRadius_units = clearance_units * sum(abs(pathNormal));
squareProgressRadius = clearance_units * sum(abs(direction_units)) / directionNorm2_units2;

%% Section 2: Project Convex Space-Time Cells Onto Path Progress And Time
for regionIndex = 1:numel(request.Regions_units)
    interval_s = request.InitialState.time_s + [0, request.MotionHorizon_s];
    if isfield(request.Coverage, 'ActiveTimeInterval_s')
        interval_s = request.Coverage.ActiveTimeInterval_s(regionIndex, :);
    end
    vertices_units = bmtpEngine.separation.regionOnInterval( ...
        request.Regions_units{regionIndex}, request.Coverage, regionIndex, interval_s);
    endpointVertices_units = [vertices_units(:, :, 1); vertices_units(:, :, end)];
    coordinateScale_units = max([1; abs(endpointVertices_units(:)); ...
        abs(initial_units(:)); abs(request.GoalState.position_units(:)); clearance_units]);
    coordinateGuard_units = 4096 * eps(coordinateScale_units);
    cellMinimum_units = min(endpointVertices_units, [], 1) - ...
        clearance_units - coordinateGuard_units;
    cellMaximum_units = max(endpointVertices_units, [], 1) + ...
        clearance_units + coordinateGuard_units;
    boxesAreDisjoint = any( ...
        cellMaximum_units < pathMinimum_units - coordinateGuard_units | ...
        cellMinimum_units > pathMaximum_units + coordinateGuard_units);
    normalResidual_units = (endpointVertices_units - initial_units) * pathNormal.';
    normalGuard_units = 4096 * eps(max([ ...
        1; abs(normalResidual_units); squareNormalRadius_units]));
    missesPathLine = min(normalResidual_units) > ...
        squareNormalRadius_units + normalGuard_units || ...
        max(normalResidual_units) < -squareNormalRadius_units - normalGuard_units;
    progress = (endpointVertices_units - initial_units) * direction_units.' / directionNorm2_units2;
    progressGuard = 4096 * eps(max([1; abs(progress); squareProgressRadius]));
    missesPathExtent = max(progress) < -squareProgressRadius - progressGuard || ...
        min(progress) > 1 + squareProgressRadius + progressGuard;
    if boxesAreDisjoint || missesPathLine || missesPathExtent
        continue
    end
    first_units = clearanceEnvelope(vertices_units(:, :, 1), clearance_units);
    last_units  = clearanceEnvelope(vertices_units(:, :, end), clearance_units);
    spaceTimePoints = [first_units, repmat(interval_s(1), size(first_units, 1), 1); ...
        last_units, repmat(interval_s(2), size(last_units, 1), 1)];
    normalResidual_units = (spaceTimePoints(:, 1:2) - initial_units) * pathNormal.';
    [positivePointIndices, negativePointIndices] = ndgrid( ...
        find(normalResidual_units > 0), find(normalResidual_units < 0));
    positivePointIndices = positivePointIndices(:);
    negativePointIndices = negativePointIndices(:);
    crossingFraction = normalResidual_units(positivePointIndices) ./ ...
        (normalResidual_units(positivePointIndices) - normalResidual_units(negativePointIndices));
    pathTimeSection = [spaceTimePoints(normalResidual_units == 0, :); ...
        spaceTimePoints(positivePointIndices, :) + crossingFraction .* ...
        (spaceTimePoints(negativePointIndices, :) - spaceTimePoints(positivePointIndices, :))];
    if isempty(pathTimeSection)
        continue;
    end
    pathTimeSection = unique([ ...
        (pathTimeSection(:, 1:2) - initial_units) * direction_units.' / directionNorm2_units2, ...
        pathTimeSection(:, 3)], 'rows');
    if size(pathTimeSection, 1) > 2 && rank(pathTimeSection - pathTimeSection(1, :)) == 2
        hullIndices     = convhull(pathTimeSection(:, 1), pathTimeSection(:, 2));
        pathTimeSection = pathTimeSection(hullIndices(1:end - 1), :);
    elseif size(pathTimeSection, 1) > 2
        sectionDirection = pathTimeSection(end, :) - pathTimeSection(1, :);
        [~, sortOrder]    = sort(pathTimeSection * sectionDirection.');
        pathTimeSection   = pathTimeSection(sortOrder([1, end]), :);
    end
    low_s                 = Inf;
    high_s                = -Inf;
    firstStartOccupancy_s = Inf;
    for edgeIndex = 1:size(pathTimeSection, 1)
        firstPoint    = pathTimeSection(edgeIndex, :);
        secondPoint   = pathTimeSection(mod(edgeIndex, size(pathTimeSection, 1)) + 1, :);
        progressRange = [max(0, min(firstPoint(1), secondPoint(1))), ...
            min(1, max(firstPoint(1), secondPoint(1)))];
        if progressRange(1) > progressRange(2)
            continue;
        end
        if firstPoint(1) == secondPoint(1)
            relative_s = inverseProgress(progressPower, phaseTiming, progressRange(1));
            low_s      = min(low_s, min(firstPoint(2), secondPoint(2)) - ...
                request.InitialState.time_s - relative_s);
            high_s = max(high_s, max(firstPoint(2), secondPoint(2)) - ...
                request.InitialState.time_s - relative_s);
            if firstPoint(1) == 0
                firstStartOccupancy_s = min(firstStartOccupancy_s, ...
                    min(firstPoint(2), secondPoint(2)));
            end
            continue;
        end
        slope_s     = (secondPoint(2) - firstPoint(2)) / (secondPoint(1) - firstPoint(1));
        intercept_s = firstPoint(2) - slope_s * firstPoint(1);
        if progressRange(1) == 0
            firstStartOccupancy_s = min(firstStartOccupancy_s, intercept_s);
        end
        for phaseIndex = 1:numel(durations_s)
            progressOverlap = [max(progressRange(1), progressPower(phaseIndex, 1)), ...
                min(progressRange(2), sum(progressPower(phaseIndex, :)))];
            if progressOverlap(1) > progressOverlap(2)
                continue;
            end
            localTau = [invertProgress(progressPower(phaseIndex, :), progressOverlap(1)), ...
                invertProgress(progressPower(phaseIndex, :), progressOverlap(2))];
            delayPower_s    = slope_s * progressPower(phaseIndex, :);
            delayPower_s(1) = delayPower_s(1) + intercept_s - ...
                request.InitialState.time_s - phaseTiming.StartTime_s(phaseIndex);
            delayPower_s(2)     = delayPower_s(2) - durations_s(phaseIndex);
            derivativePower_s   = delayPower_s(2:end) .* (1:5);
            lastDerivativeIndex = find(derivativePower_s ~= 0, 1, 'last');
            stationaryTau = [];
            if ~isempty(lastDerivativeIndex)
                if slope_s >= 0
                    derivativeLower_s = slope_s * minimumProgressDerivative(phaseIndex) - ...
                        durations_s(phaseIndex);
                    derivativeUpper_s = slope_s * maximumProgressDerivative(phaseIndex) - ...
                        durations_s(phaseIndex);
                else
                    derivativeLower_s = slope_s * maximumProgressDerivative(phaseIndex) - ...
                        durations_s(phaseIndex);
                    derivativeUpper_s = slope_s * minimumProgressDerivative(phaseIndex) - ...
                        durations_s(phaseIndex);
                end
                derivativeGuard_s = 4096 * eps(max([ ...
                    1; abs(derivativeLower_s); abs(derivativeUpper_s); ...
                    abs(durations_s(phaseIndex))]));
                derivativeIsOneSided = derivativeLower_s > derivativeGuard_s || ...
                    derivativeUpper_s < -derivativeGuard_s;
                if ~derivativeIsOneSided
                    stationaryTau = roots( ...
                        derivativePower_s(lastDerivativeIndex:-1:1));
                end
            end
            stationaryTau = real(stationaryTau(abs(imag(stationaryTau)) <= ...
                64 * eps(max(1, abs(stationaryTau)))));
            stationaryTauIsInRange = stationaryTau >= localTau(1) & ...
                stationaryTau <= localTau(2);
            localTau = [localTau, ...
                reshape(stationaryTau(stationaryTauIsInRange), 1, [])]; %#ok<AGROW>
            values_s = delayPower_s(1) + localTau .* (delayPower_s(2) + ...
                localTau .* (delayPower_s(3) + localTau .* (delayPower_s(4) + ...
                localTau .* (delayPower_s(5) + localTau .* delayPower_s(6)))));
            low_s    = min(low_s, min(values_s));
            high_s   = max(high_s, max(values_s));
        end
    end
    if low_s <= high_s
        forbidden_s(end + 1, :) = [low_s, high_s]; %#ok<AGROW>
    end
    % Waiting occupies the initial point for the entire delay, not just at
    % departure.
    if isfinite(firstStartOccupancy_s)
        forbidden_s(end + 1, :) = [firstStartOccupancy_s - request.InitialState.time_s, maximumWait_s]; %#ok<AGROW>
    end
    if mod(regionIndex, 128) == 0
        provenDelay_s = firstGapAfterForbiddenIntervals(forbidden_s);
        if provenDelay_s > maximumWait_s
            diagnostics = struct('DepartureDelay_s', provenDelay_s, 'Available', false);
            controls_units        = zeros(0, request.Degree + 1, 2);
            durations_s           = zeros(0, 1);
            prescribedPower_units = [];
            return
        end
    end
end

%% Section 3: Select The First Gap And Export The Complete Motion
wait_s = firstGapAfterForbiddenIntervals(forbidden_s);
diagnostics = struct('DepartureDelay_s', wait_s, 'Available', wait_s <= maximumWait_s);
if ~diagnostics.Available
    controls_units        = zeros(0, request.Degree + 1, 2);
    durations_s           = zeros(0, 1);
    prescribedPower_units = [];
    return
end
if wait_s > 0
    controls_units  = cat(1, reshape(repmat(initial_units, request.Degree + 1, 1), ...
        1, request.Degree + 1, 2), controls_units);
    durations_s     = [wait_s; durations_s];
    waitingPower_units          = zeros(1, 2, request.Degree + 1);
    waitingPower_units(:, :, 1) = initial_units;
    prescribedPower_units       = cat(1, waitingPower_units, prescribedPower_units);
end
end

%% Section 4: Local Functions
function wait_s = firstGapAfterForbiddenIntervals(forbidden_s)
    % Return the first nonnegative wait outside the supplied closed intervals.
    forbidden_s = sortrows(forbidden_s, 1);
    wait_s = 0;
    for forbiddenIndex = 1:size(forbidden_s, 1)
        if forbidden_s(forbiddenIndex, 1) > wait_s
            break;
        end
        if forbidden_s(forbiddenIndex, 2) >= wait_s
            wait_s = forbidden_s(forbiddenIndex, 2) + ...
                64 * eps(max(1, abs(forbidden_s(forbiddenIndex, 2))));
        end
    end
end

function vertices_units = clearanceEnvelope(vertices_units, gap_units)
    % A square Minkowski envelope contains the required Euclidean clearance.
    offsets_units = gap_units * [-1, -1; 1, -1; 1, 1; -1, 1];
    vertices_units = reshape(permute(vertices_units + ...
        reshape(offsets_units.', 1, 2, 4), [1, 3, 2]), [], 2);
    hullIndices   = convhull(vertices_units(:, 1), vertices_units(:, 2));
    vertices_units = vertices_units(hullIndices(1:end - 1), :);
end

function relative_s = inverseProgress(progressPower, phaseTiming, position)
    % Map one scalar path position back to relative motion time.
    phaseIndex = find(sum(progressPower, 2) >= position, 1);
    if isempty(phaseIndex)
        phaseIndex = size(progressPower, 1);
    end
    relative_s = phaseTiming.StartTime_s(phaseIndex) + ...
        phaseTiming.SegmentTime_s(phaseIndex) * ...
        invertProgress(progressPower(phaseIndex, :), position);
end

function localTau = invertProgress(coefficients, position)
    % Most overlap endpoints are exact phase boundaries; their inverse is
    % known. Avoid repeatedly bisecting these same endpoint values.
    if position <= coefficients(1)
        localTau = 0;
        return
    end
    if position >= sum(coefficients)
        localTau = 1;
        return
    end
    low  = 0;
    high = 1;
    for iteration = 1:48
        middle = (low + high) / 2;
        value  = coefficients(1) + middle * (coefficients(2) + ...
            middle * (coefficients(3) + middle * (coefficients(4) + ...
            middle * (coefficients(5) + middle * coefficients(6)))));
        if value < position
            low = middle;
        else
            high = middle;
        end
    end
    localTau = (low + high) / 2;
end
