function [guess, isFeasible, message, timeBudgetReached, ...
        failureReason] = createAzElHs3InitialGuess( ...
        seedRoute_deg, meshTau, obstacleField, initialState, goalState, ...
        limits, options, durationLowerBound_s, candidateTimer, ...
        remainingTime_s)
%% Section 0: Header & Readme
% SYNTAX
%   [guess, isFeasible, message, timeBudgetReached, failureReason] = ...
%       azElInternal.createAzElHs3InitialGuess( ...
%       seedRoute_deg, meshTau, obstacleField, initialState, goalState, ...
%       limits, options, durationLowerBound_s, candidateTimer, ...
%       remainingTime_s)
%**************************************************************************
% PURPOSE
%   - Fit one bounded, dynamically consistent HS-3 warm start to a route.
%   - Try deterministic durations and report the measured failure mode.
%**************************************************************************
% INPUTS
%   - seedRoute_deg (N-by-2 numeric matrix)
%       Ordered azimuth and elevation route vertices.
%   - meshTau (M-by-1 numeric vector)
%       Strictly increasing normalized knot times.
%   - obstacleField (scalar packed-obstacle struct)
%       Canonical original or safety-adjusted obstacle geometry.
%   - initialState, goalState (scalar structs)
%       Endpoint time, position, velocity, and acceleration states.
%   - limits, options (scalar structs)
%       Resolved physical limits and planner controls.
%   - durationLowerBound_s (nonnegative numeric scalar)
%       Necessary dynamic duration bound.
%   - candidateTimer (tic timer token)
%       Timer shared with the candidate solve.
%   - remainingTime_s (nonnegative numeric scalar)
%       Candidate time budget.
%**************************************************************************
% OUTPUTS
%   - guess (scalar struct)
%       HS-3 knot states, midpoint states, and jerk controls.
%   - isFeasible, timeBudgetReached (logical scalars)
%       Verified warm-start feasibility and time-limit status.
%   - message, failureReason (string scalars)
%       Actionable status and machine-readable failure reason.
%**************************************************************************
% UNITS
%   - Position is degrees. Time is seconds. Velocity, acceleration, and
%     jerk use degrees per second, second squared, and second cubed.
%**************************************************************************

%% Section 1: Prepare Deterministic Duration Trials

route_deg = removeDuplicateRoutePoints(seedRoute_deg);
routeStep_deg = diff(route_deg, 1, 1);
routeArc_deg = [0; cumsum(hypot( ...
    routeStep_deg(:, 1), routeStep_deg(:, 2)))];
if routeArc_deg(end) <= eps
    route_deg = [initialState.position_deg; goalState.position_deg];
end
availableDuration_s = goalState.time_s - initialState.time_s;
if options.GoalTimeMode == "fixedarrival"
    durationGuess_s = availableDuration_s;
else
    geometricDuration_s = max(durationLowerBound_s, ...
        routeArc_deg(end) / max(min(limits.maxVelocity_deg_s), eps));
    routeHeading_rad = atan2(routeStep_deg(:, 2), routeStep_deg(:, 1));
    headingChange_rad = diff(routeHeading_rad);
    headingChange_rad = atan2( ...
        sin(headingChange_rad), cos(headingChange_rad));
    totalTurnFraction = sum(abs(headingChange_rad)) / pi;
    % Turns need time to redirect bounded velocity and acceleration. This
    % factor orders trials. It does not change a constraint.
    durationStretch = min(3, 1.5 + 0.5 * totalTurnFraction);
    durationGuess_s = min(availableDuration_s, max( ...
        durationStretch * geometricDuration_s, ...
        1.05 * durationLowerBound_s));
    hasTimedRouteDuration = isfield(options, "SeedRouteDuration_s") && ...
        isfinite(options.SeedRouteDuration_s) && ...
        options.SeedRouteDuration_s > 0;
    if hasTimedRouteDuration
        durationGuess_s = min(availableDuration_s, max( ...
            durationLowerBound_s, options.SeedRouteDuration_s));
    elseif isfield(options, "SeedSnapshotTime_s") && ...
            isfinite(options.SeedSnapshotTime_s) && ...
            options.SeedSnapshotTime_s > initialState.time_s
        snapshotCenteredDuration_s = 2 * ...
            (options.SeedSnapshotTime_s - initialState.time_s);
        durationGuess_s = min(availableDuration_s, max( ...
            durationLowerBound_s, snapshotCenteredDuration_s));
    end
end
durationGuess_s = max(durationGuess_s, 1e-6);
durationCandidates_s = durationGuess_s;
canTryLongerDuration = options.GoalTimeMode == "earliestarrival" && ...
    availableDuration_s > 0;
if canTryLongerDuration
    shortestTrialDuration_s = min(availableDuration_s, max( ...
        durationLowerBound_s, 1e-6));
    durationCandidates_s = adaptiveDurationTrialSequence( ...
        shortestTrialDuration_s, durationGuess_s, ...
        availableDuration_s, 33);
    durationCandidates_s = sort(durationCandidates_s, "ascend");
end

%% Section 2: Fit The First Verified Duration

guess = emptySolution();
isFeasible = false;
timeBudgetReached = false;
failureReason = "initialGuessInfeasible";
lastFitMessage = "No duration was evaluated.";
lastCompletedFitMessage = "No duration trial completed.";
evaluatedDurationCount = 0;
for durationIndex = 1:numel(durationCandidates_s)
    if candidateTimeBudgetReached(candidateTimer, remainingTime_s)
        timeBudgetReached = true;
        message = "The initial-guess time budget expired after " + ...
            string(evaluatedDurationCount) + " duration trials. " + ...
            "Last completed trial: " + lastCompletedFitMessage;
        return;
    end
    finalTime_s = initialState.time_s + ...
        durationCandidates_s(durationIndex);
    [trialGuess, trialIsFeasible, lastFitMessage, ...
        trialTimeBudgetReached, trialFailureReason] = ...
        fitHs3SeedAtDuration( ...
        route_deg, meshTau, obstacleField, initialState, goalState, ...
        limits, options, finalTime_s, candidateTimer, remainingTime_s);
    evaluatedDurationCount = evaluatedDurationCount + 1;
    failureReason = trialFailureReason;
    if trialTimeBudgetReached
        timeBudgetReached = true;
        message = "The initial-guess time budget expired after " + ...
            string(evaluatedDurationCount) + " duration trials. " + ...
            "Last completed trial: " + lastCompletedFitMessage;
        return;
    end
    lastCompletedFitMessage = "Duration " + ...
        string(durationCandidates_s(durationIndex)) + ...
        " seconds: " + lastFitMessage;
    if trialIsFeasible
        guess = trialGuess;
        isFeasible = true;
        failureReason = "";
        message = "A bounded HS-3-consistent initial guess was found " + ...
            "after " + string(evaluatedDurationCount) + ...
            " duration trials.";
        return;
    end
end
if options.GoalTimeMode == "fixedarrival"
    message = "The fixed-arrival horizon has no bounded HS-3 initial " + ...
        "guess. " + lastFitMessage;
else
    message = "No bounded HS-3 initial guess was found in " + ...
        string(evaluatedDurationCount) + ...
        " deterministic adaptive duration trials. " + lastFitMessage;
end
end

%% Section 3: Local Functions

function durationCandidates_s = adaptiveDurationTrialSequence( ...
        lowerDuration_s, preferredDuration_s, upperDuration_s, ...
        maximumTrialCount)
% PURPOSE
%   - Expand around the preferred duration before the distant horizon.
preferredDuration_s = min(upperDuration_s, max( ...
    lowerDuration_s, preferredDuration_s));
durationCandidates_s = preferredDuration_s;
baseStep_s = max(1e-3, 0.05 * max(1, preferredDuration_s));
expansionIndex = 0;
while numel(durationCandidates_s) < maximumTrialCount
    expansionIndex = expansionIndex + 1;
    expansionLevel = ceil(expansionIndex / 2);
    direction = 1;
    if mod(expansionIndex, 2) == 0
        direction = -1;
    end
    trialDuration_s = preferredDuration_s + direction * ...
        baseStep_s * 2 ^ (expansionLevel - 1);
    trialDuration_s = min(upperDuration_s, max( ...
        lowerDuration_s, trialDuration_s));
    isNewDuration = all(abs(durationCandidates_s - trialDuration_s) > ...
        eps(max(1, upperDuration_s)));
    if isNewDuration
        durationCandidates_s(end + 1, 1) = ...
            trialDuration_s; %#ok<AGROW>
    end
    coveredLowerBound = any(durationCandidates_s == lowerDuration_s);
    coveredUpperBound = any(durationCandidates_s == upperDuration_s);
    if coveredLowerBound && coveredUpperBound && ~isNewDuration
        break;
    end
end
end

function [guess, isFeasible, message, timeBudgetReached, ...
        failureReason] = fitHs3SeedAtDuration( ...
        route_deg, meshTau, obstacleField, initialState, goalState, ...
        limits, options, finalTime_s, candidateTimer, remainingTime_s)
% PURPOSE
%   - Fit one joint HS-3 chain to a route and its local separators.
duration_s = finalTime_s - initialState.time_s;
[fitTau, associationTau] = azElInternal.azElHs3CorridorTau(meshTau);
segmentCount = numel(meshTau) - 1;
zeroKnotControl = zeros(segmentCount + 1, 2);
zeroMidpointControl = zeros(segmentCount, 2);
freeSolution = azElInternal.propagateAzElHs3Control( ...
    initialState.time_s, finalTime_s, meshTau, ...
    endpointState(initialState), zeroKnotControl, zeroMidpointControl);
freeSampleState = [freeSolution.KnotState; freeSolution.MidpointState];
[freeContinuousState, ~] = continuousSeedControlValues( ...
    freeSolution, meshTau);
baseControl = boundaryQuinticControl( ...
    meshTau, initialState, goalState, duration_s);
verificationTolerance = max(1e-8, ...
    0.1 * options.NlpConstraintTolerance);
guess = emptySolution();
isFeasible = false;
failureReason = "initialGuessInfeasible";
if candidateTimeBudgetReached(candidateTimer, remainingTime_s)
    timeBudgetReached = true;
    message = "The joint HS-3 route projection expired.";
    return;
end
targetPosition_deg = azElInternal.sampleAzElSeedRoute( ...
    route_deg, fitTau, initialState.time_s, finalTime_s, options);
associationPosition_deg = azElInternal.sampleAzElSeedRoute( ...
    route_deg, associationTau, initialState.time_s, finalTime_s, options);
[stateResponse, fitPositionResponse, continuousStateResponse, ...
    continuousJerkResponse] = seedControlResponses( ...
    meshTau, duration_s, fitTau);
[freeFitState, ~] = azElInternal.sampleAzElHs3Solution( ...
    freeSolution, meshTau, fitTau);
referenceCorridor = azElInternal.buildAzElHs3OptimizationCorridor( ...
    freeSolution, meshTau, obstacleField, initialState.time_s, ...
    associationPosition_deg, fitTau);
referenceCorridor.GeometryTimeInvariant = isfield( ...
    options, "ObstacleGeometryTimeInvariant") && ...
    logical(options.ObstacleGeometryTimeInvariant);
[timeOrderedControl, projectionIsFeasible, message, ...
    timeBudgetReached, failureReason] = ...
    azElInternal.fitAzElHs3InitialControl( ...
    stateResponse, fitPositionResponse, continuousStateResponse, ...
    continuousJerkResponse, freeSampleState, freeContinuousState, ...
    freeFitState(:, 1:2), targetPosition_deg, endpointState(goalState), ...
    baseControl, limits, options, referenceCorridor, obstacleField, ...
    initialState.time_s, finalTime_s, candidateTimer, remainingTime_s);
if ~projectionIsFeasible || timeBudgetReached
    return;
end
knotControl = timeOrderedControl(1:2:end, :);
midpointControl = timeOrderedControl(2:2:end, :);
guess = azElInternal.propagateAzElHs3Control( ...
    initialState.time_s, finalTime_s, meshTau, ...
    endpointState(initialState), knotControl, midpointControl);
knotControlViolation = ...
    abs(knotControl) - limits.maxJerk_deg_s3;
midpointControlViolation = ...
    abs(midpointControl) - limits.maxJerk_deg_s3;
controlViolation = max([0; knotControlViolation(:); ...
    midpointControlViolation(:)]);
terminalStateDefect = guess.KnotState(end, :) - endpointState(goalState);
motionIsFeasible = all(isfinite([ ...
    guess.KnotState(:); guess.MidpointState(:); ...
    knotControl(:); midpointControl(:)])) && ...
    controlViolation <= verificationTolerance && ...
    max(abs(terminalStateDefect)) <= verificationTolerance;
if ~motionIsFeasible
    message = "The joint HS-3 seed exceeded a control or endpoint " + ...
        "tolerance.";
    return;
end
isFeasible = true;
failureReason = "";
message = "The joint HS-3 seed passed motion and corridor checks. " + ...
    "The optimizer will perform the required collision certificate.";
end

function [stateResponse, fitPositionResponse, ...
        continuousStateResponse, continuousJerkResponse] = ...
        seedControlResponses(meshTau, duration_s, fitTau)
% PURPOSE
%   - Build exact affine state maps for time-ordered jerk ordinates.
segmentCount = numel(meshTau) - 1;
controlCount = 2 * segmentCount + 1;
sampleCount = 2 * segmentCount + 1;
stateResponse = zeros(3 * sampleCount, controlCount);
fitPositionResponse = zeros(numel(fitTau), controlCount);
continuousStateResponse = zeros(12 * segmentCount, controlCount);
continuousJerkResponse = zeros(segmentCount, controlCount);
for controlIndex = 1:controlCount
    timeOrderedControl = zeros(controlCount, 2);
    timeOrderedControl(controlIndex, 1) = 1;
    responseSolution = azElInternal.propagateAzElHs3Control( ...
        0, duration_s, meshTau, zeros(1, 6), ...
        timeOrderedControl(1:2:end, :), ...
        timeOrderedControl(2:2:end, :));
    sampleState = [responseSolution.KnotState(:, [1 3 5]); ...
        responseSolution.MidpointState(:, [1 3 5])];
    stateResponse(:, controlIndex) = sampleState(:);
    [fitState, ~] = azElInternal.sampleAzElHs3Solution( ...
        responseSolution, meshTau, fitTau);
    fitPositionResponse(:, controlIndex) = fitState(:, 1);
    [continuousState, continuousJerk] = ...
        continuousSeedControlValues(responseSolution, meshTau);
    continuousState = continuousState(:, [1 3 5]);
    continuousStateResponse(:, controlIndex) = continuousState(:);
    continuousJerkResponse(:, controlIndex) = continuousJerk(:, 1);
end
end

function [stateControlPoints, interiorJerkControl] = ...
        continuousSeedControlValues(solution, meshTau)
% PURPOSE
%   - Evaluate the same interior Bernstein controls used by the NLP.
segmentCount = numel(meshTau) - 1;
stateControlPoints = zeros(4 * segmentCount, 6);
interiorJerkControl = zeros(segmentCount, 2);
duration_s = solution.FinalTime_s - solution.InitialTime_s;
for segmentIndex = 1:segmentCount
    segmentDuration_s = duration_s * ...
        (meshTau(segmentIndex + 1) - meshTau(segmentIndex));
    firstControl = solution.KnotControl(segmentIndex, :);
    midpointControl = solution.MidpointControl(segmentIndex, :);
    lastControl = solution.KnotControl(segmentIndex + 1, :);
    statePower = azElInternal.buildAzElHs3SegmentPolynomials( ...
        solution.KnotState(segmentIndex, :), firstControl, ...
        midpointControl, lastControl, segmentDuration_s);
    stateBernstein = azElInternal.powerToBernstein(statePower);
    row = 4 * (segmentIndex - 1) + (1:4);
    stateControlPoints(row, :) = stateBernstein(2:end - 1, :);
    linearControl = -3 * firstControl + ...
        4 * midpointControl - lastControl;
    interiorJerkControl(segmentIndex, :) = ...
        firstControl + linearControl / 2;
end
end

function control = boundaryQuinticControl( ...
        meshTau, initialState, goalState, duration_s)
% PURPOSE
%   - Supply an exact endpoint-feasible quadratic-jerk starting point.
midpointTau = 0.5 * (meshTau(1:end - 1) + meshTau(2:end));
timeOrderedTau = zeros(2 * numel(meshTau) - 1, 1);
timeOrderedTau(1:2:end) = meshTau;
timeOrderedTau(2:2:end) = midpointTau;
control = zeros(numel(timeOrderedTau), 2);
higherCoefficientMatrix = [1 1 1; 3 4 5; 6 12 20];
initialEndpoint = endpointState(initialState);
goalEndpoint = endpointState(goalState);
for axisIndex = 1:2
    stateColumn = [axisIndex, axisIndex + 2, axisIndex + 4];
    firstState = initialEndpoint(stateColumn);
    lastState = goalEndpoint(stateColumn);
    fixedCoefficient = [firstState(1); ...
        duration_s * firstState(2); ...
        0.5 * duration_s ^ 2 * firstState(3)];
    higherTarget = [ ...
        lastState(1) - sum(fixedCoefficient); ...
        duration_s * lastState(2) - fixedCoefficient(2) - ...
        2 * fixedCoefficient(3); ...
        duration_s ^ 2 * lastState(3) - 2 * fixedCoefficient(3)];
    higherCoefficient = higherCoefficientMatrix \ higherTarget;
    control(:, axisIndex) = (6 * higherCoefficient(1) + ...
        24 * higherCoefficient(2) * timeOrderedTau + ...
        60 * higherCoefficient(3) * timeOrderedTau .^ 2) / ...
        duration_s ^ 3;
end
end

function solution = emptySolution()
% PURPOSE
%   - Define the stable empty HS-3 solution schema.
solution = struct( ...
    "InitialTime_s", NaN, ...
    "FinalTime_s", NaN, ...
    "KnotState", zeros(0, 6), ...
    "MidpointState", zeros(0, 6), ...
    "KnotControl", zeros(0, 2), ...
    "MidpointControl", zeros(0, 2));
end

function state = endpointState(endpoint)
% PURPOSE
%   - Assemble one six-component endpoint row.
state = [endpoint.position_deg, endpoint.velocity_deg_s, ...
    endpoint.acceleration_deg_s2];
end

function route_deg = removeDuplicateRoutePoints(route_deg)
% PURPOSE
%   - Remove only consecutive zero-length seed edges.
route_deg = double(route_deg);
keepPoint = [true; any(abs(diff(route_deg, 1, 1)) > 1e-12, 2)];
route_deg = route_deg(keepPoint, :);
end

function reached = candidateTimeBudgetReached( ...
        candidateTimer, remainingTime_s)
% PURPOSE
%   - Check the candidate timer only when its limit is finite.
reached = isfinite(remainingTime_s) && ...
    toc(candidateTimer) >= remainingTime_s;
end
