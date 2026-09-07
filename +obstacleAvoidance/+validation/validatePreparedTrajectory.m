function validation = validatePreparedTrajectory(trajectory, obstacles, initialState, goalState, limits, options)
%% Section 0: Header & Readme
% SYNTAX
%   validation = validatePreparedTrajectory( ...
%       trajectory, obstacles, initialState, goalState, limits, options)
% PURPOSE
%   Check the complete motion using already prepared obstacle histories.
% INPUTS
%   trajectory and normalized planning inputs; obstacles must be prepared.
% OUTPUTS
%   validation: endpoint, continuous constraint, collision, and timing checks.
% UNITS
%   Positions are coordinate units; time is seconds; derivatives retain physical units.

%% Section 1: Evaluate Prepared Inputs
if nargin == 0
    validation = createEmptyValidation();
    return;
end
validationTimer = tic;
hasMovingGoal   = isfield(goalState, "targetTime_s") && ~isempty(goalState.targetTime_s);
if (options.WrapX || options.WrapY) && (~isempty(obstacles) || hasMovingGoal)
    error("validateTrajectory:UnsupportedWrappedGeometry", "Wrapped validation is supported only for obstacle-free " + "fixed-position goals.");
end
[time_s, position_units, velocity_units_s, acceleration_units_s2, ...
    jerk_units_s3, historySizesMatch, timeIsFinite, ...
    timeIsStrictlyIncreasing, historyIsFinite] = readAndCheckMotionHistory(trajectory);

%% Section 2: Validate Endpoints And Continuous Polynomial Bounds

stateTolerance = max(10 * options.ConstraintTolerance, 1e-7);
[initialStateMatched, terminalStateMatched, goalTimeSatisfied] = checkEndpointStates(time_s, position_units, velocity_units_s, acceleration_units_s2, historyIsFinite, timeIsFinite, initialState, goalState, options, stateTolerance);
[continuousBounds, dynamics, polynomialChecks]                 = obstacleAvoidance.validation.validatePolynomialTrajectory(trajectory.Polynomial, time_s, position_units, velocity_units_s, acceleration_units_s2, jerk_units_s3, initialState, goalState, limits, options, stateTolerance, @obstacleAvoidance.validation.certifyPolynomialRange);

%% Section 3: Certify Continuous Collision Freedom

collisionTimer = tic;
[collisionFree, collisionResolved, seedCorridorCertified, ...
    planeCertificateCertified, minimumClearance_units, ...
    collisionCheckCount, unresolvedIntervalCount, certificateRejectionReason] = obstacleAvoidance.validation.checkObstacleClearance(trajectory, obstacles, limits, options, timeIsStrictlyIncreasing && historyIsFinite && continuousBounds.Valid);
collisionCheckingElapsedTime_s = toc(collisionTimer);
safetyMarginPolicySatisfied    = safetyMarginProvenanceSatisfied(obstacles);
wrapPolicySatisfied           = continuousBounds.PositionWithinLimits;

%% Section 4: Assemble The Stable Validation Record

checkNames = ["history shape", "finite increasing time", ...
    "finite histories", "initial state", "terminal state", "goal time", ...
    "polynomial format", "polynomial initial time", ...
    "polynomial time base", "polynomial segment continuity", ...
    "polynomial endpoint states", "polynomial sampled histories", ...
    "continuous limits", "polynomial dynamics", "collision freedom", ...
    "collision resolution", "safety-margin provenance", ...
    "coordinate-wrap policy"];
checkValues = [historySizesMatch, timeIsStrictlyIncreasing, ...
    historyIsFinite, initialStateMatched, terminalStateMatched, ...
    goalTimeSatisfied, polynomialChecks.FormatValid, ...
    polynomialChecks.InitialTimeMatched, ...
    polynomialChecks.TimeBaseConsistent, ...
    polynomialChecks.SegmentContinuity, ...
    polynomialChecks.EndpointStatesMatched, ...
    polynomialChecks.HistoryConsistent, continuousBounds.Valid, ...
    dynamics.Consistent, collisionFree, collisionResolved, ...
    safetyMarginPolicySatisfied, wrapPolicySatisfied];
passed = all(checkValues);
issues = checkNames(~checkValues).';
if passed
    message = "Independent continuous validation passed.";
else
    message = "Independent validation failed: " + strjoin(issues, ", ") + ".";
end
validation = createEmptyValidation();
validation.Passed                           = passed;
validation.Message                          = message;
validation.HistorySizesMatch                = historySizesMatch;
validation.TimeIsFinite                     = timeIsFinite;
validation.TimeIsStrictlyIncreasing         = timeIsStrictlyIncreasing;
validation.HistoryIsFinite                  = historyIsFinite;
validation.InitialStateMatched              = initialStateMatched;
validation.TerminalStateMatched             = terminalStateMatched;
validation.GoalTimeSatisfied                = goalTimeSatisfied;
validation.PolynomialFormatValid            = polynomialChecks.FormatValid;
validation.PolynomialInitialTimeMatched     = polynomialChecks.InitialTimeMatched;
validation.PolynomialTimeBaseConsistent     = polynomialChecks.TimeBaseConsistent;
validation.PolynomialSegmentContinuity      = polynomialChecks.SegmentContinuity;
validation.PolynomialEndpointStatesMatched  = polynomialChecks.EndpointStatesMatched;
validation.PolynomialHistoryConsistent      = polynomialChecks.HistoryConsistent;
validation.MaximumSegmentContinuityResidual = polynomialChecks.MaximumSegmentContinuityResidual;
validation.MaximumPolynomialHistoryResidual = polynomialChecks.MaximumHistoryResidual;
validation.PositionWithinLimits             = continuousBounds.PositionWithinLimits;
validation.VelocityWithinLimits             = continuousBounds.VelocityWithinLimits;
validation.AccelerationWithinLimits         = continuousBounds.AccelerationWithinLimits;
validation.JerkWithinLimits                 = continuousBounds.JerkWithinLimits;
validation.DynamicsConsistent               = dynamics.Consistent;
validation.MaximumDynamicsResidual          = dynamics.MaximumResidual;
validation.CollisionFree                    = collisionFree;
validation.CollisionResolved                = collisionResolved;
validation.SeedCorridorCertified            = seedCorridorCertified;
validation.PlaneCertificateCertified        = planeCertificateCertified;
validation.MinimumClearance_units             = minimumClearance_units;
validation.CollisionCheckCount              = collisionCheckCount;
validation.UnresolvedIntervalCount          = unresolvedIntervalCount;
validation.SafetyMarginPolicySatisfied      = safetyMarginPolicySatisfied;
validation.WrapPolicySatisfied       = wrapPolicySatisfied;
validation.PeakVelocity_units_s               = maximumAbsolute(velocity_units_s);
validation.PeakAcceleration_units_s2          = maximumAbsolute(acceleration_units_s2);
validation.PeakJerk_units_s3                  = maximumAbsolute(jerk_units_s3);
validation.Issues                           = issues;
validation.CollisionCheckingElapsedTime_s   = collisionCheckingElapsedTime_s;
validation.ElapsedTime_s                    = toc(validationTimer);
validation.CertificateRejectionReason       = certificateRejectionReason;
end

%% Section 5: Local Functions

function [time_s, position_units, velocity_units_s, acceleration_units_s2, jerk_units_s3, historySizesMatch, timeIsFinite, timeIsStrictlyIncreasing, historyIsFinite] = readAndCheckMotionHistory(trajectory)
    % Normalize sampled histories and check their shared shape and finiteness.
    requiredFields = {'time_s', 'position_units', 'velocity_units_s', ...
        'acceleration_units_s2', 'jerk_units_s3', 'Polynomial'};
    if ~isstruct(trajectory) || ~isscalar(trajectory) || ~all(isfield(trajectory, requiredFields))
        error("validateTrajectory:InvalidTrajectory", "trajectory must contain sampled histories and Polynomial data.");
    end
    time_s                   = double(trajectory.time_s(:));
    position_units             = double(trajectory.position_units);
    velocity_units_s           = double(trajectory.velocity_units_s);
    acceleration_units_s2      = double(trajectory.acceleration_units_s2);
    jerk_units_s3              = double(trajectory.jerk_units_s3);
    sampleCount              = numel(time_s);
    historySizesMatch        = isequal(size(position_units), [sampleCount 2]) && isequal(size(velocity_units_s), [sampleCount 2]) && isequal(size(acceleration_units_s2), [sampleCount 2]) && isequal(size(jerk_units_s3), [sampleCount 2]);
    timeIsFinite             = ~isempty(time_s) && all(isfinite(time_s));
    timeIsStrictlyIncreasing = timeIsFinite && all(diff(time_s) > 0);
    historyIsFinite          = sampleCount > 0 && historySizesMatch && all(isfinite(position_units), "all") && all(isfinite(velocity_units_s), "all") && all(isfinite(acceleration_units_s2), "all") && all(isfinite(jerk_units_s3), "all");
end

function [initialStateMatched, terminalStateMatched, goalTimeSatisfied] = checkEndpointStates(time_s, position_units, velocity_units_s, acceleration_units_s2, historyIsFinite, timeIsFinite, initialState, goalState, options, stateTolerance)
    % Compare sampled endpoints with the requested states and arrival policy.
    initialStateMatched = historyIsFinite && max(abs([ position_units(1, :) - initialState.position_units, velocity_units_s(1, :) - initialState.velocity_units_s, acceleration_units_s2(1, :) - initialState.acceleration_units_s2])) <= stateTolerance;
    if timeIsFinite
        goalPosition_units = obstacleAvoidance.input.goalPositionAtTime(goalState, time_s(end));
    else
        goalPosition_units = [NaN NaN];
    end
    terminalStateMatched = historyIsFinite && max(abs([ position_units(end, :) - goalPosition_units, velocity_units_s(end, :) - goalState.velocity_units_s, acceleration_units_s2(end, :) - goalState.acceleration_units_s2])) <= stateTolerance;
    if options.GoalTimeMode == "fixedArrival"
        goalTimeSatisfied = timeIsFinite && abs(time_s(end) - goalState.time_s) <= stateTolerance;
    else
        goalTimeSatisfied = timeIsFinite && time_s(end) <= goalState.time_s + stateTolerance && time_s(end) > initialState.time_s;
    end
end

function satisfied = safetyMarginProvenanceSatisfied(obstacles)
    % Require original geometry and a finite, nonnegative safety margin.
    if isempty(obstacles)
        satisfied = true;
        return;
    end
    hasFields = all(isfield(obstacles, {'originalX_units', 'originalY_units', 'safetyMargin_units'}));
    satisfied = hasFields && all(isfinite([obstacles.safetyMargin_units])) && all([obstacles.safetyMargin_units] >= 0);
end

function peak = maximumAbsolute(values)
    % Return per-coordinate sampled peaks or documented NaN values.
    if isempty(values)
        peak = [NaN NaN];
    else
        peak = max(abs(values), [], 1);
    end
end

function validation = createEmptyValidation()
    % Define the public validation fields and empty values in one place.
    validation = struct();
    validation.Passed                           = false;
    validation.Message                          = "No trajectory was validated.";
    validation.HistorySizesMatch                = false;
    validation.TimeIsFinite                     = false;
    validation.TimeIsStrictlyIncreasing         = false;
    validation.HistoryIsFinite                  = false;
    validation.InitialStateMatched              = false;
    validation.TerminalStateMatched             = false;
    validation.GoalTimeSatisfied                = false;
    validation.PolynomialFormatValid            = false;
    validation.PolynomialInitialTimeMatched     = false;
    validation.PolynomialTimeBaseConsistent     = false;
    validation.PolynomialSegmentContinuity      = false;
    validation.PolynomialEndpointStatesMatched  = false;
    validation.PolynomialHistoryConsistent      = false;
    validation.MaximumSegmentContinuityResidual = NaN;
    validation.MaximumPolynomialHistoryResidual = NaN;
    validation.PositionWithinLimits             = false;
    validation.VelocityWithinLimits             = false;
    validation.AccelerationWithinLimits         = false;
    validation.JerkWithinLimits                 = false;
    validation.DynamicsConsistent               = false;
    validation.MaximumDynamicsResidual          = NaN;
    validation.CollisionFree                    = false;
    validation.CollisionResolved                = false;
    validation.SeedCorridorCertified            = false;
    validation.PlaneCertificateCertified        = false;
    validation.MinimumClearance_units             = NaN;
    validation.CollisionCheckCount              = 0;
    validation.UnresolvedIntervalCount          = 0;
    validation.SafetyMarginPolicySatisfied      = false;
    validation.WrapPolicySatisfied       = false;
    validation.PeakVelocity_units_s               = [NaN NaN];
    validation.PeakAcceleration_units_s2          = [NaN NaN];
    validation.PeakJerk_units_s3                  = [NaN NaN];
    validation.Issues                           = strings(0, 1);
    validation.CollisionCheckingElapsedTime_s   = 0;
    validation.ElapsedTime_s                    = 0;
    validation.CertificateRejectionReason       = "";
end
