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
%   Positions are degrees; time is seconds; derivatives retain physical units.

%% Section 1: Evaluate Prepared Inputs
if nargin == 0
    validation = createEmptyValidation();
    return;
end
validationTimer = tic;
hasMovingGoal   = isfield(goalState, "targetTime_s") && ~isempty(goalState.targetTime_s);
if options.AllowAzimuthWrapping && (~isempty(obstacles) || hasMovingGoal)
    error("validateTrajectory:UnsupportedWrappedGeometry", "Wrapped validation is supported only for obstacle-free " + "fixed-position goals.");
end
[time_s, position_deg, velocity_deg_s, acceleration_deg_s2, ...
    jerk_deg_s3, historySizesMatch, timeIsFinite, ...
    timeIsStrictlyIncreasing, historyIsFinite] = readAndCheckMotionHistory(trajectory);

%% Section 2: Validate Endpoints And Continuous Polynomial Bounds

stateTolerance = max(10 * options.ConstraintTolerance, 1e-7);
[initialStateMatched, terminalStateMatched, goalTimeSatisfied] = checkEndpointStates(time_s, position_deg, velocity_deg_s, acceleration_deg_s2, historyIsFinite, timeIsFinite, initialState, goalState, options, stateTolerance);
[continuousBounds, dynamics, polynomialChecks]                 = obstacleAvoidance.validation.validatePolynomialTrajectory(trajectory.Polynomial, time_s, position_deg, velocity_deg_s, acceleration_deg_s2, jerk_deg_s3, initialState, goalState, limits, options, stateTolerance, @obstacleAvoidance.validation.certifyPolynomialRange);

%% Section 3: Certify Continuous Collision Freedom

collisionTimer = tic;
[collisionFree, collisionResolved, seedCorridorCertified, ...
    planeCertificateCertified, minimumClearance_deg, ...
    collisionCheckCount, unresolvedIntervalCount, certificateRejectionReason] = obstacleAvoidance.validation.checkObstacleClearance(trajectory, obstacles, limits, options, timeIsStrictlyIncreasing && historyIsFinite && continuousBounds.Valid);
collisionCheckingElapsedTime_s = toc(collisionTimer);
safetyMarginPolicySatisfied    = safetyMarginProvenanceSatisfied(obstacles);
azimuthWrapPolicySatisfied     = options.AllowAzimuthWrapping || continuousBounds.PositionWithinLimits;

%% Section 4: Assemble The Stable Validation Record

checkNames = ["history shape", "finite increasing time", ...
    "finite histories", "initial state", "terminal state", "goal time", ...
    "polynomial format", "polynomial initial time", ...
    "polynomial time base", "polynomial segment continuity", ...
    "polynomial endpoint states", "polynomial sampled histories", ...
    "continuous limits", "polynomial dynamics", "collision freedom", ...
    "collision resolution", "safety-margin provenance", ...
    "azimuth-wrap policy"];
checkValues = [historySizesMatch, timeIsStrictlyIncreasing, ...
    historyIsFinite, initialStateMatched, terminalStateMatched, ...
    goalTimeSatisfied, polynomialChecks.FormatValid, ...
    polynomialChecks.InitialTimeMatched, ...
    polynomialChecks.TimeBaseConsistent, ...
    polynomialChecks.SegmentContinuity, ...
    polynomialChecks.EndpointStatesMatched, ...
    polynomialChecks.HistoryConsistent, continuousBounds.Valid, ...
    dynamics.Consistent, collisionFree, collisionResolved, ...
    safetyMarginPolicySatisfied, azimuthWrapPolicySatisfied];
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
validation.MinimumClearance_deg             = minimumClearance_deg;
validation.CollisionCheckCount              = collisionCheckCount;
validation.UnresolvedIntervalCount          = unresolvedIntervalCount;
validation.SafetyMarginPolicySatisfied      = safetyMarginPolicySatisfied;
validation.AzimuthWrapPolicySatisfied       = azimuthWrapPolicySatisfied;
validation.PeakVelocity_deg_s               = maximumAbsolute(velocity_deg_s);
validation.PeakAcceleration_deg_s2          = maximumAbsolute(acceleration_deg_s2);
validation.PeakJerk_deg_s3                  = maximumAbsolute(jerk_deg_s3);
validation.Issues                           = issues;
validation.CollisionCheckingElapsedTime_s   = collisionCheckingElapsedTime_s;
validation.ElapsedTime_s                    = toc(validationTimer);
validation.CertificateRejectionReason       = certificateRejectionReason;
end

%% Section 5: Local Functions

function [time_s, position_deg, velocity_deg_s, acceleration_deg_s2, jerk_deg_s3, historySizesMatch, timeIsFinite, timeIsStrictlyIncreasing, historyIsFinite] = readAndCheckMotionHistory(trajectory)
    % Normalize sampled histories and check their shared shape and finiteness.
    requiredFields = {'time_s', 'position_deg', 'velocity_deg_s', ...
        'acceleration_deg_s2', 'jerk_deg_s3', 'Polynomial'};
    if ~isstruct(trajectory) || ~isscalar(trajectory) || ~all(isfield(trajectory, requiredFields))
        error("validateTrajectory:InvalidTrajectory", "trajectory must contain sampled histories and Polynomial data.");
    end
    time_s                   = double(trajectory.time_s(:));
    position_deg             = double(trajectory.position_deg);
    velocity_deg_s           = double(trajectory.velocity_deg_s);
    acceleration_deg_s2      = double(trajectory.acceleration_deg_s2);
    jerk_deg_s3              = double(trajectory.jerk_deg_s3);
    sampleCount              = numel(time_s);
    historySizesMatch        = isequal(size(position_deg), [sampleCount 2]) && isequal(size(velocity_deg_s), [sampleCount 2]) && isequal(size(acceleration_deg_s2), [sampleCount 2]) && isequal(size(jerk_deg_s3), [sampleCount 2]);
    timeIsFinite             = ~isempty(time_s) && all(isfinite(time_s));
    timeIsStrictlyIncreasing = timeIsFinite && all(diff(time_s) > 0);
    historyIsFinite          = sampleCount > 0 && historySizesMatch && all(isfinite(position_deg), "all") && all(isfinite(velocity_deg_s), "all") && all(isfinite(acceleration_deg_s2), "all") && all(isfinite(jerk_deg_s3), "all");
end

function [initialStateMatched, terminalStateMatched, goalTimeSatisfied] = checkEndpointStates(time_s, position_deg, velocity_deg_s, acceleration_deg_s2, historyIsFinite, timeIsFinite, initialState, goalState, options, stateTolerance)
    % Compare sampled endpoints with the requested states and arrival policy.
    initialStateMatched = historyIsFinite && max(abs([ position_deg(1, :) - initialState.position_deg, velocity_deg_s(1, :) - initialState.velocity_deg_s, acceleration_deg_s2(1, :) - initialState.acceleration_deg_s2])) <= stateTolerance;
    if timeIsFinite
        goalPosition_deg = obstacleAvoidance.input.goalPositionAtTime(goalState, time_s(end));
    else
        goalPosition_deg = [NaN NaN];
    end
    terminalStateMatched = historyIsFinite && max(abs([ position_deg(end, :) - goalPosition_deg, velocity_deg_s(end, :) - goalState.velocity_deg_s, acceleration_deg_s2(end, :) - goalState.acceleration_deg_s2])) <= stateTolerance;
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
    hasFields = all(isfield(obstacles, {'originalAz_deg', 'originalEl_deg', 'safetyMargin_deg'}));
    satisfied = hasFields && all(isfinite([obstacles.safetyMargin_deg])) && all([obstacles.safetyMargin_deg] >= 0);
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
    validation.MinimumClearance_deg             = NaN;
    validation.CollisionCheckCount              = 0;
    validation.UnresolvedIntervalCount          = 0;
    validation.SafetyMarginPolicySatisfied      = false;
    validation.AzimuthWrapPolicySatisfied       = false;
    validation.PeakVelocity_deg_s               = [NaN NaN];
    validation.PeakAcceleration_deg_s2          = [NaN NaN];
    validation.PeakJerk_deg_s3                  = [NaN NaN];
    validation.Issues                           = strings(0, 1);
    validation.CollisionCheckingElapsedTime_s   = 0;
    validation.ElapsedTime_s                    = 0;
    validation.CertificateRejectionReason       = "";
end
