function [candidate, diagnostics] = createTimedOrthogonalOpeningMotion( ...
        obstacles, initialState, goalState, limits, options)
%% Section 0: Header & Readme
% SYNTAX
%   [candidate, diagnostics] = ...
%       obstacleAvoidance.planner.createTimedOrthogonalOpeningMotion()
%   [candidate, diagnostics] = ...
%       obstacleAvoidance.planner.createTimedOrthogonalOpeningMotion( ...
%       obstacles, initialState, goalState, limits, options)
%**************************************************************************
% PURPOSE
%   - Detect one input-defined orthogonal blocker that opens at a topology
%     event and create its analytic event-compatible rest-to-rest motion.
%**************************************************************************
% INPUTS
%   - obstacles (canonical or prepared scalar obstacle struct)
%       One protected history with one stationary topology-changing opening.
%   - initialState, goalState (normalized scalar structs)
%       Fixed two-coordinate rest endpoints with axis-aligned displacement.
%   - limits (normalized scalar struct)
%       Positive per-axis velocity, acceleration, and jerk limits plus
%       workspace intervals required by independent validation.
%   - options (resolved scalar planner-options struct)
%       Earliest-arrival policy and public sampling, constraint, collision,
%       and time-resolution tolerances are required.
%**************************************************************************
% OUTPUTS
%   - candidate (stable scalar planner candidate)
%       Success remains false while pending validation and becomes true only
%       when the unchanged public validator accepts the complete motion.
%   - diagnostics (stable scalar struct)
%       Detection, scoped physical lower bound, candidate gap, validation,
%       elapsed time, and a named rejection outside the supported invariant.
%**************************************************************************
% UNITS
%   - Position and clearance are degrees. Time is seconds. Derivatives use
%     degrees per second and its second and third powers.
%**************************************************************************

%% Section 1: Validate The Supported Request

candidate = candidateTemplate();
diagnostics = diagnosticTemplate();
if nargin == 0
    return;
end
if nargin ~= 5
    error("createTimedOrthogonalOpeningMotion:InvalidCall", ...
        "Use zero inputs or obstacles, states, limits, and options.");
end
timer = tic;
try
requestCertificate = ...
    obstacleAvoidance.planner.certifyTimedOpeningRequestLowerBound( ...
    diagnostics, obstacles, initialState, goalState, limits, options);
if ~requestCertificate.Passed
    rejectRequest(mapCertificateRejection(requestCertificate.TerminationReason));
end
delta_deg = goalState.position_deg - initialState.position_deg;
motionAxis = find(abs(delta_deg) > 32 * requestCertificate.CoordinateGuard_deg);
motionAxis = motionAxis(1);
motionSign = sign(delta_deg(motionAxis));
totalProgress_deg = abs(delta_deg(motionAxis));
obstacles = obstacleAvoidance.obstacles.prepareDynamic( ...
    obstacleAvoidance.obstacles.combineObstacles(obstacles));

%% Section 2: Detect The Persistent Opening Event

eventTime_s = requestCertificate.EventTime_s;
blockingFaceProgress_deg = requestCertificate.BlockingFaceProgress_deg;
clearanceTolerance_deg = requestCertificate.ClearanceTolerance_deg;
certificateReserve_deg = max(clearanceTolerance_deg / 100, ...
    requestCertificate.CoordinateGuard_deg / 4);
eventFacePoint_deg = initialState.position_deg;
eventFacePoint_deg(motionAxis) = eventFacePoint_deg(motionAxis) + ...
    motionSign * blockingFaceProgress_deg;
openClearance_deg = obstacleAvoidance.geometry.pointPolygonClearance( ...
    obstacleAvoidance.obstacles.shapeAtTime(obstacles(1), eventTime_s), ...
    eventFacePoint_deg);
if openClearance_deg <= clearanceTolerance_deg + certificateReserve_deg
    rejectRequest("openedCenterlineNotClear");
end

%% Section 3: Derive The Event Bound And Constant-Jerk Motion

velocityLimit_deg_s = limits.maxVelocity_deg_s(motionAxis);
accelerationLimit_deg_s2 = limits.maxAcceleration_deg_s2(motionAxis);
jerkLimit_deg_s3 = limits.maxJerk_deg_s3(motionAxis);
jerkRampTime_s = accelerationLimit_deg_s2 / jerkLimit_deg_s3;
constantAccelerationTime_s = velocityLimit_deg_s / ...
    accelerationLimit_deg_s2 - jerkRampTime_s;
accelerationTime_s = 2 * jerkRampTime_s + constantAccelerationTime_s;
accelerationDistance_deg = velocityLimit_deg_s * accelerationTime_s / 2;
maximumEventProgress_deg = blockingFaceProgress_deg - clearanceTolerance_deg;
candidateEventProgress_deg = maximumEventProgress_deg - certificateReserve_deg;
remainingProgress_deg = totalProgress_deg - maximumEventProgress_deg;
supportedSwitchingRegime = constantAccelerationTime_s >= 0 && ...
    candidateEventProgress_deg > accelerationDistance_deg && ...
    remainingProgress_deg > accelerationDistance_deg && ...
    totalProgress_deg > 2 * accelerationDistance_deg;
if ~supportedSwitchingRegime
    rejectRequest("unsupportedSwitchingRegime");
end
minimumPrefixTime_s = accelerationTime_s + ...
    (candidateEventProgress_deg - accelerationDistance_deg) / ...
    velocityLimit_deg_s;
waitTime_s = eventTime_s - initialState.time_s - minimumPrefixTime_s;
minimumSuffixTime_s = accelerationTime_s + ...
    (remainingProgress_deg - accelerationDistance_deg) / ...
    velocityLimit_deg_s;
lowerBoundArrival_s = eventTime_s + minimumSuffixTime_s;
candidateArrival_s = lowerBoundArrival_s + ...
    certificateReserve_deg / velocityLimit_deg_s;
if waitTime_s < 0 || candidateArrival_s > ...
        goalState.time_s + options.ConstraintTolerance
    rejectRequest("eventTimeWindowInfeasible");
end
delayedInitialState = initialState;
delayedInitialState.time_s = initialState.time_s + waitTime_s;
directCandidate = bmtpEngine.createDirectMotion( ...
    delayedInitialState, goalState, limits, options);
if ~directCandidate.Success
    rejectRequest("directMotionConstructionFailed");
end
cruiseStartTime_s = delayedInitialState.time_s + accelerationTime_s;
candidate = prependWaitAndEventSplits( ...
    directCandidate, initialState, eventTime_s, cruiseStartTime_s, ...
    certificateReserve_deg / velocityLimit_deg_s, options);
candidate.Message = "Event-compatible candidate awaiting independent validation.";
candidate.TerminationReason = "candidatePendingValidation";
diagnostics.Attempted = true;
diagnostics.EventTime_s = eventTime_s;
diagnostics.BlockingFaceProgress_deg = blockingFaceProgress_deg;
diagnostics.OpenCenterlineClearance_deg = openClearance_deg;
diagnostics.ClearanceTolerance_deg = clearanceTolerance_deg;
diagnostics.CertificateReserve_deg = certificateReserve_deg;
diagnostics.EventCompatibleLowerBound_s = lowerBoundArrival_s;
diagnostics.WaitTime_s = waitTime_s;
diagnostics.CandidateDuration_s = candidate.MotionDuration_s;
diagnostics.LowerBoundGap_s = candidate.time_s(end) - lowerBoundArrival_s;
diagnostics.LowerBoundScope = ...
    "Trajectories remaining on the blocked side until the opening event.";
% The event word alone does not exclude a pre-event exterior route.  Keep
% the request-wide fields empty until post-validation certification proves
% that alternative route cannot arrive sooner.
diagnostics.RequestLowerBound_s = NaN;
diagnostics.AllRouteCertificatePassed = false;
diagnostics.InfimumGap_s = NaN;
diagnostics.InfimumGapWithinPolicy = false;

%% Section 4: Accept Only Independent Validation

validationTimer = tic;
validation = obstacleAvoidance.validateTrajectory( ...
    candidate, obstacles, initialState, goalState, limits, options);
diagnostics.ValidationElapsedTime_s = toc(validationTimer);
diagnostics.CollisionCheckingElapsedTime_s = ...
    validation.CollisionCheckingElapsedTime_s;
candidate.Validation = validation;
diagnostics.Validation = validation;
diagnostics.SegmentCount = candidate.Polynomial.SegmentCount;
if validation.Passed
    candidate.Success = true;
    candidate.OptimizerFeasible = true;
    candidate.Message = "The event-compatible motion passed independent validation.";
    candidate.TerminationReason = "goalReached";
    diagnostics.Success = true;
    diagnostics.Message = candidate.Message;
    diagnostics.TerminationReason = "goalReached";
    diagnostics.AllRouteCertificate = requestCertificate;
    diagnostics.AllRouteCertificatePassed = requestCertificate.Passed;
    if requestCertificate.Passed
        diagnostics.RequestLowerBound_s = requestCertificate.LowerBound_s;
        diagnostics.InfimumGap_s = candidate.MotionDuration_s - ...
            diagnostics.RequestLowerBound_s;
    end
else
    candidate.Success = false;
    candidate.OptimizerFeasible = false;
    candidate.Message = "The event candidate failed independent validation.";
    candidate.TerminationReason = "independentValidationFailed";
    diagnostics.Message = candidate.Message;
    diagnostics.TerminationReason = candidate.TerminationReason;
end
catch rejection
    if string(rejection.identifier) ~= ...
            "createTimedOrthogonalOpeningMotion:ExpectedRejection"
        rethrow(rejection);
    end
    [candidate, diagnostics] = reject(candidate, diagnostics, ...
        string(rejection.message), timer);
    return;
end
diagnostics.ElapsedTime_s = toc(timer);
end

%% Section 5: Local Functions

function candidate = prependWaitAndEventSplits( ...
        direct, initialState, eventTime_s, cruiseStartTime_s, ...
        firstApproachDuration_s, options)
% Re-express the delayed direct polynomial with a dwell and dyadic event splits.
breakTime_s = [initialState.time_s; direct.Polynomial.SegmentStartTime_s; ...
    direct.Polynomial.FinalTime_s; eventTime_s];
approachDuration_s = firstApproachDuration_s;
while eventTime_s - approachDuration_s > cruiseStartTime_s
    breakTime_s(end + 1, 1) = eventTime_s - approachDuration_s; %#ok<AGROW>
    approachDuration_s = 2 * approachDuration_s;
end
breakTime_s = unique(sort(breakTime_s));
candidate = bmtpEngine.createDelayedMotion( ...
    direct, initialState, breakTime_s, options.SampleTime_s, ...
    "timedOrthogonalOpening");
candidate.Success = false;
candidate.OptimizerFeasible = false;
end

function candidate = candidateTemplate()
% Define the stable candidate shape consumed by the public validator.
emptyInitialState = struct("time_s", 0, "position_deg", [0 0], ...
    "velocity_deg_s", [0 0], "acceleration_deg_s2", [0 0]);
candidate = bmtpEngine.createMotionRecord( ...
    struct(), emptyInitialState, [], [], 1, "timedOrthogonalOpening");
candidate.Message = "The timed-opening kernel was not run.";
candidate.UsedStraightProgress = true;
candidate.Validation = obstacleAvoidance.validateTrajectory();
end

function diagnostics = diagnosticTemplate()
% Define stable detection, bound, validation, and rejection evidence.
diagnostics = struct("Attempted", false, "Success", false, ...
    "Message", "The timed-opening kernel was not attempted.", ...
    "TerminationReason", "notRun", "EventTime_s", NaN, ...
    "BlockingFaceProgress_deg", NaN, "OpenCenterlineClearance_deg", NaN, ...
    "ClearanceTolerance_deg", NaN, "CertificateReserve_deg", NaN, ...
    "EventCompatibleLowerBound_s", NaN, ...
    "WaitTime_s", NaN, "CandidateDuration_s", NaN, ...
    "LowerBoundGap_s", NaN, ...
    "LowerBoundScope", "notEstablished", ...
    "RequestLowerBound_s", NaN, ...
    "AllRouteCertificate", ...
    obstacleAvoidance.planner.certifyTimedOpeningRequestLowerBound(), ...
    "AllRouteCertificatePassed", false, "InfimumGap_s", NaN, ...
    "InfimumGapWithinPolicy", false, ...
    "SegmentCount", 0, "Validation", obstacleAvoidance.validateTrajectory(), ...
    "ValidationElapsedTime_s", 0, "CollisionCheckingElapsedTime_s", 0, ...
    "ElapsedTime_s", 0);
end

function reason = mapCertificateRejection(reason)
% Preserve the motion kernel's public rejection vocabulary.
reason = string(reason);
if reason == "nonstationaryObstacleHistory"
    reason = "nonstationaryOpeningHistory";
elseif reason == "protectedBlockingFaceNotFound"
    reason = "orthogonalBlockingFaceNotFound";
elseif reason == "kinematicConditioningFailed" || startsWith(reason, "switching")
    reason = "unsupportedSwitchingRegime";
elseif contains(reason, "Ring") || contains(reason, "Cavity") || ...
        contains(reason, "Containment") || contains(reason, "Geometry")
    reason = "orthogonalBlockingFaceNotFound";
end
end

function rejectRequest(reason)
% Stop one expected unsupported branch for the public function to record.
error("createTimedOrthogonalOpeningMotion:ExpectedRejection", "%s", reason);
end

function [candidate, diagnostics] = reject(candidate, diagnostics, reason, timer)
% Return one named, fail-closed expected rejection with elapsed work.
[candidate.TerminationReason, diagnostics.TerminationReason] = deal(reason);
message = "Timed-opening request rejected: " + reason + ".";
[candidate.Message, diagnostics.Message] = deal(message);
diagnostics.ElapsedTime_s = toc(timer);
end
