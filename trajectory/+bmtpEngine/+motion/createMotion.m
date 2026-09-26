function preparedMotion = createMotion(solverRequest, controlPoint_units, segmentTime_s, ...
        suppliedPowerCoefficients_units, splitSegment, splitProgress)
%% Section 0: Header & Readme
% SYNTAX
%   preparedMotion = bmtpEngine.motion.createMotion(solverRequest, controlPoint_units, segmentTime_s)
%   preparedMotion = bmtpEngine.motion.createMotion(solverRequest, controlPoint_units, segmentTime_s, ...
%       suppliedPowerCoefficients_units, splitSegment, splitProgress)
%**************************************************************************
% PURPOSE
%   - Turn proposed Bezier controls and segment times into one motion for
%     later checks. Match the requested start and goal states, split selected
%     segments, and keep one polynomial for validation and output.
%   - When arrival time can change, lengthen the segment times together if
%     the curve needs more time to meet the motion-rate limits.
%**************************************************************************
% INPUTS
%   - solverRequest (scalar struct)
%       Checked start and goal states, motion limits, time available, and
%       options for arrival timing and tolerances.
%   - controlPoint_units (S-by-(D+1)-by-2 numeric array)
%       S is the number of segments; each degree-D Bezier curve has D+1
%       controls. The last dimension holds two position axes.
%   - segmentTime_s (S-by-1 positive finite numeric vector)
%       Proposed duration of each segment.
%   - suppliedPowerCoefficients_units (S-by-2-by-(D+1) numeric array, optional)
%       Position polynomial for each axis, ordered as constant, u, u^2,
%       and so on, with segment fraction u from 0 to 1. A fully finite axis
%       takes priority over converted controls; an axis with NaN uses them.
%   - splitSegment (S-by-1 logical array, optional)
%       True for each segment to split into two; defaults to all segments.
%   - splitProgress (S-by-1 numeric array, optional)
%       Where to split each selected segment, between 0 and 1; defaults to 0.5.
%**************************************************************************
% OUTPUTS
%   - preparedMotion (scalar struct)
%       Prepared controls, final polynomial, segment times, duration check,
%       and SourceSegmentIndex linking each output piece to an input segment.
%       Success = false if total duration exceeds the available time or a
%       curve-join correction is too large. Message and TerminationReason
%       explain either outcome.
%       MotionProof.Passed reports the rate check separately and can be false
%       even when Success is true. The caller validates the complete motion.
%**************************************************************************
% UNITS
%   - Position is coordinate units and time is seconds.
%**************************************************************************

%% Section 1: Set Endpoint States And Split The Input Curve

if nargin < 4
    suppliedPowerCoefficients_units = [];
end
if nargin < 5
    splitSegment = true(size(controlPoint_units, 1), 1);
end
if nargin < 6
    splitProgress = repmat(0.5, numel(splitSegment), 1);
end

% First make the controls match the requested start and goal states. After
% splitting, keep each piece's source segment for later checks and reports.
controlPoint_units = bmtpEngine.motion.imposeEndpointControls(controlPoint_units, segmentTime_s, ...
    solverRequest.InitialState, solverRequest.GoalState);
[controlPoint_units, segmentTime_s, suppliedPowerCoefficients_units, sourceSegmentIndex] = ...
    bmtpEngine.motion.subdivideMotion(controlPoint_units, segmentTime_s, ...
    suppliedPowerCoefficients_units, splitSegment, splitProgress);

%% Section 2: Size Durations And Respect The Arrival Mode

% The polynomial builder may move controls slightly to make neighboring
% degree-five curves meet in position, velocity, acceleration, and jerk.
% Check the rate limits on this corrected curve, which is the one retained.
motionPolynomial = bmtpEngine.motion.createPowerPolynomial( ...
    controlPoint_units, segmentTime_s, 0, suppliedPowerCoefficients_units);
polynomialControlPoint_units = bmtpEngine.motion.powerToBernstein(motionPolynomial.positionPower_units);
requiredTime_s = bmtpEngine.motion.findRequiredPolynomialTime( ...
    motionPolynomial.positionPower_units, segmentTime_s, solverRequest.Limits);

% Required time comes from the curve's highest speed, acceleration, and
% jerk. For example, a piece assigned 1 second but needing 2 seconds calls
% for at least a 2x time scale. Fixed arrival keeps its assigned times;
% otherwise stretch every piece by the same factor so the curve stays the
% same while its motion slows. The tiny extra factor covers rounding.
isFixedArrival = solverRequest.Options.GoalTimeMode == "fixedArrival";
durationScale  = 1;
if ~isFixedArrival
    durationScale = max(1, max(requiredTime_s ./ segmentTime_s)) * (1 + 64 * eps);
    segmentTime_s = segmentTime_s * durationScale;
end
motionDuration_s            = sum(segmentTime_s);
durationMatchesFixedArrival = isFixedArrival && ...
    abs(sum(segmentTime_s) - solverRequest.MotionHorizon_s) <= 64 * eps(solverRequest.MotionHorizon_s);
if durationMatchesFixedArrival
    % Correct only a rounding-sized mismatch. A materially late motion
    % must not be made to appear on time by changing its last segment.
    for passIndex = 1:2
        segmentTime_s(end) = segmentTime_s(end) + (solverRequest.MotionHorizon_s - sum(segmentTime_s));
    end
end

% Adding start time + durations can round slightly away from the requested
% arrival time. When the fixed duration matches, record the supplied arrival
% time directly. Otherwise, calculate arrival from the segment durations.
finalTime_s = solverRequest.InitialState.time_s + sum(segmentTime_s);
if durationMatchesFixedArrival
    finalTime_s = solverRequest.GoalState.time_s;
end

%% Section 3: Reject Excessive Join Changes Or Insufficient Time

% Joining curves may move controls slightly. Reject a larger change because
% it would alter the path optimized for obstacle clearance. Allow the larger
% of the collision-clearance tolerance or one part per million of the
% coordinate scale. One part per million of 1000 units is 0.001 units.
% The optimizer's ConstraintTolerance does not set this distance allowance.
coordinateScale_units         = max(1, max(abs(controlPoint_units), [], 'all'));
joinCorrectionTolerance_units = max( ...
    solverRequest.Options.CollisionClearanceTolerance_units, 1e-6 * coordinateScale_units);
joinCorrectionDistance_units   = motionPolynomial.ContinuityProjectionDisplacement_units;
joinCorrectionExceedsTolerance = joinCorrectionDistance_units > joinCorrectionTolerance_units;
motionFitsAvailableTime        = motionDuration_s <= solverRequest.MotionHorizon_s + ...
    solverRequest.Options.ArrivalTimeTolerance_s;

success           = motionFitsAvailableTime && ~joinCorrectionExceedsTolerance;
message           = "";
terminationReason = "";
if joinCorrectionExceedsTolerance
    message           = "The C3 join projection would move the supplied controls beyond the join-repair tolerance.";
    terminationReason = "continuityProjectionExceedsTolerance";
elseif ~motionFitsAvailableTime
    if isFixedArrival
        message           = "The proven minimum exceeds the fixed arrival.";
        terminationReason = "fixedArrivalInfeasible";
    else
        message           = "The proven motion exceeds the goal horizon.";
        terminationReason = "timeWindowInfeasible";
    end
end

%% Section 4: Retain One Polynomial For Checks And Output

% Keep the corrected polynomial, including axes converted from controls, so
% later checks and output use the same curve. MotionProof records whether
% every assigned duration meets its required time. This preparation result
% still needs the caller's complete motion validation.
durationCheck = struct( ...
    "Passed",           all(segmentTime_s >= requiredTime_s), ...
    "SegmentTime_s",    segmentTime_s, ...
    "MaximumViolation", max([0; requiredTime_s - segmentTime_s]));
preparedMotion = struct( ...
    "Success",                                success, ...
    "Message",                                message, ...
    "TerminationReason",                      terminationReason, ...
    "ControlPoint_units",                     polynomialControlPoint_units, ...
    "GivenPower_units",                       motionPolynomial.positionPower_units, ...
    "SegmentTime_s",                          segmentTime_s, ...
    "SourceSegmentIndex",                     sourceSegmentIndex, ...
    "FinalTime_s",                            finalTime_s, ...
    "RequiredSegmentTime_s",                  requiredTime_s, ...
    "DilationScale",                          durationScale, ...
    "MotionProof",                            durationCheck, ...
    "ContinuityProjectionDisplacement_units", joinCorrectionDistance_units);
end
