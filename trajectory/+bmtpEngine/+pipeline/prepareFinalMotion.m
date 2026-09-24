function preparedMotion = prepareFinalMotion(solverRequest, controlPoint_units, segmentTime_s, ...
        suppliedPowerCoefficients_units, splitSegment, splitProgress)
%% Section 0: Header & Readme
% SYNTAX
%   preparedMotion = bmtpEngine.pipeline.prepareFinalMotion(solverRequest, controlPoint_units, segmentTime_s)
%   preparedMotion = bmtpEngine.pipeline.prepareFinalMotion(solverRequest, controlPoint_units, segmentTime_s, ...
%       suppliedPowerCoefficients_units, splitSegment, splitProgress)
%**************************************************************************
% PURPOSE
%   - Set endpoint states, correct curve joins, and assign final durations
%     once. Retain the resulting polynomial for all later checks and output.
%**************************************************************************
% INPUTS
%   - solverRequest (scalar struct)
%       Checked BMTP inputs, motion limits, available time, and arrival mode.
%       ArrivalTimeTolerance_s allows small timing errors. Curve-join changes
%       use the larger of CollisionClearanceTolerance_units and one part per
%       million of the coordinate scale; ConstraintTolerance is not used here.
%   - controlPoint_units (S-by-(D+1)-by-2 numeric array)
%       Selected composite Bezier control points.
%   - segmentTime_s (positive finite numeric vector)
%       Selected per-segment durations.
%   - suppliedPowerCoefficients_units (S-by-2-by-(D+1) numeric array, optional)
%       Coefficients of p(u) = c0 + c1*u + c2*u^2 + ... for segment progress
%       u from 0 to 1. A finite coefficient set for an axis takes priority over
%       converted controls; an axis filled with NaN uses the converted controls.
%   - splitSegment (S-by-1 logical array, optional)
%       True for each segment to split into two; defaults to all segments.
%   - splitProgress (S-by-1 numeric array, optional)
%       Where to split each selected segment, between 0 and 1; defaults to 0.5.
%**************************************************************************
% OUTPUTS
%   - preparedMotion (scalar struct)
%       Prepared controls, complete power coefficients, times, duration checks,
%       and SourceSegmentIndex mapping each piece to its input segment.
%       Success = false if the motion exceeds the available time or a join repair
%       would move a control point beyond the allowed tolerance. These
%       expected failures return a message instead of throwing an error.
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

controlPoint_units = bmtpEngine.motion.imposeEndpointControls(controlPoint_units, segmentTime_s, ...
    solverRequest.InitialState, solverRequest.GoalState);
[controlPoint_units, segmentTime_s, suppliedPowerCoefficients_units, sourceSegmentIndex] = ...
    bmtpEngine.motion.subdivideMotion(controlPoint_units, segmentTime_s, ...
    suppliedPowerCoefficients_units, splitSegment, splitProgress);

%% Section 2: Check Required Durations And Keep The Requested Timing

% Conversion may make small corrections so position, velocity, acceleration,
% and jerk match at curve joins. Check durations for both the input controls
% and controls reconstructed from that polynomial.
motionPolynomial = bmtpEngine.motion.createPowerPolynomial( ...
    controlPoint_units, segmentTime_s, 0, suppliedPowerCoefficients_units);
polynomialControlPoint_units = bmtpEngine.motion.powerToBernstein(motionPolynomial.positionPower_units);
requiredTime_s = max( ...
    bmtpEngine.motion.findRequiredSegmentTime(controlPoint_units, solverRequest.Limits), ...
    bmtpEngine.motion.findRequiredSegmentTime(polynomialControlPoint_units, solverRequest.Limits));

% These control-point bounds may ask for more time than the curve needs.
% The caller also checks the polynomial throughout each segment before
% accepting the motion. Keep fixed-arrival times; otherwise allow a longer
% duration to meet these bounds, with a small floating-point allowance.
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
    % Correct the tiny rounding error from summing segment durations so
    % their total equals the fixed time allowed for the motion.
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

% Joining curves may correct small numerical differences left by the solver.
% Reject a larger change: it would alter the motion whose clearance was solved.
% The limit is one part per million of the coordinate scale, or the collision
% clearance tolerance if larger. This covers measured solver errors of about
% two parts in ten million on fixed-arrival motions.
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

% The corrected polynomial defines the physical curve from this point on.
% Keep every coefficient, including axes originally converted from controls,
% so checking, subdivision, and output never reconstruct its joins again.
% The common duration scale above changes time without changing this curve.
% Check whether the assigned durations meet the control-point bounds.
% Passing this check does not replace the final polynomial validation.
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
