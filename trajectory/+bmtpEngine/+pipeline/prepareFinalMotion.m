function preparedMotion = prepareFinalMotion(solverRequest, controlPoint_units, segmentTime_s, ...
        suppliedPowerCoefficients_units, splitSegment, splitProgress)
%% Section 0: Header & Readme
% SYNTAX
%   preparedMotion = bmtpEngine.pipeline.prepareFinalMotion(solverRequest, controlPoint_units, segmentTime_s)
%   preparedMotion = bmtpEngine.pipeline.prepareFinalMotion(solverRequest, controlPoint_units, segmentTime_s, ...
%       suppliedPowerCoefficients_units, splitSegment, splitProgress)
%**************************************************************************
% PURPOSE
%   - Set the endpoint position, velocity, and acceleration, split selected
%     curve segments, and prepare the motion for continuous checks.
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
%       Prepared controls, times, and duration checks. Success = false if
%       the motion exceeds the available time or correcting a curve join
%       would move a control point beyond the allowed tolerance. These
%       expected failures return a message instead of throwing an error.
%**************************************************************************
% UNITS
%   - Position is coordinate units and time is seconds.
%**************************************************************************

%% Section 1: Resolve The Requested Split Layout

if nargin < 4
    suppliedPowerCoefficients_units = [];
end
if nargin < 5
    splitSegment = true(size(controlPoint_units, 1), 1);
end
splitSegment = logical(splitSegment(:));
if nargin < 6
    splitProgress = repmat(0.5, numel(splitSegment), 1);
end
splitProgress = splitProgress(:);

% An unchanged segment produces one output segment; a split produces two.
% For [false true false], output counts are [1 2 1] and starts are [1 2 4].
outputCountBySegment     = 1 + double(splitSegment);
firstOutputSegmentIndex = 1 + [0; cumsum(outputCountBySegment(1:end - 1))];

%% Section 2: Split Supplied Polynomial Coefficients Without Changing The Curve

if ~isempty(suppliedPowerCoefficients_units)
    % Split the polynomial coefficients directly when they were supplied.
    % Converting through control points could add roundoff to coefficients
    % that are known exactly, including zero higher-degree terms.
    curveDegree = size(controlPoint_units, 2) - 1;
    subdividedPowerCoefficients_units = NaN(sum(outputCountBySegment), 2, curveDegree + 1);
    for segmentIndex = 1:numel(splitSegment)
        outputSegmentIndex = firstOutputSegmentIndex(segmentIndex);
        if ~splitSegment(segmentIndex)
            subdividedPowerCoefficients_units(outputSegmentIndex, :, :) = ...
                suppliedPowerCoefficients_units(segmentIndex, :, :);
            continue
        end
        % Both new pieces use progress v from 0 to 1. At split location f,
        % the original progress is u = f x v on the left and
        % u = f + (1 - f) x v on the right. Expand these to get new coefficients.
        splitLocation = splitProgress(segmentIndex);
        for powerIndex = 0:curveDegree
            subdividedPowerCoefficients_units(outputSegmentIndex, :, powerIndex + 1) = ...
                suppliedPowerCoefficients_units(segmentIndex, :, powerIndex + 1) * splitLocation ^ powerIndex;
            rightCoefficient_units = zeros(1, 2);
            for sourcePowerIndex = powerIndex:curveDegree
                rightCoefficient_units = rightCoefficient_units + nchoosek(sourcePowerIndex, powerIndex) * ...
                    suppliedPowerCoefficients_units(segmentIndex, :, sourcePowerIndex + 1) * ...
                    splitLocation ^ (sourcePowerIndex - powerIndex) * (1 - splitLocation) ^ powerIndex;
            end
            subdividedPowerCoefficients_units(outputSegmentIndex + 1, :, powerIndex + 1) = ...
                rightCoefficient_units;
        end
    end
    suppliedPowerCoefficients_units = subdividedPowerCoefficients_units;
end

%% Section 3: Set Endpoint States And Split The Curve

controlPoint_units = bmtpEngine.motion.imposeEndpointControls(controlPoint_units, segmentTime_s, ...
    solverRequest.InitialState, solverRequest.GoalState);
controlPoint_units = subdivideControls(controlPoint_units, splitSegment, splitProgress);

% Divide time in the same ratio as the curve. For example, splitting a
% 10-second segment at 0.3 gives durations of 3 and 7 seconds.
subdividedSegmentTime_s = repelem(segmentTime_s(:), outputCountBySegment);
subdividedSegmentTime_s = subdividedSegmentTime_s(:);
for segmentIndex = find(splitSegment).'
    outputSegmentIndex = firstOutputSegmentIndex(segmentIndex);
    subdividedSegmentTime_s(outputSegmentIndex:outputSegmentIndex + 1) = ...
        segmentTime_s(segmentIndex) * [splitProgress(segmentIndex); 1 - splitProgress(segmentIndex)];
end
segmentTime_s = subdividedSegmentTime_s;

%% Section 4: Check Required Durations And Keep The Requested Timing

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

%% Section 5: Reject Excessive Join Changes Or Insufficient Time

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

%% Section 6: Return The Prepared Motion And Checks

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
    "ControlPoint_units",                     controlPoint_units, ...
    "ProvenControlPoint_units",               polynomialControlPoint_units, ...
    "SegmentTime_s",                          segmentTime_s, ...
    "FinalTime_s",                            finalTime_s, ...
    "RequiredSegmentTime_s",                  requiredTime_s, ...
    "DilationScale",                          durationScale, ...
    "MotionProof",                            durationCheck, ...
    "ContinuityProjectionDisplacement_units", joinCorrectionDistance_units);
preparedMotion.GivenPower_units = suppliedPowerCoefficients_units;
end

%% Section 7: Local Functions

function subdividedControls_units = subdivideControls(controlPoint_units, splitSegment, splitProgress)
    % Split selected curves into two pieces without changing their paths.
    % restrictBezier computes the control points for each requested portion.
    segmentCount             = size(controlPoint_units, 1);
    curveDegree              = size(controlPoint_units, 2) - 1;
    subdividedControls_units = zeros(segmentCount + nnz(splitSegment), curveDegree + 1, 2);
    outputSegmentIndex       = 1;
    for segmentIndex = 1:segmentCount
        if ~splitSegment(segmentIndex)
            subdividedControls_units(outputSegmentIndex, :, :) = controlPoint_units(segmentIndex, :, :);
            outputSegmentIndex = outputSegmentIndex + 1;
            continue
        end
        segmentControls_units = squeeze(controlPoint_units(segmentIndex, :, :));
        splitLocation         = splitProgress(segmentIndex);
        subdividedControls_units(outputSegmentIndex, :, :) = ...
            bmtpEngine.motion.restrictBezier(segmentControls_units, [0, splitLocation]);
        subdividedControls_units(outputSegmentIndex + 1, :, :) = ...
            bmtpEngine.motion.restrictBezier(segmentControls_units, [splitLocation, 1]);
        outputSegmentIndex = outputSegmentIndex + 2;
    end
end
