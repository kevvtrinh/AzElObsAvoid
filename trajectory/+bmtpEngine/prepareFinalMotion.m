function preparedMotion = prepareFinalMotion(request, controlPoint_units, segmentTime_s)
%% Section 0: Header & Readme
% SYNTAX
%   preparedMotion = bmtpEngine.prepareFinalMotion( ...
%       request, controlPoint_units, segmentTime_s)
%
% PURPOSE
%   - Impose exact rest-to-rest endpoints, split the selected curve, and
%     increase segment times enough to satisfy derivative-control bounds.
%
% INPUTS
%   - request (scalar struct)
%       Checked BMTP request, limits, horizon, and goal-time policy.
%   - controlPoint_units (S-by-(D+1)-by-2 numeric array)
%       Selected composite Bezier control points.
%   - segmentTime_s (positive finite scalar)
%       Selected common segment time.
%
% OUTPUTS
%   - preparedMotion (scalar struct)
%       Prepared controls, time, timing certificate, and expected failure.
%
% UNITS
%   - Position is coordinate units and time is seconds.
%

%% Section 1: Set Endpoint Derivatives And Split The Curve

controlPoint_units(1, 1:3, :) = reshape(repmat(request.InitialState.position_units, 3, 1), 1, 3, 2);
controlPoint_units(end, end - 2:end, :) = reshape(repmat(request.GoalState.position_units, 3, 1), 1, 3, 2);
controlPoint_units = subdivideMidpoint(controlPoint_units);
segmentTime_s    = segmentTime_s / 2;

%% Section 2: Find And Apply The Required Segment Time

exportPolynomial          = bmtpEngine.createPowerPolynomial(controlPoint_units, 1, 0);
certifiedControlPoint_units = powerToBernsteinControls(exportPolynomial.positionPower_units);
requiredTime_s            = max(bmtpEngine.findRequiredSegmentTime(controlPoint_units, request.Limits), bmtpEngine.findRequiredSegmentTime(certifiedControlPoint_units, request.Limits));
dilationScale             = max(1, requiredTime_s / segmentTime_s) * (1 + 64 * eps);
segmentTime_s             = segmentTime_s * dilationScale;
minimumDuration_s         = size(controlPoint_units, 1) * segmentTime_s;
isFixedArrival            = request.Options.GoalTimeMode == "fixedArrival";
success                   = minimumDuration_s <= request.MotionHorizon_s + request.Options.ConstraintTolerance;
message                   = "";
terminationReason         = "";
if ~success
    reasons  = ["timeWindowInfeasible", "fixedArrivalInfeasible"];
    messages = ["The certified motion exceeds the goal horizon.", ...
        "The certified minimum exceeds the fixed arrival."];
    message           = messages(1 + isFixedArrival);
    terminationReason = reasons(1 + isFixedArrival);
elseif isFixedArrival
    fixedScale    = request.MotionHorizon_s / minimumDuration_s;
    segmentTime_s = segmentTime_s * fixedScale;
    dilationScale = dilationScale * fixedScale;
end

%% Section 3: Return The Prepared Representation

motionCertificate = createMotionCertificate(segmentTime_s, requiredTime_s);
preparedMotion    = struct("Success", success, ...
    "Message", message, ...
    "TerminationReason", terminationReason, ...
    "ControlPoint_units", controlPoint_units, ...
    "CertifiedControlPoint_units", certifiedControlPoint_units, ...
    "SegmentTime_s", segmentTime_s, ...
    "RequiredSegmentTime_s", requiredTime_s, ...
    "DilationScale", dilationScale, ...
    "ArrivalAtHorizon", isFixedArrival, ...
    "MotionCertificate", motionCertificate);
end

%% Section 4: Local Functions

function subdivided_units = subdivideMidpoint(controlPoint_units)
    % Split each Bezier span in half using de Casteljau subdivision.
    segmentCount   = size(controlPoint_units, 1);
    degree         = size(controlPoint_units, 2) - 1;
    subdivided_units = zeros(2 * segmentCount, degree + 1, 2);
    % Process each segment while assembling the complete motion or interval result.
    for segmentIndex = 1:segmentCount
        work_units  = squeeze(controlPoint_units(segmentIndex, :, :));
        left_units  = zeros(degree + 1, 2);
        right_units = zeros(degree + 1, 2);
        left_units(1, :) = work_units(1, :);
        right_units(end, :) = work_units(end, :);
        % Repeat the level alternatives needed to refine the current solution.
        for levelIndex = 1:degree
            work_units = (work_units(1:end - 1, :) + work_units(2:end, :)) / 2;
            left_units(levelIndex + 1, :) = work_units(1, :);
            right_units(end - levelIndex, :) = work_units(end, :);
        end
        subdivided_units(2 * segmentIndex - 1, :, :) = left_units;
        subdivided_units(2 * segmentIndex, :, :) = right_units;
    end
end

function controlPoint_units = powerToBernsteinControls(positionPower_units)
    % Reconstruct Bezier controls from the exported power coefficients.
    degree    = size(positionPower_units, 3) - 1;
    transform = zeros(degree + 1);
    % Process each bernstein needed to complete power to bernstein controls.
    for bernsteinIndex = 0:degree
        % Process each power needed to complete power to bernstein controls.
        for powerIndex = 0:bernsteinIndex
            transform(bernsteinIndex + 1, powerIndex + 1) = nchoosek(bernsteinIndex, powerIndex) / nchoosek(degree, powerIndex);
        end
    end
    powerPages       = permute(positionPower_units, [3 1 2]);
    controlPoint_units = permute(pagemtimes(transform, powerPages), [2 1 3]);
end

function motion = createMotionCertificate(segmentTime_s, requiredTime_s)
    % Record the derivative bound used to stretch time.
    motion = struct("Passed", segmentTime_s >= requiredTime_s, ...
        "SegmentTime_s", segmentTime_s, ...
        "RequiredSegmentTime_s", requiredTime_s, ...
        "MaximumViolation", max(0, requiredTime_s - segmentTime_s));
end
