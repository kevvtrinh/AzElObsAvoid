function preparedMotion = prepareFinalMotion( ...
        request, controlPoint_deg, segmentTime_s)
%% Section 0: Header & Readme
% SYNTAX
%   preparedMotion = bmtpEngine.prepareFinalMotion( ...
%       request, controlPoint_deg, segmentTime_s)
%**************************************************************************
% PURPOSE
%   - Impose exact rest-to-rest endpoints, split the selected curve, and
%     increase segment times enough to satisfy derivative-control bounds.
%**************************************************************************
% INPUTS
%   - request (scalar struct)
%       Checked BMTP request, limits, horizon, and goal-time policy.
%   - controlPoint_deg (S-by-(D+1)-by-2 numeric array)
%       Selected composite Bezier control points.
%   - segmentTime_s (positive finite scalar)
%       Selected common segment time.
%**************************************************************************
% OUTPUTS
%   - preparedMotion (scalar struct)
%       Prepared controls, time, timing certificate, and expected failure.
%**************************************************************************
% UNITS
%   - Position is degrees and time is seconds.
%**************************************************************************

%% Section 1: Set Endpoint Derivatives And Split The Curve

controlPoint_deg(1, 1:3, :) = reshape(repmat( ...
    request.InitialState.position_deg, 3, 1), 1, 3, 2);
controlPoint_deg(end, end - 2:end, :) = reshape(repmat( ...
    request.GoalState.position_deg, 3, 1), 1, 3, 2);
controlPoint_deg = subdivideMidpoint(controlPoint_deg);
segmentTime_s = segmentTime_s / 2;

%% Section 2: Find And Apply The Required Segment Time

exportPolynomial = bmtpEngine.createPowerPolynomial( ...
    controlPoint_deg, 1, 0);
certifiedControlPoint_deg = powerToBernsteinControls( ...
    exportPolynomial.positionPower_deg);
requiredTime_s = max( ...
    bmtpEngine.findRequiredSegmentTime(controlPoint_deg, request.Limits), ...
    bmtpEngine.findRequiredSegmentTime( ...
    certifiedControlPoint_deg, request.Limits));
dilationScale = max(1, requiredTime_s / segmentTime_s) * (1 + 64 * eps);
segmentTime_s = segmentTime_s * dilationScale;
minimumDuration_s = size(controlPoint_deg, 1) * segmentTime_s;
isFixedArrival = request.Options.GoalTimeMode == "fixedArrival";
success = minimumDuration_s <= request.MotionHorizon_s + ...
    request.Options.ConstraintTolerance;
message = "";
terminationReason = "";
if ~success
    reasons = ["timeWindowInfeasible", "fixedArrivalInfeasible"];
    messages = ["The certified motion exceeds the goal horizon.", ...
        "The certified minimum exceeds the fixed arrival."];
    message = messages(1 + isFixedArrival);
    terminationReason = reasons(1 + isFixedArrival);
elseif isFixedArrival
    fixedScale = request.MotionHorizon_s / minimumDuration_s;
    segmentTime_s = segmentTime_s * fixedScale;
    dilationScale = dilationScale * fixedScale;
end

%% Section 3: Return The Prepared Representation

motionCertificate = createMotionCertificate( ...
    segmentTime_s, requiredTime_s);
preparedMotion = struct( ...
    "Success", success, ...
    "Message", message, ...
    "TerminationReason", terminationReason, ...
    "ControlPoint_deg", controlPoint_deg, ...
    "CertifiedControlPoint_deg", certifiedControlPoint_deg, ...
    "SegmentTime_s", segmentTime_s, ...
    "RequiredSegmentTime_s", requiredTime_s, ...
    "DilationScale", dilationScale, ...
    "ArrivalAtHorizon", isFixedArrival, ...
    "MotionCertificate", motionCertificate);
end

%% Section 4: Local Functions

function subdivided_deg = subdivideMidpoint(controlPoint_deg)
% Restrict every Bezier span to exact half intervals by de Casteljau averaging.
segmentCount = size(controlPoint_deg, 1);
degree = size(controlPoint_deg, 2) - 1;
subdivided_deg = zeros(2 * segmentCount, degree + 1, 2);
for segmentIndex = 1:segmentCount
    work_deg = squeeze(controlPoint_deg(segmentIndex, :, :));
    left_deg = zeros(degree + 1, 2);
    right_deg = zeros(degree + 1, 2);
    left_deg(1, :) = work_deg(1, :);
    right_deg(end, :) = work_deg(end, :);
    for levelIndex = 1:degree
        work_deg = (work_deg(1:end - 1, :) + work_deg(2:end, :)) / 2;
        left_deg(levelIndex + 1, :) = work_deg(1, :);
        right_deg(end - levelIndex, :) = work_deg(end, :);
    end
    subdivided_deg(2 * segmentIndex - 1, :, :) = left_deg;
    subdivided_deg(2 * segmentIndex, :, :) = right_deg;
end
end

function controlPoint_deg = powerToBernsteinControls(positionPower_deg)
% Reconstruct the exact control net represented by exported position powers.
degree = size(positionPower_deg, 3) - 1;
transform = zeros(degree + 1);
for bernsteinIndex = 0:degree
    for powerIndex = 0:bernsteinIndex
        transform(bernsteinIndex + 1, powerIndex + 1) = ...
            nchoosek(bernsteinIndex, powerIndex) / ...
            nchoosek(degree, powerIndex);
    end
end
powerPages = permute(positionPower_deg, [3 1 2]);
controlPoint_deg = permute( ...
    pagemtimes(transform, powerPages), [2 1 3]);
end

function motion = createMotionCertificate(segmentTime_s, requiredTime_s)
% Record the exact derivative-control timing inequality used for dilation.
motion = struct("Passed", segmentTime_s >= requiredTime_s, ...
    "SegmentTime_s", segmentTime_s, ...
    "RequiredSegmentTime_s", requiredTime_s, ...
    "MaximumViolation", max(0, requiredTime_s - segmentTime_s));
end
