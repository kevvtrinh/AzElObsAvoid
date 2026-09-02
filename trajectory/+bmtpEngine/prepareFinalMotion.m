function preparedMotion = prepareFinalMotion( ...
        request, controlPoint_deg, segmentTime_s, operations)
%% Section 0: Header & Readme
% SYNTAX
%   preparedMotion = bmtpEngine.prepareFinalMotion( ...
%       request, controlPoint_deg, segmentTime_s, operations)
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
%   - operations (scalar struct of function handles)
%       Polynomial and subdivision kernels owned by BMTP solve.
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
controlPoint_deg = operations.subdivideMidpoint(controlPoint_deg);
segmentTime_s = segmentTime_s / 2;

%% Section 2: Find And Apply The Required Segment Time

exportPolynomial = operations.createPowerPolynomial( ...
    controlPoint_deg, 1, 0);
certifiedControlPoint_deg = operations.powerToBernsteinControls( ...
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

motionCertificate = operations.createMotionCertificate( ...
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
