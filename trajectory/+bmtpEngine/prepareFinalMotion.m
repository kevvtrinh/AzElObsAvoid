function preparedMotion = prepareFinalMotion(request, controlPoint_units, segmentTime_s, prescribedPower_units)
%% Section 0: Header & Readme
% SYNTAX
%   preparedMotion = bmtpEngine.prepareFinalMotion( ...
%       request, controlPoint_units, segmentTime_s)
%
% PURPOSE
%   - Impose physical endpoint states, split the selected curve, and
%     dilate eligible rest motion to satisfy derivative-control bounds.
%
% INPUTS
%   - request (scalar struct)
%       Checked BMTP request, limits, horizon, and goal-time policy.
%   - controlPoint_units (S-by-(D+1)-by-2 numeric array)
%       Selected composite Bezier control points.
%   - segmentTime_s (positive finite scalar)
%       Selected per-segment durations.
%   - prescribedPower_units (optional normalized analytic axis coefficients)
%       Preserved exactly through subdivision and independently certified.
%
% OUTPUTS
%   - preparedMotion (scalar struct)
%       Prepared controls, time, timing certificate, and expected failure.
%
% UNITS
%   - Position is coordinate units and time is seconds.
%

%% Section 1: Set Endpoint Derivatives And Split The Curve

if nargin<4, prescribedPower_units = []; end
if ~isempty(prescribedPower_units)
    % Restrict the prescribed power polynomial directly. Returning through
    % absolute degree-eight controls would lose its known low-degree form.
    degree = size(controlPoint_units,2)-1;
    refined = NaN(2*size(controlPoint_units,1),2,degree+1);
    for k = 0:degree
        refined(1:2:end,:,k+1) = prescribedPower_units(:,:,k+1)*0.5^k;
        value = zeros(size(controlPoint_units,1),2);
        for j = k:degree
            value = value+nchoosek(j,k)*prescribedPower_units(:,:,j+1)*0.5^j;
        end
        refined(2:2:end,:,k+1) = value;
    end
    prescribedPower_units = refined;
end
controlPoint_units = bmtpEngine.imposeEndpointControls(controlPoint_units,segmentTime_s,request.InitialState,request.GoalState);
controlPoint_units = subdivideMidpoint(controlPoint_units);
segmentTime_s    = repelem(segmentTime_s(:), 2, 1) / 2;

%% Section 2: Find And Apply The Required Segment Time

exportPolynomial          = bmtpEngine.createPowerPolynomial(controlPoint_units, segmentTime_s, 0,prescribedPower_units);
certifiedControlPoint_units = powerToBernsteinControls(exportPolynomial.positionPower_units);
requiredTime_s            = max(bmtpEngine.findRequiredSegmentTime(controlPoint_units, request.Limits), bmtpEngine.findRequiredSegmentTime(certifiedControlPoint_units, request.Limits));
dilationScale             = max([1; requiredTime_s ./ segmentTime_s]) * (1 + 64 * eps);
% Nonzero boundary derivatives fix the physical clock. Continuous validation
% decides whether the exported proposal meets limits; it is never time-scaled.
if ~request.IsRest, dilationScale = 1; end
segmentTime_s             = segmentTime_s * dilationScale;
minimumDuration_s         = sum(segmentTime_s);
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
elseif isFixedArrival && request.IsRest
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
preparedMotion.PrescribedPower_units = prescribedPower_units;
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
    motion = struct("Passed", all(segmentTime_s >= requiredTime_s), ...
        "SegmentTime_s", segmentTime_s, ...
        "RequiredSegmentTime_s", requiredTime_s, ...
        "MaximumViolation", max([0; requiredTime_s - segmentTime_s]));
end
