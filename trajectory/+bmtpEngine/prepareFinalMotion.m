function preparedMotion = prepareFinalMotion(request, controlPoint_units, segmentTime_s, ...
        prescribedPower_units, splitMask, splitFraction)
%% Section 0: Header & Readme
% SYNTAX
%   preparedMotion = bmtpEngine.prepareFinalMotion(request, controlPoint_units, segmentTime_s)
%   preparedMotion = bmtpEngine.prepareFinalMotion(request, controlPoint_units, segmentTime_s, ...
%       prescribedPower_units, splitMask, splitFraction)
%**************************************************************************
% PURPOSE
%   - Impose physical endpoint states, split the selected curve, and retain
%     its physical clock for continuous derivative certification.
%**************************************************************************
% INPUTS
%   - request (scalar struct)
%       Checked BMTP request, limits, horizon, and goal-time policy.
%   - controlPoint_units (S-by-(D+1)-by-2 numeric array)
%       Selected composite Bezier control points.
%   - segmentTime_s (positive finite numeric vector)
%       Selected per-segment durations.
%   - prescribedPower_units (S-by-2-by-(D+1) numeric array, optional)
%       Normalized analytic axis coefficients, preserved exactly through
%       subdivision and independently certified.
%   - splitMask (S-by-1 logical array, optional)
%       Spans to subdivide; defaults to every span.
%   - splitFraction (S-by-1 numeric array, optional)
%       Interior split locations; defaults to 0.5.
%**************************************************************************
% OUTPUTS
%   - preparedMotion (scalar struct)
%       Prepared controls, time, timing certificate, and expected failure.
%       An infeasible horizon is reported as Success = false, not thrown.
%**************************************************************************
% UNITS
%   - Position is coordinate units and time is seconds.
%**************************************************************************

%% Section 1: Resolve The Requested Split Layout

if nargin < 4
    prescribedPower_units = [];
end
if nargin < 5
    splitMask = true(size(controlPoint_units, 1), 1);
end
splitMask = logical(splitMask(:));
if nargin < 6
    splitFraction = repmat(0.5, numel(splitMask), 1);
end
splitFraction = splitFraction(:);
repeatCount   = 1 + double(splitMask);
outputStart   = 1 + [0; cumsum(repeatCount(1:end - 1))];

%% Section 2: Restrict The Prescribed Power Polynomial Exactly

if ~isempty(prescribedPower_units)
    % Restrict the prescribed power polynomial directly. Returning through
    % absolute Bernstein controls would lose its known low-degree form.
    degree  = size(controlPoint_units, 2) - 1;
    refined = NaN(sum(repeatCount), 2, degree + 1);
    for spanIndex = 1:numel(splitMask)
        targetIndex = outputStart(spanIndex);
        if ~splitMask(spanIndex)
            refined(targetIndex, :, :) = prescribedPower_units(spanIndex, :, :);
            continue
        end
        fraction = splitFraction(spanIndex);
        for powerIndex = 0:degree
            refined(targetIndex, :, powerIndex + 1) = ...
                prescribedPower_units(spanIndex, :, powerIndex + 1) * fraction ^ powerIndex;
            value = zeros(1, 2);
            for sourceIndex = powerIndex:degree
                value = value + nchoosek(sourceIndex, powerIndex) * ...
                    prescribedPower_units(spanIndex, :, sourceIndex + 1) * ...
                    fraction ^ (sourceIndex - powerIndex) * (1 - fraction) ^ powerIndex;
            end
            refined(targetIndex + 1, :, powerIndex + 1) = value;
        end
    end
    prescribedPower_units = refined;
end

%% Section 3: Set Endpoint Derivatives And Split The Curve

controlPoint_units = bmtpEngine.imposeEndpointControls(controlPoint_units, segmentTime_s, ...
    request.InitialState, request.GoalState);
controlPoint_units = subdivideControls(controlPoint_units, splitMask, splitFraction);
refinedTime_s      = repelem(segmentTime_s(:), repeatCount);
refinedTime_s      = refinedTime_s(:);
for spanIndex = find(splitMask).'
    targetIndex = outputStart(spanIndex);
    refinedTime_s(targetIndex:targetIndex + 1) = ...
        segmentTime_s(spanIndex) * [splitFraction(spanIndex); 1 - splitFraction(spanIndex)];
end
segmentTime_s = refinedTime_s;

%% Section 4: Record Sufficient Control Bounds And Preserve The Clock

exportPolynomial            = bmtpEngine.createPowerPolynomial( ...
    controlPoint_units, segmentTime_s, 0, prescribedPower_units);
certifiedControlPoint_units = bmtpEngine.powerToBernstein(exportPolynomial.positionPower_units);
requiredTime_s = max(bmtpEngine.findRequiredSegmentTime(controlPoint_units, request.Limits), ...
    bmtpEngine.findRequiredSegmentTime(certifiedControlPoint_units, request.Limits));
% Control hull bounds are sufficient, not necessary. The exported polynomial
% is checked continuously against the actual physical limits before success.
isFixedArrival = request.Options.GoalTimeMode == "fixedArrival";
dilationScale  = 1;
if ~isFixedArrival
    dilationScale = max(1, max(requiredTime_s ./ segmentTime_s)) * (1 + 64 * eps);
    segmentTime_s = segmentTime_s * dilationScale;
end
minimumDuration_s = sum(segmentTime_s);
clockIsAtPrescribedEndpoint = isFixedArrival && ...
    abs(sum(segmentTime_s) - request.MotionHorizon_s) <= 64 * eps(request.MotionHorizon_s);
if clockIsAtPrescribedEndpoint
    % Close only accumulation roundoff at the prescribed physical endpoint.
    for passIndex = 1:2
        segmentTime_s(end) = segmentTime_s(end) + (request.MotionHorizon_s - sum(segmentTime_s));
    end
end

%% Section 5: Report The Expected Horizon Failure

success           = minimumDuration_s <= request.MotionHorizon_s + request.Options.ConstraintTolerance;
message           = "";
terminationReason = "";
if ~success
    reasons  = ["timeWindowInfeasible", "fixedArrivalInfeasible"];
    messages = ["The certified motion exceeds the goal horizon.", ...
        "The certified minimum exceeds the fixed arrival."];
    message           = messages(1 + isFixedArrival);
    terminationReason = reasons(1 + isFixedArrival);
end

%% Section 6: Return The Prepared Representation

motionCertificate = createMotionCertificate(segmentTime_s, requiredTime_s);
preparedMotion    = struct( ...
    "Success",                     success, ...
    "Message",                     message, ...
    "TerminationReason",           terminationReason, ...
    "ControlPoint_units",          controlPoint_units, ...
    "CertifiedControlPoint_units", certifiedControlPoint_units, ...
    "SegmentTime_s",               segmentTime_s, ...
    "RequiredSegmentTime_s",       requiredTime_s, ...
    "DilationScale",               dilationScale, ...
    "MotionCertificate",           motionCertificate);
preparedMotion.PrescribedPower_units = prescribedPower_units;
end

%% Section 7: Local Functions

function subdivided_units = subdivideControls(controlPoint_units, splitMask, splitFraction)
    % Restrict selected spans exactly, reusing the one de Casteljau restriction.
    segmentCount     = size(controlPoint_units, 1);
    degree           = size(controlPoint_units, 2) - 1;
    subdivided_units = zeros(segmentCount + nnz(splitMask), degree + 1, 2);
    targetIndex      = 1;
    for segmentIndex = 1:segmentCount
        if ~splitMask(segmentIndex)
            subdivided_units(targetIndex, :, :) = controlPoint_units(segmentIndex, :, :);
            targetIndex = targetIndex + 1;
            continue
        end
        span_units = squeeze(controlPoint_units(segmentIndex, :, :));
        fraction   = splitFraction(segmentIndex);
        subdivided_units(targetIndex, :, :)     = bmtpEngine.restrictBezier(span_units, [0, fraction]);
        subdivided_units(targetIndex + 1, :, :) = bmtpEngine.restrictBezier(span_units, [fraction, 1]);
        targetIndex = targetIndex + 2;
    end
end

function motion = createMotionCertificate(segmentTime_s, requiredTime_s)
    % This is a sufficient control-hull diagnostic, not the final physical test.
    motion = struct( ...
        "Passed",           all(segmentTime_s >= requiredTime_s), ...
        "SegmentTime_s",    segmentTime_s, ...
        "MaximumViolation", max([0; requiredTime_s - segmentTime_s]));
end
