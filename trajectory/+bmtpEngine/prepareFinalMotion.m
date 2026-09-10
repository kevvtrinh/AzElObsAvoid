function preparedMotion = prepareFinalMotion(request, controlPoint_units, segmentTime_s, prescribedPower_units, splitMask, splitFraction)
%% Section 0: Header & Readme
% SYNTAX: preparedMotion = bmtpEngine.prepareFinalMotion( request, controlPoint_units,
%   segmentTime_s)
% PURPOSE: Impose physical endpoint states, split the selected curve, and retain its physical clock
%   for continuous derivative certification.
% INPUTS: request (scalar struct) Checked BMTP request, limits, horizon, and goal-time policy.
%   controlPoint_units (S-by-(D+1)-by-2 numeric array) Selected composite Bezier control points.
%   segmentTime_s (positive finite scalar) Selected per-segment durations. prescribedPower_units
%   (optional normalized analytic axis coefficients) Preserved exactly through subdivision and
%   independently certified. splitMask (optional logical S-by-1): spans to subdivide; default all.
%   splitFraction (optional S-by-1): interior split locations; default 0.5.
% OUTPUTS: preparedMotion (scalar struct) Prepared controls, time, timing certificate, and expected
%   failure.
% UNITS: Position is coordinate units and time is seconds.

%% Section 1: Set Endpoint Derivatives And Split The Curve
if nargin<4, prescribedPower_units = []; end
if nargin<5, splitMask=true(size(controlPoint_units,1),1); end
splitMask=logical(splitMask(:));
if nargin<6, splitFraction=repmat(0.5,numel(splitMask),1); end
splitFraction=splitFraction(:);
repeatCount=1+double(splitMask);
outputStart=1+[0;cumsum(repeatCount(1:end-1))];
if ~isempty(prescribedPower_units)
    % Restrict the prescribed power polynomial directly. Returning through
    % absolute Bernstein controls would lose its known low-degree form.
    degree = size(controlPoint_units,2)-1;
    refined = NaN(sum(repeatCount),2,degree+1);
    for span=1:numel(splitMask)
        target=outputStart(span);
        if ~splitMask(span)
            refined(target,:,:)=prescribedPower_units(span,:,:);
            continue;
        end
        for k=0:degree
            refined(target,:,k+1)=prescribedPower_units(span,:,k+1)*splitFraction(span)^k;
            value=zeros(1,2);
            for j=k:degree
                value=value+nchoosek(j,k)*prescribedPower_units(span,:,j+1)*splitFraction(span)^(j-k)*(1-splitFraction(span))^k;
            end
            refined(target+1,:,k+1)=value;
        end
    end
    prescribedPower_units = refined;
end
controlPoint_units = bmtpEngine.imposeEndpointControls(controlPoint_units,segmentTime_s,request.InitialState,request.GoalState);
controlPoint_units = subdivideControls(controlPoint_units,splitMask,splitFraction);
refinedTime_s=repelem(segmentTime_s(:),repeatCount); refinedTime_s=refinedTime_s(:);
for span=find(splitMask).'
    target=outputStart(span);
    refinedTime_s(target:target+1)=segmentTime_s(span)*[splitFraction(span);1-splitFraction(span)];
end
segmentTime_s=refinedTime_s;

%% Section 2: Record Sufficient Control Bounds And Preserve The Clock
exportPolynomial          = bmtpEngine.createPowerPolynomial(controlPoint_units, segmentTime_s, 0,prescribedPower_units);
certifiedControlPoint_units = powerToBernsteinControls(exportPolynomial.positionPower_units);
requiredTime_s            = max(bmtpEngine.findRequiredSegmentTime(controlPoint_units, request.Limits), bmtpEngine.findRequiredSegmentTime(certifiedControlPoint_units, request.Limits));
% Control hull bounds are sufficient, not necessary. The exported polynomial
% is checked continuously against the actual physical limits before success.
minimumDuration_s         = sum(segmentTime_s);
isFixedArrival            = request.Options.GoalTimeMode == "fixedArrival";
if isFixedArrival && abs(sum(segmentTime_s)-request.MotionHorizon_s)<=64*eps(request.MotionHorizon_s)
    % Close only accumulation roundoff at the prescribed physical endpoint.
    for pass=1:2
        segmentTime_s(end)=segmentTime_s(end)+(request.MotionHorizon_s-sum(segmentTime_s));
    end
end
success                   = minimumDuration_s <= request.MotionHorizon_s + request.Options.ConstraintTolerance;
message                   = "";
terminationReason         = "";
if ~success
    reasons  = ["timeWindowInfeasible", "fixedArrivalInfeasible"];
    messages = ["The certified motion exceeds the goal horizon.", ...
        "The certified minimum exceeds the fixed arrival."];
    message           = messages(1 + isFixedArrival);
    terminationReason = reasons(1 + isFixedArrival);

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
    "DilationScale", 1, ...
    "ArrivalAtHorizon", isFixedArrival, ...
    "MotionCertificate", motionCertificate);
preparedMotion.PrescribedPower_units = prescribedPower_units;
end

%% Section 4: Local Functions
function subdivided_units = subdivideControls(controlPoint_units,splitMask,splitFraction)
    % Restrict selected spans exactly using de Casteljau subdivision.
    segmentCount   = size(controlPoint_units, 1);
    degree         = size(controlPoint_units, 2) - 1;
    subdivided_units = zeros(segmentCount+nnz(splitMask), degree + 1, 2);
    target=1;
    for segmentIndex = 1:segmentCount
        if ~splitMask(segmentIndex)
            subdivided_units(target,:,:)=controlPoint_units(segmentIndex,:,:);
            target=target+1;
            continue;
        end
        work_units  = squeeze(controlPoint_units(segmentIndex, :, :));
        left_units  = zeros(degree + 1, 2);
        right_units = zeros(degree + 1, 2);
        left_units(1, :) = work_units(1, :);
        right_units(end, :) = work_units(end, :);
        for levelIndex = 1:degree
            work_units = (1-splitFraction(segmentIndex))*work_units(1:end - 1, :)+splitFraction(segmentIndex)*work_units(2:end, :);
            left_units(levelIndex + 1, :) = work_units(1, :);
            right_units(end - levelIndex, :) = work_units(end, :);
        end
        subdivided_units(target, :, :) = left_units;
        subdivided_units(target+1, :, :) = right_units;
        target=target+2;
    end
end

function controlPoint_units = powerToBernsteinControls(positionPower_units)
    % Reconstruct Bezier controls from the exported power coefficients.
    degree    = size(positionPower_units, 3) - 1;
    transform = zeros(degree + 1);
    for bernsteinIndex = 0:degree
        for powerIndex = 0:bernsteinIndex
            transform(bernsteinIndex + 1, powerIndex + 1) = nchoosek(bernsteinIndex, powerIndex) / nchoosek(degree, powerIndex);
        end
    end
    powerPages       = permute(positionPower_units, [3 1 2]);
    controlPoint_units = permute(pagemtimes(transform, powerPages), [2 1 3]);
end

function motion = createMotionCertificate(segmentTime_s, requiredTime_s)
    % This is a sufficient control-hull diagnostic, not the final physical test.
    motion = struct("Passed", all(segmentTime_s >= requiredTime_s), "Kind", "sufficientControlHull", ...
        "SegmentTime_s", segmentTime_s, ...
        "RequiredSegmentTime_s", requiredTime_s, ...
        "MaximumViolation", max([0; requiredTime_s - segmentTime_s]));
end
