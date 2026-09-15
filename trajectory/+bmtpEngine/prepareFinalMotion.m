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
certifiedControlPoint_units = bmtpEngine.powerToBernstein(exportPolynomial.positionPower_units);
requiredTime_s            = max(bmtpEngine.findRequiredSegmentTime(controlPoint_units, request.Limits), bmtpEngine.findRequiredSegmentTime(certifiedControlPoint_units, request.Limits));
% Control hull bounds are sufficient, not necessary. The exported polynomial
% is checked continuously against the actual physical limits before success.
isFixedArrival            = request.Options.GoalTimeMode == "fixedArrival";
dilationScale=1;
if ~isFixedArrival
    dilationScale=max(1,max(requiredTime_s./segmentTime_s))*(1+64*eps);
    segmentTime_s=segmentTime_s*dilationScale;
end
minimumDuration_s         = sum(segmentTime_s);
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
    "DilationScale", dilationScale, ...
    "MotionCertificate", motionCertificate);
preparedMotion.PrescribedPower_units = prescribedPower_units;
end

%% Section 4: Local Functions
function subdivided_units = subdivideControls(controlPoint_units,splitMask,splitFraction)
    % Restrict selected spans exactly, reusing the one de Casteljau restriction.
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
        span_units = squeeze(controlPoint_units(segmentIndex, :, :));
        subdivided_units(target, :, :) = bmtpEngine.restrictBezier(span_units,[0,splitFraction(segmentIndex)]);
        subdivided_units(target+1, :, :) = bmtpEngine.restrictBezier(span_units,[splitFraction(segmentIndex),1]);
        target=target+2;
    end
end

function motion = createMotionCertificate(segmentTime_s, requiredTime_s)
    % This is a sufficient control-hull diagnostic, not the final physical test.
    motion = struct("Passed", all(segmentTime_s >= requiredTime_s), ...
        "SegmentTime_s", segmentTime_s, ...
        "MaximumViolation", max([0; requiredTime_s - segmentTime_s]));
end
