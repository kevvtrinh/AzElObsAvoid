function [A,Aeq,beq,lb,ub,jerkMap] = createTrajectoryConstraints(segmentCount,degree,boundaryControls,limits,variableCount,activePlaneCount,segmentRatio,physicalTimes_s)
%% Section 0: Header & Readme
% SYNTAX: [A,Aeq,beq,lb,ub,jerkMap] =
%   bmtpEngine.createTrajectoryConstraints(S,D,endpoints,limits,N,R,ratios,times)
% PURPOSE: Assemble shared workspace, endpoint, C3 and derivative constraints.
% INPUTS: Span count/degree, imposed endpoint controls, limits, decision size, active-plane count
%   and positive span ratios; optional physical times request the quadratic-jerk map used by the
%   fixed-clock objective.
% OUTPUTS: Sparse inequality/equality rows and bounds, leaving plane rows empty.
% UNITS: Coordinate units, seconds and physical derivatives.

%% Section 1: Allocate Bounds And Prescribed Endpoint Rows
segmentRatio = segmentRatio(:);
controlCount = segmentCount*(degree+1)*2;
powerIndex = controlCount+(1:4);
baseInequalityCount = 4*segmentCount*(3*degree-3);
inequalityCount = baseInequalityCount+activePlaneCount*(degree+2);
equalityCount = 13+8*(segmentCount-1);
A = spalloc(inequalityCount,variableCount,6*inequalityCount);
Aeq = spalloc(equalityCount,variableCount,8*equalityCount);
beq = zeros(equalityCount,1);
lb = -Inf(variableCount,1); ub = Inf(variableCount,1);
domain_units = [limits.xInterval_units;limits.yInterval_units];
lb(1:controlCount) = repmat(domain_units(:,1),segmentCount*(degree+1),1);
ub(1:controlCount) = repmat(domain_units(:,2),segmentCount*(degree+1),1);
lb(powerIndex) = 0; lb(powerIndex(2)) = eps;
equalityIndex = 0;
for axisIndex = 1:2
    firstColumn = axisIndex;
    lastColumn = controlCount-2+axisIndex;
    equalityIndex = equalityIndex+1;
    Aeq(equalityIndex,firstColumn) = 1;
    beq(equalityIndex) = boundaryControls(1,1,axisIndex);
    equalityIndex = equalityIndex+1;
    Aeq(equalityIndex,lastColumn) = 1;
    beq(equalityIndex) = boundaryControls(end,end,axisIndex);
    for endpointOrder = 1:2
        equalityIndex = equalityIndex+1;
        Aeq(equalityIndex,[firstColumn+2*endpointOrder,firstColumn]) = [1,-1];
        beq(equalityIndex) = boundaryControls(1,endpointOrder+1,axisIndex)-boundaryControls(1,1,axisIndex);
        equalityIndex = equalityIndex+1;
        Aeq(equalityIndex,[lastColumn-2*endpointOrder,lastColumn]) = [1,-1];
        beq(equalityIndex) = boundaryControls(end,degree-endpointOrder+1,axisIndex)-boundaryControls(end,end,axisIndex);
    end
end

%% Section 2: Preserve Span/Derivative/Axis Continuity Row Order
differences = {1,[-1,1],[1,-2,1],[-1,3,-3,1]};
joins = (1:segmentCount-1).';
% Multiply before dividing, as in the scalar equations, to preserve roundoff.
for order = 0:3
    if segmentCount==1, break; end
    coefficients = differences{order+1};
    rowScale = max(segmentRatio(1:end-1),segmentRatio(2:end)).^order;
    leftValues = coefficients.*segmentRatio(2:end).^order./rowScale;
    rightValues = -coefficients.*segmentRatio(1:end-1).^order./rowScale;
    leftColumns = (joins-1)*(degree+1)+degree-order+(1:order+1);
    rightColumns = joins*(degree+1)+(1:order+1);
    rowIndices = repmat((joins-1)*4+order+1,1,order+1);
    rows = sparse([rowIndices(:);rowIndices(:)],[leftColumns(:);rightColumns(:)], ...
        [leftValues(:);rightValues(:)],4*(segmentCount-1),segmentCount*(degree+1));
    Aeq(13:end-1,1:controlCount) = Aeq(13:end-1,1:controlCount)+kron(rows,speye(2));
end
Aeq(end,powerIndex(1)) = 1; beq(end) = 1;

%% Section 3: Batch The Same Signed Derivative Rows Across Spans
limitValues = [limits.maxVelocity_units_s;limits.maxAcceleration_units_s2;limits.maxJerk_units_s3];
jerkMap = sparse(6*segmentCount,variableCount);
for order = 1:3
    derivativeCount = degree-order+1;
    scale = factorial(degree)/factorial(degree-order);
    derivativeRows = spdiags(repmat(scale*differences{order+1},derivativeCount,1),0:order,derivativeCount,degree+1);
    signedRows = kron(kron(derivativeRows,speye(2)),[1;-1]);
    precedingRows = 4*((order-1)*(degree+1)-(order-1)*order/2);
    targets = reshape(precedingRows+(1:size(signedRows,1)).'+ ...
        (0:segmentCount-1)*(baseInequalityCount/segmentCount),[],1);
    A(targets,1:controlCount) = kron(speye(segmentCount),signedRows);
    axisLimits = repmat(limitValues(order,:),derivativeCount,1);
    values = -repelem(reshape(axisLimits.',[],1),2)*segmentRatio.'.^order;
    A(targets,powerIndex(order+1)) = values(:);
    if ~isempty(physicalTimes_s) && order==3
        jerkMap(:,1:controlCount) = kron(spdiags(1./physicalTimes_s(:).^3,0,segmentCount,segmentCount), ...
            kron(derivativeRows,speye(2)));
    end
end
end
