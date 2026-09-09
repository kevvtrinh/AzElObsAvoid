function [result,diagnostics] = solveStaticCorridor(request,warmStart,diagnostics,target_units,reserve_units)
%% Section 0: Header & Readme
% SYNTAX: [result,diagnostics] = bmtpEngine.solveStaticCorridor(request,warm,diagnostics,target,reserve)
% PURPOSE: Optimize a monotone static motion against exact source envelopes.
%   Integrate a quadratic Bernstein jerk on each phase for the free axis.
%   The limiting axis follows its analytic profile with a common time scale.
% INPUTS: Validated static earliest-arrival request, clock guide, diagnostics,
%   required separation target, and numerical reserve.
% OUTPUTS: Quintic motion proposal. The caller must prepare and independently
%   validate every output against the complete source geometry.
% UNITS: Coordinate units and seconds; powers use normalized local time.

%% Section 1: Refine The Analytic Clock Near Guide Turns
assert(~isfield(request.Coverage,'ActiveTimeInterval_s'), ...
    'bmtpEngine:InvalidCorridorRequest','The monotone corridor requires static geometry.');
% Keep solver feasibility residuals inside an additional numerical reserve.
% The public clearance and validation tolerance remain unchanged.
reserve_units=reserve_units+10*request.Options.ConstraintTolerance;
% Resolve guide turns on the analytic clock before optimizing the free axis.
% These knots add motion degrees of freedom; they do not alter source geometry.
originalTimes_s=warmStart.SegmentTime_s; originalPowers_units=warmStart.FixedPower_units;
breaks_s=[0;cumsum(originalTimes_s)];
[~,axisIndex]=max(warmStart.AxisMinimumTime_s);
guide_units=warmStart.ClockGuide.Route_units;
guideEvents_units=guide_units(2:end-1,axisIndex);
direction=sign(guide_units(end,axisIndex)-guide_units(1,axisIndex));
lowerTime_s=zeros(size(guideEvents_units)); upperTime_s=repmat(breaks_s(end),size(guideEvents_units));
axisPowers_units=reshape(originalPowers_units(:,axisIndex,:),numel(originalTimes_s),6);
for iteration=1:48
    middleTime_s=(lowerTime_s+upperTime_s)/2;
    span=max(1,min(numel(originalTimes_s),sum(middleTime_s>=breaks_s(1:end-1).',2)));
    tau=(middleTime_s-breaks_s(span))./originalTimes_s(span);
    coordinate_units=sum(axisPowers_units(span,:).*tau.^(0:5),2);
    before=direction*coordinate_units<direction*guideEvents_units;
    lowerTime_s(before)=middleTime_s(before); upperTime_s(~before)=middleTime_s(~before);
end
response_s=max(request.Limits.maxVelocity_units_s./request.Limits.maxAcceleration_units_s2 + ...
    request.Limits.maxAcceleration_units_s2./request.Limits.maxJerk_units_s3);
extraTimes_s=sort(reshape((lowerTime_s+upperTime_s)/2+[-response_s,0,response_s],[],1));
minimumGap_s=min(0.5,response_s/4);
refinedBreaks_s=breaks_s;
% Bound refinement cost for detailed coastlines. Every original exclusion
% facet still participates in the corridor and independent certificate.
if numel(extraTimes_s)>24
    extraTimes_s=extraTimes_s(unique(round(linspace(1,numel(extraTimes_s),24))));
end
for eventTime_s=extraTimes_s.'
    if eventTime_s>0 && eventTime_s<breaks_s(end) && min(abs(refinedBreaks_s-eventTime_s))>=minimumGap_s
        refinedBreaks_s(end+1,1)=eventTime_s; %#ok<AGROW>
    end
end
refinedBreaks_s=sort(refinedBreaks_s);
refinedTimes_s=diff(refinedBreaks_s); refinedPowers_units=zeros(numel(refinedTimes_s),2,6);
for part=1:numel(refinedTimes_s)
    span=find(breaks_s<=refinedBreaks_s(part)+64*eps(breaks_s(end)),1,'last');
    span=min(span,numel(originalTimes_s));
    startTau=(refinedBreaks_s(part)-breaks_s(span))/originalTimes_s(span);
    tauWidth=refinedTimes_s(part)/originalTimes_s(span);
    for k=0:5
        for j=k:5
            refinedPowers_units(part,:,k+1)=refinedPowers_units(part,:,k+1)+ ...
                nchoosek(j,k)*originalPowers_units(span,:,j+1)*startTau^(j-k)*tauWidth^k;
        end
    end
end
warmStart.SegmentCount=numel(refinedTimes_s);
warmStart.SegmentTime_s=refinedTimes_s;
warmStart.FixedPower_units=refinedPowers_units;

%% Section 2: Integrate The Free Coordinate Through Shared Physical States
degree = 5; segmentCount = warmStart.SegmentCount;
diagnostics.OptimizerSpanCount=segmentCount;
jerkCount = segmentCount*(degree-2);
quadratureCount = request.Degree;
variableCount = jerkCount+4+segmentCount*quadratureCount;
powerIndex = jerkCount+(1:4);
lengthIndex = jerkCount+4+(1:segmentCount*quadratureCount);
result = struct('Success',false,'ControlPoint_units',zeros(0,request.Degree+1,2), ...
    'PositionPower_units',[],'SegmentTime_s',NaN,'SolverMessage',"No certified corridor proposal was found.");
diagnostics.ConicSolver = bmtpEngine.accumulateConicDiagnostics();
diagnostics.TrajectorySocpCount = 0;
corridor = bmtpEngine.createStaticCorridor(request,warmStart);
if ~corridor.Available, return; end
axisIndex = corridor.AxisIndex; freeAxis = 3-axisIndex;
origin_units = request.InitialState.position_units;
referenceTimes_s = warmStart.SegmentTime_s;
breaks_s = [0;cumsum(referenceTimes_s)];
basis = cell(4,segmentCount); terminal = zeros(3,jerkCount);
for segment = 1:segmentCount
    jerk = zeros(degree-2,jerkCount);
    jerk(:,(segment-1)*(degree-2)+(1:degree-2)) = eye(degree-2);
    basis{4,segment} = jerk;
    for order = 2:-1:0
        count = degree-order+1;
        basis{order+1,segment} = terminal(order+1,:)+referenceTimes_s(segment)/(count-1)* ...
            tril(ones(count,count-1),-1)*basis{order+2,segment};
    end
    for order = 0:2, terminal(order+1,:) = basis{order+1,segment}(end,:); end
end
conversion = zeros(degree+1,degree+1);
for k = 0:degree
    for j = 0:k, conversion(k+1,j+1) = nchoosek(k,j)/nchoosek(degree,j); end
end
axisPower_units = reshape(warmStart.FixedPower_units(:,axisIndex,1:6),segmentCount,6);
axisPower_units(:,1) = axisPower_units(:,1)-origin_units(axisIndex);
axisControls_units = axisPower_units*conversion.';

%% Section 3: Pull Every Facet Interval Back To The Analytic Clock
events_units = unique(corridor.Intervals_units(:,1:2));
direction = sign(request.GoalState.position_units(axisIndex)-origin_units(axisIndex));
lower_s = zeros(size(events_units)); upper_s = repmat(breaks_s(end),size(events_units));
for iteration = 1:48
    middle_s = (lower_s+upper_s)/2;
    segment = max(1,min(segmentCount,sum(middle_s>=breaks_s(1:end-1).',2)));
    tau = (middle_s-breaks_s(segment))./referenceTimes_s(segment);
    values_units = sum(axisPower_units(segment,:).*tau.^(0:degree),2);
    left = direction*values_units<direction*events_units;
    lower_s(left) = middle_s(left); upper_s(~left) = middle_s(~left);
end
eventTimes_s = (lower_s+upper_s)/2;
[~,eventIndex] = ismember(corridor.Intervals_units(:,1:2),events_units);
intervalTimes_s = sort(eventTimes_s(eventIndex),2);
rows = cell(0,1); bounds = cell(0,1);
domain_units = [request.Limits.xInterval_units;request.Limits.yInterval_units]-origin_units.';
limitValues = [request.Limits.maxVelocity_units_s;request.Limits.maxAcceleration_units_s2;request.Limits.maxJerk_units_s3];
for segment = 1:segmentCount
    for order = 0:3
        localBasis = basis{order+1,segment};
        localRows = sparse(2*size(localBasis,1),variableCount);
        localRows(:,1:jerkCount) = [localBasis;-localBasis];
        if order==0
            localBounds = [repmat(domain_units(freeAxis,2),size(localBasis,1),1); ...
                repmat(-domain_units(freeAxis,1),size(localBasis,1),1)];
        else
            localRows(:,powerIndex(order+1)) = -limitValues(order,freeAxis);
            localBounds = zeros(size(localRows,1),1);
        end
        rows{end+1} = localRows; bounds{end+1} = localBounds; %#ok<AGROW>
    end
    active = find(intervalTimes_s(:,1)<breaks_s(segment+1) & intervalTimes_s(:,2)>breaks_s(segment));
    for piece = active.'
        localInterval = (max(breaks_s(segment),min(breaks_s(segment+1),intervalTimes_s(piece,:)))- ...
            breaks_s(segment))/referenceTimes_s(segment);
        restricted = bmtpEngine.restrictBezier([basis{1,segment},axisControls_units(segment,:).'],localInterval);
        facet = corridor.Intervals_units(piece,3); side = corridor.Intervals_units(piece,4);
        slope = corridor.Slopes(facet); intercept_units = corridor.Intercepts_units(facet);
        localRows = sparse(degree+1,variableCount);
        localRows(:,1:jerkCount) = -side*restricted(:,1:jerkCount);
        localBounds = -side*(slope*restricted(:,end)+intercept_units)-(target_units+reserve_units)*hypot(1,slope);
        rows{end+1} = localRows; bounds{end+1} = localBounds; %#ok<AGROW>
    end
end
A = vertcat(rows{:}); b = vertcat(bounds{:});
Aeq = sparse(4,variableCount); Aeq(1:3,1:jerkCount) = terminal; Aeq(4,powerIndex(1)) = 1;
beq = [request.GoalState.position_units(freeAxis)-origin_units(freeAxis);0;0;1];
for segment=1:segmentCount-1
    Aeq(end+1,1:jerkCount)=basis{4,segment}(end,:)-basis{4,segment+1}(1,:);
    beq(end+1,1)=0;
end
lower = -Inf(variableCount,1); upper = Inf(variableCount,1);
lower(powerIndex) = 1;
upper(powerIndex) = (request.MotionHorizon_s/breaks_s(end)).^(0:3);
lower(lengthIndex) = 0;

%% Section 4: Minimize Length At The Bound, Or Solve The Required Dilation
emptyCone = secondordercone(sparse(2,variableCount),zeros(2,1),sparse(variableCount,1),0);
timeCones = repmat(emptyCone,2,1);
for k = 1:2
    coneA = sparse(2,variableCount);
    coneA(1,powerIndex(k+1)) = 2; coneA(2,powerIndex(k)) = 1; coneA(2,powerIndex(k+2)) = -1;
    coneD = sparse(variableCount,1); coneD(powerIndex([k,k+2])) = 1;
    timeCones(k) = secondordercone(coneA,zeros(2,1),coneD,0);
end
% Positive Gauss-Legendre weights give a convex quadrature of actual speed.
% This avoids minimizing the larger degree-five control-polygon perimeter.
indices = (1:quadratureCount-1).';
offDiagonal = indices./sqrt(4*indices.^2-1);
[eigenvectors,eigenvalues] = eig(diag(offDiagonal,1)+diag(offDiagonal,-1));
nodes = (diag(eigenvalues)+1)/2;
weights = eigenvectors(1,:).'.^2;
velocityBasis = zeros(quadratureCount,degree);
for k = 0:degree-1
    velocityBasis(:,k+1) = nchoosek(degree-1,k)*nodes.^k.*(1-nodes).^(degree-1-k);
end
lengthCones = repmat(emptyCone,segmentCount*quadratureCount,1);
for segment = 1:segmentCount
    for k = 1:quadratureCount
        coneA = sparse(2,variableCount);
        coneA(freeAxis,1:jerkCount) = referenceTimes_s(segment)*velocityBasis(k,:)*basis{2,segment};
        coneB = zeros(2,1); coneB(axisIndex) = -degree*velocityBasis(k,:)*diff(axisControls_units(segment,:)).';
        coneD = sparse(variableCount,1); coneD(lengthIndex((segment-1)*quadratureCount+k)) = 1;
        lengthCones((segment-1)*quadratureCount+k) = secondordercone(coneA,coneB,coneD,0);
    end
end
objective = zeros(variableCount,1); objective(lengthIndex) = repmat(weights,segmentCount,1);
fixedUpper = upper; fixedUpper(powerIndex) = 1;
timer = tic;
[x,~,exitFlag,output] = coneprog(objective,lengthCones,A,b,Aeq,beq,lower,fixedUpper,request.TrajectoryOptions);
output.TotalTime_s = toc(timer);
diagnostics.ConicSolver = bmtpEngine.accumulateConicDiagnostics(diagnostics.ConicSolver,output);
if ~linearFeasible(x,A,b,Aeq,beq,lower,fixedUpper,request.Options.ConstraintTolerance)
    objective(:) = 0; objective(powerIndex(4)) = 1; timer = tic;
    [x,~,exitFlag,output] = coneprog(objective,timeCones,A,b,Aeq,beq,lower,upper,request.TrajectoryOptions);
    output.TotalTime_s = toc(timer);
    diagnostics.ConicSolver = bmtpEngine.accumulateConicDiagnostics(diagnostics.ConicSolver,output);
    if linearFeasible(x,A,b,Aeq,beq,lower,upper,request.Options.ConstraintTolerance)
        upper(powerIndex(4)) = x(powerIndex(4)); objective(:) = 0; objective(lengthIndex) = repmat(weights,segmentCount,1); timer = tic;
        [shortX,~,shortFlag,shortOutput] = coneprog(objective,[timeCones;lengthCones],A,b,Aeq,beq,lower,upper,request.TrajectoryOptions);
        shortOutput.TotalTime_s = toc(timer);
        diagnostics.ConicSolver = bmtpEngine.accumulateConicDiagnostics(diagnostics.ConicSolver,shortOutput);
        if linearFeasible(shortX,A,b,Aeq,beq,lower,upper,request.Options.ConstraintTolerance)
            x = shortX; exitFlag = shortFlag; output = shortOutput;
        end
    end
end
% Refine actual speed-length from a feasible clock. The conic time solution
% can be feasible without satisfying its path-length optimality conditions.
refinement=struct('Attempted',false,'Accepted',false,'Allowance_s',0, ...
    'ArrivalCost_s',0,'InitialLength_units',NaN,'FinalLength_units',NaN, ...
    'ElapsedTime_s',0,'ExitFlags',zeros(0,1),'Iterations',zeros(0,1));
if linearFeasible(x,A,b,Aeq,beq,lower,upper,request.Options.ConstraintTolerance)
    refinement.Attempted=true;
    refinementTimer=tic;
    allowance_s=0;
    if isfield(request.Options,'PathLengthTimeAllowance_s')
        allowance_s=request.Options.PathLengthTimeAllowance_s;
    end
    refinement.Allowance_s=allowance_s;
    earliestDuration_s=x(powerIndex(4))^(1/3)*breaks_s(end);
    speedBasis=zeros(segmentCount*quadratureCount,jerkCount);
    fixedSpeed=zeros(segmentCount*quadratureCount,1);
    for segment=1:segmentCount
        indices=(segment-1)*quadratureCount+(1:quadratureCount);
        speedBasis(indices,:)=referenceTimes_s(segment)*velocityBasis*basis{2,segment};
        fixedSpeed(indices)=degree*velocityBasis*diff(axisControls_units(segment,:)).';
    end
    objective=@(z) lengthObjective(z,speedBasis,fixedSpeed,repmat(weights,segmentCount,1));
    settings=optimoptions('fmincon','Algorithm','sqp','Display','none', ...
        'SpecifyObjectiveGradient',true,'ConstraintTolerance',1e-10, ...
        'OptimalityTolerance',1e-9,'StepTolerance',1e-12,'MaxIterations',200, ...
        'MaxFunctionEvaluations',500);
    validRows=any(Aeq(:,1:jerkCount)~=0,2);
    initialLength_units=curveLength(x(1:jerkCount),basis,axisControls_units,referenceTimes_s);
    bestLength_units=initialLength_units;
    refinement.InitialLength_units=initialLength_units;
    % First shorten at the earliest feasible clock. Spend additional arrival
    % time only for at least a one-percent further length reduction.
    durations_s=unique([earliestDuration_s,min(request.MotionHorizon_s,earliestDuration_s+allowance_s)]);
    for duration_s=durations_s
        scale=duration_s/breaks_s(end);
        fixedPower=scale.^(0:3).';
        [z,~,polishFlag,polishOutput]=fmincon(objective,x(1:jerkCount),full(A(:,1:jerkCount)), ...
            b-A(:,powerIndex)*fixedPower,full(Aeq(validRows,1:jerkCount)),beq(validRows), ...
            [],[],[],settings);
        refinement.ExitFlags(end+1,1)=polishFlag;
        refinement.Iterations(end+1,1)=polishOutput.iterations;
        trial=x; trial(1:jerkCount)=z; trial(powerIndex)=fixedPower;
        trial(lengthIndex)=hypot(speedBasis*z,fixedSpeed);
        trialUpper=upper; trialUpper(powerIndex(4))=fixedPower(4);
        if ~linearFeasible(trial,A,b,Aeq,beq,lower,trialUpper,request.Options.ConstraintTolerance), continue; end
        length_units=curveLength(z,basis,axisControls_units,referenceTimes_s);
        spendingTime=duration_s>earliestDuration_s+request.Options.ConstraintTolerance;
        requiredGain_units=max(1e-8,1e-8*bestLength_units);
        if spendingTime, requiredGain_units=0.01*bestLength_units; end
        if length_units<bestLength_units-requiredGain_units
            x=trial; upper=trialUpper; bestLength_units=length_units;
            refinement.Accepted=true;
        end
    end
    refinement.ArrivalCost_s=x(powerIndex(4))^(1/3)*breaks_s(end)-earliestDuration_s;
    refinement.FinalLength_units=bestLength_units;
    refinement.ElapsedTime_s=toc(refinementTimer);
end
diagnostics.PathLengthRefinement=refinement;
diagnostics.TrajectorySocpCount = diagnostics.ConicSolver.CallCount;
diagnostics.FinalTrajectoryExitFlag = exitFlag;
diagnostics.Converged = exitFlag>0;
diagnostics.IterationCount = 1;
diagnostics.SolverMessage = string(output.message);
diagnostics.Identifier = "monotoneStaticCorridor";
diagnostics.ConstraintRepresentation = "integratedQuinticCorridor";
if ~linearFeasible(x,A,b,Aeq,beq,lower,upper,request.Options.ConstraintTolerance), return; end

%% Section 5: Preserve Integrated Continuity And Export Exact Powers
% Correct solver endpoint roundoff globally in the integrated jerk variables.
% The complete final polynomial and all original obstacle pairs are checked.
continuityRows=[1:3,5:size(Aeq,1)];
continuity=Aeq(continuityRows,1:jerkCount); target=beq(continuityRows);
x(1:jerkCount)=x(1:jerkCount)+continuity.'*((continuity*continuity.')\(target-continuity*x(1:jerkCount)));
powers_units = zeros(segmentCount,2,request.Degree+1);
controls_units = zeros(segmentCount,request.Degree+1,2);
export = zeros(request.Degree+1,degree+1);
for k = 0:request.Degree
    for j = 0:min(k,degree), export(k+1,j+1) = nchoosek(k,j)/nchoosek(request.Degree,j); end
end
for segment = 1:segmentCount
    powers_units(segment,axisIndex,1:6) = axisPower_units(segment,:);
    powers_units(segment,axisIndex,1) = powers_units(segment,axisIndex,1)+origin_units(axisIndex);
    powers_units(segment,freeAxis,1) = basis{1,segment}(1,:)*x(1:jerkCount)+origin_units(freeAxis);
    powers_units(segment,freeAxis,2) = basis{2,segment}(1,:)*x(1:jerkCount)*referenceTimes_s(segment);
    powers_units(segment,freeAxis,3) = basis{3,segment}(1,:)*x(1:jerkCount)*referenceTimes_s(segment)^2/2;
    jerkControls = basis{4,segment}*x(1:jerkCount);
    jerkPowers = [jerkControls(1);2*diff(jerkControls(1:2));diff(jerkControls,2)];
    for k = 0:degree-3
        powers_units(segment,freeAxis,k+4) = jerkPowers(k+1)*referenceTimes_s(segment)^3/((k+1)*(k+2)*(k+3));
    end
    controls_units(segment,:,:) = export*squeeze(powers_units(segment,:,1:degree+1)).';
end
result.Success = true;
result.ControlPoint_units = controls_units;
result.PositionPower_units = powers_units;
result.SegmentTime_s = x(powerIndex(4))^(1/3)*referenceTimes_s;
result.CertificateEventTime_s=x(powerIndex(4))^(1/3)*eventTimes_s;
result.SolverMessage = "The exact monotone corridor returned an integrated quintic proposal.";
end

%% Section 6: Local Functions
function feasible = linearFeasible(x,A,b,Aeq,beq,lower,upper,tolerance)
    feasible = ~isempty(x) && all(isfinite(x)) && ...
        max([A*x-b;abs(Aeq*x-beq);lower-x;x-upper])<=tolerance;
end

function [value,gradient]=lengthObjective(z,basis,fixed,weights)
    free=basis*z;
    speed=hypot(free,fixed);
    value=weights.'*speed;
    ratio=zeros(size(free)); moving=speed>0;
    ratio(moving)=free(moving)./speed(moving);
    gradient=basis.'*(weights.*ratio);
end

function length_units=curveLength(jerk,basis,axisControls_units,times_s)
    % Measure the continuous curve for selection, independent of plot sampling
    % and of the quadrature used by the convex optimization objective.
    degree=4;
    conversion=zeros(degree+1);
    for k=0:degree
        for j=0:k
            conversion(k+1,j+1)=nchoosek(degree,k)*nchoosek(k,j)*(-1)^(k-j);
        end
    end
    length_units=0;
    for segment=1:numel(times_s)
        freePower=conversion*(times_s(segment)*basis{2,segment}*jerk);
        fixedPower=conversion*(5*diff(axisControls_units(segment,:)).');
        speed=@(tau) hypot(polyval(flipud(freePower),tau),polyval(flipud(fixedPower),tau));
        length_units=length_units+integral(speed,0,1,'AbsTol',1e-10,'RelTol',1e-10);
    end
end
