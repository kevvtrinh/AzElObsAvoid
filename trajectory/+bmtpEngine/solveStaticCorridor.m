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

%% Section 1: Integrate The Free Coordinate Through Shared Physical States
assert(~isfield(request.Coverage,'ActiveTimeInterval_s'), ...
    'bmtpEngine:InvalidCorridorRequest','The monotone corridor requires static geometry.');
% Keep solver feasibility residuals inside an additional numerical reserve.
% The public clearance and validation tolerance remain unchanged.
reserve_units=reserve_units+10*request.Options.ConstraintTolerance;
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

%% Section 2: Pull Every Facet Interval Back To The Analytic Clock
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

%% Section 3: Minimize Length At The Bound, Or Solve The Required Dilation
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
diagnostics.TrajectorySocpCount = diagnostics.ConicSolver.CallCount;
diagnostics.FinalTrajectoryExitFlag = exitFlag;
diagnostics.Converged = exitFlag>0;
diagnostics.IterationCount = 1;
diagnostics.SolverMessage = string(output.message);
diagnostics.Identifier = "monotoneStaticCorridor";
diagnostics.ConstraintRepresentation = "integratedQuinticCorridor";
if ~linearFeasible(x,A,b,Aeq,beq,lower,upper,request.Options.ConstraintTolerance), return; end

%% Section 4: Preserve Integrated Continuity And Export Exact Powers
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

function feasible = linearFeasible(x,A,b,Aeq,beq,lower,upper,tolerance)
    feasible = ~isempty(x) && all(isfinite(x)) && ...
        max([A*x-b;abs(Aeq*x-beq);lower-x;x-upper])<=tolerance;
end
