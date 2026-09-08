function [result,diagnostics] = solveStaticCorridor(request,warmStart,diagnostics,target_units,reserve_units)
%% Section 0: Header & Readme
% SYNTAX: [result,diagnostics] = bmtpEngine.solveStaticCorridor(request,warm,diagnostics,target,reserve)
% PURPOSE: Optimize a monotone static motion against exact source envelopes.
%   Integrate a quadratic Bernstein jerk on each phase for the free axis.
%   The limiting axis follows its prescribed analytic profile.
% INPUTS: Validated static earliest-arrival request, clock guide, diagnostics,
%   required separation target, and numerical reserve.
% OUTPUTS: Quintic motion proposal. The caller must prepare and independently
%   validate every output against the complete source geometry.
% UNITS: Coordinate units and seconds; powers use normalized local time.

%% Section 1: Integrate The Free Coordinate Through Shared Physical States
assert(~isfield(request.Coverage,'ActiveTimeInterval_s'), ...
    'bmtpEngine:InvalidCorridorRequest','The monotone corridor requires static geometry.');
degree = 5; segmentCount = warmStart.SegmentCount;
jerkCount = segmentCount*(degree-2);
quadratureCount = request.Degree;
variableCount = jerkCount+segmentCount*quadratureCount;
lengthIndex = jerkCount+(1:segmentCount*quadratureCount);
result = struct('Success',false,'ControlPoint_units',zeros(0,request.Degree+1,2), ...
    'PositionPower_units',[],'SegmentTime_s',NaN,'SolverMessage',"No certified corridor proposal was found.");
diagnostics.ConicSolver = bmtpEngine.accumulateConicDiagnostics();
diagnostics.TrajectorySocpCount = 0;
corridor = bmtpEngine.createStaticCorridor(request,warmStart);
diagnostics.TerminationReason = corridor.TerminationReason;
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
conversion = zeros(degree+1,4);
for k = 0:degree
    for j = 0:min(k,3), conversion(k+1,j+1) = nchoosek(k,j)/nchoosek(degree,j); end
end
axisPower_units = reshape(warmStart.FixedPower_units(:,axisIndex,1:4),segmentCount,4);
axisPower_units(:,1) = axisPower_units(:,1)-origin_units(axisIndex);
axisControls_units = axisPower_units*conversion.';

%% Section 2: Pull Every Facet Interval Back To The Analytic Clock
events_units = unique(reshape(corridor.Intervals_units(:,1:2),[],1));
direction = sign(request.GoalState.position_units(axisIndex)-origin_units(axisIndex));
lower_s = zeros(size(events_units)); upper_s = repmat(breaks_s(end),size(events_units));
for iteration = 1:48
    middle_s = (lower_s+upper_s)/2;
    segment = max(1,min(segmentCount,sum(middle_s>=breaks_s(1:end-1).',2)));
    tau = (middle_s-breaks_s(segment))./referenceTimes_s(segment);
    values_units = axisPower_units(segment,1)+tau.*(axisPower_units(segment,2)+ ...
        tau.*(axisPower_units(segment,3)+tau.*axisPower_units(segment,4)));
    left = direction*values_units<direction*events_units;
    lower_s(left) = middle_s(left); upper_s(~left) = middle_s(~left);
end
eventTimes_s = (lower_s+upper_s)/2;
[~,eventIndex] = ismember(corridor.Intervals_units(:,1:2),events_units);
intervalTimes_s = sort(reshape(eventTimes_s(eventIndex),size(eventIndex)),2);
% Each rest endpoint bounds displacement by the integrals of velocity,
% acceleration, and jerk limits. Their intersection is a necessary bound,
% not a feasibility certificate. Test facet endpoints and midpoints before
% assembling an optimization that cannot attain this prescribed clock.
checkTimes_s = [intervalTimes_s(:, 1); mean(intervalTimes_s, 2); intervalTimes_s(:, 2)];
remainingTimes_s = breaks_s(end) - checkTimes_s;
forwardReach_units = min([request.Limits.maxVelocity_units_s(freeAxis) * checkTimes_s, 0.5 * request.Limits.maxAcceleration_units_s2(freeAxis) * checkTimes_s .^ 2, request.Limits.maxJerk_units_s3(freeAxis) * checkTimes_s .^ 3 / 6], [], 2);
backwardReach_units = min([request.Limits.maxVelocity_units_s(freeAxis) * remainingTimes_s, 0.5 * request.Limits.maxAcceleration_units_s2(freeAxis) * remainingTimes_s .^ 2, request.Limits.maxJerk_units_s3(freeAxis) * remainingTimes_s .^ 3 / 6], [], 2);
goalCoordinate_units = request.GoalState.position_units(freeAxis) - origin_units(freeAxis);
lowerReach_units = max(-forwardReach_units, goalCoordinate_units - backwardReach_units);
upperReach_units = min(forwardReach_units, goalCoordinate_units + backwardReach_units);
segment = max(1, min(segmentCount, sum(checkTimes_s >= breaks_s(1:end - 1).', 2)));
tau = (checkTimes_s - breaks_s(segment)) ./ referenceTimes_s(segment);
clockCoordinate_units = axisPower_units(segment, 1) + tau .* (axisPower_units(segment, 2) + tau .* (axisPower_units(segment, 3) + tau .* axisPower_units(segment, 4)));
facets = repmat(corridor.Intervals_units(:, 3), 3, 1);
sides = repmat(corridor.Intervals_units(:, 4), 3, 1);
requiredCoordinate_units = sides .* (corridor.Slopes(facets) .* clockCoordinate_units + corridor.Intercepts_units(facets)) + (target_units + reserve_units) * hypot(1, corridor.Slopes(facets));
availableCoordinate_units = upperReach_units;
availableCoordinate_units(sides < 0) = -lowerReach_units(sides < 0);
if any(requiredCoordinate_units > availableCoordinate_units + request.Options.ConstraintTolerance)
    diagnostics.TerminationReason = "freeAxisReachabilityBound";
    return;
end
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
            localBounds = repmat(limitValues(order,freeAxis),size(localRows,1),1);
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
Aeq = sparse(3,variableCount); Aeq(:,1:jerkCount) = terminal;
beq = [request.GoalState.position_units(freeAxis)-origin_units(freeAxis);0;0];
lower = -Inf(variableCount,1); upper = Inf(variableCount,1);
lower(lengthIndex) = 0;

% At the fixed clock the speed cones only bound unbounded length epigraphs.
% Feasibility therefore depends entirely on the linear jerk constraints.
% Omit the unused length columns for a cheap LP check.
feasibilityOptions = optimoptions('linprog', 'Display', 'none', 'MaxIterations', request.TrajectoryOptions.MaxIterations);
feasibilityTimer = tic;
[feasibleJerk, ~, feasibilityFlag, feasibilityOutput] = linprog(zeros(jerkCount, 1), A(:, 1:jerkCount), b, Aeq(:, 1:jerkCount), beq, [], [], feasibilityOptions);
diagnostics.LinearFeasibility = struct('ExitFlag', feasibilityFlag, 'ElapsedTime_s', toc(feasibilityTimer), 'Message', string(feasibilityOutput.message));
if isempty(feasibleJerk)
    diagnostics.TerminationReason = "linearClockCorridorUnresolved";
    return;
end

%% Section 3: Minimize Length At The Prescribed Physical Clock
emptyCone = secondordercone(sparse(2,variableCount),zeros(2,1),sparse(variableCount,1),0);
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
timer = tic;
[x,~,exitFlag,output] = coneprog(objective,lengthCones,A,b,Aeq,beq,lower,upper,request.TrajectoryOptions);
output.TotalTime_s = toc(timer);
diagnostics.ConicSolver = bmtpEngine.accumulateConicDiagnostics(diagnostics.ConicSolver,output);
diagnostics.TrajectorySocpCount = diagnostics.ConicSolver.CallCount;
if ~linearFeasible(x,A,b,Aeq,beq,lower,upper,request.Options.ConstraintTolerance)
    diagnostics.SolverMessage = "The physical-clock corridor is unresolved.";
    diagnostics.TerminationReason = "clockCorridorUnresolved";
    return;
end
diagnostics.FinalTrajectoryExitFlag = exitFlag;
diagnostics.Converged = exitFlag>0;
diagnostics.IterationCount = 1;
diagnostics.SolverMessage = string(output.message);
diagnostics.Identifier = "monotoneStaticCorridor";
diagnostics.ConstraintRepresentation = "integratedQuinticCorridor";

%% Section 4: Preserve Integrated Continuity And Export Exact Powers
% Correct solver endpoint roundoff globally in the integrated jerk variables.
% The complete final polynomial and all original obstacle pairs are checked.
x(1:jerkCount) = x(1:jerkCount)+terminal.'*((terminal*terminal.')\(beq(1:3)-terminal*x(1:jerkCount)));
powers_units = zeros(segmentCount,2,request.Degree+1);
controls_units = zeros(segmentCount,request.Degree+1,2);
export = zeros(request.Degree+1,degree+1);
for k = 0:request.Degree
    for j = 0:min(k,degree), export(k+1,j+1) = nchoosek(k,j)/nchoosek(request.Degree,j); end
end
for segment = 1:segmentCount
    powers_units(segment,axisIndex,1:4) = axisPower_units(segment,:);
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
result.SegmentTime_s = referenceTimes_s;
result.SolverMessage = "The exact monotone corridor returned an integrated quintic proposal.";
end

function feasible = linearFeasible(x,A,b,Aeq,beq,lower,upper,tolerance)
    feasible = ~isempty(x) && all(isfinite(x)) && ...
        max([A*x-b;abs(Aeq*x-beq);lower-x;x-upper])<=tolerance;
end
