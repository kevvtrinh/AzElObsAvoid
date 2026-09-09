function [result,diagnostics] = solveQuinticTrajectory(request,warmStart,diagnostics,target_units,reserve_units)
%% Section 0: Header & Readme
% SYNTAX: [motion,diagnostics] = bmtpEngine.solveQuinticTrajectory(request,warm,diagnostics,target,reserve)
% PURPOSE: Optimize a static detour using integrated quadratic-jerk phases.
%   Initialize the clock conically, vary local knot states, durations, and jerk,
%   then repair on the final clock with length and bounded jerk-variation cost.
% INPUTS: Validated earliest-arrival static request, visibility warm start,
%   diagnostic record, obstacle target, and numerical reserve.
% OUTPUTS: A polynomial proposal and full solver diagnostics. The caller must
%   prepare and independently validate every returned motion before success.
% UNITS: Coordinate units and seconds; jerk uses coordinate units/s^3.

%% Section 1: Initialize One Integrated Quintic Model
assert(request.Options.GoalTimeMode=="earliestArrival" && ~isfield(request.Coverage,'ActiveTimeInterval_s'), ...
    'bmtpEngine:InvalidQuinticRequest','The variable quintic clock requires static geometry and earliest arrival.');
% Keep solver feasibility residuals inside an additional numerical reserve.
% The public clearance and validation tolerance remain unchanged.
reserve_units=reserve_units+10*request.Options.ConstraintTolerance;
segmentCount=warmStart.SegmentCount;
degree=request.Degree;
diagnostics.OptimizerSpanCount=segmentCount;
regionCount=numel(request.Regions_units);
emptyPlane=struct('Active',false,'Verified',false,'ExitFlag',-2,'Normal',zeros(2,2), ...
    'Offset_units',zeros(1,2),'SignedGap_units',NaN);
planes=repmat(emptyPlane,segmentCount,regionCount);
for k=1:segmentCount
    for j=1:regionCount
        planes(k,j)=bmtpEngine.solveSeparatingLine(squeeze(warmStart.ControlPoint_units(k,:,:)),request.Regions_units{j},target_units,reserve_units);
    end
end
result=struct('Success',false,'SolverMessage',"Quintic clock initialization failed.", ...
    'ControlPoint_units',zeros(0,degree+1,2),'SegmentTime_s',NaN,'PositionPower_units',[]);
diagnostics.Identifier="quinticJerkClock";
diagnostics.ConstraintRepresentation="integratedQuinticVariableClock";
[controls_units,times_s,~,initial]=solveQuinticStep(request,planes,warmStart.SegmentRatio,reserve_units,false,request.MotionHorizon_s);
diagnostics.TrajectorySocpCount=initial.SolveCount;
diagnostics.ConicSolver=bmtpEngine.accumulateConicDiagnostics(bmtpEngine.accumulateConicDiagnostics(),initial);
if isempty(controls_units) || any(~isfinite(controls_units),'all'), return; end
conversion=zeros(degree+1,6);
for k=0:degree
    for j=0:min(k,5), conversion(k+1,j+1)=nchoosek(k,j)/nchoosek(degree,j); end
end
controls_units=permute(pagemtimes(conversion,permute(initial.PositionPower_units,[3,2,1])),[3,1,2]);
for k=1:segmentCount
    for j=1:regionCount
        planes(k,j)=bmtpEngine.solveSeparatingLine(squeeze(controls_units(k,:,:)),request.Regions_units{j},target_units,reserve_units);
    end
end

%% Section 2: Refine Phase Boundaries And Repair The Complete Motion
jerkPower_units_s3=initial.PositionPower_units(:,:,4:6).*reshape([6,24,60],1,1,3)./times_s.^3;
jerks_units_s3=permute(pagemtimes([1,0,0;1,0.5,0;1,1,1],permute(jerkPower_units_s3,[3,2,1])),[3,1,2]);
[times_s,nonlinearFlag,nonlinear]=refinePhaseTimes(request,times_s,jerks_units_s3,planes,reserve_units);
diagnostics.NonlinearSolver=nonlinear;
if any(~isfinite(times_s) | times_s<=0)
    result.SolverMessage="The phase-time optimizer did not return positive finite durations."; return;
end
% Repair on the optimized physical clock with a small interior time reserve
% for rest states. This avoids a degenerate length solve at the conic time bound.
repairDuration_s=sum(times_s);
if request.IsRest, repairDuration_s=min(request.MotionHorizon_s,repairDuration_s*(1+1e-5)); end
[controls_units,times_s,exitFlag,final]=solveQuinticStep(request,planes,times_s/mean(times_s),reserve_units,true,repairDuration_s);
diagnostics.TrajectorySocpCount=diagnostics.TrajectorySocpCount+final.SolveCount;
diagnostics.ConicSolver=bmtpEngine.accumulateConicDiagnostics(diagnostics.ConicSolver,final);
diagnostics.FinalTrajectoryExitFlag=exitFlag;
if isempty(controls_units) || any(~isfinite(controls_units),'all')
    result.SolverMessage="Quintic clock repair failed."; return;
end
controls_units=permute(pagemtimes(conversion,permute(final.PositionPower_units,[3,2,1])),[3,1,2]);

result.Success=true;
result.ControlPoint_units=controls_units;
result.SegmentTime_s=times_s;
result.PositionPower_units=final.PositionPower_units;
result.SolverMessage="An integrated quintic motion was optimized with variable phase times.";
diagnostics.SolverMessage=result.SolverMessage;
diagnostics.Converged=exitFlag>0 && nonlinearFlag>0;
diagnostics.IterationCount=1;
end

%% Section 3: Convex Integrated-Jerk Subproblem
function [controls_units,times_s,exitFlag,output] = solveQuinticStep(request,planes,ratios,reserve_units,minimizeLength,fixedDuration_s)
    segmentCount=size(planes,1); degree=5; limits=request.Limits;
    if minimizeLength, planes=bmtpEngine.removeRedundantPlanes(planes,limits,reserve_units); end
    start_units=request.InitialState.position_units; goal_units=request.GoalState.position_units;
    horizon_s=request.MotionHorizon_s; options=request.TrajectoryOptions;
    %% Section 1: Construct The Integrated Polynomial Basis
    origin_units=start_units;
    goal_units=goal_units-origin_units; start_units=[0,0];
    domain_units=[limits.xInterval_units-origin_units(1);limits.yInterval_units-origin_units(2)];
    jerkCount=segmentCount*(degree-2); stateCount=2*jerkCount;
    fixedPhysicalClock=~request.IsRest || minimizeLength;
    referenceTime_s=fixedDuration_s;
    if ~fixedPhysicalClock
        [~,profileTimes_s]=bmtpEngine.createJerkLimitedChord(start_units,goal_units,limits,degree);
        referenceTime_s=sum(profileTimes_s);
    end
    referenceTimes_s=referenceTime_s*ratios(:)/sum(ratios);
    powerIndex=stateCount+(1:4);
    lengthCount=segmentCount*degree;
    planeCount=nnz([planes.Active]);
    variableCount=stateCount+4+lengthCount+minimizeLength*segmentCount;
    lengthIndex=stateCount+4+(1:lengthCount);
    basis=cell(4,segmentCount);
    stateMaps=zeros(3,jerkCount);
    for segment=1:segmentCount
        jerk=zeros(degree-2,jerkCount); jerk(:,(segment-1)*(degree-2)+(1:degree-2))=eye(degree-2);
        basis{4,segment}=jerk;
        for order=2:-1:0
            count=degree-order+1;
            integrated=referenceTimes_s(segment)/(count-1)*tril(ones(count,count-1),-1)*basis{order+2,segment};
            integrated=integrated+stateMaps(order+1,:);
            basis{order+1,segment}=integrated;
        end
        for order=0:2, stateMaps(order+1,:)=basis{order+1,segment}(end,:); end
    end
    % Integrate the initial physical state on the same fixed clock as jerk.
    offsets=cell(4,segmentCount);
    stateOffset=[zeros(1,2);request.InitialState.velocity_units_s;request.InitialState.acceleration_units_s2];
    for segment=1:segmentCount
        offsets{4,segment}=zeros(degree-2,2);
        for order=2:-1:0
            count=degree-order+1;
            offsets{order+1,segment}=stateOffset(order+1,:)+ ...
                referenceTimes_s(segment)/(count-1)*tril(ones(count,count-1),-1)*offsets{order+2,segment};
        end
        for order=0:2, stateOffset(order+1,:)=offsets{order+1,segment}(end,:); end
    end
    lb=-Inf(variableCount,1); ub=Inf(variableCount,1);
    lb(powerIndex)=0; lb(powerIndex(2))=eps;
    lb(lengthIndex)=0;
    maximumClockScale=horizon_s/referenceTime_s;
    timePowers=[1;maximumClockScale;maximumClockScale^2;maximumClockScale^3];
    ub(powerIndex)=timePowers;
    if fixedPhysicalClock, lb(powerIndex)=1; ub(powerIndex)=1; end
    %% Section 2: Impose Physical Continuity And Bounds
    Aeq=spalloc(7+2*(segmentCount-1),variableCount,8*stateCount);
    beq=zeros(size(Aeq,1),1); row=0;
    for axis=1:2
        for order=0:2
            row=row+1; Aeq(row,axis:2:stateCount)=basis{order+1,segmentCount}(end,:);
            terminalState=[goal_units;request.GoalState.velocity_units_s;request.GoalState.acceleration_units_s2];
            beq(row)=terminalState(order+1,axis)-offsets{order+1,segmentCount}(end,axis);
        end
    end
    for segment=1:segmentCount-1
        for axis=1:2
            row=row+1;
            Aeq(row,axis:2:stateCount)=basis{4,segment}(end,:)-basis{4,segment+1}(1,:);
        end
    end
    row=row+1; Aeq(row,powerIndex(1))=1; beq(row)=1;
    inequalityCount=4*segmentCount*(4*degree-2)+planeCount*(degree+2);
    A=spalloc(inequalityCount,variableCount,10*inequalityCount); b=zeros(inequalityCount,1); row=0;
    limitValues=[limits.maxVelocity_units_s;limits.maxAcceleration_units_s2;limits.maxJerk_units_s3];
    for segment=1:segmentCount
        columns=1:stateCount;
        for order=0:3
            count=degree-order+1;
            rows=kron(kron(basis{order+1,segment},speye(2)),[1;-1]);
            indices=row+(1:size(rows,1)); row=indices(end);
            A(indices,columns)=rows;
            if order==0
                b(indices)=repmat([domain_units(1,2);-domain_units(1,1);domain_units(2,2);-domain_units(2,1)],count,1);
            else
                values=repmat(limitValues(order,:),count,1);
                A(indices,powerIndex(order+1))=-repelem(reshape(values.',[],1),2);
            end
            offset=reshape(offsets{order+1,segment}.',[],1);
            b(indices)=b(indices)-reshape([offset.';-offset.'],[],1);
        end
    end
    for segment=1:segmentCount
        columns=1:stateCount;
        for region=1:size(planes,2)
            plane=planes(segment,region);
            if ~plane.Active, continue; end
            plane.Offset_units=plane.Offset_units+(plane.Normal*origin_units.').';
            [localRows,offset_units]=fixedPlaneRows(plane,degree,2*(degree+1),1);
            rows=sparse(degree+2,variableCount);
            rows(:,columns)=localRows*kron(basis{1,segment},speye(2));
            indices=row+(1:size(rows,1)); row=indices(end);
            A(indices,:)=rows;
            b(indices)=-reserve_units-offset_units-localRows*reshape(offsets{1,segment}.',[],1);
        end
    end
    assert(row==inequalityCount);

    %% Section 3: Solve The Existing Time And Length Objectives
    cones=createTimePowerCones(variableCount,powerIndex);
    empty=secondordercone(sparse(2,variableCount),zeros(2,1),sparse(variableCount,1),0);
    lengthCones=repmat(empty,lengthCount,1);
    for segment=1:segmentCount
        columns=1:stateCount;
        for control=1:degree
            index=(segment-1)*degree+control;
            coneA=sparse(2,variableCount);
            coneA(:,columns)=kron(referenceTimes_s(segment)/degree*basis{2,segment}(control,:),speye(2));
            coneD=sparse(variableCount,1); coneD(lengthIndex(index))=1;
            lengthOffset=referenceTimes_s(segment)/degree*offsets{2,segment}(control,:);
            lengthCones(index)=secondordercone(coneA,-lengthOffset.',coneD,0);
        end
    end
    f=zeros(variableCount,1); f(powerIndex(4))=1;
    if fixedPhysicalClock, cones=lengthCones; f(:)=0; f(lengthIndex)=1; end
    if minimizeLength
        jerkMap=sparse(6*segmentCount,variableCount);
        for span=1:segmentCount
            jerkMap((span-1)*6+(1:6),1:stateCount)=kron(basis{4,span},speye(2));
        end
        smoothIndices=variableCount-segmentCount+(1:segmentCount);
        cones=[cones;bmtpEngine.createVariationCone(jerkMap,referenceTimes_s,limits,smoothIndices)];
        lb(smoothIndices)=0; f(smoothIndices)=0.005*norm(goal_units-start_units);
    end
    timer=tic;
    [x,~,exitFlag,output]=coneprog(f,cones,A,b,Aeq,beq,lb,ub,options);
    output.TotalTime_s=toc(timer); output.SolveCount=1; output.OptimizationConverged=exitFlag>0;
    controls_units=zeros(0,degree+1,2); times_s=NaN;
    if isempty(x) || any(~isfinite(x)) || (exitFlag<=0 && exitFlag~=-7), return; end

    %% Section 4: Export Controls And Powers From Integrated States
    % Distribute endpoint roundoff through the integrated jerk variables.
    % Bounds and separation are checked again on the resulting motion.
    terminal=Aeq(1:end-1,1:stateCount);
    residual=beq(1:end-1)-terminal*x(1:stateCount);
    x(1:stateCount)=x(1:stateCount)+terminal.'*((terminal*terminal.')\residual);
    output.ProjectedEndpointResidual=norm(beq(1:end-1)-terminal*x(1:stateCount),Inf);
    % A stalled conic solve can return a finite but unusable initialization.
    % Check after endpoint roundoff correction, before spending nonlinear
    % iterations on it. Final motion validation remains separate and exact.
    output.InitializationResidual=max([0;A*x-b;abs(Aeq*x-beq);lb-x;x-ub]);
    if ~minimizeLength && output.InitializationResidual>request.Options.ConstraintTolerance, return; end
    times_s=x(powerIndex(4))^(1/3)*referenceTimes_s;
    states=reshape(x(1:stateCount),2,jerkCount).';
    controls_units=zeros(segmentCount,degree+1,2); powers_units=zeros(segmentCount,2,degree+1);
    for segment=1:segmentCount
        controls_units(segment,:,:)=basis{1,segment}*states+offsets{1,segment}+origin_units;
        powers_units(segment,:,1)=basis{1,segment}(1,:)*states+offsets{1,segment}(1,:)+origin_units;
        powers_units(segment,:,2)=(basis{2,segment}(1,:)*states+offsets{2,segment}(1,:))*referenceTimes_s(segment);
        powers_units(segment,:,3)=(basis{3,segment}(1,:)*states+offsets{3,segment}(1,:))*referenceTimes_s(segment)^2/2;
        jerkPowers=[1,0,0;-2,2,0;1,-2,1]*basis{4,segment}*states;
        for power=0:degree-3
            powers_units(segment,:,power+4)=jerkPowers(power+1,:)*referenceTimes_s(segment)^3/((power+1)*(power+2)*(power+3));
        end
    end
    output.PositionPower_units=powers_units;
end

function soc = createTimePowerCones(variableCount, powerIndex)
    % Create p0*p2>=p1^2 and p1*p3>=p2^2 as standard cones.
    emptyCone = secondordercone(zeros(2, variableCount), zeros(2, 1), zeros(variableCount, 1), 0);
    soc       = repmat(emptyCone, 2, 1);
    % Process each cone needed to build time power cones.
    for coneIndex = 1:2
        coneA = zeros(2, variableCount);
        coneA(1, powerIndex(coneIndex + 1)) = 2;
        coneA(2, powerIndex(coneIndex)) = 1;
        coneA(2, powerIndex(coneIndex + 2)) = -1;
        coneD = zeros(variableCount, 1);
        coneD(powerIndex([coneIndex coneIndex + 2])) = 1;
        soc(coneIndex) = secondordercone(coneA, zeros(2, 1), coneD, 0);
    end
end

function [rows, offset_units] = fixedPlaneRows(plane, degree, variableCount, segmentIndex)
    % Multiply a fixed separating line by variable trajectory controls.
    % Exact degree-N by degree-one Bernstein product weights.
    beta  = (0:degree + 1).' / (degree + 1);
    alpha = 1 - beta;
    controlColumns = (segmentIndex-1)*2*(degree+1)+(1:2*(degree+1)).';
    rowIndices = [repelem((1:degree+1).',2);repelem((2:degree+2).',2)];
    values = [reshape((alpha(1:end-1)*plane.Normal(1,:)).',[],1); ...
        reshape((beta(2:end)*plane.Normal(2,:)).',[],1)];
    rows = sparse(rowIndices,[controlColumns;controlColumns],values,degree+2,variableCount);
    offset_units = alpha * plane.Offset_units(1) + beta * plane.Offset_units(2);
end


%% Section 4: Joint Quintic Phase-Time Optimization
function [times_s,exitFlag,output] = refinePhaseTimes(request,times_s,jerks_units_s3,planes,reserve_units)
    % Explicit shared knot states keep physical continuity local. Integrating
    % every earlier jerk into every later constraint creates a dense Jacobian.
    segmentCount=numel(times_s); stateCount=6*(segmentCount-1);
    jerkCount=4*segmentCount+2; variableCount=stateCount+jerkCount+segmentCount;
    limits=request.Limits; jerkLimit=limits.maxJerk_units_s3;
    stateScale=[max(1,abs(request.GoalState.position_units-request.InitialState.position_units)); ...
        limits.maxVelocity_units_s;limits.maxAcceleration_units_s2];
    initial=[request.InitialState.position_units;request.InitialState.velocity_units_s;request.InitialState.acceleration_units_s2];
    target=[request.GoalState.position_units;request.GoalState.velocity_units_s;request.GoalState.acceleration_units_s2];
    integrationA=tril(ones(4,3),-1)/3;
    integrationV=tril(ones(5,4),-1)/4;
    integrationP=tril(ones(6,5),-1)/5;
    constant=zeros(18,6); linear=constant; quadratic=constant; cubic=constant;
    constant(1:6,1)=1; constant(7:11,2)=1; constant(12:15,3)=1; constant(16:18,4:6)=eye(3);
    linear(1:6,2)=integrationP*ones(5,1);
    linear(7:11,3)=integrationV*ones(4,1); linear(12:15,4:6)=integrationA;
    quadratic(1:6,3)=integrationP*integrationV*ones(4,1);
    quadratic(7:11,4:6)=integrationV*integrationA;
    cubic(1:6,4:6)=integrationP*integrationV*integrationA;
    timeIndex=stateCount+jerkCount+(1:segmentCount);
    x0=zeros(variableCount,1); x0(timeIndex)=times_s;
    state=initial;
    stateMaps=cell(segmentCount,1); constraintMaps=stateMaps; bounds=stateMaps;
    stateRows=[1;7;2;8;3;9]; jerkRows=[4;10;5;11;6;12];
    identity=speye(36);
    positionRows=reshape([1:6;19:24],[],1);
    velocityRows=reshape([7:11;25:29],[],1);
    accelerationRows=reshape([12:15;30:33],[],1);
    velocityMap=spdiags(1./repmat(limits.maxVelocity_units_s.',5,1),0,10,10)*identity(velocityRows,:);
    accelerationMap=spdiags(1./repmat(limits.maxAcceleration_units_s2.',4,1),0,8,8)*identity(accelerationRows,:);
    positionMap=identity(positionRows,:);
    endpointRows=[6;24;11;29;15;33];
    endpointMap=spdiags(1./reshape(stateScale.',[],1),0,6,6)*identity(endpointRows,:);
    counts=zeros(segmentCount,1);
    for span=1:segmentCount
        if span>1
            x0(6*(span-2)+(1:6))=reshape((state./stateScale).',[],1);
        end
        jerkIndices=stateCount+4*(span-1)+(1:6);
        localJerks=squeeze(jerks_units_s3(span,:,:));
        x0(jerkIndices)=reshape((localJerks./jerkLimit).',[],1);
        map=sparse(12,variableCount);
        if span>1
            map(stateRows,6*(span-2)+(1:6))=diag(reshape(stateScale.',[],1));
        end
        map(jerkRows,jerkIndices)=diag(repmat(jerkLimit.',3,1)); stateMaps{span}=map;
        h=times_s(span); transition=constant+h*linear+h^2*quadratic+h^3*cubic;
        controls=transition*[state;localJerks]; state=controls([6,11,15],:);
        active=find([planes(span,:).Active]); planeMap=zeros(7*numel(active),12); planeBounds=zeros(size(planeMap,1),1);
        for k=1:numel(active)
            [local,offset]=fixedPlaneRows(planes(span,active(k)),5,12,1);
            rows=(k-1)*7+(1:7); planeMap(rows,:)=local; planeBounds(rows)=-reserve_units-offset;
        end
        constraintMaps{span}=[velocityMap;-velocityMap;accelerationMap;-accelerationMap;positionMap;-positionMap;sparse(planeMap)*positionMap];
        bounds{span}=[ones(36,1);repmat([limits.xInterval_units(2);limits.yInterval_units(2)],6,1); ...
            -repmat([limits.xInterval_units(1);limits.yInterval_units(1)],6,1);planeBounds];
        counts(span)=numel(bounds{span});
    end
    starts=[0;cumsum(counts)];
    lb=[-Inf(stateCount,1);-ones(jerkCount,1);eps*ones(segmentCount,1)];
    ub=[Inf(stateCount,1);ones(jerkCount,1);repmat(request.MotionHorizon_s,segmentCount,1)];
    totalTimeRow=sparse(ones(1,segmentCount),timeIndex,ones(1,segmentCount),1,variableCount);
    difference=sparse(4*segmentCount,variableCount);
    for span=1:segmentCount
        difference((span-1)*4+(1:4),stateCount+4*(span-1)+(1:6))=kron([-1,1,0;0,-1,1],speye(2));
    end
    variationGram=(difference.'*difference)/(16*segmentCount);
    % Each normalized jerk difference is in [-2,2], so this entire penalty
    % is between zero and the single arrival allowance. This regularizes
    % phase generation itself; the final repair does not add another delay.
    variationAllowance_s=request.Options.PathLengthTimeAllowance_s;
    lastJacobian=sparse(0,variableCount);
    settings=optimoptions('fmincon','Algorithm','interior-point','Display','none', ...
        'SpecifyObjectiveGradient',true,'SpecifyConstraintGradient',true, ...
        'HessianFcn',@hessian,'InitBarrierParam',1e-5,'SubproblemAlgorithm','cg', ...
        'MaxIterations',200,'MaxFunctionEvaluations',1000, ...
        'ConstraintTolerance',1e-10,'OptimalityTolerance',1e-7,'StepTolerance',1e-12,'ScaleProblem',true);
    timer=tic;
    [x,~,exitFlag,output]=fmincon(@objective,x0,totalTimeRow,request.MotionHorizon_s,[],[],lb,ub,@constraints,settings);
    output.TotalTime_s=toc(timer); output.VariableCount=variableCount;
    output.ConstraintJacobianNonzeros=nnz(lastJacobian);
    output.JerkVariationPenalty_s=variationAllowance_s*(x.'*variationGram*x);
    times_s=x(timeIndex);

    function [value,gradient]=objective(x)
        value=sum(x(timeIndex))+variationAllowance_s*(x.'*variationGram*x);
        gradient=full(totalTimeRow.'+2*variationAllowance_s*variationGram*x);
    end
    function [residual,equality,gradient,equalityGradient]=constraints(x)
        residual=zeros(starts(end),1); equality=zeros(6*segmentCount,1);
        blocks=cell(segmentCount,1); endpointBlocks=blocks;
        for span=1:segmentCount
            h=x(timeIndex(span)); map=stateMaps{span};
            local=reshape(map*x,6,2);
            if span==1, local(1:3,:)=initial; end
            transition=constant+h*linear+h^2*quadratic+h^3*cubic;
            first=linear+2*h*quadratic+3*h^2*cubic;
            controls=transition*local;
            derivatives=kron(speye(2),sparse(transition))*map;
            derivatives(:,timeIndex(span))=reshape(first*local,[],1);
            rows=starts(span)+(1:counts(span));
            residual(rows)=constraintMaps{span}*controls(:)-bounds{span};
            blocks{span}=constraintMaps{span}*derivatives;
            endpointBlocks{span}=endpointMap*derivatives;
            next=target;
            if span<segmentCount
                next=reshape(x(6*(span-1)+(1:6)),2,3).'.*stateScale;
                endpointBlocks{span}(:,6*(span-1)+(1:6))=endpointBlocks{span}(:,6*(span-1)+(1:6))-speye(6);
            end
            equality(6*(span-1)+(1:6))=reshape(((controls([6,11,15],:)-next)./stateScale).',[],1);
        end
        lastJacobian=vertcat(blocks{:});
        gradient=lastJacobian.'; equalityGradient=vertcat(endpointBlocks{:}).';
    end
    function curvature=hessian(x,lambda)
        curvature=2*variationAllowance_s*variationGram;
        for span=1:segmentCount
            ti=timeIndex(span); h=x(ti); map=stateMaps{span}; local=reshape(map*x,6,2);
            if span==1, local(1:3,:)=initial; end
            rows=starts(span)+(1:counts(span));
            weights=reshape(constraintMaps{span}.'*lambda.ineqnonlin(rows)+endpointMap.'*lambda.eqnonlin(6*(span-1)+(1:6)),18,2);
            first=linear+2*h*quadratic+3*h^2*cubic; second=2*quadratic+6*h*cubic;
            cross=reshape(first.'*weights,1,12)*map;
            curvature(ti,:)=curvature(ti,:)+cross;
            curvature(:,ti)=curvature(:,ti)+cross.';
            curvature(ti,ti)=curvature(ti,ti)+sum(weights.*(second*local),'all');
        end
    end
end
