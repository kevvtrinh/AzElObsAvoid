function [result,diagnostics] = solveCubicTrajectory(request,warmStart,diagnostics,target_units,reserve_units)
%% Section 0: Header & Readme
% SYNTAX: [motion,diagnostics] = bmtpEngine.solveCubicTrajectory(request,warm,diagnostics,target,reserve)
% PURPOSE: Optimize a static detour using integrated constant-jerk phases.
%   Initialize the clock conically, vary phase boundaries and jerk jointly,
%   then solve the resulting clock ratios for time and control-polygon length.
% INPUTS: Validated earliest-arrival static request, visibility warm start,
%   diagnostic record, obstacle target, and numerical reserve.
% OUTPUTS: A polynomial proposal and full solver diagnostics. The caller must
%   prepare and independently validate every returned motion before success.
% UNITS: Coordinate units and seconds; jerk uses coordinate units/s^3.

%% Section 1: Initialize One Integrated Cubic Model
assert(request.Options.GoalTimeMode=="earliestArrival" && ~isfield(request.Coverage,'ActiveTimeInterval_s'), ...
    'bmtpEngine:InvalidCubicRequest','The variable cubic clock requires static geometry and earliest arrival.');
segmentCount=warmStart.SegmentCount;
degree=request.Degree;
regionCount=numel(request.Regions_units);
emptyPlane=struct('Active',false,'Verified',false,'ExitFlag',-2,'Normal',zeros(2,2), ...
    'Offset_units',zeros(1,2),'SignedGap_units',NaN);
planes=repmat(emptyPlane,segmentCount,regionCount);
for k=1:segmentCount
    for j=1:regionCount
        planes(k,j)=bmtpEngine.solveSeparatingLine(squeeze(warmStart.ControlPoint_units(k,:,:)),request.Regions_units{j},target_units,reserve_units);
    end
end
result=struct('Success',false,'SolverMessage',"Cubic clock initialization failed.", ...
    'ControlPoint_units',zeros(0,degree+1,2),'SegmentTime_s',NaN,'PositionPower_units',[]);
diagnostics.Identifier="cubicJerkClock";
diagnostics.ConstraintRepresentation="integratedCubicVariableClock";
[controls_units,times_s,~,initial]=solveCubicStep(request,planes,warmStart.SegmentRatio,reserve_units,false,request.MotionHorizon_s);
diagnostics.TrajectorySocpCount=initial.SolveCount;
diagnostics.ConicSolver=bmtpEngine.accumulateConicDiagnostics(bmtpEngine.accumulateConicDiagnostics(),initial);
if isempty(controls_units) || any(~isfinite(controls_units),'all'), return; end
conversion=zeros(degree+1,4);
for k=0:degree
    for j=0:min(k,3), conversion(k+1,j+1)=nchoosek(k,j)/nchoosek(degree,j); end
end
controls_units=permute(pagemtimes(conversion,permute(initial.PositionPower_units,[3,2,1])),[3,1,2]);
for k=1:segmentCount
    for j=1:regionCount
        planes(k,j)=bmtpEngine.solveSeparatingLine(squeeze(controls_units(k,:,:)),request.Regions_units{j},target_units,reserve_units);
    end
end

%% Section 2: Refine Phase Boundaries And Repair The Complete Motion
jerks_units_s3=6*initial.PositionPower_units(:,:,4)./times_s.^3;
[times_s,nonlinearFlag,nonlinear]=refinePhaseTimes(request,times_s,jerks_units_s3,planes,reserve_units);
diagnostics.NonlinearSolver=nonlinear;
if any(~isfinite(times_s) | times_s<=0)
    result.SolverMessage="The phase-time optimizer did not return positive finite durations."; return;
end
[controls_units,times_s,exitFlag,final]=solveCubicStep(request,planes,times_s/mean(times_s),reserve_units,true,sum(times_s));
diagnostics.TrajectorySocpCount=diagnostics.TrajectorySocpCount+final.SolveCount;
diagnostics.ConicSolver=bmtpEngine.accumulateConicDiagnostics(diagnostics.ConicSolver,final);
diagnostics.FinalTrajectoryExitFlag=exitFlag;
if isempty(controls_units) || any(~isfinite(controls_units),'all')
    result.SolverMessage="Cubic clock repair failed."; return;
end
controls_units=permute(pagemtimes(conversion,permute(final.PositionPower_units,[3,2,1])),[3,1,2]);
final.PositionPower_units(:,:,5:degree+1)=0;
result.Success=true;
result.ControlPoint_units=controls_units;
result.SegmentTime_s=times_s;
result.PositionPower_units=final.PositionPower_units;
result.SolverMessage="An integrated cubic motion was optimized with variable phase times.";
diagnostics.SolverMessage=result.SolverMessage;
diagnostics.Converged=exitFlag>0 && nonlinearFlag>0;
diagnostics.IterationCount=1;
end

%% Section 3: Convex Integrated-Jerk Subproblem
function [controls_units,times_s,exitFlag,output] = solveCubicStep(request,planes,ratios,reserve_units,minimizeLength,fixedDuration_s)
    segmentCount=size(planes,1); degree=3; limits=request.Limits;
    start_units=request.InitialState.position_units; goal_units=request.GoalState.position_units;
    horizon_s=request.MotionHorizon_s; options=request.TrajectoryOptions;
    %% Section 1: Construct The Integrated Polynomial Basis
    origin_units=start_units;
    goal_units=goal_units-origin_units; start_units=[0,0];
    domain_units=[limits.xInterval_units-origin_units(1);limits.yInterval_units-origin_units(2)];
    jerkCount=segmentCount*(degree-2); stateCount=2*jerkCount;
    referenceTime_s=fixedDuration_s;
    if request.IsRest
        [~,profileTimes_s]=bmtpEngine.createJerkLimitedChord(start_units,goal_units,limits,degree);
        referenceTime_s=sum(profileTimes_s);
    end
    referenceTimes_s=referenceTime_s*ratios(:)/sum(ratios);
    powerIndex=stateCount+(1:4);
    lengthCount=segmentCount*degree;
    planeCount=nnz([planes.Active]);
    variableCount=stateCount+4+lengthCount;
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
        offsets{4,segment}=zeros(1,2);
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
    if ~request.IsRest, lb(powerIndex)=1; ub(powerIndex)=1; end
    %% Section 2: Impose Physical Continuity And Bounds
    Aeq=spalloc(7,variableCount,6*stateCount);
    beq=zeros(size(Aeq,1),1); row=0;
    for axis=1:2
        for order=0:2
            row=row+1; Aeq(row,axis:2:stateCount)=basis{order+1,segmentCount}(end,:);
            terminalState=[goal_units;request.GoalState.velocity_units_s;request.GoalState.acceleration_units_s2];
            beq(row)=terminalState(order+1,axis)-offsets{order+1,segmentCount}(end,axis);
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
    if ~request.IsRest, cones=lengthCones; f(:)=0; f(lengthIndex)=1; end
    timer=tic;
    [x,~,exitFlag,output]=coneprog(f,cones,A,b,Aeq,beq,lb,ub,options);
    output.TotalTime_s=toc(timer); output.SolveCount=1; output.OptimizationConverged=exitFlag>0;
    if minimizeLength && request.IsRest && ~isempty(x) && all(isfinite(x)) && (exitFlag>0 || exitFlag==-7)
        ub(powerIndex(4))=x(powerIndex(4));
        f(:)=0; f(lengthIndex)=1; timer=tic;
        [shortX,~,shortFlag,shortOutput]=coneprog(f,[cones;lengthCones],A,b,Aeq,beq,lb,ub,options);
        elapsed=output.TotalTime_s+toc(timer);
        converged=output.OptimizationConverged && shortFlag>0;
        if ~isempty(shortX) && all(isfinite(shortX)) && (shortFlag>0 || shortFlag==-7)
            x=shortX; exitFlag=shortFlag; output=shortOutput;
        end
        output.TotalTime_s=elapsed; output.SolveCount=2; output.OptimizationConverged=converged;
    end
    controls_units=zeros(0,degree+1,2); times_s=NaN;
    if isempty(x) || any(~isfinite(x)) || (exitFlag<=0 && exitFlag~=-7), return; end

    %% Section 4: Export Controls And Powers From Integrated States
    % Distribute endpoint roundoff through the integrated jerk variables.
    % Bounds and separation are checked again on the resulting motion.
    terminal=Aeq(1:6,1:stateCount);
    residual=beq(1:6)-terminal*x(1:stateCount);
    x(1:stateCount)=x(1:stateCount)+terminal.'*((terminal*terminal.')\residual);
    output.ProjectedEndpointResidual=norm(beq(1:6)-terminal*x(1:stateCount),Inf);
    times_s=x(powerIndex(4))^(1/3)*referenceTimes_s;
    states=reshape(x(1:stateCount),2,jerkCount).';
    controls_units=zeros(segmentCount,degree+1,2); powers_units=zeros(segmentCount,2,degree+1);
    for segment=1:segmentCount
        controls_units(segment,:,:)=basis{1,segment}*states+offsets{1,segment}+origin_units;
        powers_units(segment,:,1)=basis{1,segment}(1,:)*states+offsets{1,segment}(1,:)+origin_units;
        powers_units(segment,:,2)=(basis{2,segment}(1,:)*states+offsets{2,segment}(1,:))*referenceTimes_s(segment);
        powers_units(segment,:,3)=(basis{3,segment}(1,:)*states+offsets{3,segment}(1,:))*referenceTimes_s(segment)^2/2;
        jerkPowers=basis{4,segment}*states;
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

%% Section 4: Joint Phase-Time Optimization
function [times_s,exitFlag,output] = refinePhaseTimes(request,times_s,jerks_units_s3,planes,reserve_units)
    % Integrate bounded jerk exactly; shared states enforce physical C2 continuity.
    %% Section 1: Prepare Phase Boundaries And Irredundant Bounds
    segmentCount=numel(times_s); variableCount=3*segmentCount; limits=request.Limits;
    jerkLimit_units_s3=limits.maxJerk_units_s3;
    velocityLimit_units_s=limits.maxVelocity_units_s;
    accelerationLimit_units_s2=limits.maxAcceleration_units_s2;
    x0=[reshape((jerks_units_s3./jerkLimit_units_s3).',[],1);cumsum(times_s(:))];
    lb=[-Inf(2*segmentCount,1);zeros(segmentCount,1)];
    ub=[Inf(2*segmentCount,1);repmat(request.MotionHorizon_s,segmentCount,1)];
    phaseDifferenceMap=spdiags(ones(segmentCount,1)*[-1,1],[-1,0],segmentCount,segmentCount);
    transform=blkdiag(speye(2*segmentCount),phaseDifferenceMap);
    % Keep jerk bounds as linear inequalities so the optimizer does not
    % shift a saturated initial jerk away from its analytic endpoint states.
    jerkRows=[speye(2*segmentCount),sparse(2*segmentCount,segmentCount)];
    positionScale=max(1,abs(request.GoalState.position_units-request.InitialState.position_units));
    endpointScale=[positionScale;velocityLimit_units_s;accelerationLimit_units_s2];
    planeMaps=cell(segmentCount,1); planeBounds_units=cell(segmentCount,1); counts=zeros(segmentCount,1);
    beta=(0:4)'/4; alpha=1-beta;
    for setupIndex=1:segmentCount
        active=find([planes(setupIndex,:).Active]); map=zeros(5*numel(active),8); bounds=zeros(size(map,1),1);
        for k=1:numel(active)
            plane=planes(setupIndex,active(k)); rows=(k-1)*5+(1:5);
            for axis=1:2
                map(rows(1:4),(0:3)*2+axis)=diag(alpha(1:4)*plane.Normal(1,axis));
                map(rows(2:5),(0:3)*2+axis)=map(rows(2:5),(0:3)*2+axis)+diag(beta(2:5)*plane.Normal(2,axis));
            end
            bounds(rows)=-reserve_units-alpha*plane.Offset_units(1)-beta*plane.Offset_units(2);
        end
        planeMaps{setupIndex}=map; planeBounds_units{setupIndex}=bounds; counts(setupIndex)=36+numel(bounds);
    end
    % Per phase: velocity (12), acceleration (8), workspace (16), then
    % five exact product coefficients for each active separating plane.
    starts=[0;cumsum(counts)]; constraintCount=starts(end)+1;
    keep=true(constraintCount,1);
    for setupIndex=1:segmentCount
        % Shared boundary states need one bound; fixed endpoint states need none.
        keep(starts(setupIndex)+[1,2,7,8,13,14,17,18,21,22,29,30])=false;
        active=find([planes(setupIndex,:).Active]);
        for k=1:numel(active)
            plane=planes(setupIndex,active(k));
            if setupIndex>1 && isequal(plane.Normal,planes(setupIndex-1,active(k)).Normal) && isequal(plane.Offset_units,planes(setupIndex-1,active(k)).Offset_units)
                keep(starts(setupIndex)+36+5*(k-1)+1)=false;
            end
        end
    end
    keep(starts(segmentCount)+[5,6,11,12,15,16,19,20,27,28,35,36])=false;
    % These maps integrate physical p, v, a, and j into position, velocity,
    % and acceleration Bernstein coefficients on one constant-jerk phase.
    constantMap=zeros(9,4); linearTimeMap=constantMap; quadraticTimeMap=constantMap; cubicTimeMap=constantMap;
    constantMap(1:4,1)=1; constantMap(5:7,2)=1; constantMap(8:9,3)=1;
    linearTimeMap(2:4,2)=[1/3;2/3;1]; linearTimeMap(6:7,3)=[1/2;1]; linearTimeMap(9,4)=1;
    quadraticTimeMap(3:4,3)=[1/6;1/2]; quadraticTimeMap(7,4)=1/2; cubicTimeMap(4,4)=1/6;
    settings=optimoptions('fmincon','Algorithm','interior-point','Display','none', ...
        'SpecifyObjectiveGradient',true,'SpecifyConstraintGradient',true,'HessianFcn',@hessian, ...
        'InitBarrierParam',1e-5,'MaxIterations',100,'MaxFunctionEvaluations',500,'SubproblemAlgorithm','cg','ScaleProblem',true, ...
        'ConstraintTolerance',1e-10,'OptimalityTolerance',1e-7,'StepTolerance',1e-12);
    %% Section 2: Optimize The Joint Cubic Model
    timer=tic;
    [x,~,exitFlag,output]=fmincon(@objective,x0,[jerkRows;-jerkRows;sparse(segmentCount,2*segmentCount),-phaseDifferenceMap],[ones(4*segmentCount,1);-eps*ones(segmentCount,1)],[],[],lb,ub,@constraints,settings);
    output.TotalTime_s=toc(timer); times_s=phaseDifferenceMap*x(2*segmentCount+(1:segmentCount));

    %% Section 3: Objective, Constraints, And Exact Adjoint Hessian
        function [f,g]=objective(x)
            f=x(end); g=zeros(variableCount,1); g(end)=1;
        end
        function [inequalityResidual,equalityResidual,inequalityGradient,equalityGradient]=constraints(x)
            [inequalityResidual,equalityResidual,inequalityJacobian,equalityJacobian]=evaluate(x,[]);
            inequalityResidual=inequalityResidual(keep);
            inequalityGradient=sparse((inequalityJacobian(keep,:)*transform).');
            equalityGradient=sparse((equalityJacobian*transform).');
        end
        function lagrangianHessian=hessian(x,lambda)
            full=zeros(constraintCount,1); full(keep)=lambda.ineqnonlin; lambda.ineqnonlin=full;
            [~,~,~,~,lagrangianHessian]=evaluate(x,lambda); lagrangianHessian=sparse(transform.'*lagrangianHessian*transform);
        end
        function [inequalityResidual,equalityResidual,inequalityJacobian,equalityJacobian,lagrangianHessian]=evaluate(x,lambda)
            x=transform*x;
            useH=~isempty(lambda);
            inequalityResidual=zeros(constraintCount,1,'like',x);
            inequalityJacobian=zeros(constraintCount,variableCount,'like',x);
            motionState=zeros(4,2,'like',x); motionState(1:3,:)=[request.InitialState.position_units;request.InitialState.velocity_units_s;request.InitialState.acceleration_units_s2];
            stateJacobian=zeros(4,2,variableCount,'like',x); lagrangianHessian=zeros(variableCount,variableCount,'like',x);
            stateByPhase=zeros(4,2,segmentCount,'like',x);
            stateJacobianByPhase=zeros(4,2,variableCount,segmentCount,'like',x);
            weightsByPhase=zeros(9,2,segmentCount,'like',x);
            duration_s=x(2*segmentCount+(1:segmentCount));
            for span=1:segmentCount
                ti=2*segmentCount+span; ji=(span-1)*2+(1:2); motionState(4,:)=x(ji).'.*jerkLimit_units_s3;
                stateJacobian(4,:,:)=0; stateJacobian(4,1,ji(1))=jerkLimit_units_s3(1); stateJacobian(4,2,ji(2))=jerkLimit_units_s3(2);
                if useH, stateByPhase(:,:,span)=motionState; stateJacobianByPhase(:,:,:,span)=stateJacobian; end
                transition=constantMap+duration_s(span)*linearTimeMap+duration_s(span)^2*quadraticTimeMap+duration_s(span)^3*cubicTimeMap;
                first=linearTimeMap+2*duration_s(span)*quadraticTimeMap+3*duration_s(span)^2*cubicTimeMap;
                stateControls=transition*motionState;
                derivatives=reshape(kron(eye(2),transition)*reshape(stateJacobian,8,variableCount),9,2,variableCount);
                derivatives(:,:,ti)=derivatives(:,:,ti)+first*motionState;
                normalizedVelocity=reshape((stateControls(5:7,:)./velocityLimit_units_s).',[],1);
                normalizedAcceleration=reshape((stateControls(8:9,:)./accelerationLimit_units_s2).',[],1);
                positionControl_units=reshape(stateControls(1:4,:).',[],1);
                velocityJacobian=reshape(permute(derivatives(5:7,:,:)./reshape(velocityLimit_units_s,1,2,1),[2,1,3]),6,variableCount);
                accelerationJacobian=reshape(permute(derivatives(8:9,:,:)./reshape(accelerationLimit_units_s2,1,2,1),[2,1,3]),4,variableCount);
                positionJacobian=reshape(permute(derivatives(1:4,:,:),[2,1,3]),8,variableCount);
                upper=repmat([limits.xInterval_units(2);limits.yInterval_units(2)],4,1);
                lower=repmat([limits.xInterval_units(1);limits.yInterval_units(1)],4,1);
                rows=starts(span)+(1:counts(span));
                inequalityResidual(rows)=[normalizedVelocity-1;-normalizedVelocity-1;normalizedAcceleration-1;-normalizedAcceleration-1;positionControl_units-upper;lower-positionControl_units;planeMaps{span}*positionControl_units-planeBounds_units{span}];
                inequalityJacobian(rows,:)=[velocityJacobian;-velocityJacobian;accelerationJacobian;-accelerationJacobian;positionJacobian;-positionJacobian;planeMaps{span}*positionJacobian];
                if useH
                    mu=lambda.ineqnonlin(rows); weights=zeros(9,2);
                    weights(5:7,:)=reshape(mu(1:6)-mu(7:12),2,3).'./velocityLimit_units_s;
                    weights(8:9,:)=reshape(mu(13:16)-mu(17:20),2,2).'./accelerationLimit_units_s2;
                    weights(1:4,:)=reshape(mu(21:28)-mu(29:36)+planeMaps{span}.'*mu(37:end),2,4).';
                    weightsByPhase(:,:,span)=weights;
                end
                motionState(1:3,:)=stateControls([4,7,9],:); stateJacobian(1:3,:,:)=derivatives([4,7,9],:,:);
            end
            inequalityResidual(end)=sum(duration_s)-request.MotionHorizon_s; inequalityJacobian(end,2*segmentCount+(1:segmentCount))=1;
            goal=[request.GoalState.position_units;request.GoalState.velocity_units_s;request.GoalState.acceleration_units_s2];
            equalityResidual=reshape(((motionState(1:3,:)-goal)./endpointScale).',[],1);
            equalityJacobian=reshape(permute(stateJacobian(1:3,:,:)./reshape(endpointScale,3,2,1),[2,1,3]),6,variableCount);
            if useH
                % Curvature enters only through each phase duration. Propagate
                % state weights backward instead of carrying dense state Hessians.
                adjoint=reshape(lambda.eqnonlin,2,3).'./endpointScale;
                for span=segmentCount:-1:1
                    ti=2*segmentCount+span; motionState=stateByPhase(:,:,span); weights=weightsByPhase(:,:,span);
                    transition=constantMap+duration_s(span)*linearTimeMap+duration_s(span)^2*quadraticTimeMap+duration_s(span)^3*cubicTimeMap;
                    first=linearTimeMap+2*duration_s(span)*quadraticTimeMap+3*duration_s(span)^2*cubicTimeMap;
                    second=2*quadraticTimeMap+6*duration_s(span)*cubicTimeMap;
                    firstWeights=first.'*weights+first([4,7,9],:).'*adjoint;
                    cross=firstWeights(:).'*reshape(stateJacobianByPhase(:,:,:,span),8,variableCount);
                    lagrangianHessian(ti,:)=lagrangianHessian(ti,:)+cross; lagrangianHessian(:,ti)=lagrangianHessian(:,ti)+cross.';
                    lagrangianHessian(ti,ti)=lagrangianHessian(ti,ti)+sum(weights.*(second*motionState),'all')+sum(adjoint.*(second([4,7,9],:)*motionState),'all');
                    previous=transition.'*weights+transition([4,7,9],:).'*adjoint; adjoint=previous(1:3,:);
                end
            end
        end
end
