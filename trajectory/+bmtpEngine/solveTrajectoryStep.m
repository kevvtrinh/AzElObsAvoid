function [controlPoint_units, segmentTime_s, exitFlag, output] = solveTrajectoryStep(segmentCount, degree, initialState, goalState, limits, planes, reserve_units, maximumMotionDuration_s, options, segmentRatio, fixedClock, fixedControl_units, minimizeLength)
%% Section 0: Header & Readme
% SYNTAX: [controlPoint_units, segmentTime_s, exitFlag, output] = bmtpEngine.solveTrajectoryStep(
%   segmentCount, degree, initialState, goalState, limits, planes, reserve_units,
%   maximumMotionDuration_s, options, segmentRatio, fixedClock, fixedControl_units, minimizeLength)
% PURPOSE: Solve one convex trajectory step for fixed separating lines, timing policy, and
%   derivative limits.
% INPUTS: segmentCount, degree (positive integer scalars) Composite Bezier representation size.
%   initialState, goalState (full normalized state structs) Prescribed physical endpoint positions,
%   velocities, and accelerations.
%   limits (scalar struct) Workspace, velocity, acceleration, and jerk limits.
%   planes (S-by-R struct array) Fixed active separating-line constraints. Optional TimeFraction
%   restricts a plane to a closed part of a fixed-duration motion span.
%   reserve_units (nonnegative scalar) Numerical separation reserve.
%   maximumMotionDuration_s (positive scalar) Upper bound on the internal minimum-time solve.
%   options (coneprog options) Numerical solver controls. Optional mesh ratio, fixed-clock,
%   prescribed-control, and length-objective inputs follow.
% OUTPUTS: controlPoint_units (S-by-(D+1)-by-2 numeric array) Solved control points, or an empty
%   array on expected solve failure.
%   segmentTime_s (scalar numeric) Per-segment durations, or NaN on expected solve failure.
%   exitFlag (numeric scalar), output (solver record) Original coneprog status and measured solver
%   time.
% UNITS: Position is coordinate units and time is seconds.

%% Section 1: Create Decision Bounds And Continuity Rows
controlCount           = segmentCount * (degree + 1) * 2;
if nargin < 10, segmentRatio = ones(segmentCount, 1); end
if nargin < 11, fixedClock = false; end
if nargin < 13, minimizeLength = true; end
originalPlaneCount=nnz([planes.Active]);
partialPlanes=false;
if isfield(planes,'TimeFraction') && ~isempty(planes)
    fractions=reshape([planes.TimeFraction],2,[]).';
    validateattributes(fractions,{'numeric'},{'real','finite','ncols',2,'>=',0,'<=',1});
    assert(all(fractions(:,1)<fractions(:,2)),'bmtpEngine:InvalidPlaneTimeScope');
    partialPlanes=any(fractions~=[0,1],'all');
    assert(fixedClock || ~partialPlanes,'bmtpEngine:InvalidPlaneTimeScope');
end
% Half-spaces on different physical intervals cannot eliminate each other.
if fixedClock && ~partialPlanes && originalPlaneCount>segmentCount*degree
    planes=bmtpEngine.removeRedundantPlanes(planes,limits,2*reserve_units);
end
prescribedAxis = nargin>=12 && ~isempty(fixedControl_units) && any(isfinite(fixedControl_units(:)));
% Larger clocks have surplus phases that can oscillate under length alone.
% Preserve the compact eight-span steering solve used on sparse clocks.
intrinsicVariation=fixedClock && degree==5 && ~prescribedAxis && segmentCount>8;
start_units = initialState.position_units; goal_units = goalState.position_units;
isRest = all([initialState.velocity_units_s initialState.acceleration_units_s2 goalState.velocity_units_s goalState.acceleration_units_s2]==0);
assert(fixedClock || isRest,'bmtpEngine:NonrestRelaxedClock','Nonzero boundary states require physical fixed durations.');
boundaryControls = bmtpEngine.imposeEndpointControls(zeros(segmentCount,degree+1,2), ...
    maximumMotionDuration_s*segmentRatio/sum(segmentRatio),initialState,goalState);
powerIndex             = controlCount + (1:4);
lengthCount = segmentCount * degree;
planeActiveBySegment = reshape([planes.Active], size(planes));
activePlaneCount = nnz(planeActiveBySegment);
planeCountBySegment = sum(planeActiveBySegment, 2);
% Share elastic variables only when they dominate the length-cone variables.
% The weighted maximum penalizes every retained plane; zero slack recovers
% the same hard corridor, which is independently checked before acceptance.
sharedSlack = fixedClock && originalPlaneCount>lengthCount;
slackCount = fixedClock*activePlaneCount;
if sharedSlack, slackCount=nnz(planeCountBySegment); end
variableCount          = controlCount + 4 + lengthCount + slackCount + intrinsicVariation*segmentCount;
physicalTimes_s = maximumMotionDuration_s*segmentRatio/sum(segmentRatio);
jerkTimes_s = []; if intrinsicVariation, jerkTimes_s = physicalTimes_s; end
[A,Aeq,beq,lb,ub,jerkMap] = bmtpEngine.createTrajectoryConstraints( ...
    segmentCount,degree,boundaryControls,limits,variableCount,activePlaneCount,segmentRatio,jerkTimes_s);
if nargin>=12 && ~isempty(fixedControl_units)
    fixedValues = reshape(permute(fixedControl_units,[3,2,1]),[],1);
    indices = find(isfinite(fixedValues));
    lb(indices) = fixedValues(indices);
    ub(indices) = fixedValues(indices);
end

%% Section 2: Add Separating-Line Bounds
baseInequalityCount = 4*segmentCount*(3*degree-3);
inequalityCount = size(A,1);
b               = zeros(inequalityCount, 1);
inequalityIndex = baseInequalityCount;
slackIndex = controlCount+4+lengthCount;
for segmentIndex = 1:segmentCount
    if sharedSlack && planeCountBySegment(segmentIndex)>0
        slackIndex = slackIndex+1;
    end
    for regionIndex = reshape(find(planeActiveBySegment(segmentIndex, :)), 1, [])
        plane = planes(segmentIndex, regionIndex);
        [rows, offset_units] = bmtpEngine.createPlaneRows(plane, degree, variableCount, segmentIndex);
        targets = inequalityIndex + (1:size(rows, 1));
        A(targets, :) = rows;
        if fixedClock
            if ~sharedSlack, slackIndex=slackIndex+1; end
            A(targets,slackIndex) = -1;
        end
        b(targets) = -(1+fixedClock)*reserve_units - offset_units;
        inequalityIndex = targets(end);
    end
end

%% Section 3: Create The Objective And Solve
cones = bmtpEngine.createTimePowerCones(variableCount, powerIndex);
f = zeros(variableCount, 1);
f(powerIndex(4)) = 1;
maximumSegmentTime_s = maximumMotionDuration_s / sum(segmentRatio);
timePowers_s         = [1; maximumSegmentTime_s; ...
    maximumSegmentTime_s ^ 2; maximumSegmentTime_s ^ 3];
ub(powerIndex) = timePowers_s;
lengthIndex = controlCount+4+(1:lengthCount);
lb(lengthIndex) = 0;
if fixedClock
    lb(powerIndex) = timePowers_s;
    f(:) = 0;
    f(lengthIndex) = 1;
    slackIndices = controlCount+4+lengthCount+(1:slackCount);
    lb(slackIndices) = 0;
    % Elastic sequential convex programming: penalize clearance slack in the
    % same distance units as control-polygon length. Only independently clear
    % motion may be accepted by the outer solve.
    f(slackIndices) = 1e3;
    if sharedSlack, f(slackIndices)=1e3*planeCountBySegment(planeCountBySegment>0); end
end
emptyCone = secondordercone(sparse(2,variableCount),zeros(2,1),sparse(variableCount,1),0);
lengthCones = repmat(emptyCone,lengthCount,1);
for k = 1:segmentCount
    for j = 1:degree
        row = (k-1)*degree+j;
        coneA = sparse(2,variableCount);
        coneA(:,controlIndexOf(k,j,1:2,degree)) = eye(2);
        coneA(:,controlIndexOf(k,j-1,1:2,degree)) = -eye(2);
        coneD = sparse(variableCount,1); coneD(lengthIndex(row)) = 1;
        lengthCones(row) = secondordercone(coneA,zeros(2,1),coneD,0);
    end
end
if fixedClock
    cones=lengthCones;
    if intrinsicVariation
        smoothIndices=variableCount-segmentCount+(1:segmentCount);
        cones=[cones;bmtpEngine.createVariationCone(jerkMap,physicalTimes_s,limits,smoothIndices)];
        lb(smoothIndices)=0;
        f(smoothIndices)=0.005*norm(goal_units-start_units);
    end
end
solverTimer = tic;
solverTimes_s=[]; if intrinsicVariation, solverTimes_s=physicalTimes_s; end
[x, ~, exitFlag, output] = solveConic(f, cones, A, b, Aeq, beq, lb, ub, options, prescribedAxis,solverTimes_s,limits);
output.TotalTime_s = toc(solverTimer);
output.SolveCount = 1;
output.OptimizationConverged = exitFlag>0;
output.IntrinsicJerkVariation = intrinsicVariation;
if ~fixedClock && minimizeLength && ~isempty(x) && all(isfinite(x)) && (exitFlag>0 || exitFlag==-7)
    % Lexicographic optimization: preserve the first time-power value while
    % minimizing path length. Unconstrained lateral motion must not be chosen
    % arbitrarily merely because another axis determines the arrival time.
    ub(powerIndex(4)) = x(powerIndex(4));
    f(:) = 0; f(lengthIndex) = 1;
    timer = tic;
    [shortX,~,shortFlag,shortOutput] = solveConic(f,[cones;lengthCones],A,b,Aeq,beq,lb,ub,options,prescribedAxis,[],limits);
    bothConverged = output.OptimizationConverged && shortFlag>0;
    shortElapsed_s = toc(timer);
    elapsed_s = output.TotalTime_s+shortElapsed_s;
    if ~isempty(shortX) && all(isfinite(shortX)) && (shortFlag>0 || shortFlag==-7)
        x = shortX; exitFlag = shortFlag; output = shortOutput;
    end
    output.TotalTime_s = elapsed_s;
    output.SolveCount = 2;
    output.OptimizationConverged = bothConverged;
end
output.OriginalPlaneCount=originalPlaneCount;
output.RetainedPlaneCount=activePlaneCount;
if fixedClock && ~isempty(x), output.MaximumClearanceSlack_units = max(x(slackIndices)); end
% A stalled finite iterate remains a proposal, never a feasibility certificate.
% Independent physical checks decide whether it can become returned motion.
if (exitFlag <= 0 && exitFlag ~= -7) || isempty(x) || any(~isfinite(x))
    controlPoint_units = zeros(0, degree + 1, 2);
    segmentTime_s    = NaN;
    return;
end
segmentTime_s    = max(x(powerIndex(4)), 0) ^ (1 / 3) * segmentRatio;
if fixedClock, segmentTime_s=physicalTimes_s; end
controlPoint_units = permute(reshape(x(1:controlCount), 2, degree + 1, segmentCount), [3 2 1]);
end

%% Section 4: Local Functions
function [x,value,exitFlag,output] = solveConic(f,cones,A,b,Aeq,beq,lb,ub,options,prescribedAxis,phaseTimes_s,limits)
    transform=[]; center=[]; objectiveOffset=0;
    if ~isempty(phaseTimes_s)
        % Use local position, velocity, acceleration and quadratic jerk as
        % unknowns, avoiding high-order differences of absolute positions.
        count=numel(phaseTimes_s); variableCount=numel(f);
        transform=speye(variableCount); center=zeros(variableCount,1);
        jerkMap=sparse(6*count,variableCount);
        domain=[limits.xInterval_units;limits.yInterval_units]; origin=mean(domain,2);
        % Exact unit-duration integration template; only physical powers of
        % time change between phases. Columns are p, v, a, j0, j1, j2.
        unitBasis=[1,0,0,0,0,0;1,1/5,0,0,0,0;1,2/5,1/20,0,0,0; ...
            1,3/5,3/20,1/60,0,0;1,4/5,3/10,1/20,1/60,0; ...
            1,1,1/2,1/10,1/20,1/60];
        for span=1:count
            h=phaseTimes_s(span); basis=unitBasis.*[1,h,h^2,h^3,h^3,h^3];
            for axis=1:2
                columns=((span-1)*6+(0:5))*2+axis;
                transform(columns,columns)=basis; center(columns)=origin(axis);
            end
            jerkMap((span-1)*6+(1:6),(span-1)*12+(7:12))=speye(6);
        end
        b=b-A*center; A=A*transform;
        beq=beq-Aeq*center; Aeq=Aeq*transform;
        for k=1:numel(cones)
            cone=cones(k);
            cones(k)=secondordercone(cone.A*transform,cone.b-cone.A*center, ...
                transform.'*cone.d,cone.gamma-cone.d.'*center);
        end
        cones(end-count+1:end)=bmtpEngine.createVariationCone(jerkMap,phaseTimes_s,limits,variableCount-count+(1:count));
        objectiveOffset=f.'*center; f=transform.'*f;
        fixed=find(lb==ub); upperRows=find(isfinite(ub) & lb~=ub); lowerRows=find(isfinite(lb) & lb~=ub);
        A=[A;transform(upperRows,:);-transform(lowerRows,:)];
        b=[b;ub(upperRows)-center(upperRows);center(lowerRows)-lb(lowerRows)];
        assert(nnz(transform(fixed,setdiff(1:variableCount,fixed)))==0);
        fixedValues=transform(fixed,fixed)\(lb(fixed)-center(fixed));
        lb(:)=-Inf; ub(:)=Inf; lb(fixed)=fixedValues; ub(fixed)=fixedValues;
    end
    % Eliminate prescribed variables exactly. Leaving a complete analytic
    % axis as equal bounds produces redundant, poorly scaled solver rows.
    fixed = find(lb==ub & isfinite(lb));
    if (isempty(phaseTimes_s) && ~prescribedAxis) || isempty(fixed)
        [x,value,exitFlag,output] = coneprog(f,cones,A,b,Aeq,beq,lb,ub,options);
        return;
    end
    free = find(lb~=ub);
    fixedValues = lb(fixed);
    b = b-A(:,fixed)*fixedValues;
    beq = beq-Aeq(:,fixed)*fixedValues;
    A = A(:,free); Aeq = Aeq(:,free);
    % Constant rows are checked at the existing conic tolerance; the complete
    % physical motion is still subject to the unchanged independent validator.
    keep = any(A~=0,2) | b < -options.ConstraintTolerance;
    A = A(keep,:); b = b(keep);
    keep = any(Aeq~=0,2) | abs(beq)>options.ConstraintTolerance;
    Aeq = Aeq(keep,:); beq = beq(keep);
    if ~isempty(phaseTimes_s)
        scale=max(max(abs(A),[],2),1e-20); A=A./scale; b=b./scale;
        scale=max(max(abs(Aeq),[],2),1e-20); Aeq=Aeq./scale; beq=beq./scale;
    end
    for k = 1:numel(cones)
        cone = cones(k);
        cones(k) = secondordercone(cone.A(:,free),cone.b-cone.A(:,fixed)*fixedValues, ...
            cone.d(free),cone.gamma-cone.d(fixed).'*fixedValues);
    end
    [reduced,value,exitFlag,output] = coneprog(f(free),cones,A,b,Aeq,beq,lb(free),ub(free),options);
    x = [];
    if ~isempty(reduced)
        x = zeros(size(f)); x(fixed) = fixedValues; x(free) = reduced;
        value = value+f(fixed).'*fixedValues;
        if ~isempty(transform), x=center+transform*x; value=value+objectiveOffset; end
    end
end

function index = controlIndexOf(segmentIndex, controlIndex, axisIndex, degree)
    % Map trajectory controls into the conic decision vector.
    index = ((segmentIndex - 1) * (degree + 1) + controlIndex) * 2 + axisIndex;
end
