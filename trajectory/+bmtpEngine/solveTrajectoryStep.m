function [controlPoint_units, segmentTime_s, exitFlag, output] = solveTrajectoryStep(segmentCount, degree, start_units, goal_units, limits, planes, reserve_units, maximumMotionDuration_s, options, segmentRatio, fixedClock, fixedControl_units)
%% Section 0: Header & Readme
% SYNTAX
%   [controlPoint_units, segmentTime_s, exitFlag, output] = ...
%       bmtpEngine.solveTrajectoryStep( ...
%       segmentCount, degree, start_units, goal_units, limits, planes, ...
%       reserve_units, maximumMotionDuration_s, options)
%
% PURPOSE
%   - Solve one convex trajectory step for fixed separating lines, timing
%     policy, and derivative limits.
%
% INPUTS
%   - segmentCount, degree (positive integer scalars)
%       Composite Bezier representation size.
%   - start_units, goal_units (1-by-2 numeric rows)
%       Fixed endpoint positions.
%   - limits (scalar struct)
%       Workspace, velocity, acceleration, and jerk limits.
%   - planes (S-by-R struct array)
%       Fixed active separating-line constraints.
%   - reserve_units (nonnegative scalar)
%       Numerical separation reserve.
%   - maximumMotionDuration_s (positive scalar)
%       Upper bound on the internal minimum-time solve.
%   - options (coneprog options)
%       Numerical solver controls.
%
% OUTPUTS
%   - controlPoint_units (S-by-(D+1)-by-2 numeric array)
%       Solved control points, or an empty array on expected solve failure.
%   - segmentTime_s (scalar numeric)
%       Per-segment durations, or NaN on expected solve failure.
%   - exitFlag (numeric scalar), output (solver record)
%       Original coneprog status and measured solver time.
%
% UNITS
%   - Position is coordinate units and time is seconds.
%

%% Section 1: Create Decision Bounds And Continuity Rows

controlCount           = segmentCount * (degree + 1) * 2;
if nargin < 10, segmentRatio = ones(segmentCount, 1); end
if nargin < 11, fixedClock = false; end
prescribedAxis = nargin>=12 && ~isempty(fixedControl_units) && any(isfinite(fixedControl_units(:)));
powerIndex             = controlCount + (1:4);
lengthCount = segmentCount * degree;
activePlaneCount       = nnz(reshape([planes.Active], size(planes)));
slackCount = fixedClock * activePlaneCount;
variableCount          = controlCount + 4 + lengthCount + slackCount;
differenceCoefficients = {1, [-1 1], [1 -2 1], [-1 3 -3 1]};
baseInequalityCount    = 4 * segmentCount * (3 * degree - 3);
inequalityCount        = baseInequalityCount + activePlaneCount * (degree + 2);
equalityCount          = 13 + 6 * (segmentCount - 1);
A                      = spalloc(inequalityCount, variableCount, 6 * inequalityCount);
Aeq                    = spalloc(equalityCount, variableCount, 8 * equalityCount);
beq                    = zeros(equalityCount, 1);
lb                     = -Inf(variableCount, 1);
ub                     = Inf(variableCount, 1);
domain_units             = [limits.xInterval_units; limits.yInterval_units];
lb(1:controlCount) = repmat(domain_units(:, 1), segmentCount * (degree + 1), 1);
ub(1:controlCount) = repmat(domain_units(:, 2), segmentCount * (degree + 1), 1);
if nargin>=12 && ~isempty(fixedControl_units)
    fixedValues = reshape(permute(fixedControl_units,[3,2,1]),[],1);
    indices = find(isfinite(fixedValues));
    lb(indices) = fixedValues(indices);
    ub(indices) = fixedValues(indices);
end
lb(powerIndex) = 0;
lb(powerIndex(2)) = eps;
equalityIndex = 0;
% Evaluate each coordinate axis and combine its limiting result.
for axisIndex = 1:2
    equalityIndex = equalityIndex + 1;
    Aeq(equalityIndex, ...
        controlIndexOf(1, 0, axisIndex, degree)) = 1; %#ok<SPRIX>
    beq(equalityIndex) = start_units(axisIndex);
    equalityIndex = equalityIndex + 1;
    Aeq(equalityIndex, controlIndexOf(segmentCount, degree, axisIndex, degree)) = 1; %#ok<SPRIX>
    beq(equalityIndex) = goal_units(axisIndex);
    % Process each endpoint order needed to find trajectory step.
    for endpointOrder = 1:2
        equalityIndex = equalityIndex + 1;
        indices       = controlIndexOf(1, [endpointOrder 0], axisIndex, degree);
        Aeq(equalityIndex, indices) = [1 -1]; %#ok<SPRIX>
        equalityIndex = equalityIndex + 1;
        indices       = controlIndexOf(segmentCount, [degree - endpointOrder degree], axisIndex, degree);
        Aeq(equalityIndex, indices) = [1 -1]; %#ok<SPRIX>
    end
end
% Process each segment while assembling the complete motion or interval result.
for segmentIndex = 1:segmentCount - 1
    % Process each order needed to find trajectory step.
    for order = 0:2
        coefficients     = differenceCoefficients{order + 1};
        coefficientIndex = 0:order;
        % Evaluate each coordinate axis and combine its limiting result.
        for axisIndex = 1:2
            equalityIndex = equalityIndex + 1;
            left          = controlIndexOf(segmentIndex, degree - order + coefficientIndex, axisIndex, degree);
            right         = controlIndexOf(segmentIndex + 1, coefficientIndex, axisIndex, degree);
            rowScale = max(segmentRatio(segmentIndex:segmentIndex+1))^order;
            Aeq(equalityIndex, left) = coefficients * segmentRatio(segmentIndex+1)^order / rowScale; %#ok<SPRIX>
            Aeq(equalityIndex, right) = ...
                Aeq(equalityIndex, right) - coefficients * segmentRatio(segmentIndex)^order / rowScale; %#ok<SPRIX>
        end
    end
end
equalityIndex = equalityIndex + 1;
Aeq(equalityIndex, powerIndex(1)) = 1;
beq(equalityIndex) = 1;

%% Section 2: Create Derivative And Separating-Line Bounds

limitValues = [limits.maxVelocity_units_s; ...
    limits.maxAcceleration_units_s2; limits.maxJerk_units_s3];
inequalityIndex = 0;
% Process each segment while assembling the complete motion or interval result.
for segmentIndex = 1:segmentCount
    controlColumns = (segmentIndex - 1) * 2 * (degree + 1) + (1:2 * (degree + 1));
    % Process each order needed to find trajectory step.
    for order = 1:3
        coefficients    = differenceCoefficients{order + 1};
        scale           = factorial(degree) / factorial(degree - order);
        derivativeCount = degree - order + 1;
        derivativeRows  = spdiags(repmat(scale * coefficients, derivativeCount, 1), 0:order, derivativeCount, degree + 1);
        signedRows      = kron(kron(derivativeRows, speye(2)), [1; -1]);
        targets         = inequalityIndex + (1:size(signedRows, 1));
        A(targets, controlColumns) = signedRows; %#ok<SPRIX>
        axisLimits = repmat(limitValues(order, :), derivativeCount, 1);
        A(targets, powerIndex(order + 1)) = ...
            -repelem(reshape(axisLimits.', [], 1), 2) * segmentRatio(segmentIndex)^order; %#ok<SPRIX>
        inequalityIndex = targets(end);
    end
end
b               = zeros(inequalityCount, 1);
inequalityIndex = baseInequalityCount;
slackIndex = controlCount+4+lengthCount;
% Process each segment while assembling the complete motion or interval result.
for segmentIndex = 1:segmentCount
    % Process each geometric region while constructing or checking the region topology.
    for regionIndex = 1:size(planes, 2)
        plane = planes(segmentIndex, regionIndex);
        if ~plane.Active
            continue;
        end
        [rows, offset_units] = fixedPlaneRows(plane, degree, variableCount, segmentIndex);
        targets = inequalityIndex + (1:size(rows, 1));
        A(targets, :) = rows; %#ok<SPRIX>
        if fixedClock
            slackIndex = slackIndex+1;
            A(targets,slackIndex) = -1;
        end
        b(targets) = -(1+fixedClock)*reserve_units - offset_units;
        inequalityIndex = targets(end);
    end
end

%% Section 3: Create The Objective And Solve

cones = createTimePowerCones(variableCount, powerIndex);
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
if fixedClock, cones = lengthCones; end
solverTimer = tic;
[x, ~, exitFlag, output] = solveConic(f, cones, A, b, Aeq, beq, lb, ub, options, prescribedAxis);
output.TotalTime_s = toc(solverTimer);
output.SolveCount = 1;
output.OptimizationConverged = exitFlag>0;
if ~fixedClock && ~isempty(x) && all(isfinite(x)) && (exitFlag>0 || exitFlag==-7)
    % Lexicographic optimization: preserve the first time-power value while
    % minimizing path length. Unconstrained lateral motion must not be chosen
    % arbitrarily merely because another axis determines the arrival time.
    ub(powerIndex(4)) = x(powerIndex(4));
    f(:) = 0; f(lengthIndex) = 1;
    timer = tic;
    [shortX,~,shortFlag,shortOutput] = solveConic(f,[cones;lengthCones],A,b,Aeq,beq,lb,ub,options,prescribedAxis);
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
if fixedClock && ~isempty(x), output.MaximumClearanceSlack_units = max(x(slackIndices)); end
% A stalled finite iterate remains a proposal, never a feasibility certificate.
% Independent physical checks decide whether it can become returned motion.
if (exitFlag <= 0 && exitFlag ~= -7) || isempty(x) || any(~isfinite(x))
    controlPoint_units = zeros(0, degree + 1, 2);
    segmentTime_s    = NaN;
    return;
end
segmentTime_s    = max(x(powerIndex(4)), 0) ^ (1 / 3) * segmentRatio;
controlPoint_units = permute(reshape(x(1:controlCount), 2, degree + 1, segmentCount), [3 2 1]);
end

%% Section 4: Local Functions

function [x,value,exitFlag,output] = solveConic(f,cones,A,b,Aeq,beq,lb,ub,options,prescribedAxis)
    % Eliminate prescribed variables exactly. Leaving a complete analytic
    % axis as equal bounds produces redundant, poorly scaled solver rows.
    fixed = find(lb==ub & isfinite(lb));
    if ~prescribedAxis || isempty(fixed)
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
    end
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

function index = controlIndexOf(segmentIndex, controlIndex, axisIndex, degree)
    % Map trajectory controls into the conic decision vector.
    index = ((segmentIndex - 1) * (degree + 1) + controlIndex) * 2 + axisIndex;
end
