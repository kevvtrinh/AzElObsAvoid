function [controlPoint_units, segmentTime_s, exitFlag, output, constraintBase] = ...
        solveTrajectoryStep(request, step)
%% Section 0: Header & Readme
% SYNTAX
%   [controlPoint_units, segmentTime_s, exitFlag, output, constraintBase] = ...
%       bmtpEngine.optimization.solveTrajectoryStep(request, step)
%**************************************************************************
% PURPOSE
%   - Solve one convex trajectory step for fixed separating lines, timing
%     policy, and derivative limits.
%**************************************************************************
% INPUTS
%   - request (scalar struct)
%       The engine solve request from createSolveRequest. This step reads
%       Degree, InitialState, GoalState, Limits, and TrajectoryOptions.
%   - step (scalar struct)
%       What this one step is asked to do, every field required:
%       SegmentCount (positive integer), Planes (S-by-R struct array of
%       fixed separating lines whose TimeFraction scopes each active plane
%       to a closed part of the span), RoundoffReserve_units (nonnegative
%       scalar), MaximumMotionDuration_s (positive scalar upper bound on the
%       internal minimum-time solve), SegmentRatio (S-by-1 positive relative
%       segment durations), FixedClock (logical: prescribe the segment
%       clock), IntrinsicVariationEnabled (logical: surplus fixed-clock
%       spans use the integrated-snap tie-break), and ConstraintBase (the
%       reusable invariant constraint arrays from an earlier step with the
%       same formulation, or struct() to build them).
%**************************************************************************
% OUTPUTS
%   - controlPoint_units (S-by-(D+1)-by-2 numeric array)
%       Solved control points, or an empty array on expected solve failure.
%       Invalid input throws an error.
%   - segmentTime_s (S-by-1 numeric vector)
%       Per-segment durations, or NaN on expected solve failure.
%   - exitFlag (numeric scalar)
%       Original coneprog status.
%   - output (scalar struct)
%       Solver status, diagnostics, and measured solver time.
%   - constraintBase (scalar struct)
%       Invariant constraint arrays for reuse with the same formulation.
%**************************************************************************
% UNITS
%   - Position is coordinate units and time is seconds.
%**************************************************************************

%% Section 1: Read The Step And The Request, Then Create Decision Bounds
validateattributes(step, {'struct'}, {'scalar'});
stepFields = ["SegmentCount", "Planes", "RoundoffReserve_units", "MaximumMotionDuration_s", ...
    "SegmentRatio", "FixedClock", "IntrinsicVariationEnabled", "ConstraintBase"];
assert(all(isfield(step, stepFields)), 'bmtpEngine:InvalidStep', ...
    'A trajectory step declares every one of: %s.', strjoin(stepFields, ', '));
segmentCount              = step.SegmentCount;
planes                    = step.Planes;
roundoffReserve_units     = step.RoundoffReserve_units;
maximumMotionDuration_s   = step.MaximumMotionDuration_s;
segmentRatio              = step.SegmentRatio;
fixedClock                = step.FixedClock;
intrinsicVariationEnabled = step.IntrinsicVariationEnabled;
constraintBase            = step.ConstraintBase;
degree       = request.Degree;
initialState = request.InitialState;
goalState    = request.GoalState;
limits       = request.Limits;
options      = request.TrajectoryOptions;
controlCount = segmentCount * (degree + 1) * 2;
originalPlaneCount = nnz([planes.Active]);
partialPlanes      = false;
if ~isempty(planes)
    fractions = reshape([planes.TimeFraction], 2, []).';
    validateattributes(fractions, {'numeric'}, ...
        {'real', 'finite', 'ncols', 2, '>=', 0, '<=', 1});
    assert(all(fractions(:, 1) < fractions(:, 2)), ...
        'bmtpEngine:InvalidPlaneTimeScope', ...
        'Every plane time scope must have positive duration.');
    partialPlanes = any(fractions ~= [0, 1], 'all');
    assert(fixedClock || ~partialPlanes, ...
        'bmtpEngine:InvalidPlaneTimeScope', ...
        'Partial plane time scopes require a fixed trajectory clock.');
end
% Half-spaces on different physical intervals cannot eliminate each other.
if fixedClock && ~partialPlanes && originalPlaneCount > segmentCount * degree
    planes = bmtpEngine.separation.removeRedundantPlanes(planes, limits, 2 * roundoffReserve_units);
end
% Larger clocks have surplus phases that can oscillate under length alone.
% Preserve the compact eight-span steering solve used on sparse clocks.
intrinsicVariation = intrinsicVariationEnabled && fixedClock && ...
    degree == 5 && segmentCount > 8;
start_units        = initialState.position_units;
goal_units         = goalState.position_units;
endpointDerivatives = [initialState.velocity_units_s, initialState.acceleration_units_s2, ...
    goalState.velocity_units_s, goalState.acceleration_units_s2];
isRest = all(endpointDerivatives == 0);
assert(fixedClock || isRest, 'bmtpEngine:NonrestRelaxedClock', ...
    'Nonzero boundary states require physical fixed durations.');
boundaryControls = bmtpEngine.motion.imposeEndpointControls( ...
    zeros(segmentCount, degree + 1, 2), ...
    maximumMotionDuration_s * segmentRatio / sum(segmentRatio), initialState, goalState);
powerIndex = controlCount + (1:4);
lengthCount = segmentCount * degree;
planeActiveBySegment = reshape([planes.Active], size(planes));
activePlaneCount     = nnz(planeActiveBySegment);
planeCountBySegment  = sum(planeActiveBySegment, 2);
% Share elastic variables only when they dominate the length-cone variables.
% The weighted maximum penalizes every retained plane; zero slack recovers
% the same hard corridor, which is independently checked before acceptance.
sharedSlack = fixedClock && originalPlaneCount > lengthCount;
slackCount  = fixedClock * activePlaneCount;
if sharedSlack
    slackCount = nnz(planeCountBySegment);
end
variableCount = controlCount + 4 + lengthCount + slackCount + ...
    intrinsicVariation;
slackColumnByPair = zeros(size(planeActiveBySegment));
if fixedClock && sharedSlack
    segmentSlackColumn = controlCount + 4 + lengthCount + cumsum(planeCountBySegment > 0);
    for segmentIndex = reshape(find(planeCountBySegment > 0), 1, [])
        slackColumnByPair(segmentIndex, planeActiveBySegment(segmentIndex, :)) = ...
            segmentSlackColumn(segmentIndex);
    end
elseif fixedClock
    nextSlackColumn = controlCount + 4 + lengthCount;
    for segmentIndex = 1:segmentCount
        for regionIndex = reshape(find(planeActiveBySegment(segmentIndex, :)), 1, [])
            nextSlackColumn = nextSlackColumn + 1;
            slackColumnByPair(segmentIndex, regionIndex) = nextSlackColumn;
        end
    end
end
physicalTimes_s = maximumMotionDuration_s * segmentRatio / sum(segmentRatio);
jerkTimes_s = [];
if intrinsicVariation
    jerkTimes_s = physicalTimes_s;
end
constraintLimits = limits;
% A finite stalled cone iterate can carry a constraint residual larger than
% floating-point roundoff. Keep jerk controls strictly inside their physical
% bounds so exact endpoint reconstruction remains provable.
if ~intrinsicVariation
    constraintLimits.maxJerk_units_s3 = ...
        limits.maxJerk_units_s3 .* (1 - sqrt(eps));
end
initialPlanePairs = planeActiveBySegment;
if fixedClock
    initialPlanePairs(:) = false;
end
constraintKey = struct( ...
    "SegmentCount",     segmentCount, ...
    "Degree",           degree, ...
    "BoundaryControls", boundaryControls, ...
    "Limits",           constraintLimits, ...
    "VariableCount",    variableCount, ...
    "SegmentRatio",     segmentRatio, ...
    "JerkTimes_s",      jerkTimes_s);
canReuseConstraintBase = isstruct(constraintBase) && isscalar(constraintBase) && ...
    isfield(constraintBase, 'Key') && isequaln(constraintBase.Key, constraintKey);
if ~canReuseConstraintBase
    [A, Aeq, beq, lb, ub, jerkMap] = bmtpEngine.optimization.createTrajectoryConstraints( ...
        segmentCount, degree, boundaryControls, constraintLimits, variableCount, ...
        segmentRatio, jerkTimes_s);
    constraintBase = struct("A", A, "Aeq", Aeq, "beq", beq, ...
        "lb", lb, "ub", ub, "jerkMap", jerkMap, "Key", constraintKey);
else
    A       = constraintBase.A;
    Aeq     = constraintBase.Aeq;
    beq     = constraintBase.beq;
    lb      = constraintBase.lb;
    ub      = constraintBase.ub;
    jerkMap = constraintBase.jerkMap;
end
% Fix endpoint position, velocity, and acceleration controls in the solver's
% own variable space. Leaving them as approximate equality rows allows a
% stalled finite iterate to satisfy derivative bounds before exact endpoint
% reconstruction changes its terminal jerk.
if ~intrinsicVariation
    exactControl_units = NaN(segmentCount, degree + 1, 2);
    exactControl_units(1, 1:3, :)           = boundaryControls(1, 1:3, :);
    exactControl_units(end, end - 2:end, :) = boundaryControls(end, end - 2:end, :);
    fixedValues         = reshape(permute(exactControl_units, [3, 2, 1]), [], 1);
    fixedControlIndices = find(isfinite(fixedValues));
    lb(fixedControlIndices) = fixedValues(fixedControlIndices);
    ub(fixedControlIndices) = fixedValues(fixedControlIndices);
end

%% Section 2: Add Separating-Line Bounds
baseInequalityCount = 4 * segmentCount * (3 * degree - 3);
b = zeros(baseInequalityCount, 1);
[planeRows, planeBounds] = bmtpEngine.separation.createSelectedPlaneRows(planes, ...
    initialPlanePairs, degree, variableCount, slackColumnByPair, ...
    (1 + fixedClock) * roundoffReserve_units);
A = [A; planeRows];
b = [b; planeBounds];

%% Section 3: Create The Objective And Solve
f                    = zeros(variableCount, 1);
f(powerIndex(4))     = 1;
maximumSegmentTime_s = maximumMotionDuration_s / sum(segmentRatio);
timePowers_s         = [1; maximumSegmentTime_s; ...
    maximumSegmentTime_s ^ 2; maximumSegmentTime_s ^ 3];
ub(powerIndex) = timePowers_s;
lengthIndex = controlCount + 4 + (1:lengthCount);
lb(lengthIndex) = 0;
if fixedClock
    lb(powerIndex) = timePowers_s;
    f(:) = 0;
    f(lengthIndex) = 1;
    slackIndices = controlCount + 4 + lengthCount + (1:slackCount);
    lb(slackIndices) = 0;
    % Elastic sequential convex programming: penalize clearance slack in the
    % same distance units as control-polygon length. Only independently clear
    % motion may be accepted by the outer solve.
    f(slackIndices) = 1e3;
    if sharedSlack
        f(slackIndices) = 1e3 * planeCountBySegment(planeCountBySegment > 0);
    end
    emptyCone = secondordercone( ...
        sparse(2, variableCount), zeros(2, 1), sparse(variableCount, 1), 0);
    lengthCones = repmat(emptyCone, lengthCount, 1);
    for segmentIndex = 1:segmentCount
        for controlIndex = 1:degree
            lengthConeIndex = (segmentIndex - 1) * degree + controlIndex;
            coneA = sparse(2, variableCount);
            coneA(:, bmtpEngine.optimization.controlIndexOf( ...
                segmentIndex, controlIndex, 1:2, degree)) = eye(2);
            coneA(:, bmtpEngine.optimization.controlIndexOf( ...
                segmentIndex, controlIndex - 1, 1:2, degree)) = -eye(2);
            coneD = sparse(variableCount, 1);
            coneD(lengthIndex(lengthConeIndex)) = 1;
            lengthCones(lengthConeIndex) = secondordercone( ...
                coneA, zeros(2, 1), coneD, 0);
        end
    end
    cones = lengthCones;
    if intrinsicVariation
        smoothIndex = variableCount;
        cones = [cones; bmtpEngine.optimization.createVariationCone( ...
            jerkMap, physicalTimes_s, limits, smoothIndex)];
        lb(smoothIndex) = 0;
        f(smoothIndex)  = 0.005 * norm(goal_units - start_units);
    end
else
    cones = bmtpEngine.optimization.createTimePowerCones(variableCount, powerIndex);
end
solverTimer = tic;
solverTimes_s = [];
if intrinsicVariation
    solverTimes_s = physicalTimes_s;
end
% The conic program is one product; constraint generation grows its
% inequality rows between solves.
program = struct('f', f, 'cones', cones, 'A', A, 'b', b, 'Aeq', Aeq, 'beq', beq, ...
    'lb', lb, 'ub', ub);
retainedPlanePairs = initialPlanePairs;
solveCount         = 0;
constraintGenerationComplete   = ~fixedClock;
maximumPlaneConstraintResidual = NaN;
while true
    [x, exitFlag, output] = solveConic(program, options, ~intrinsicVariation, ...
        solverTimes_s, limits);
    solveCount = solveCount + 1;
    if ~fixedClock || ~bmtpEngine.optimization.hasUsableConicIterate(x, exitFlag)
        break
    end
    [violatedPairs, maximumOmittedResidual] = bmtpEngine.separation.findViolatedPlanePairs( ...
        x, planes, planeActiveBySegment, ...
        retainedPlanePairs, degree, slackColumnByPair, 2 * roundoffReserve_units, ...
        options.ConstraintTolerance);
    if ~any(violatedPairs, 'all')
        loadedResidual = -Inf;
        if size(program.A, 1) > baseInequalityCount
            loadedResidual = max(program.A(baseInequalityCount + 1:end, :) * x - ...
                program.b(baseInequalityCount + 1:end));
        end
        maximumPlaneConstraintResidual = max(loadedResidual, maximumOmittedResidual);
        constraintGenerationComplete = ...
            maximumPlaneConstraintResidual <= options.ConstraintTolerance;
        break
    end
    retainedPlanePairs = retainedPlanePairs | violatedPairs;
    [newRows, newBounds] = bmtpEngine.separation.createSelectedPlaneRows(planes, ...
        violatedPairs, degree, variableCount, slackColumnByPair, 2 * roundoffReserve_units);
    program.A = [program.A; newRows];
    program.b = [program.b; newBounds];
end
output.TotalTime_s                    = toc(solverTimer);
output.SolveCount                     = solveCount;
output.OptimizationConverged          = exitFlag > 0;
output.OriginalPlaneCount             = originalPlaneCount;
output.RetainedPlaneCount             = activePlaneCount;
output.LoadedPlanePairCount           = nnz(retainedPlanePairs);
output.ConstraintGenerationApplied    = fixedClock;
output.ConstraintGenerationRoundCount = max(0, solveCount - 1);
output.ConstraintGenerationComplete   = constraintGenerationComplete;
output.MaximumPlaneConstraintResidual = maximumPlaneConstraintResidual;
output.IntrinsicJerkVariation          = intrinsicVariation;
if fixedClock && ~isempty(x)
    output.MaximumClearanceSlack_units = max(x(slackIndices));
end
% A stalled finite iterate remains a proposal, never a feasibility proof.
% Independent physical checks decide whether it can become returned motion.
if ~bmtpEngine.optimization.hasUsableConicIterate(x, exitFlag)
    controlPoint_units = zeros(0, degree + 1, 2);
    segmentTime_s      = NaN;
    return
end
segmentTime_s = max(x(powerIndex(4)), 0) ^ (1 / 3) * segmentRatio;
if fixedClock
    segmentTime_s = physicalTimes_s;
end
controlPoint_units = permute(reshape(x(1:controlCount), 2, degree + 1, segmentCount), [3 2 1]);
end

%% Section 4: Local Functions
function [x, exitFlag, output] = solveConic(program, options, givenAxis, phaseTimes_s, limits)
    % Solve one conic program (f, cones, A, b, Aeq, beq, lb, ub) in a locally
    % conditioned variable space when physical phase times are given, then
    % restore the original decision vector.
    f     = program.f;
    cones = program.cones;
    A     = program.A;
    b     = program.b;
    Aeq   = program.Aeq;
    beq   = program.beq;
    lb    = program.lb;
    ub    = program.ub;
    transform = [];
    center    = [];
    if ~isempty(phaseTimes_s)
        % Use local position, velocity, acceleration and quadratic jerk as
        % unknowns, avoiding high-order differences of absolute positions.
        phaseCount    = numel(phaseTimes_s);
        variableCount = numel(f);
        transform     = speye(variableCount);
        center        = zeros(variableCount, 1);
        jerkMap       = sparse(6 * phaseCount, variableCount);
        domain_units  = [limits.xInterval_units; limits.yInterval_units];
        origin_units  = mean(domain_units, 2);
        % Exact unit-duration integration template; only physical powers of
        % time change between phases. Columns are p, v, a, j0, j1, j2.
        unitBasis = [1, 0, 0, 0, 0, 0; 1, 1 / 5, 0, 0, 0, 0; ...
            1, 2 / 5, 1 / 20, 0, 0, 0; ...
            1, 3 / 5, 3 / 20, 1 / 60, 0, 0; ...
            1, 4 / 5, 3 / 10, 1 / 20, 1 / 60, 0; ...
            1, 1, 1 / 2, 1 / 10, 1 / 20, 1 / 60];
        for phaseIndex = 1:phaseCount
            phaseTime_s = phaseTimes_s(phaseIndex);
            basis = unitBasis .* [1, phaseTime_s, phaseTime_s ^ 2, ...
                phaseTime_s ^ 3, phaseTime_s ^ 3, phaseTime_s ^ 3];
            for axisIndex = 1:2
                columns = ((phaseIndex - 1) * 6 + (0:5)) * 2 + axisIndex;
                transform(columns, columns) = basis;
                center(columns)             = origin_units(axisIndex);
            end
            jerkRows    = (phaseIndex - 1) * 6 + (1:6);
            jerkColumns = (phaseIndex - 1) * 12 + (7:12);
            jerkMap(jerkRows, jerkColumns) = speye(6);
        end
        b = b - A * center;
        A = A * transform;
        beq = beq - Aeq * center;
        Aeq = Aeq * transform;
        for coneIndex = 1:numel(cones)
            cone = cones(coneIndex);
            cones(coneIndex) = secondordercone( ...
                cone.A * transform, cone.b - cone.A * center, ...
                transform.' * cone.d, cone.gamma - cone.d.' * center);
        end
        cones(end) = bmtpEngine.optimization.createVariationCone( ...
            jerkMap, phaseTimes_s, limits, variableCount);
        f = transform.' * f;
        fixedIndices    = find(lb == ub);
        upperRowIndices = find(isfinite(ub) & lb ~= ub);
        lowerRowIndices = find(isfinite(lb) & lb ~= ub);
        A = [A; transform(upperRowIndices, :); -transform(lowerRowIndices, :)];
        b = [b; ub(upperRowIndices) - center(upperRowIndices); ...
            center(lowerRowIndices) - lb(lowerRowIndices)];
        freeTransformIndices = setdiff(1:variableCount, fixedIndices);
        assert(nnz(transform(fixedIndices, freeTransformIndices)) == 0);
        fixedValues = transform(fixedIndices, fixedIndices) \ ...
            (lb(fixedIndices) - center(fixedIndices));
        lb(:) = -Inf;
        ub(:) = Inf;
        lb(fixedIndices) = fixedValues;
        ub(fixedIndices) = fixedValues;
    end
    % Eliminate given variables exactly. Leaving a complete analytic
    % axis as equal bounds produces redundant, poorly scaled solver rows.
    fixedIndices = find(lb == ub & isfinite(lb));
    if (isempty(phaseTimes_s) && ~givenAxis) || isempty(fixedIndices)
        [x, ~, exitFlag, output] = coneprog(f, cones, A, b, Aeq, beq, lb, ub, options);
        return
    end
    freeIndices = find(lb ~= ub);
    fixedValues = lb(fixedIndices);
    b   = b - A(:, fixedIndices) * fixedValues;
    beq = beq - Aeq(:, fixedIndices) * fixedValues;
    A   = A(:, freeIndices);
    Aeq = Aeq(:, freeIndices);
    % Constant rows are checked at the existing conic tolerance; the complete
    % physical motion is still subject to the unchanged independent validator.
    keepRows = any(A ~= 0, 2) | b < -options.ConstraintTolerance;
    A = A(keepRows, :);
    b = b(keepRows);
    keepRows = any(Aeq ~= 0, 2) | abs(beq) > options.ConstraintTolerance;
    Aeq = Aeq(keepRows, :);
    beq = beq(keepRows);
    if ~isempty(phaseTimes_s)
        inequalityScale = max(max(abs(A), [], 2), 1e-20);
        A = A ./ inequalityScale;
        b = b ./ inequalityScale;
        equalityScale = max(max(abs(Aeq), [], 2), 1e-20);
        Aeq = Aeq ./ equalityScale;
        beq = beq ./ equalityScale;
    end
    for coneIndex = 1:numel(cones)
        cone = cones(coneIndex);
        cones(coneIndex) = secondordercone( ...
            cone.A(:, freeIndices), cone.b - cone.A(:, fixedIndices) * fixedValues, ...
            cone.d(freeIndices), cone.gamma - cone.d(fixedIndices).' * fixedValues);
    end
    [reduced, ~, exitFlag, output] = coneprog( ...
        f(freeIndices), cones, A, b, Aeq, beq, ...
        lb(freeIndices), ub(freeIndices), options);
    x = [];
    if ~isempty(reduced)
        x = zeros(size(f));
        x(fixedIndices) = fixedValues;
        x(freeIndices)  = reduced;
        if ~isempty(transform)
            x = center + transform * x;
        end
    end
end
