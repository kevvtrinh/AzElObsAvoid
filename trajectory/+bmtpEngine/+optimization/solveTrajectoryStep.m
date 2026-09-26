function [controlPoint_units, segmentTime_s, exitFlag, solverOutput, savedTrajectoryConstraints] = ...
    solveTrajectoryStep(solverRequest, trajectoryStep)
%% Section 0: Header & Readme
% SYNTAX
%   [controlPoint_units, segmentTime_s, exitFlag, solverOutput, savedTrajectoryConstraints] = ...
%       bmtpEngine.optimization.solveTrajectoryStep(solverRequest, trajectoryStep)
%**************************************************************************
% PURPOSE
%   - Solve one BMTP trajectory step with the requested clock formulation.
%     The caller checks the complete returned motion before accepting it.
%**************************************************************************
% INPUTS
%   - solverRequest (scalar struct)
%       Checked states, limits, degree, and both coneprog option sets.
%   - trajectoryStep (scalar struct)
%       Formulation is "physicalClock" for physical time powers, endpoint
%       rates, clearance slack, and optional jerk smoothing. "scaledClock"
%       scales time powers by their maximum and keeps endpoints at rest.
%       Each retains its own line-loading and solve-failure rules.
%       SegmentCount is a positive integer. Planes is S-by-R, with each
%       TimeFraction selecting part of a segment. Other required fields are
%       RoundoffReserve_units (nonnegative), MaximumMotionDuration_s
%       (positive for variable clocks), MinimumMotionDuration_s (zero when
%       unused), SegmentRatio (S positive values for variable clocks, or []
%       for scaledClock's common time), FixedClock (logical),
%       IntrinsicVariationEnabled (logical, false for scaledClock), and
%       ConstraintBase (matching saved constraints or struct()). A fixed
%       clock also requires SegmentTime_s (S-by-1 positive physical times)
%       and empty MaximumMotionDuration_s and SegmentRatio.
%**************************************************************************
% OUTPUTS
%   - controlPoint_units (S-by-(D+1)-by-2 numeric array)
%       Solved control points, or an empty array on expected solve failure.
%       Invalid input throws an error.
%   - segmentTime_s (numeric scalar or S-element numeric array)
%       Segment durations. A fixed clock returns the supplied SegmentTime_s;
%       a variable scaledClock step with [] SegmentRatio returns one common
%       segment time. Expected solve failure returns NaN.
%   - exitFlag (numeric scalar)
%       Original coneprog status.
%   - solverOutput (scalar struct)
%       Solver status, diagnostics, and measured solver time.
%       FailureStage, FailureKind, and AlternativeGuideEligible are empty or false when the iterate is usable.
%   - savedTrajectoryConstraints (scalar struct)
%       Constraints and their inputs, for reuse when the next step matches.
%**************************************************************************
% UNITS
%   - Position is coordinate units and time is seconds.
%**************************************************************************

%% Section 1: Validate Inputs And Resolve The Formulation

validateattributes(trajectoryStep, {'struct'}, {'scalar'});
requiredStepFields = ["Formulation", "SegmentCount", "Planes", "RoundoffReserve_units", ...
    "MaximumMotionDuration_s", "MinimumMotionDuration_s", "SegmentRatio", "FixedClock", ...
    "IntrinsicVariationEnabled", "ConstraintBase"];
assert(all(isfield(trajectoryStep, requiredStepFields)), 'bmtpEngine:InvalidStep', ...
    'A trajectory step declares every one of: %s.', strjoin(requiredStepFields, ', '));
formulationName             = string(trajectoryStep.Formulation);
segmentCount                = trajectoryStep.SegmentCount;
separatingPlanes            = trajectoryStep.Planes;
roundoffReserve_units       = trajectoryStep.RoundoffReserve_units;
maximumMotionDuration_s     = trajectoryStep.MaximumMotionDuration_s;
minimumMotionDuration_s     = trajectoryStep.MinimumMotionDuration_s;
segmentTimeRatios           = trajectoryStep.SegmentRatio;
hasFixedSegmentTimes        = trajectoryStep.FixedClock;
allowJerkVariationObjective = trajectoryStep.IntrinsicVariationEnabled;
savedTrajectoryConstraints  = trajectoryStep.ConstraintBase;

assert(isscalar(formulationName) && any(formulationName == ["physicalClock", "scaledClock"]), ...
    'bmtpEngine:InvalidStep', 'Formulation must be physicalClock or scaledClock.');
validateattributes(segmentCount, {'numeric'}, {'real', 'finite', 'scalar', 'integer', 'positive'});
validateattributes(separatingPlanes, {'struct'}, {'2d'});
assert(size(separatingPlanes, 1) == segmentCount && isfield(separatingPlanes, 'Active'), ...
    'bmtpEngine:InvalidStep', 'Planes must have SegmentCount rows and an Active field.');
validateattributes(roundoffReserve_units, {'numeric'}, {'real', 'finite', 'scalar', 'nonnegative'});
validateattributes(hasFixedSegmentTimes, {'logical'}, {'scalar'});
validateattributes(allowJerkVariationObjective, {'logical'}, {'scalar'});
validateattributes(savedTrajectoryConstraints, {'struct'}, {'scalar'});
formulation = resolveFormulation(formulationName, solverRequest, hasFixedSegmentTimes);
assert(formulation.AllowJerkVariation || ~allowJerkVariationObjective, ...
    'bmtpEngine:InvalidStep', 'scaledClock cannot enable intrinsic jerk variation.');
if hasFixedSegmentTimes
    assert(isfield(trajectoryStep, 'SegmentTime_s'), 'bmtpEngine:InvalidStep', ...
        'A fixed clock requires SegmentTime_s.');
    assert(isempty(maximumMotionDuration_s) && isempty(segmentTimeRatios), ...
        'bmtpEngine:InvalidStep', ...
        'A fixed clock requires empty MaximumMotionDuration_s and SegmentRatio.');
    validateattributes(minimumMotionDuration_s, {'numeric'}, ...
        {'real', 'finite', 'scalar', 'nonnegative'});
    assert(minimumMotionDuration_s == 0, 'bmtpEngine:InvalidStep', ...
        'A fixed clock requires MinimumMotionDuration_s = 0.');
    segmentTimeRatios = trajectoryStep.SegmentTime_s;
    validateattributes(segmentTimeRatios, {'numeric'}, ...
        {'real', 'finite', 'positive', 'size', [segmentCount, 1]});
    returnsCommonSegmentTime = false;
else
    validateattributes(maximumMotionDuration_s, {'numeric'}, {'real', 'finite', 'scalar', 'positive'});
    validateattributes(minimumMotionDuration_s, {'numeric'}, ...
        {'real', 'finite', 'scalar', 'nonnegative', '<=', maximumMotionDuration_s});
    assert(formulation.AllowMinimumDuration || minimumMotionDuration_s == 0, ...
        'bmtpEngine:InvalidStep', 'physicalClock requires MinimumMotionDuration_s = 0.');
    returnsCommonSegmentTime = isempty(segmentTimeRatios);
    assert(~returnsCommonSegmentTime || formulation.AllowCommonSegmentTime, ...
        'bmtpEngine:InvalidStep', 'Only scaledClock accepts an empty SegmentRatio.');
    if returnsCommonSegmentTime
        segmentTimeRatios = ones(segmentCount, 1);
    end
    validateattributes(segmentTimeRatios, {'numeric'}, ...
        {'real', 'finite', 'positive', 'numel', segmentCount});
end
if formulation.ScaleTimePowers
    % The scaled formulation always used a column of doubles.
    segmentTimeRatios = double(segmentTimeRatios(:));
end

degree        = solverRequest.Degree;
initialState  = solverRequest.InitialState;
goalState     = solverRequest.GoalState;
limits        = solverRequest.Limits;
solverOptions = formulation.SolverOptions;

controlVariableCount   = segmentCount * (degree + 1) * 2;
originalPlaneCount     = nnz([separatingPlanes.Active]);
hasPartialSegmentLines = false;
if ~isempty(separatingPlanes)
    lineTimeFractions = reshape([separatingPlanes.TimeFraction], 2, []).';
    validateattributes(lineTimeFractions, {'numeric'}, ...
        {'real', 'finite', 'ncols', 2, '>=', 0, '<=', 1});
    assert(all(lineTimeFractions(:, 1) < lineTimeFractions(:, 2)), ...
        'bmtpEngine:InvalidPlaneTimeScope', ...
        'Every plane time scope must have positive duration.');
    hasPartialSegmentLines = any(lineTimeFractions ~= [0, 1], 'all');
    assert(hasFixedSegmentTimes || ~hasPartialSegmentLines || ...
        formulation.AllowPartialScopesWithVariableClock, ...
        'bmtpEngine:InvalidPlaneTimeScope', ...
        'Partial plane time scopes require a fixed trajectory clock.');
end
% A line that applies from 2 to 3 s cannot replace one that applies from
% 5 to 6 s. Remove redundant lines only when they cover whole segments.
if formulation.RemoveRedundantLines && hasFixedSegmentTimes && ...
        ~hasPartialSegmentLines && originalPlaneCount > segmentCount * degree
    separatingPlanes = bmtpEngine.separation.removeRedundantPlanes( ...
        separatingPlanes, limits, 2 * roundoffReserve_units);
end
% More quintic segments give the curve freedom to wiggle even when its
% control polygon is short. The optional smoothness objective is used only
% with fixed times and more than eight segments.
useJerkVariationObjective = allowJerkVariationObjective && hasFixedSegmentTimes && ...
    degree == 5 && segmentCount > 8;
start_units         = initialState.position_units;
goal_units          = goalState.position_units;
if formulation.PinEndpointControls
    endpointMotionRates = [initialState.velocity_units_s, initialState.acceleration_units_s2, ...
        goalState.velocity_units_s, goalState.acceleration_units_s2];
    % Nonzero endpoint rates require an actual fixed duration.
    assert(hasFixedSegmentTimes || all(endpointMotionRates == 0), ...
        'bmtpEngine:NonrestRelaxedClock', ...
        'Nonzero boundary states require physical fixed durations.');
end

%% Section 2: Build Endpoint, Join, And Motion-Limit Constraints

% Keep each formulation's original variable layout. The physical variable
% clock reserves unused edge bounds; the scaled variable clock does not.
timePowerIndices     = controlVariableCount + (1:4);
edgeLengthBoundCount = (hasFixedSegmentTimes || formulation.ReserveEdgeBoundsAtVariableClock) * ...
    segmentCount * degree;
planeActiveBySegment = reshape([separatingPlanes.Active], size(separatingPlanes));
activePlaneCount     = nnz(planeActiveBySegment);
planeCountBySegment  = sum(planeActiveBySegment, 2);
if hasFixedSegmentTimes
    % A fixed step solves on exactly the durations its lines were built on.
    % Use them as the segment ratios with a time scale of one. Every
    % rate-limit row then forms limit x duration^k directly, with the single
    % rounding any direct evaluation has, and the returned durations are the
    % supplied ones. The time powers are pinned before the solve (eliminated
    % in the physical formulation, fixed bounds in the scaled one), so the
    % scale has no numerical effect: a power-of-two scale near the mean
    % duration gave results identical to 17 digits on every example.
    fixedSegmentTime_s = segmentTimeRatios;
    maximumTimeScale_s = 1;
    % The rows form limit x duration^k and join weights of up to 3 x
    % duration^3 divided by the larger neighbour's cube. Segments from one
    % nanosecond to 1e9 s keep every one of those a normal double with more
    % than 250 orders of magnitude to spare; shorter or longer segments are
    % outside this solver's supported clock and are refused here.
    assert(all(fixedSegmentTime_s >= 1e-9 & fixedSegmentTime_s <= 1e9), 'bmtpEngine:InvalidStep', ...
        'Fixed segment times must lie between 1e-9 s and 1e9 s.');
else
    maximumTimeScale_s = maximumMotionDuration_s / sum(segmentTimeRatios);
    fixedSegmentTime_s = maximumMotionDuration_s * segmentTimeRatios / sum(segmentTimeRatios);
end
if formulation.PinEndpointControls
    endpointControlPoint_units = bmtpEngine.motion.imposeEndpointControls( ...
        zeros(segmentCount, degree + 1, 2), fixedSegmentTime_s, initialState, goalState);
else
    % Repeated positions make the first and last velocity and acceleration zero.
    endpointControlPoint_units = zeros(segmentCount, degree + 1, 2);
    endpointControlPoint_units(1, 1:3, :)           = repmat(reshape(start_units, 1, 1, 2), 1, 3, 1);
    endpointControlPoint_units(end, end - 2:end, :) = repmat(reshape(goal_units, 1, 1, 2), 1, 3, 1);
end
% Slack allows a temporary violation of a separating-line bound while the
% solver seeks a clear curve. With many lines, use one slack value per segment
% instead of per pair. Its cost is weighted by that segment's line count.
% The caller still checks obstacle clearance before accepting the motion.
shareSlackBySegment = formulation.UseClearanceSlack && hasFixedSegmentTimes && ...
    originalPlaneCount > edgeLengthBoundCount;
slackVariableCount  = formulation.UseClearanceSlack * hasFixedSegmentTimes * activePlaneCount;
if shareSlackBySegment
    slackVariableCount = nnz(planeCountBySegment);
end
decisionVariableCount = controlVariableCount + 4 + edgeLengthBoundCount + slackVariableCount + ...
    useJerkVariationObjective;
slackColumnByPair = zeros(size(planeActiveBySegment));
if shareSlackBySegment
    segmentSlackColumn = controlVariableCount + 4 + edgeLengthBoundCount + cumsum(planeCountBySegment > 0);
    for segmentIndex = reshape(find(planeCountBySegment > 0), 1, [])
        slackColumnByPair(segmentIndex, planeActiveBySegment(segmentIndex, :)) = ...
            segmentSlackColumn(segmentIndex);
    end
elseif slackVariableCount > 0
    nextSlackColumn = controlVariableCount + 4 + edgeLengthBoundCount;
    for segmentIndex = 1:segmentCount
        for regionIndex = reshape(find(planeActiveBySegment(segmentIndex, :)), 1, [])
            nextSlackColumn = nextSlackColumn + 1;
            slackColumnByPair(segmentIndex, regionIndex) = nextSlackColumn;
        end
    end
end
jerkObjectiveTime_s = [];
if useJerkVariationObjective
    jerkObjectiveTime_s = fixedSegmentTime_s;
end
constraintLimits = limits;
% A stalled solver may leave a small constraint error. Reserve a small
% fraction of the jerk limit here so later exact endpoint reconstruction
% does not immediately push jerk beyond its physical limit.
if formulation.ReserveJerkLimit && ~useJerkVariationObjective
    constraintLimits.maxJerk_units_s3 = ...
        limits.maxJerk_units_s3 .* (1 - sqrt(eps));
end
% Reuse matrices only when all their construction inputs match. Changing
% line constraints does not require rebuilding these shared motion rows.
sharedConstraintInputs = struct( ...
    "SegmentCount",     segmentCount, ...
    "Degree",           degree, ...
    "BoundaryControls", endpointControlPoint_units, ...
    "Limits",           constraintLimits, ...
    "VariableCount",    decisionVariableCount, ...
    "SegmentRatio",     segmentTimeRatios);
if formulation.KeepJerkTimesInKey
    sharedConstraintInputs.JerkTimes_s = jerkObjectiveTime_s;
end
canReuseSharedConstraints = isstruct(savedTrajectoryConstraints) && isscalar(savedTrajectoryConstraints) && ...
    isfield(savedTrajectoryConstraints, 'Key') && isequaln(savedTrajectoryConstraints.Key, sharedConstraintInputs);
if ~canReuseSharedConstraints
    [inequalityMatrix, equalityMatrix, equalityValues, lowerBounds, upperBounds, jerkControlMap] = ...
        bmtpEngine.optimization.createTrajectoryConstraints( ...
        segmentCount, degree, endpointControlPoint_units, constraintLimits, decisionVariableCount, ...
        segmentTimeRatios, jerkObjectiveTime_s);
    savedTrajectoryConstraints = struct( ...
        "A",   inequalityMatrix, ...
        "Aeq", equalityMatrix, ...
        "beq", equalityValues, ...
        "lb",  lowerBounds, ...
        "ub",  upperBounds);
    if formulation.KeepJerkTimesInKey
        savedTrajectoryConstraints.jerkMap = jerkControlMap;
    end
    savedTrajectoryConstraints.Key = sharedConstraintInputs;
else
    inequalityMatrix = savedTrajectoryConstraints.A;
    equalityMatrix   = savedTrajectoryConstraints.Aeq;
    equalityValues   = savedTrajectoryConstraints.beq;
    lowerBounds      = savedTrajectoryConstraints.lb;
    upperBounds      = savedTrajectoryConstraints.ub;
    jerkControlMap   = [];
    if useJerkVariationObjective
        jerkControlMap = savedTrajectoryConstraints.jerkMap;
    end
end
if formulation.ScaleTimePowers
    % The scaled clock stays near one while these columns preserve physical rate limits.
    for derivativeOrder = 1:3
        inequalityMatrix(:, timePowerIndices(derivativeOrder + 1)) = ...
            inequalityMatrix(:, timePowerIndices(derivativeOrder + 1)) * ...
            maximumTimeScale_s ^ derivativeOrder;
    end
end
% Set equal lower/upper bounds on the endpoint controls so position,
% velocity, and acceleration are exact in the returned variable values.
% Relying only on approximate equality rows could require a later endpoint
% correction that changes the end jerk.
if formulation.PinEndpointControls && ~useJerkVariationObjective
    fixedEndpointControl_units = NaN(segmentCount, degree + 1, 2);
    fixedEndpointControl_units(1, 1:3, :)           = endpointControlPoint_units(1, 1:3, :);
    fixedEndpointControl_units(end, end - 2:end, :) = endpointControlPoint_units(end, end - 2:end, :);
    fixedVariableValues = reshape(permute(fixedEndpointControl_units, [3, 2, 1]), [], 1);
    fixedControlIndices = find(isfinite(fixedVariableValues));
    lowerBounds(fixedControlIndices) = fixedVariableValues(fixedControlIndices);
    upperBounds(fixedControlIndices) = fixedVariableValues(fixedControlIndices);
end

%% Section 3: Add The Initial Separating-Line Rows

% Fixed clocks start without line rows. The scaled variable clock also does
% so when more than 2048 pairs are active.
motionLimitRowCount        = 4 * segmentCount * (3 * degree - 3);
addLineConstraintsAsNeeded = hasFixedSegmentTimes || ...
    activePlaneCount > formulation.MaximumFullyLoadedPairCount;
initiallyLoadedPairs = planeActiveBySegment;
if addLineConstraintsAsNeeded
    initiallyLoadedPairs(:) = false;
end
inequalityBounds = zeros(motionLimitRowCount, 1);
[lineConstraintRows, lineConstraintBounds] = bmtpEngine.separation.createSelectedPlaneRows(separatingPlanes, ...
    initiallyLoadedPairs, degree, decisionVariableCount, slackColumnByPair, ...
    formulation.LineRowReserveFactor * roundoffReserve_units);
inequalityMatrix = [inequalityMatrix; lineConstraintRows];
inequalityBounds = [inequalityBounds; lineConstraintBounds];

%% Section 4: Choose The Arrival, Path-Length, And Smoothness Objectives

% Minimize the cubic shared time scale for earliest arrival. With fixed
% durations, minimize control-polygon length instead.
objectiveWeights = zeros(decisionVariableCount, 1);
objectiveWeights(timePowerIndices(4)) = 1;
edgeLengthBoundIndices = controlVariableCount + 4 + (1:edgeLengthBoundCount);
lowerBounds(edgeLengthBoundIndices) = 0;
if formulation.ScaleTimePowers
    upperBounds(timePowerIndices) = 1;
    if hasFixedSegmentTimes
        lowerBounds(timePowerIndices) = 1;
    else
        minimumTimeFraction = minimumMotionDuration_s / maximumMotionDuration_s;
        lowerBounds(timePowerIndices) = [1; minimumTimeFraction; ...
            minimumTimeFraction ^ 2; minimumTimeFraction ^ 3];
    end
else
    maximumTimePowers = [1; maximumTimeScale_s; ...
        maximumTimeScale_s ^ 2; maximumTimeScale_s ^ 3];
    upperBounds(timePowerIndices) = maximumTimePowers;
    if hasFixedSegmentTimes
        lowerBounds(timePowerIndices) = maximumTimePowers;
    end
end
solverCones = repmat(secondordercone( ...
    sparse(2, decisionVariableCount), zeros(2, 1), sparse(decisionVariableCount, 1), 0), 0, 1);
if ~hasFixedSegmentTimes || formulation.IncludeTimePowerConesAtFixedClock
    solverCones = bmtpEngine.optimization.createTimePowerCones(decisionVariableCount, timePowerIndices);
end
if hasFixedSegmentTimes
    objectiveWeights(:) = 0;
    objectiveWeights(edgeLengthBoundIndices) = 1;
    slackIndices = controlVariableCount + 4 + edgeLengthBoundCount + (1:slackVariableCount);
    lowerBounds(slackIndices) = 0;
    % Slack and control-polygon length have the same distance units. A cost
    % of 1000 per unit makes line violations expensive compared with length.
    % A shared slack value pays that cost once for every line it relaxes.
    if formulation.UseClearanceSlack
        objectiveWeights(slackIndices) = 1e3;
        if shareSlackBySegment
            objectiveWeights(slackIndices) = 1e3 * planeCountBySegment(planeCountBySegment > 0);
        end
    end
    % Each edge bound z satisfies norm(nextControl - currentControl) <= z.
    % Their sum measures the control polygon, not the exact curve length.
    emptyCone = secondordercone( ...
        sparse(2, decisionVariableCount), zeros(2, 1), sparse(decisionVariableCount, 1), 0);
    edgeLengthCones = repmat(emptyCone, edgeLengthBoundCount, 1);
    for segmentIndex = 1:segmentCount
        for controlPointIndex = 1:degree
            edgeIndex   = (segmentIndex - 1) * degree + controlPointIndex;
            leftSideMap = sparse(2, decisionVariableCount);
            % Each segment stores controls as [P0x P0y P1x P1y ...]. The
            % previous control is two entries before the current one.
            currentControlIndices  = ((segmentIndex - 1) * (degree + 1) + controlPointIndex) * 2 + (1:2);
            previousControlIndices = currentControlIndices - 2;
            leftSideMap(:, currentControlIndices)  = eye(2);
            leftSideMap(:, previousControlIndices) = -eye(2);
            rightSideWeights = sparse(decisionVariableCount, 1);
            rightSideWeights(edgeLengthBoundIndices(edgeIndex)) = 1;
            edgeLengthCones(edgeIndex) = secondordercone( ...
                leftSideMap, zeros(2, 1), rightSideWeights, 0);
        end
    end
    solverCones = [solverCones; edgeLengthCones];
    if useJerkVariationObjective
        % The jerk-variation cost is dimensionless. Scale its weight by
        % start-to-goal distance to compare it with the length objective.
        smoothnessVariableIndex = decisionVariableCount;
        solverCones             = [solverCones; bmtpEngine.optimization.createVariationCone( ...
            jerkControlMap, fixedSegmentTime_s, limits, smoothnessVariableIndex)];
        lowerBounds(smoothnessVariableIndex) = 0;
        objectiveWeights(smoothnessVariableIndex) = 0.005 * norm(goal_units - start_units);
    end
end

%% Section 5: Solve And Add The Selected Violated Line Constraints

solverTimer             = tic;
localStateSegmentTime_s = [];
if useJerkVariationObjective
    localStateSegmentTime_s = fixedSegmentTime_s;
end
% Keep the solver matrices together while line rows are appended. After
% each fixed-time solve, check every unloaded pair and add the largest
% violation per segment. Later rounds continue checking all remaining pairs.
solverProblem = struct( ...
    'f',     objectiveWeights, ...
    'cones', solverCones, ...
    'A',     inequalityMatrix, ...
    'b',     inequalityBounds, ...
    'Aeq',   equalityMatrix, ...
    'beq',   equalityValues, ...
    'lb',    lowerBounds, ...
    'ub',    upperBounds);
loadedPlanePairs             = initiallyLoadedPairs;
solveCount                   = 0;
allLineConstraintsSatisfied  = ~addLineConstraintsAsNeeded;
maximumLineViolation         = NaN;
savedSolverValues            = [];
savedExitFlag                = NaN;
savedSolverOutput            = struct();
savedPlanePairs              = false(size(planeActiveBySegment));
savedMaximumLineViolation    = NaN;
savedLineConstraintsSatisfied = false;
savedSolveIndex              = 0;
while true
    attemptedPlanePairs = loadedPlanePairs;
    [attemptedSolverValues, attemptedExitFlag, attemptedSolverOutput] = solveWithFixedValues( ...
        solverProblem, solverOptions, formulation.PinEndpointControls && ~useJerkVariationObjective, ...
        localStateSegmentTime_s, limits);
    solveCount = solveCount + 1;
    lastAttemptExitFlag = attemptedExitFlag;
    if ~bmtpEngine.optimization.hasUsableConicIterate(attemptedSolverValues, attemptedExitFlag)
        break
    end

    violatedPairs = false(size(planeActiveBySegment));
    maximumAttemptLineViolation = NaN;
    attemptLineConstraintsSatisfied = ~addLineConstraintsAsNeeded;
    if addLineConstraintsAsNeeded
        [violatedPairs, maximumUnloadedLineViolation] = bmtpEngine.separation.findViolatedPlanePairs( ...
            attemptedSolverValues, separatingPlanes, planeActiveBySegment, ...
            attemptedPlanePairs, degree, slackColumnByPair, ...
            formulation.ViolationReserveFactor * roundoffReserve_units, ...
            solverOptions.ConstraintTolerance);
        if ~any(violatedPairs, 'all') || formulation.KeepLastUsableIterate
            % Loaded rows must pass as well as pairs that were not loaded.
            maximumLoadedLineViolation = -Inf;
            if size(solverProblem.A, 1) > motionLimitRowCount
                maximumLoadedLineViolation = max(solverProblem.A(motionLimitRowCount + 1:end, :) * ...
                    attemptedSolverValues - solverProblem.b(motionLimitRowCount + 1:end));
            end
            maximumAttemptLineViolation = max(maximumLoadedLineViolation, maximumUnloadedLineViolation);
            attemptLineConstraintsSatisfied = ~any(violatedPairs, 'all') && ...
                maximumAttemptLineViolation <= solverOptions.ConstraintTolerance;
        end
    end
    if formulation.KeepLastUsableIterate
        % A later failed round returns these values with their matching counts.
        savedSolverValues             = attemptedSolverValues;
        savedExitFlag                 = attemptedExitFlag;
        savedSolverOutput             = attemptedSolverOutput;
        savedPlanePairs               = attemptedPlanePairs;
        savedMaximumLineViolation     = maximumAttemptLineViolation;
        savedLineConstraintsSatisfied = attemptLineConstraintsSatisfied;
        savedSolveIndex               = solveCount;
    else
        maximumLineViolation        = maximumAttemptLineViolation;
        allLineConstraintsSatisfied = attemptLineConstraintsSatisfied;
    end
    if ~addLineConstraintsAsNeeded || ~any(violatedPairs, 'all')
        break
    end
    loadedPlanePairs = attemptedPlanePairs | violatedPairs;
    [newLineRows, newLineBounds] = bmtpEngine.separation.createSelectedPlaneRows(separatingPlanes, ...
        violatedPairs, degree, decisionVariableCount, slackColumnByPair, ...
        formulation.LineRowReserveFactor * roundoffReserve_units);
    solverProblem.A = [solverProblem.A; newLineRows];
    solverProblem.b = [solverProblem.b; newLineBounds];
end

%% Section 6: Return The Candidate And Solver Measurements

solverValues = attemptedSolverValues;
exitFlag     = attemptedExitFlag;
solverOutput = attemptedSolverOutput;
reportedPlanePairs = loadedPlanePairs;
returnedSolveIndex = 0;
if bmtpEngine.optimization.hasUsableConicIterate(solverValues, exitFlag)
    returnedSolveIndex = solveCount;
end
if formulation.KeepLastUsableIterate
    if ~isempty(savedSolverValues)
        solverValues = savedSolverValues;
        exitFlag     = savedExitFlag;
        solverOutput = savedSolverOutput;
        reportedPlanePairs = savedPlanePairs;
    else
        reportedPlanePairs = attemptedPlanePairs;
    end
    maximumLineViolation        = savedMaximumLineViolation;
    allLineConstraintsSatisfied = savedLineConstraintsSatisfied;
    returnedSolveIndex          = savedSolveIndex;
end
solverOutput.TotalTime_s = toc(solverTimer);
solverOutput.SolveCount = solveCount;
solverOutput.OptimizationConverged = exitFlag > 0;
solverOutput.OriginalPlaneCount = originalPlaneCount;
solverOutput.RetainedPlaneCount = activePlaneCount;
solverOutput.LoadedPlanePairCount = nnz(reportedPlanePairs);
solverOutput.ConstraintGenerationApplied = addLineConstraintsAsNeeded;
solverOutput.ConstraintGenerationRoundCount = max(0, solveCount - 1);
solverOutput.ConstraintGenerationComplete = allLineConstraintsSatisfied;
solverOutput.MaximumPlaneConstraintResidual = maximumLineViolation;
solverOutput.IntrinsicJerkVariation = useJerkVariationObjective;
if formulation.UseClearanceSlack && hasFixedSegmentTimes && ~isempty(solverValues)
    solverOutput.MaximumClearanceSlack_units = max(solverValues(slackIndices));
end
solverOutput.ReturnedSolveIndex             = returnedSolveIndex;
solverOutput.LastAttemptExitFlag            = lastAttemptExitFlag;
solverOutput.TerminatedAfterRetainedIterate = returnedSolveIndex > 0 && ...
    returnedSolveIndex < solveCount;
solverOutput.AttemptedLoadedPlanePairCount  = nnz(attemptedPlanePairs);
% Finite values from a stalled solve remain a candidate for full motion
% checks. A solver status alone cannot establish physical feasibility.
iterateIsUsable = bmtpEngine.optimization.hasUsableConicIterate(solverValues, exitFlag);
solverOutput    = labelSolveFailure(solverOutput, exitFlag, iterateIsUsable);
if ~iterateIsUsable
    controlPoint_units = zeros(0, degree + 1, 2);
    segmentTime_s      = NaN;
    return
end
% Recover the shared time scale from its cubic variable, then multiply by
% each segment's ratio. Fixed-time solves keep the exact supplied durations.
if hasFixedSegmentTimes
    segmentTime_s = fixedSegmentTime_s;
elseif formulation.ScaleTimePowers
    segmentTime_s = maximumTimeScale_s * max(solverValues(timePowerIndices(4)), 0) ^ (1 / 3);
    if ~returnsCommonSegmentTime
        segmentTime_s = segmentTime_s * segmentTimeRatios;
    end
else
    segmentTime_s = max(solverValues(timePowerIndices(4)), 0) ^ (1 / 3) * segmentTimeRatios;
end
controlPoint_units = permute(reshape(solverValues(1:controlVariableCount), 2, degree + 1, segmentCount), [3 2 1]);
end

%% Section 7: Local Functions

function formulation = resolveFormulation(formulationName, solverRequest, hasFixedSegmentTimes)
    % Set the numerical choices together so the solve never selects by name.
    physicalClock = struct();
    scaledClock   = struct();
    % The physical clock uses tight solver tolerances; the scaled clock uses its tested defaults.
    physicalClock.SolverOptions = solverRequest.TrajectoryOptions;
    scaledClock.SolverOptions   = solverRequest.TimedTrajectoryOptions;
    % The scaled clock keeps time powers near one; the physical clock uses seconds.
    physicalClock.ScaleTimePowers = false;
    scaledClock.ScaleTimePowers   = true;
    % The physical clock leaves jerk roundoff room unless its smoothing cost is active.
    physicalClock.ReserveJerkLimit = true;
    scaledClock.ReserveJerkLimit   = false;
    % The physical clock fixes endpoint controls; the scaled clock uses equality rows.
    physicalClock.PinEndpointControls = true;
    scaledClock.PinEndpointControls   = false;
    % Only the physical fixed clock has clearance slack variables.
    physicalClock.UseClearanceSlack = true;
    scaledClock.UseClearanceSlack   = false;
    % Only physical whole-segment fixed-clock lines may be reduced.
    physicalClock.RemoveRedundantLines = true;
    scaledClock.RemoveRedundantLines   = false;
    % Physical fixed-clock line rows use twice the reserve; scaled rows use it once.
    physicalClock.LineRowReserveFactor = 1 + hasFixedSegmentTimes;
    scaledClock.LineRowReserveFactor   = 1;
    % Physical violation checks use twice the reserve; scaled checks use it once.
    physicalClock.ViolationReserveFactor = 2;
    scaledClock.ViolationReserveFactor   = 1;
    % Physical variable clocks load all lines; scaled clocks defer sets above 2048.
    physicalClock.MaximumFullyLoadedPairCount = Inf;
    scaledClock.MaximumFullyLoadedPairCount   = 2048;
    % The scaled clock retains a usable iterate if the next line round fails.
    physicalClock.KeepLastUsableIterate = false;
    scaledClock.KeepLastUsableIterate   = true;
    % The scaled variable clock accepts lines that cover only part of a segment.
    physicalClock.AllowPartialScopesWithVariableClock = false;
    scaledClock.AllowPartialScopesWithVariableClock   = true;
    % The physical variable clock reserves unused edge bounds; the scaled clock does not.
    physicalClock.ReserveEdgeBoundsAtVariableClock = true;
    scaledClock.ReserveEdgeBoundsAtVariableClock   = false;
    % The scaled fixed clock includes time-power cones even with pinned powers.
    physicalClock.IncludeTimePowerConesAtFixedClock = false;
    scaledClock.IncludeTimePowerConesAtFixedClock   = true;
    % Physical constraint keys include jerk times; scaled keys omit them.
    physicalClock.KeepJerkTimesInKey = true;
    scaledClock.KeepJerkTimesInKey   = false;
    % Only the scaled clock has a lower arrival bound and a scalar-time request.
    physicalClock.AllowMinimumDuration = false;
    scaledClock.AllowMinimumDuration   = true;
    physicalClock.AllowCommonSegmentTime = false;
    scaledClock.AllowCommonSegmentTime   = true;
    % Intrinsic jerk variation belongs to physical-clock quintic refinement.
    physicalClock.AllowJerkVariation = true;
    scaledClock.AllowJerkVariation   = false;

    if formulationName == "physicalClock"
        formulation = physicalClock;
    else
        formulation = scaledClock;
    end
end

function [solverValues, exitFlag, solverOutput] = solveWithFixedValues( ...
        solverProblem, solverOptions, eliminateFixedValues, localStateSegmentTime_s, limits)
    % With known segment durations, solve using local physical states to
    % reduce roundoff in control-point differences. Remove fixed variables
    % when requested or when using this transformation, then restore every
    % original solver variable in the returned vector.
    objectiveWeights   = solverProblem.f;
    solverCones        = solverProblem.cones;
    inequalityMatrix   = solverProblem.A;
    inequalityBounds   = solverProblem.b;
    equalityMatrix     = solverProblem.Aeq;
    equalityValues     = solverProblem.beq;
    lowerBounds        = solverProblem.lb;
    upperBounds        = solverProblem.ub;
    solverToControlMap = [];
    coordinateOffset   = [];
    if ~isempty(localStateSegmentTime_s)
        % Replace absolute position controls with local position, velocity,
        % acceleration, and three quadratic jerk controls. The relation is
        % originalValues = coordinateOffset + solverToControlMap x localValues.
        segmentCount             = numel(localStateSegmentTime_s);
        decisionVariableCount    = numel(objectiveWeights);
        solverToControlMap       = speye(decisionVariableCount);
        coordinateOffset         = zeros(decisionVariableCount, 1);
        jerkControlMap           = sparse(6 * segmentCount, decisionVariableCount);
        workspaceIntervals_units = [limits.xInterval_units; limits.yInterval_units];
        workspaceCenter_units    = mean(workspaceIntervals_units, 2);
        % Integrating quadratic jerk three times gives these six position
        % controls for a one-second segment. Columns are p, v, a, j0, j1, j2;
        % multiply by duration, duration^2, or duration^3 for actual seconds.
        unitDurationControlMap = [1, 0, 0, 0, 0, 0; 1, 1 / 5, 0, 0, 0, 0; ...
            1, 2 / 5, 1 / 20, 0, 0, 0; ...
            1, 3 / 5, 3 / 20, 1 / 60, 0, 0; ...
            1, 4 / 5, 3 / 10, 1 / 20, 1 / 60, 0; ...
            1, 1, 1 / 2, 1 / 10, 1 / 20, 1 / 60];
        for segmentIndex = 1:segmentCount
            segmentDuration_s = localStateSegmentTime_s(segmentIndex);
            segmentControlMap = unitDurationControlMap .* [1, segmentDuration_s, segmentDuration_s ^ 2, ...
                segmentDuration_s ^ 3, segmentDuration_s ^ 3, segmentDuration_s ^ 3];
            for axisIndex = 1:2
                segmentCoordinateIndices = ((segmentIndex - 1) * 6 + (0:5)) * 2 + axisIndex;
                solverToControlMap(segmentCoordinateIndices, segmentCoordinateIndices) = segmentControlMap;
                coordinateOffset(segmentCoordinateIndices) = workspaceCenter_units(axisIndex);
            end
            jerkRowIndices    = (segmentIndex - 1) * 6 + (1:6);
            jerkColumnIndices = (segmentIndex - 1) * 12 + (7:12);
            jerkControlMap(jerkRowIndices, jerkColumnIndices) = speye(6);
        end
        % Substitute the same change of variables into every linear row,
        % vector-length constraint, and objective so the physical problem agrees.
        inequalityBounds = inequalityBounds - inequalityMatrix * coordinateOffset;
        inequalityMatrix = inequalityMatrix * solverToControlMap;
        equalityValues   = equalityValues - equalityMatrix * coordinateOffset;
        equalityMatrix   = equalityMatrix * solverToControlMap;
        for coneIndex = 1:numel(solverCones)
            cone = solverCones(coneIndex);
            solverCones(coneIndex) = secondordercone( ...
                cone.A * solverToControlMap, cone.b - cone.A * coordinateOffset, ...
                solverToControlMap.' * cone.d, cone.gamma - cone.d.' * coordinateOffset);
        end
        solverCones(end) = bmtpEngine.optimization.createVariationCone( ...
            jerkControlMap, localStateSegmentTime_s, limits, decisionVariableCount);
        objectiveWeights = solverToControlMap.' * objectiveWeights;
        % Original coordinate bounds become linear rows in the local values.
        % Preserve exact fixed values as equal lower/upper bounds.
        fixedVariableIndices = find(lowerBounds == upperBounds);
        upperRowIndices      = find(isfinite(upperBounds) & lowerBounds ~= upperBounds);
        lowerRowIndices      = find(isfinite(lowerBounds) & lowerBounds ~= upperBounds);
        inequalityMatrix     = [inequalityMatrix; solverToControlMap(upperRowIndices, :); ...
            -solverToControlMap(lowerRowIndices, :)];
        inequalityBounds = [inequalityBounds; upperBounds(upperRowIndices) - coordinateOffset(upperRowIndices); ...
            coordinateOffset(lowerRowIndices) - lowerBounds(lowerRowIndices)];
        nonFixedVariableIndices = setdiff(1:decisionVariableCount, fixedVariableIndices);
        % Each fixed original value must depend only on fixed local values.
        assert(nnz(solverToControlMap(fixedVariableIndices, nonFixedVariableIndices)) == 0);
        fixedVariableValues = solverToControlMap(fixedVariableIndices, fixedVariableIndices) \ ...
            (lowerBounds(fixedVariableIndices) - coordinateOffset(fixedVariableIndices));
        lowerBounds(:) = -Inf;
        upperBounds(:) = Inf;
        lowerBounds(fixedVariableIndices) = fixedVariableValues;
        upperBounds(fixedVariableIndices) = fixedVariableValues;
    end
    % Substitute fixed values into every row and solve only for the free
    % variables. This avoids redundant rows for values already known exactly.
    fixedVariableIndices = find(lowerBounds == upperBounds & isfinite(lowerBounds));
    if (isempty(localStateSegmentTime_s) && ~eliminateFixedValues) || isempty(fixedVariableIndices)
        [solverValues, ~, exitFlag, solverOutput] = coneprog( ...
            objectiveWeights, solverCones, inequalityMatrix, inequalityBounds, ...
            equalityMatrix, equalityValues, lowerBounds, upperBounds, solverOptions);
        return
    end
    freeVariableIndices = find(lowerBounds ~= upperBounds);
    fixedVariableValues = lowerBounds(fixedVariableIndices);
    inequalityBounds    = inequalityBounds - inequalityMatrix(:, fixedVariableIndices) * fixedVariableValues;
    equalityValues      = equalityValues - equalityMatrix(:, fixedVariableIndices) * fixedVariableValues;
    inequalityMatrix    = inequalityMatrix(:, freeVariableIndices);
    equalityMatrix      = equalityMatrix(:, freeVariableIndices);
    % A row with no remaining variable coefficients is already decided.
    % Remove it only if its constant value satisfies the solver tolerance;
    % retain a violated constant row so the solver can report infeasibility.
    rowsToKeep       = any(inequalityMatrix ~= 0, 2) | inequalityBounds < -solverOptions.ConstraintTolerance;
    inequalityMatrix = inequalityMatrix(rowsToKeep, :);
    inequalityBounds = inequalityBounds(rowsToKeep);
    rowsToKeep       = any(equalityMatrix ~= 0, 2) | abs(equalityValues) > solverOptions.ConstraintTolerance;
    equalityMatrix   = equalityMatrix(rowsToKeep, :);
    equalityValues   = equalityValues(rowsToKeep);
    if ~isempty(localStateSegmentTime_s)
        % Divide both sides by the largest coefficient in each row. The
        % positive floor prevents division by zero without changing the bound.
        inequalityRowScale = max(max(abs(inequalityMatrix), [], 2), 1e-20);
        inequalityMatrix   = inequalityMatrix ./ inequalityRowScale;
        inequalityBounds   = inequalityBounds ./ inequalityRowScale;
        equalityRowScale   = max(max(abs(equalityMatrix), [], 2), 1e-20);
        equalityMatrix     = equalityMatrix ./ equalityRowScale;
        equalityValues     = equalityValues ./ equalityRowScale;
    end
    for coneIndex = 1:numel(solverCones)
        cone = solverCones(coneIndex);
        solverCones(coneIndex) = secondordercone( ...
            cone.A(:, freeVariableIndices), cone.b - cone.A(:, fixedVariableIndices) * fixedVariableValues, ...
            cone.d(freeVariableIndices), cone.gamma - cone.d(fixedVariableIndices).' * fixedVariableValues);
    end
    [freeVariableValues, ~, exitFlag, solverOutput] = coneprog( ...
        objectiveWeights(freeVariableIndices), solverCones, inequalityMatrix, inequalityBounds, ...
        equalityMatrix, equalityValues, ...
        lowerBounds(freeVariableIndices), upperBounds(freeVariableIndices), solverOptions);
    % Put fixed and solved values back in their original positions, then
    % undo the local-state transformation when one was used.
    solverValues = [];
    if ~isempty(freeVariableValues)
        solverValues = zeros(size(objectiveWeights));
        solverValues(fixedVariableIndices) = fixedVariableValues;
        solverValues(freeVariableIndices)  = freeVariableValues;
        if ~isempty(solverToControlMap)
            solverValues = coordinateOffset + solverToControlMap * solverValues;
        end
    end
end

function solverOutput = labelSolveFailure(solverOutput, exitFlag, iterateIsUsable)
    % Tell the calling optimizer what a failed solve means to the planner.
    % An iteration limit or an infeasible subproblem may be answered with a
    % different route. Any other solver status is a numerical failure and
    % must not be hidden behind another attempt.
    solverOutput.FailureStage             = "";
    solverOutput.FailureKind              = "";
    solverOutput.AlternativeGuideEligible = false;
    if iterateIsUsable
        return
    end
    if exitFlag == 0
        solverOutput.FailureStage             = "optimization";
        solverOutput.FailureKind              = "trajectorySolverIterationLimit";
        solverOutput.AlternativeGuideEligible = true;
    elseif exitFlag == -2
        solverOutput.FailureStage             = "proposal";
        solverOutput.FailureKind              = "trajectorySubproblemInfeasible";
        solverOutput.AlternativeGuideEligible = true;
    else
        solverOutput.FailureStage = "numericalSolver";
        solverOutput.FailureKind  = "optimizerIterateUnavailable";
    end
end
