function [controlPoint_units, segmentTime_s, exitFlag, solverOutput, savedTrajectoryConstraints] = ...
    solveTrajectoryStep(solverRequest, trajectoryStep)
%% Section 0: Header & Readme
% SYNTAX
%   [controlPoint_units, segmentTime_s, exitFlag, solverOutput, savedTrajectoryConstraints] = ...
%       bmtpEngine.optimization.solveTrajectoryStep(solverRequest, trajectoryStep)
%**************************************************************************
% PURPOSE
%   - Solve for a new curve using fixed separating lines and duration ratios.
%     Seek an earlier arrival, or shorten and optionally smooth the path at a
%     fixed arrival time. The caller still checks the complete returned motion.
%**************************************************************************
% INPUTS
%   - solverRequest (scalar struct)
%       The engine solve request from createSolveRequest. This step reads
%       Degree, InitialState, GoalState, Limits, and TrajectoryOptions.
%   - trajectoryStep (scalar struct)
%       What this one step is asked to do, every field required:
%       SegmentCount (positive integer), Planes (S-by-R struct array of
%       fixed separating lines whose TimeFraction selects the part of the
%       segment where each line applies), RoundoffReserve_units (nonnegative
%       scalar), MaximumMotionDuration_s (positive scalar upper bound on the
%       internal minimum-time solve), SegmentRatio (S-by-1 positive relative
%       segment durations), FixedClock (logical: prescribe the segment
%       durations), IntrinsicVariationEnabled (logical: allow a jerk-variation
%       objective for fixed-time quintic curves with more than eight segments),
%       and ConstraintBase (saved workspace, endpoint, join, and motion-limit
%       constraints from a matching earlier step, or struct() to build them).
%**************************************************************************
% OUTPUTS
%   - controlPoint_units (S-by-(D+1)-by-2 numeric array)
%       Solved control points, or an empty array on expected solve failure.
%       Invalid input throws an error.
%   - segmentTime_s (S-by-1 numeric vector)
%       Per-segment durations, or NaN on expected solve failure.
%   - exitFlag (numeric scalar)
%       Original coneprog status.
%   - solverOutput (scalar struct)
%       Solver status, diagnostics, and measured solver time.
%   - savedTrajectoryConstraints (scalar struct)
%       Constraints and their inputs, for reuse when the next step matches.
%**************************************************************************
% UNITS
%   - Position is coordinate units and time is seconds.
%**************************************************************************

%% Section 1: Read Inputs And Resolve Timing And Line Coverage

validateattributes(trajectoryStep, {'struct'}, {'scalar'});
requiredStepFields = ["SegmentCount", "Planes", "RoundoffReserve_units", "MaximumMotionDuration_s", ...
    "SegmentRatio", "FixedClock", "IntrinsicVariationEnabled", "ConstraintBase"];
assert(all(isfield(trajectoryStep, requiredStepFields)), 'bmtpEngine:InvalidStep', ...
    'A trajectory step declares every one of: %s.', strjoin(requiredStepFields, ', '));
segmentCount                = trajectoryStep.SegmentCount;
separatingPlanes            = trajectoryStep.Planes;
roundoffReserve_units       = trajectoryStep.RoundoffReserve_units;
maximumMotionDuration_s     = trajectoryStep.MaximumMotionDuration_s;
segmentTimeRatios           = trajectoryStep.SegmentRatio;
hasFixedSegmentTimes        = trajectoryStep.FixedClock;
allowJerkVariationObjective = trajectoryStep.IntrinsicVariationEnabled;
savedTrajectoryConstraints  = trajectoryStep.ConstraintBase;

degree        = solverRequest.Degree;
initialState  = solverRequest.InitialState;
goalState     = solverRequest.GoalState;
limits        = solverRequest.Limits;
solverOptions = solverRequest.TrajectoryOptions;

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
    assert(hasFixedSegmentTimes || ~hasPartialSegmentLines, ...
        'bmtpEngine:InvalidPlaneTimeScope', ...
        'Partial plane time scopes require a fixed trajectory clock.');
end
% A line that applies from 2 to 3 s cannot replace one that applies from
% 5 to 6 s. Remove redundant lines only when they cover whole segments.
if hasFixedSegmentTimes && ~hasPartialSegmentLines && originalPlaneCount > segmentCount * degree
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
endpointMotionRates = [initialState.velocity_units_s, initialState.acceleration_units_s2, ...
    goalState.velocity_units_s, goalState.acceleration_units_s2];
endpointsAreAtRest = all(endpointMotionRates == 0);
% Nonzero endpoint velocity or acceleration depends on the actual segment
% duration, so this formulation requires fixed times for those endpoints.
assert(hasFixedSegmentTimes || endpointsAreAtRest, 'bmtpEngine:NonrestRelaxedClock', ...
    'Nonzero boundary states require physical fixed durations.');

%% Section 2: Build Endpoint, Join, And Motion-Limit Constraints

% Solver variables contain x/y controls, four time powers, edge-length bounds,
% optional clearance slack, and an optional smoothness-cost bound.
endpointControlPoint_units = bmtpEngine.motion.imposeEndpointControls( ...
    zeros(segmentCount, degree + 1, 2), ...
    maximumMotionDuration_s * segmentTimeRatios / sum(segmentTimeRatios), initialState, goalState);
timePowerIndices     = controlVariableCount + (1:4);
edgeLengthBoundCount = segmentCount * degree;
planeActiveBySegment = reshape([separatingPlanes.Active], size(separatingPlanes));
activePlaneCount     = nnz(planeActiveBySegment);
planeCountBySegment  = sum(planeActiveBySegment, 2);
% Slack allows a temporary violation of a separating-line bound while the
% solver seeks a clear curve. With many lines, use one slack value per segment
% instead of per pair. Its cost is weighted by that segment's line count.
% The caller still checks obstacle clearance before accepting the motion.
shareSlackBySegment = hasFixedSegmentTimes && originalPlaneCount > edgeLengthBoundCount;
slackVariableCount  = hasFixedSegmentTimes * activePlaneCount;
if shareSlackBySegment
    slackVariableCount = nnz(planeCountBySegment);
end
decisionVariableCount = controlVariableCount + 4 + edgeLengthBoundCount + slackVariableCount + ...
    useJerkVariationObjective;
slackColumnByPair = zeros(size(planeActiveBySegment));
if hasFixedSegmentTimes && shareSlackBySegment
    segmentSlackColumn = controlVariableCount + 4 + edgeLengthBoundCount + cumsum(planeCountBySegment > 0);
    for segmentIndex = reshape(find(planeCountBySegment > 0), 1, [])
        slackColumnByPair(segmentIndex, planeActiveBySegment(segmentIndex, :)) = ...
            segmentSlackColumn(segmentIndex);
    end
elseif hasFixedSegmentTimes
    nextSlackColumn = controlVariableCount + 4 + edgeLengthBoundCount;
    for segmentIndex = 1:segmentCount
        for regionIndex = reshape(find(planeActiveBySegment(segmentIndex, :)), 1, [])
            nextSlackColumn = nextSlackColumn + 1;
            slackColumnByPair(segmentIndex, regionIndex) = nextSlackColumn;
        end
    end
end
fixedSegmentTime_s  = maximumMotionDuration_s * segmentTimeRatios / sum(segmentTimeRatios);
jerkObjectiveTime_s = [];
if useJerkVariationObjective
    jerkObjectiveTime_s = fixedSegmentTime_s;
end
constraintLimits = limits;
% A stalled solver may leave a small constraint error. Reserve a small
% fraction of the jerk limit here so later exact endpoint reconstruction
% does not immediately push jerk beyond its physical limit.
if ~useJerkVariationObjective
    constraintLimits.maxJerk_units_s3 = ...
        limits.maxJerk_units_s3 .* (1 - sqrt(eps));
end
initiallyLoadedPairs = planeActiveBySegment;
if hasFixedSegmentTimes
    initiallyLoadedPairs(:) = false;
end
% Reuse matrices only when all their construction inputs match. Changing
% line constraints does not require rebuilding these shared motion rows.
sharedConstraintInputs = struct( ...
    "SegmentCount",     segmentCount, ...
    "Degree",           degree, ...
    "BoundaryControls", endpointControlPoint_units, ...
    "Limits",           constraintLimits, ...
    "VariableCount",    decisionVariableCount, ...
    "SegmentRatio",     segmentTimeRatios, ...
    "JerkTimes_s",      jerkObjectiveTime_s);
canReuseSharedConstraints = isstruct(savedTrajectoryConstraints) && isscalar(savedTrajectoryConstraints) && ...
    isfield(savedTrajectoryConstraints, 'Key') && isequaln(savedTrajectoryConstraints.Key, sharedConstraintInputs);
if ~canReuseSharedConstraints
    [inequalityMatrix, equalityMatrix, equalityValues, lowerBounds, upperBounds, jerkControlMap] = ...
        bmtpEngine.optimization.createTrajectoryConstraints( ...
        segmentCount, degree, endpointControlPoint_units, constraintLimits, decisionVariableCount, ...
        segmentTimeRatios, jerkObjectiveTime_s);
    savedTrajectoryConstraints = struct( ...
        "A",       inequalityMatrix, ...
        "Aeq",     equalityMatrix, ...
        "beq",     equalityValues, ...
        "lb",      lowerBounds, ...
        "ub",      upperBounds, ...
        "jerkMap", jerkControlMap, ...
        "Key",     sharedConstraintInputs);
else
    inequalityMatrix = savedTrajectoryConstraints.A;
    equalityMatrix   = savedTrajectoryConstraints.Aeq;
    equalityValues   = savedTrajectoryConstraints.beq;
    lowerBounds      = savedTrajectoryConstraints.lb;
    upperBounds      = savedTrajectoryConstraints.ub;
    jerkControlMap   = savedTrajectoryConstraints.jerkMap;
end
% Set equal lower/upper bounds on the endpoint controls so position,
% velocity, and acceleration are exact in the returned variable values.
% Relying only on approximate equality rows could require a later endpoint
% correction that changes the end jerk.
if ~useJerkVariationObjective
    fixedEndpointControl_units = NaN(segmentCount, degree + 1, 2);
    fixedEndpointControl_units(1, 1:3, :)           = endpointControlPoint_units(1, 1:3, :);
    fixedEndpointControl_units(end, end - 2:end, :) = endpointControlPoint_units(end, end - 2:end, :);
    fixedVariableValues = reshape(permute(fixedEndpointControl_units, [3, 2, 1]), [], 1);
    fixedControlIndices = find(isfinite(fixedVariableValues));
    lowerBounds(fixedControlIndices) = fixedVariableValues(fixedControlIndices);
    upperBounds(fixedControlIndices) = fixedVariableValues(fixedControlIndices);
end

%% Section 3: Add The Initial Separating-Line Rows

% Fixed-time solves start without line rows and add violated pairs below.
% Variable-time solves include every active line immediately.
motionLimitRowCount = 4 * segmentCount * (3 * degree - 3);
inequalityBounds    = zeros(motionLimitRowCount, 1);
[lineConstraintRows, lineConstraintBounds] = bmtpEngine.separation.createSelectedPlaneRows(separatingPlanes, ...
    initiallyLoadedPairs, degree, decisionVariableCount, slackColumnByPair, ...
    (1 + hasFixedSegmentTimes) * roundoffReserve_units);
inequalityMatrix = [inequalityMatrix; lineConstraintRows];
inequalityBounds = [inequalityBounds; lineConstraintBounds];

%% Section 4: Choose The Arrival, Path-Length, And Smoothness Objectives

% Minimize the cubic shared time scale for earliest arrival. With fixed
% durations, minimize control-polygon length instead.
objectiveWeights = zeros(decisionVariableCount, 1);
objectiveWeights(timePowerIndices(4)) = 1;
maximumTimeScale_s = maximumMotionDuration_s / sum(segmentTimeRatios);
maximumTimePowers  = [1; maximumTimeScale_s; ...
    maximumTimeScale_s ^ 2; maximumTimeScale_s ^ 3];
upperBounds(timePowerIndices) = maximumTimePowers;
edgeLengthBoundIndices = controlVariableCount + 4 + (1:edgeLengthBoundCount);
lowerBounds(edgeLengthBoundIndices) = 0;
if hasFixedSegmentTimes
    lowerBounds(timePowerIndices) = maximumTimePowers;
    objectiveWeights(:) = 0;
    objectiveWeights(edgeLengthBoundIndices) = 1;
    slackIndices = controlVariableCount + 4 + edgeLengthBoundCount + (1:slackVariableCount);
    lowerBounds(slackIndices) = 0;
    % Slack and control-polygon length have the same distance units. A cost
    % of 1000 per unit makes line violations expensive compared with length.
    % A shared slack value pays that cost once for every line it relaxes.
    objectiveWeights(slackIndices) = 1e3;
    if shareSlackBySegment
        objectiveWeights(slackIndices) = 1e3 * planeCountBySegment(planeCountBySegment > 0);
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
            leftSideMap(:, bmtpEngine.optimization.controlIndexOf( ...
                segmentIndex, controlPointIndex, 1:2, degree)) = eye(2);
            leftSideMap(:, bmtpEngine.optimization.controlIndexOf( ...
                segmentIndex, controlPointIndex - 1, 1:2, degree)) = -eye(2);
            rightSideWeights = sparse(decisionVariableCount, 1);
            rightSideWeights(edgeLengthBoundIndices(edgeIndex)) = 1;
            edgeLengthCones(edgeIndex) = secondordercone( ...
                leftSideMap, zeros(2, 1), rightSideWeights, 0);
        end
    end
    solverCones = edgeLengthCones;
    if useJerkVariationObjective
        % The jerk-variation cost is dimensionless. Scale its weight by
        % start-to-goal distance to compare it with the length objective.
        smoothnessVariableIndex = decisionVariableCount;
        solverCones             = [solverCones; bmtpEngine.optimization.createVariationCone( ...
            jerkControlMap, fixedSegmentTime_s, limits, smoothnessVariableIndex)];
        lowerBounds(smoothnessVariableIndex) = 0;
        objectiveWeights(smoothnessVariableIndex) = 0.005 * norm(goal_units - start_units);
    end
else
    solverCones = bmtpEngine.optimization.createTimePowerCones(decisionVariableCount, timePowerIndices);
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
loadedPlanePairs            = initiallyLoadedPairs;
solveCount                  = 0;
allLineConstraintsSatisfied = ~hasFixedSegmentTimes;
maximumLineViolation        = NaN;
while true
    [solverValues, exitFlag, solverOutput] = solveWithFixedValues( ...
        solverProblem, solverOptions, ~useJerkVariationObjective, ...
        localStateSegmentTime_s, limits);
    solveCount = solveCount + 1;
    if ~hasFixedSegmentTimes || ~bmtpEngine.optimization.hasUsableConicIterate(solverValues, exitFlag)
        break
    end
    [violatedPairs, maximumUnloadedLineViolation] = bmtpEngine.separation.findViolatedPlanePairs( ...
        solverValues, separatingPlanes, planeActiveBySegment, ...
        loadedPlanePairs, degree, slackColumnByPair, 2 * roundoffReserve_units, ...
        solverOptions.ConstraintTolerance);
    if ~any(violatedPairs, 'all')
        % Also check the loaded rows: no missing violations alone does not
        % establish that the full line-constraint set passes.
        maximumLoadedLineViolation = -Inf;
        if size(solverProblem.A, 1) > motionLimitRowCount
            maximumLoadedLineViolation = max(solverProblem.A(motionLimitRowCount + 1:end, :) * solverValues - ...
                solverProblem.b(motionLimitRowCount + 1:end));
        end
        maximumLineViolation        = max(maximumLoadedLineViolation, maximumUnloadedLineViolation);
        allLineConstraintsSatisfied = ...
            maximumLineViolation <= solverOptions.ConstraintTolerance;
        break
    end
    loadedPlanePairs = loadedPlanePairs | violatedPairs;
    [newLineRows, newLineBounds] = bmtpEngine.separation.createSelectedPlaneRows(separatingPlanes, ...
        violatedPairs, degree, decisionVariableCount, slackColumnByPair, 2 * roundoffReserve_units);
    solverProblem.A = [solverProblem.A; newLineRows];
    solverProblem.b = [solverProblem.b; newLineBounds];
end

%% Section 6: Return The Candidate And Solver Measurements

solverOutput.TotalTime_s = toc(solverTimer);
solverOutput.SolveCount = solveCount;
solverOutput.OptimizationConverged = exitFlag > 0;
solverOutput.OriginalPlaneCount = originalPlaneCount;
solverOutput.RetainedPlaneCount = activePlaneCount;
solverOutput.LoadedPlanePairCount = nnz(loadedPlanePairs);
solverOutput.ConstraintGenerationApplied = hasFixedSegmentTimes;
solverOutput.ConstraintGenerationRoundCount = max(0, solveCount - 1);
solverOutput.ConstraintGenerationComplete = allLineConstraintsSatisfied;
solverOutput.MaximumPlaneConstraintResidual = maximumLineViolation;
solverOutput.IntrinsicJerkVariation = useJerkVariationObjective;
if hasFixedSegmentTimes && ~isempty(solverValues)
    solverOutput.MaximumClearanceSlack_units = max(solverValues(slackIndices));
end
% Finite values from a stalled solve remain a candidate for full motion
% checks. A solver status alone cannot establish physical feasibility.
if ~bmtpEngine.optimization.hasUsableConicIterate(solverValues, exitFlag)
    controlPoint_units = zeros(0, degree + 1, 2);
    segmentTime_s      = NaN;
    return
end
% Recover the shared time scale from its cubic variable, then multiply by
% each segment's ratio. Fixed-time solves keep the exact supplied durations.
segmentTime_s = max(solverValues(timePowerIndices(4)), 0) ^ (1 / 3) * segmentTimeRatios;
if hasFixedSegmentTimes
    segmentTime_s = fixedSegmentTime_s;
end
controlPoint_units = permute(reshape(solverValues(1:controlVariableCount), 2, degree + 1, segmentCount), [3 2 1]);
end

%% Section 7: Local Functions

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
