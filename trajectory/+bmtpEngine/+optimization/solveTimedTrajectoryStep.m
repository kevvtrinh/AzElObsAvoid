function [controlPoint_units, segmentTime_s, exitFlag, output, constraintBase] = ...
        solveTimedTrajectoryStep(request, step)
%% Section 0: Header & Readme
% SYNTAX
%   [controlPoint_units, segmentTime_s, exitFlag, output, constraintBase] = ...
%       bmtpEngine.optimization.solveTimedTrajectoryStep(request, step)
%**************************************************************************
% PURPOSE
%   - Solve one convex trajectory step for fixed separating lines, timing
%     policy, and derivative limits.
%**************************************************************************
% INPUTS
%   - request (scalar struct)
%       The engine solve request from createSolveRequest. This step reads
%       Degree, the InitialState and GoalState positions, Limits, and
%       TimedTrajectoryOptions.
%   - step (scalar struct)
%       What this one step is asked to do, every field required:
%       SegmentCount (positive integer), Planes (S-by-R struct array of
%       fixed active separating lines), RoundoffReserve_units (nonnegative
%       scalar), MaximumMotionDuration_s (positive scalar upper bound or
%       fixed duration), GoalTimeMode (earliestArrival or fixedArrival),
%       MinimumMotionDuration_s (nonnegative scalar lower arrival bound, at
%       most the maximum), SegmentRatio (S-by-1 positive relative span
%       durations, or [] for one common segment time), and ConstraintBase
%       (the reusable invariant constraint arrays from an earlier step with
%       the same formulation, or struct() to build them).
%**************************************************************************
% OUTPUTS
%   - controlPoint_units (S-by-(D+1)-by-2 numeric array)
%       Solved control points, or an empty array on expected solve failure.
%       Invalid input throws an error.
%   - segmentTime_s (numeric scalar or S-by-1 numeric vector)
%       Common segment time, or the per-segment durations implied by
%       SegmentRatio. NaN on expected solve failure.
%   - exitFlag (numeric scalar)
%       Original coneprog status.
%   - output (scalar struct)
%       Solver status, diagnostics, and measured time. Finite fixed-clock -7
%       iterates are proposals requiring final independent proof.
%   - constraintBase (scalar struct)
%       Invariant constraint arrays for reuse with the same formulation.
%**************************************************************************
% UNITS
%   - Position is coordinate units and time is seconds.
%**************************************************************************

%% Section 1: Read The Step And The Request, Then Create Decision Bounds
validateattributes(step, {'struct'}, {'scalar'});
stepFields = ["SegmentCount", "Planes", "RoundoffReserve_units", "MaximumMotionDuration_s", ...
    "GoalTimeMode", "MinimumMotionDuration_s", "SegmentRatio", "ConstraintBase"];
assert(all(isfield(step, stepFields)), 'bmtpEngine:InvalidStep', ...
    'A timed trajectory step declares every one of: %s.', strjoin(stepFields, ', '));
segmentCount            = step.SegmentCount;
planes                  = step.Planes;
roundoffReserve_units   = step.RoundoffReserve_units;
maximumMotionDuration_s = step.MaximumMotionDuration_s;
goalTimeMode            = step.GoalTimeMode;
minimumMotionDuration_s = step.MinimumMotionDuration_s;
segmentRatio            = step.SegmentRatio;
constraintBase          = step.ConstraintBase;
degree      = request.Degree;
start_units = request.InitialState.position_units;
goal_units  = request.GoalState.position_units;
limits      = request.Limits;
options     = request.TimedTrajectoryOptions;
returnsCommonSegmentTime = isempty(segmentRatio);
if returnsCommonSegmentTime
    segmentRatio = ones(segmentCount, 1);
end
segmentRatio = double(segmentRatio(:));
validateattributes(segmentRatio, {'numeric'}, ...
    {'real', 'finite', 'positive', 'numel', segmentCount});
validateattributes(minimumMotionDuration_s, {'numeric'}, ...
    {'real', 'finite', 'scalar', 'nonnegative', '<=', maximumMotionDuration_s});

controlCount     = segmentCount * (degree + 1) * 2;
powerIndex       = controlCount + (1:4);
travelBoundCount = (goalTimeMode ~= "earliestArrival") * segmentCount * degree;
travelBoundIndex = controlCount + 4 + (1:travelBoundCount);
variableCount    = controlCount + 4 + travelBoundCount;

planeActiveBySegment  = reshape([planes.Active], size(planes));
maximumSegmentTime_s  = maximumMotionDuration_s / sum(segmentRatio);

boundaryControls = zeros(segmentCount, degree + 1, 2);
boundaryControls(1, 1:3, :)           = repmat(reshape(start_units, 1, 1, 2), 1, 3, 1);
boundaryControls(end, end - 2:end, :) = repmat(reshape(goal_units, 1, 1, 2), 1, 3, 1);
constraintKey = struct( ...
    "SegmentCount",     segmentCount, ...
    "Degree",           degree, ...
    "BoundaryControls", boundaryControls, ...
    "Limits",           limits, ...
    "VariableCount",    variableCount, ...
    "SegmentRatio",     segmentRatio);
canReuseConstraintBase = isstruct(constraintBase) && isscalar(constraintBase) && ...
    isfield(constraintBase, 'Key') && isequaln(constraintBase.Key, constraintKey);
if ~canReuseConstraintBase
    [A, Aeq, beq, lb, ub] = bmtpEngine.optimization.createTrajectoryConstraints( ...
        segmentCount, degree, boundaryControls, limits, variableCount, segmentRatio, []);
    constraintBase = struct("A", A, "Aeq", Aeq, "beq", beq, ...
        "lb", lb, "ub", ub, "Key", constraintKey);
else
    A   = constraintBase.A;
    Aeq = constraintBase.Aeq;
    beq = constraintBase.beq;
    lb  = constraintBase.lb;
    ub  = constraintBase.ub;
end
% The clock cones need only relative powers. Scaling the three physical-time
% columns to a unit upper bound avoids conditioning the SOCP with seconds,
% seconds squared, and seconds cubed that differ by several orders.
for derivativeOrder = 1:3
    A(:, powerIndex(derivativeOrder + 1)) = ...
        A(:, powerIndex(derivativeOrder + 1)) * maximumSegmentTime_s ^ derivativeOrder;
end
lb(travelBoundIndex) = 0;

%% Section 2: Add Separating-Line Bounds
baseInequalityCount = 4 * segmentCount * (3 * degree - 3);
initialPlanePairs   = planeActiveBySegment;
% Keep the full formulation for moderate earliest-arrival systems, where its
% single solve is both compact and better conditioned. Beyond this exact row
% count, exhaustive row generation avoids constructing the dominant dense
% plane block while still scanning every omitted pair after each solve.
maximumDirectPlanePairCount = 2048;
useConstraintGeneration = goalTimeMode == "fixedArrival" || ...
    nnz(planeActiveBySegment) > maximumDirectPlanePairCount;
if useConstraintGeneration
    initialPlanePairs(:) = false;
end
slackColumnByPair = zeros(size(planeActiveBySegment));
[planeRows, planeBounds] = bmtpEngine.separation.createSelectedPlaneRows(planes, ...
    initialPlanePairs, degree, variableCount, slackColumnByPair, roundoffReserve_units);
A = [A; planeRows];
b = [zeros(baseInequalityCount, 1); planeBounds];

%% Section 3: Create The Objective And Solve
cones = [bmtpEngine.optimization.createTimePowerCones(variableCount, powerIndex); ...
    createTravelBoundCones(variableCount, travelBoundIndex, segmentCount, degree)];
f     = zeros(variableCount, 1);
if goalTimeMode == "earliestArrival"
    f(powerIndex(4)) = 1;
else
    f(travelBoundIndex) = 1;
end
if goalTimeMode == "fixedArrival"
    lb(powerIndex) = 1;
    ub(powerIndex) = 1;
else
    ub(powerIndex) = 1;
    minimumTimeRatio = minimumMotionDuration_s / maximumMotionDuration_s;
    lb(powerIndex) = [1; minimumTimeRatio; ...
        minimumTimeRatio ^ 2; minimumTimeRatio ^ 3];
end

solverTimer        = tic;
retainedPlanePairs = initialPlanePairs;
solveCount         = 0;
returnedX                              = [];
returnedExitFlag                       = NaN;
returnedOutput                         = struct();
returnedPlanePairs                     = false(size(planeActiveBySegment));
returnedMaximumPlaneConstraintResidual = NaN;
returnedConstraintGenerationComplete   = false;
returnedSolveIndex                     = 0;
while true
    attemptedPlanePairs = retainedPlanePairs;
    [attemptX, ~, attemptExitFlag, attemptOutput] = ...
        coneprog(f, cones, A, b, Aeq, beq, lb, ub, options);
    solveCount = solveCount + 1;
    lastAttemptExitFlag = attemptExitFlag;
    if ~bmtpEngine.optimization.hasUsableConicIterate(attemptX, attemptExitFlag)
        break
    end

    violatedPairs         = false(size(planeActiveBySegment));
    attemptMaximumResidual = NaN;
    attemptIsComplete      = ~useConstraintGeneration;
    if useConstraintGeneration
        [violatedPairs, maximumOmittedResidual] = ...
            bmtpEngine.separation.findViolatedPlanePairs(attemptX, planes, planeActiveBySegment, ...
            attemptedPlanePairs, degree, slackColumnByPair, roundoffReserve_units, ...
            options.ConstraintTolerance);
        loadedResidual = -Inf;
        if size(A, 1) > baseInequalityCount
            loadedResidual = max(A(baseInequalityCount + 1:end, :) * attemptX - ...
                b(baseInequalityCount + 1:end));
        end
        attemptMaximumResidual = max(loadedResidual, maximumOmittedResidual);
        attemptIsComplete = ~any(violatedPairs, 'all') && ...
            attemptMaximumResidual <= options.ConstraintTolerance;
    end

    % Snapshot the last usable proposal before adding rows it has never
    % solved. A later numerical failure cannot erase this truthful record;
    % the caller still independently proves it before any retention.
    returnedX                              = attemptX;
    returnedExitFlag                       = attemptExitFlag;
    returnedOutput                         = attemptOutput;
    returnedPlanePairs                     = attemptedPlanePairs;
    returnedMaximumPlaneConstraintResidual = attemptMaximumResidual;
    returnedConstraintGenerationComplete   = attemptIsComplete;
    returnedSolveIndex                     = solveCount;
    if ~useConstraintGeneration || ~any(violatedPairs, 'all')
        break
    end
    retainedPlanePairs = attemptedPlanePairs | violatedPairs;
    [newRows, newBounds] = bmtpEngine.separation.createSelectedPlaneRows(planes, ...
        violatedPairs, degree, variableCount, slackColumnByPair, roundoffReserve_units);
    A = [A; newRows]; %#ok<AGROW>
    b = [b; newBounds]; %#ok<AGROW>
end
if isempty(returnedX)
    x        = attemptX;
    exitFlag = attemptExitFlag;
    output   = attemptOutput;
    returnedPlanePairs = attemptedPlanePairs;
else
    x        = returnedX;
    exitFlag = returnedExitFlag;
    output   = returnedOutput;
end
output.TotalTime_s                    = toc(solverTimer);
output.OptimizationConverged          = exitFlag > 0;
output.SolveCount                     = solveCount;
output.ConstraintGenerationApplied    = useConstraintGeneration;
output.ConstraintGenerationRoundCount = max(0, solveCount - 1);
output.ConstraintGenerationComplete   = returnedConstraintGenerationComplete;
output.MaximumPlaneConstraintResidual = returnedMaximumPlaneConstraintResidual;
output.LoadedPlanePairCount           = nnz(returnedPlanePairs);
output.ReturnedSolveIndex             = returnedSolveIndex;
output.LastAttemptExitFlag            = lastAttemptExitFlag;
output.TerminatedAfterRetainedIterate = returnedSolveIndex > 0 && ...
    returnedSolveIndex < solveCount;
output.AttemptedLoadedPlanePairCount  = nnz(attemptedPlanePairs);
% An optimality stall does not establish physical infeasibility. Every finite
% retained iterate remains only a proposal for independent proof.
if ~bmtpEngine.optimization.hasUsableConicIterate(x, exitFlag)
    controlPoint_units = zeros(0, degree + 1, 2);
    segmentTime_s      = NaN;
    return;
end
segmentTime_s = maximumSegmentTime_s * max(x(powerIndex(4)), 0) ^ (1 / 3);
if ~returnsCommonSegmentTime
    segmentTime_s = segmentTime_s * segmentRatio;
end
controlPoint_units = permute(reshape(x(1:controlCount), 2, degree + 1, segmentCount), [3 2 1]);
end

%% Section 4: Local Functions
function soc = createTravelBoundCones(variableCount, travelBoundIndex, segmentCount, degree)
    % Bound travel by the sum of Bezier control-edge lengths.
    if isempty(travelBoundIndex)
        emptyCone = secondordercone( ...
            zeros(2, variableCount), zeros(2, 1), zeros(variableCount, 1), 0);
        soc = repmat(emptyCone, 0, 1);
        return;
    end
    emptyCone  = secondordercone( ...
        zeros(2, variableCount), zeros(2, 1), zeros(variableCount, 1), 0);
    soc        = repmat(emptyCone, numel(travelBoundIndex), 1);
    boundIndex = 0;
    for segmentIndex = 1:segmentCount
        for controlIndex = 0:degree - 1
            boundIndex = boundIndex + 1;
            coneA      = zeros(2, variableCount);
            for axisIndex = 1:2
                firstIndex = bmtpEngine.optimization.controlIndexOf( ...
                    segmentIndex, controlIndex, axisIndex, degree);
                secondIndex = bmtpEngine.optimization.controlIndexOf( ...
                    segmentIndex, controlIndex + 1, axisIndex, degree);
                coneA(axisIndex, [firstIndex secondIndex]) = [-1 1];
            end
            coneC = zeros(variableCount, 1);
            coneC(travelBoundIndex(boundIndex)) = 1;
            soc(boundIndex) = secondordercone(coneA, zeros(2, 1), coneC, 0);
        end
    end
end
