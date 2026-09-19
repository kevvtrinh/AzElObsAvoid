function [controlPoint_units, segmentTime_s, exitFlag, output, constraintBase] = solveTimedTrajectoryStep( ...
    segmentCount, degree, start_units, goal_units, limits, planes, ...
    reserve_units, maximumMotionDuration_s, goalTimeMode, options, ...
    minimumMotionDuration_s, segmentRatio, constraintBase)
%% Section 0: Header & Readme
% SYNTAX
%   [controlPoint_units, segmentTime_s, exitFlag, output] = ...
%       bmtpEngine.optimization.solveTimedTrajectoryStep(segmentCount, degree, ...
%       start_units, goal_units, limits, planes, reserve_units, ...
%       maximumMotionDuration_s, goalTimeMode, options)
%   [controlPoint_units, segmentTime_s, exitFlag, output] = ...
%       bmtpEngine.optimization.solveTimedTrajectoryStep(segmentCount, degree, ...
%       start_units, goal_units, limits, planes, reserve_units, ...
%       maximumMotionDuration_s, goalTimeMode, options, ...
%       minimumMotionDuration_s, segmentRatio)
%   [controlPoint_units, segmentTime_s, exitFlag, output, constraintBase] = ...
%       bmtpEngine.optimization.solveTimedTrajectoryStep(..., ...
%       minimumMotionDuration_s, segmentRatio, constraintBase)
%**************************************************************************
% PURPOSE
%   - Solve one convex trajectory step for fixed separating lines, timing
%     policy, and derivative limits.
%**************************************************************************
% INPUTS
%   - segmentCount (positive integer scalar)
%       Number of composite Bezier segments.
%   - degree (positive integer scalar)
%       Degree of each Bezier segment.
%   - start_units (1-by-2 numeric row)
%       Fixed initial position.
%   - goal_units (1-by-2 numeric row)
%       Fixed goal position.
%   - limits (scalar struct)
%       Workspace, velocity, acceleration, and jerk limits.
%   - planes (S-by-R struct array)
%       Fixed active separating-line constraints.
%   - reserve_units (nonnegative scalar)
%       Numerical separation reserve.
%   - maximumMotionDuration_s (positive scalar)
%       Upper bound or fixed motion duration.
%   - goalTimeMode (scalar text)
%       earliestArrival or fixedArrival.
%   - options (coneprog options)
%       Numerical solver controls.
%   - minimumMotionDuration_s (nonnegative scalar, optional; default 0)
%       Lower arrival bound, at most maximumMotionDuration_s.
%   - segmentRatio (S-by-1 positive vector, optional; default all ones)
%       Relative physical span durations.
%   - constraintBase (scalar struct, optional)
%       Reusable invariant constraint arrays for the unchanged formulation.
%**************************************************************************
% OUTPUTS
%   - controlPoint_units (S-by-(D+1)-by-2 numeric array)
%       Solved control points, or an empty array on expected solve failure.
%       Invalid input throws an error.
%   - segmentTime_s (numeric scalar or S-by-1 numeric vector)
%       Common segment time, or the per-segment durations implied by
%       segmentRatio. NaN on expected solve failure.
%   - exitFlag (numeric scalar)
%       Original coneprog status.
%   - output (scalar struct)
%       Solver status, diagnostics, and measured time. Finite fixed-clock -7
%       iterates are proposals requiring final independent certification.
%   - constraintBase (scalar struct)
%       Invariant constraint arrays for reuse with the same formulation.
%**************************************************************************
% UNITS
%   - Position is coordinate units and time is seconds.
%**************************************************************************

%% Section 1: Create Decision Bounds And Continuity Rows
if nargin < 11
    minimumMotionDuration_s = 0;
end
returnsCommonSegmentTime = nargin < 12 || isempty(segmentRatio);
if returnsCommonSegmentTime
    segmentRatio = ones(segmentCount, 1);
end
if nargin < 13
    constraintBase = struct();
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
    initialPlanePairs, degree, variableCount, slackColumnByPair, reserve_units);
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
            attemptedPlanePairs, degree, slackColumnByPair, reserve_units, ...
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
    % the caller still independently certifies it before any retention.
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
        violatedPairs, degree, variableCount, slackColumnByPair, reserve_units);
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
% retained iterate remains only a proposal for independent certification.
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
