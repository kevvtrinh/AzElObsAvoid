function [controlPoint_units, segmentTime_s, exitFlag, solverOutput, savedTrajectoryConstraints] = ...
    solveTimedTrajectoryStep(solverRequest, trajectoryStep)
%% Section 0: Header & Readme
% SYNTAX
%   [controlPoint_units, segmentTime_s, exitFlag, solverOutput, savedTrajectoryConstraints] = ...
%       bmtpEngine.optimization.solveTimedTrajectoryStep(solverRequest, trajectoryStep)
%**************************************************************************
% PURPOSE
%   - Solve for a new curve using fixed separating lines and duration ratios.
%     Seek an earlier arrival, or shorten the path when arrival time is fixed.
%     The returned candidate still requires the caller's full motion checks.
%**************************************************************************
% INPUTS
%   - solverRequest (scalar struct)
%       The engine solve request from createSolveRequest. This step reads
%       Degree, the InitialState and GoalState positions, Limits, and
%       TimedTrajectoryOptions.
%   - trajectoryStep (scalar struct)
%       What this one step is asked to do, every field required:
%       SegmentCount (positive integer), Planes (S-by-R struct array of
%       fixed active separating lines), RoundoffReserve_units (nonnegative
%       scalar), MaximumMotionDuration_s (positive scalar upper bound or
%       fixed duration), GoalTimeMode (earliestArrival or fixedArrival),
%       MinimumMotionDuration_s (nonnegative scalar lower arrival bound, at
%       most the maximum), SegmentRatio (S-by-1 positive relative segment
%       durations, or [] for one common segment time), and ConstraintBase
%       (saved workspace, endpoint, join, and motion-limit constraints from
%       a matching earlier step, or struct() to build them).
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
%   - solverOutput (scalar struct)
%       Status, measurements, and elapsed time for the returned solver values.
%       A finite -7 result may be returned for checking, but is not yet accepted.
%   - savedTrajectoryConstraints (scalar struct)
%       Constraints and their inputs, for reuse when the next step matches.
%**************************************************************************
% UNITS
%   - Position is coordinate units and time is seconds.
%**************************************************************************

%% Section 1: Read Inputs And Build Shared Motion Constraints

validateattributes(trajectoryStep, {'struct'}, {'scalar'});
requiredStepFields = ["SegmentCount", "Planes", "RoundoffReserve_units", "MaximumMotionDuration_s", ...
    "GoalTimeMode", "MinimumMotionDuration_s", "SegmentRatio", "ConstraintBase"];
assert(all(isfield(trajectoryStep, requiredStepFields)), 'bmtpEngine:InvalidStep', ...
    'A timed trajectory step declares every one of: %s.', strjoin(requiredStepFields, ', '));
segmentCount               = trajectoryStep.SegmentCount;
separatingPlanes           = trajectoryStep.Planes;
roundoffReserve_units      = trajectoryStep.RoundoffReserve_units;
maximumMotionDuration_s    = trajectoryStep.MaximumMotionDuration_s;
goalTimeMode               = trajectoryStep.GoalTimeMode;
minimumMotionDuration_s    = trajectoryStep.MinimumMotionDuration_s;
segmentTimeRatios          = trajectoryStep.SegmentRatio;
savedTrajectoryConstraints = trajectoryStep.ConstraintBase;

degree        = solverRequest.Degree;
start_units   = solverRequest.InitialState.position_units;
goal_units    = solverRequest.GoalState.position_units;
limits        = solverRequest.Limits;
solverOptions = solverRequest.TimedTrajectoryOptions;

returnsCommonSegmentTime = isempty(segmentTimeRatios);
if returnsCommonSegmentTime
    segmentTimeRatios = ones(segmentCount, 1);
end
segmentTimeRatios = double(segmentTimeRatios(:));
validateattributes(segmentTimeRatios, {'numeric'}, ...
    {'real', 'finite', 'positive', 'numel', segmentCount});
validateattributes(minimumMotionDuration_s, {'numeric'}, ...
    {'real', 'finite', 'scalar', 'nonnegative', '<=', maximumMotionDuration_s});

% Solver values contain x/y control coordinates, four time-power variables,
% and (for fixed arrival) one length bound for every control-polygon edge.
controlVariableCount   = segmentCount * (degree + 1) * 2;
timePowerIndices       = controlVariableCount + (1:4);
edgeLengthBoundCount   = (goalTimeMode ~= "earliestArrival") * segmentCount * degree;
edgeLengthBoundIndices = controlVariableCount + 4 + (1:edgeLengthBoundCount);
decisionVariableCount  = controlVariableCount + 4 + edgeLengthBoundCount;

planeActiveBySegment = reshape([separatingPlanes.Active], size(separatingPlanes));
maximumTimeScale_s   = maximumMotionDuration_s / sum(segmentTimeRatios);

% Three identical controls at each endpoint set zero velocity and
% acceleration there. This timed step uses the requested endpoint positions.
endpointControlPoint_units = zeros(segmentCount, degree + 1, 2);
endpointControlPoint_units(1, 1:3, :)           = repmat(reshape(start_units, 1, 1, 2), 1, 3, 1);
endpointControlPoint_units(end, end - 2:end, :) = repmat(reshape(goal_units, 1, 1, 2), 1, 3, 1);
% Reuse the shared matrices only when every input used to build them matches.
% Obstacle-line rows are added separately below.
sharedConstraintInputs = struct( ...
    "SegmentCount",     segmentCount, ...
    "Degree",           degree, ...
    "BoundaryControls", endpointControlPoint_units, ...
    "Limits",           limits, ...
    "VariableCount",    decisionVariableCount, ...
    "SegmentRatio",     segmentTimeRatios);
canReuseSharedConstraints = isstruct(savedTrajectoryConstraints) && isscalar(savedTrajectoryConstraints) && ...
    isfield(savedTrajectoryConstraints, 'Key') && isequaln(savedTrajectoryConstraints.Key, sharedConstraintInputs);
if ~canReuseSharedConstraints
    [inequalityMatrix, equalityMatrix, equalityValues, lowerBounds, upperBounds] = ...
        bmtpEngine.optimization.createTrajectoryConstraints( ...
        segmentCount, degree, endpointControlPoint_units, limits, decisionVariableCount, segmentTimeRatios, []);
    savedTrajectoryConstraints = struct( ...
        "A",   inequalityMatrix, ...
        "Aeq", equalityMatrix, ...
        "beq", equalityValues, ...
        "lb",  lowerBounds, ...
        "ub",  upperBounds, ...
        "Key", sharedConstraintInputs);
else
    inequalityMatrix = savedTrajectoryConstraints.A;
    equalityMatrix   = savedTrajectoryConstraints.Aeq;
    equalityValues   = savedTrajectoryConstraints.beq;
    lowerBounds      = savedTrajectoryConstraints.lb;
    upperBounds      = savedTrajectoryConstraints.ub;
end
% Use time as a fraction of its allowed maximum. For example, powers of
% 1000 seconds are 1000, 1e6, and 1e9; scaled time powers stay between 0 and 1.
% This avoids giving the solver columns with very different numeric scales.
% Multiplying the matrix columns by the matching physical-time powers keeps
% the speed, acceleration, and jerk constraints in physical units.
for derivativeOrder = 1:3
    inequalityMatrix(:, timePowerIndices(derivativeOrder + 1)) = ...
        inequalityMatrix(:, timePowerIndices(derivativeOrder + 1)) * maximumTimeScale_s ^ derivativeOrder;
end
lowerBounds(edgeLengthBoundIndices) = 0;

%% Section 2: Choose Which Separating-Line Rows To Load First

motionLimitRowCount  = 4 * segmentCount * (3 * degree - 3);
initiallyLoadedPairs = planeActiveBySegment;
% Load all lines for earliest arrival with at most 2048 active pairs. For
% larger problems or fixed arrival, begin without line rows. Check every
% omitted pair after each solve, then add the largest violation per segment.
% Repeat until no omitted pair exceeds tolerance or the solve fails.
maximumFullyLoadedPairCount = 2048;
addLineConstraintsAsNeeded  = goalTimeMode == "fixedArrival" || ...
    nnz(planeActiveBySegment) > maximumFullyLoadedPairCount;
if addLineConstraintsAsNeeded
    initiallyLoadedPairs(:) = false;
end
% Zero column indices mean this step has no variables that relax line bounds.
slackColumnByPair = zeros(size(planeActiveBySegment));
[lineConstraintRows, lineConstraintBounds] = bmtpEngine.separation.createSelectedPlaneRows(separatingPlanes, ...
    initiallyLoadedPairs, degree, decisionVariableCount, slackColumnByPair, roundoffReserve_units);
inequalityMatrix = [inequalityMatrix; lineConstraintRows];
inequalityBounds = [zeros(motionLimitRowCount, 1); lineConstraintBounds];

%% Section 3: Solve And Add The Selected Violated Constraints

solverCones = [bmtpEngine.optimization.createTimePowerCones(decisionVariableCount, timePowerIndices); ...
    createTravelBoundCones(decisionVariableCount, edgeLengthBoundIndices, segmentCount, degree)];
objectiveWeights = zeros(decisionVariableCount, 1);
% Minimize the cubic scaled time to seek an earlier arrival.
% For fixed arrival, minimize the sum of control-polygon edge lengths.
if goalTimeMode == "earliestArrival"
    objectiveWeights(timePowerIndices(4)) = 1;
else
    objectiveWeights(edgeLengthBoundIndices) = 1;
end
% A fixed duration uses all four scaled powers equal to 1. Otherwise bound
% them between the requested minimum-duration powers and the maximum 1.
if goalTimeMode == "fixedArrival"
    lowerBounds(timePowerIndices) = 1;
    upperBounds(timePowerIndices) = 1;
else
    upperBounds(timePowerIndices) = 1;
    minimumTimeFraction = minimumMotionDuration_s / maximumMotionDuration_s;
    lowerBounds(timePowerIndices) = [1; minimumTimeFraction; ...
        minimumTimeFraction ^ 2; minimumTimeFraction ^ 3];
end

solverTimer      = tic;
loadedPlanePairs = initiallyLoadedPairs;
solveCount       = 0;

savedSolverValues             = [];
savedExitFlag                 = NaN;
savedSolverOutput             = struct();
savedPlanePairs               = false(size(planeActiveBySegment));
savedMaximumLineViolation     = NaN;
savedLineConstraintsSatisfied = false;
savedSolveIndex               = 0;
while true
    attemptedPlanePairs = loadedPlanePairs;
    [attemptedSolverValues, ~, attemptedExitFlag, attemptedSolverOutput] = ...
        coneprog(objectiveWeights, solverCones, inequalityMatrix, inequalityBounds, ...
        equalityMatrix, equalityValues, lowerBounds, upperBounds, solverOptions);
    solveCount          = solveCount + 1;
    lastAttemptExitFlag = attemptedExitFlag;
    if ~bmtpEngine.optimization.hasUsableConicIterate(attemptedSolverValues, attemptedExitFlag)
        break
    end

    violatedPairs                   = false(size(planeActiveBySegment));
    maximumAttemptLineViolation     = NaN;
    attemptLineConstraintsSatisfied = ~addLineConstraintsAsNeeded;
    if addLineConstraintsAsNeeded
        % A positive row violation means a line bound was exceeded. Check
        % both already-loaded rows and every pair not yet loaded.
        [violatedPairs, maximumUnloadedLineViolation] = ...
            bmtpEngine.separation.findViolatedPlanePairs( ...
            attemptedSolverValues, separatingPlanes, planeActiveBySegment, ...
            attemptedPlanePairs, degree, slackColumnByPair, roundoffReserve_units, ...
            solverOptions.ConstraintTolerance);
        maximumLoadedLineViolation = -Inf;
        if size(inequalityMatrix, 1) > motionLimitRowCount
            maximumLoadedLineViolation = max(inequalityMatrix(motionLimitRowCount + 1:end, :) * attemptedSolverValues - ...
                inequalityBounds(motionLimitRowCount + 1:end));
        end
        maximumAttemptLineViolation     = max(maximumLoadedLineViolation, maximumUnloadedLineViolation);
        attemptLineConstraintsSatisfied = ~any(violatedPairs, 'all') && ...
            maximumAttemptLineViolation <= solverOptions.ConstraintTolerance;
    end

    % Save values and diagnostics together before adding more constraints.
    % If the next solve fails, return this candidate with its own checks and
    % loaded-pair count. The caller must still validate the complete motion.
    savedSolverValues             = attemptedSolverValues;
    savedExitFlag                 = attemptedExitFlag;
    savedSolverOutput             = attemptedSolverOutput;
    savedPlanePairs               = attemptedPlanePairs;
    savedMaximumLineViolation     = maximumAttemptLineViolation;
    savedLineConstraintsSatisfied = attemptLineConstraintsSatisfied;
    savedSolveIndex               = solveCount;
    if ~addLineConstraintsAsNeeded || ~any(violatedPairs, 'all')
        break
    end
    loadedPlanePairs = attemptedPlanePairs | violatedPairs;
    [newLineRows, newLineBounds] = bmtpEngine.separation.createSelectedPlaneRows(separatingPlanes, ...
        violatedPairs, degree, decisionVariableCount, slackColumnByPair, roundoffReserve_units);
    inequalityMatrix = [inequalityMatrix; newLineRows]; %#ok<AGROW>
    inequalityBounds = [inequalityBounds; newLineBounds]; %#ok<AGROW>
end

%% Section 4: Return Matching Values, Durations, And Diagnostics

if isempty(savedSolverValues)
    solverValues    = attemptedSolverValues;
    exitFlag        = attemptedExitFlag;
    solverOutput    = attemptedSolverOutput;
    savedPlanePairs = attemptedPlanePairs;
else
    solverValues = savedSolverValues;
    exitFlag     = savedExitFlag;
    solverOutput = savedSolverOutput;
end
solverOutput.TotalTime_s                    = toc(solverTimer);
solverOutput.OptimizationConverged          = exitFlag > 0;
solverOutput.SolveCount                     = solveCount;
solverOutput.ConstraintGenerationApplied    = addLineConstraintsAsNeeded;
solverOutput.ConstraintGenerationRoundCount = max(0, solveCount - 1);
solverOutput.ConstraintGenerationComplete   = savedLineConstraintsSatisfied;
solverOutput.MaximumPlaneConstraintResidual = savedMaximumLineViolation;
solverOutput.LoadedPlanePairCount           = nnz(savedPlanePairs);
solverOutput.ReturnedSolveIndex             = savedSolveIndex;
solverOutput.LastAttemptExitFlag            = lastAttemptExitFlag;
solverOutput.TerminatedAfterRetainedIterate = savedSolveIndex > 0 && ...
    savedSolveIndex < solveCount;
solverOutput.AttemptedLoadedPlanePairCount  = nnz(attemptedPlanePairs);
% A retained finite result remains a candidate for the caller's checks.
% When no usable values exist, return empty controls and a NaN duration.
if ~bmtpEngine.optimization.hasUsableConicIterate(solverValues, exitFlag)
    controlPoint_units = zeros(0, degree + 1, 2);
    segmentTime_s      = NaN;
    return;
end
% Convert the cubic scaled time back to seconds, then apply each segment's
% duration ratio. Jerk limits depend on duration^3, hence the cube root.
segmentTime_s = maximumTimeScale_s * max(solverValues(timePowerIndices(4)), 0) ^ (1 / 3);
if ~returnsCommonSegmentTime
    segmentTime_s = segmentTime_s * segmentTimeRatios;
end
controlPoint_units = permute(reshape(solverValues(1:controlVariableCount), 2, degree + 1, segmentCount), [3 2 1]);
end

%% Section 5: Local Functions

function edgeLengthCones = createTravelBoundCones( ...
        decisionVariableCount, edgeLengthBoundIndices, segmentCount, degree)
    % Give each adjacent control-point pair a variable z with
    % norm(nextPoint - currentPoint) <= z. Minimizing the sum of z shortens
    % the control polygon, whose length bounds the Bezier curve length.
    if isempty(edgeLengthBoundIndices)
        emptyCone = secondordercone( ...
            zeros(2, decisionVariableCount), zeros(2, 1), zeros(decisionVariableCount, 1), 0);
        edgeLengthCones = repmat(emptyCone, 0, 1);
        return;
    end
    emptyCone = secondordercone( ...
        zeros(2, decisionVariableCount), zeros(2, 1), zeros(decisionVariableCount, 1), 0);
    edgeLengthCones = repmat(emptyCone, numel(edgeLengthBoundIndices), 1);
    edgeIndex       = 0;
    for segmentIndex = 1:segmentCount
        for controlPointIndex = 0:degree - 1
            edgeIndex   = edgeIndex + 1;
            leftSideMap = zeros(2, decisionVariableCount);
            for axisIndex = 1:2
                firstCoordinateIndex = bmtpEngine.optimization.controlIndexOf( ...
                    segmentIndex, controlPointIndex, axisIndex, degree);
                secondCoordinateIndex = bmtpEngine.optimization.controlIndexOf( ...
                    segmentIndex, controlPointIndex + 1, axisIndex, degree);
                leftSideMap(axisIndex, [firstCoordinateIndex secondCoordinateIndex]) = [-1 1];
            end
            rightSideWeights = zeros(decisionVariableCount, 1);
            rightSideWeights(edgeLengthBoundIndices(edgeIndex)) = 1;
            edgeLengthCones(edgeIndex) = secondordercone(leftSideMap, zeros(2, 1), rightSideWeights, 0);
        end
    end
end
