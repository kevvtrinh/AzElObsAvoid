function [inequalityMatrix, equalityMatrix] = ...
        createConstraintMatrices( ...
        segmentCount, duration_s, allowAzimuthWrapping, ...
        seedCorridor, corridor, corridorTau, corridorNormal, ...
        collinearityDirection, constraintLayout)
%% Section 0: Header & Readme
% SYNTAX
%   [inequalityMatrix, equalityMatrix] = ...
%       obstacleAvoidance.planner.createConstraintMatrices( ...
%       segmentCount, duration_s, allowAzimuthWrapping, ...
%       seedCorridor, corridor, corridorTau)
%   [inequalityMatrix, equalityMatrix] = ...
%       obstacleAvoidance.planner.createConstraintMatrices( ...
%       segmentCount, duration_s, allowAzimuthWrapping, ...
%       seedCorridor, corridor, corridorTau, corridorNormal)
%   [inequalityMatrix, equalityMatrix] = ...
%       obstacleAvoidance.planner.createConstraintMatrices( ...
%       segmentCount, duration_s, allowAzimuthWrapping, ...
%       seedCorridor, corridor, corridorTau, corridorNormal, ...
%       collinearityDirection)
%   [inequalityMatrix, equalityMatrix] = ...
%       obstacleAvoidance.planner.createConstraintMatrices( ...
%       segmentCount, duration_s, allowAzimuthWrapping, ...
%       seedCorridor, corridor, corridorTau, corridorNormal, ...
%       collinearityDirection, constraintLayout)
%**************************************************************************
% PURPOSE
%   - Assemble exact fixed-duration HS3 jerk Jacobians in the raw constraint
%     order consumed by fmincon's linear interfaces.
%**************************************************************************
% INPUTS
%   - segmentCount (positive integer scalar), HS3 segment count.
%   - duration_s (positive scalar), complete motion duration in seconds.
%   - allowAzimuthWrapping (logical scalar), omits azimuth position bounds.
%   - seedCorridor (struct array), continuous exterior support records.
%   - corridor (struct array), fixed obstacle association records.
%   - corridorTau (numeric column), normalized obstacle constraint times.
%   - corridorNormal (M-by-2 numeric, optional)
%       Active normals evaluated at the current variable-duration geometry.
%   - collinearityDirection (1-by-2 numeric row, optional)
%       Replaces redundant normal endpoint rows with normal-jerk equations.
%   - constraintLayout (scalar struct, optional; default empty)
%       Precomputed corridor maps, row offsets, and direct-progress direction.
%**************************************************************************
% OUTPUTS
%   - inequalityMatrix (M-by-D numeric), exact dc/djerk matrix.
%   - equalityMatrix (numeric matrix), terminal P/V/A Jacobian and optional
%       direct-path normal-jerk equations.
%**************************************************************************
% UNITS
%   - Rows retain the heterogeneous physical units of their constraints.
%**************************************************************************

%% Section 1: Assemble Ordered Constraint Blocks

% The optimizer stores one jerk value per HS3 control point and axis. The
% polynomial sensitivity model tells how changing any jerk value changes
% position, velocity, and acceleration throughout the motion. Each helper
% converts that common sensitivity into rows matching the inequality values.
% This row order is essential because fmincon pairs values and derivatives
% by row number rather than by physical meaning.

monotonicityDirection = zeros(0, 2);
if nargin >= 9 && isfield(constraintLayout, "MonotonicityDirection")
    monotonicityDirection = constraintLayout.MonotonicityDirection;
end
hasPreparedLayout = nargin >= 9 && ...
    isfield(constraintLayout, "CorridorRowOffset");
if hasPreparedLayout
    % Coefficient Jacobians do not consume sampled position/velocity maps.
    % Avoid creating those unused maps when the corridor layout is prepared.
    sensitivityTau = zeros(0, 1);
else
    sensitivityTau = corridorTau;
    constraintLayout = struct( ...
        "MonotonicityDirection", monotonicityDirection);
end
sensitivity = hs3Engine.polynomial.createAffineSensitivityModel( ...
    segmentCount, duration_s, sensitivityTau);
continuousMatrix = continuousBoundConstraintMatrix( ...
    sensitivity, allowAzimuthWrapping);
monotonicityMatrix = monotonicityConstraintMatrix( ...
    sensitivity, constraintLayout);
seedMatrix = seedCorridorConstraintMatrix(sensitivity, seedCorridor);
if nargin >= 7 && ~isempty(corridorNormal)
    % Moving-obstacle normals depend on the current trial motion, so use the
    % normals evaluated during the same constraint call.
    corridorMatrix = corridorConstraintMatrix( ...
        sensitivity, corridor, corridorNormal, constraintLayout);
else
    % Fixed geometry carries its own normal and needs no time reevaluation.
    corridorMatrix = frozenCorridorConstraintMatrix( ...
        sensitivity, corridor, constraintLayout);
end
inequalityMatrix = [ ...
    continuousMatrix; monotonicityMatrix; seedMatrix; corridorMatrix];
controlCount = sensitivity.ControlCount;
terminalMap = sensitivity.terminalStateMap;
equalityMatrix = zeros(6, 2 * controlCount);
equalityMatrix([1 3 5], 1:controlCount) = terminalMap;
equalityMatrix([2 4 6], controlCount + 1:end) = terminalMap;
if nargin >= 8 && ~isempty(collinearityDirection)
    % For a certified straight route, constrain terminal motion along the
    % tangent and every jerk control along the normal. Endpoint equations
    % alone would not prevent sideways motion between the endpoints.
    tangent = collinearityDirection(:).';
    normal = [-tangent(2), tangent(1)];
    tangentMatrix = zeros(3, 2 * controlCount);
    for stateIndex = 1:3
        stateMap = terminalMap(stateIndex, :);
        tangentMatrix(stateIndex, 1:controlCount) = tangent(1) * stateMap;
        tangentMatrix(stateIndex, controlCount + 1:end) = ...
            tangent(2) * stateMap;
    end
    normalJerkMatrix = [ ...
        normal(1) * eye(controlCount), normal(2) * eye(controlCount)];
    equalityMatrix = [tangentMatrix; normalJerkMatrix];
end
end

function matrix = corridorConstraintMatrix( ...
        sensitivity, corridor, normal, constraintLayout)
% Differentiate each sub-interval hull while geometry remains fixed in jerk.
controlCount = sensitivity.ControlCount;
recordCount = numel(corridor);
coefficientCount = size(sensitivity.positionPowerMap, 2);
if recordCount == 0
    matrix = zeros(0, 2 * controlCount);
    return;
end
if size(normal, 1) ~= recordCount || size(normal, 2) ~= 2
    error("createConstraintMatrices:InvalidCorridorNormals", ...
        "corridorNormal must contain one two-axis row per association.");
end
if isfield(constraintLayout, "CorridorRowOffset")
    if constraintLayout.PositionCoefficientCount ~= coefficientCount || ...
            numel(constraintLayout.CorridorRowOffset) ~= recordCount + 1
        error("createConstraintMatrices:InvalidConstraintLayout", ...
            "The prepared corridor layout does not match the sensitivity model.");
    end
    segmentIndex = constraintLayout.CorridorSegmentIndex;
    hullMap = constraintLayout.CorridorHullMap;
    rowOffset = constraintLayout.CorridorRowOffset;
else
    useIntervalHull = [corridor.UseIntervalHull].';
    effectiveTauEnd = [corridor.TauEnd];
    effectiveTauEnd(~useIntervalHull.') = [corridor(~useIntervalHull).Tau];
    [segmentIndex, hullMap] = ...
        hs3Engine.polynomial.createSubintervalBernsteinMap( ...
        [corridor.Tau], effectiveTauEnd, ...
        size(sensitivity.positionPowerMap, 1), coefficientCount);
    rowOffset = [0; cumsum(1 + ...
        (coefficientCount - 1) * useIntervalHull)];
end
matrix = zeros(rowOffset(end), 2 * controlCount);
selectedPowerMap = permute( ...
    sensitivity.positionPowerMap(segmentIndex, :, :), [2 3 1]);
hullByRecord = permute(pagemtimes(hullMap, selectedPowerMap), [1 3 2]);
for associationIndex = 1:recordCount
    % One row checks a single instant. An interval association uses all
    % Bernstein coefficients to cover its complete time interval.
    matrixRows = rowOffset(associationIndex) + 1: ...
        rowOffset(associationIndex + 1);
    associationRowCount = numel(matrixRows);
    positionMap = reshape( ...
        hullByRecord(1:associationRowCount, associationIndex, :), ...
        associationRowCount, controlCount);
    matrix(matrixRows, 1:controlCount) = ...
        -normal(associationIndex, 1) * positionMap;
    matrix(matrixRows, controlCount + 1:end) = ...
        -normal(associationIndex, 2) * positionMap;
end
end

function matrix = continuousBoundConstraintMatrix( ...
        sensitivity, allowAzimuthWrapping)
% Preserve the existing per-segment, per-axis Bernstein constraint order.
% A polynomial stays inside the range of its Bernstein coefficients. This
% enforces a continuous bound without relying on point samples to find peaks.
positionGradient = bernsteinViolationGradient( ...
    sensitivity.positionPowerMap);
velocityGradient = bernsteinViolationGradient( ...
    sensitivity.velocityPowerMap);
accelerationGradient = bernsteinViolationGradient( ...
    sensitivity.accelerationPowerMap);
jerkGradient = bernsteinViolationGradient(sensitivity.jerkPowerMap);
if allowAzimuthWrapping
    firstAxisGradient = cat(1, velocityGradient, ...
        accelerationGradient, jerkGradient);
else
    firstAxisGradient = cat(1, positionGradient, velocityGradient, ...
        accelerationGradient, jerkGradient);
end
secondAxisGradient = cat(1, positionGradient, velocityGradient, ...
    accelerationGradient, jerkGradient);
firstRowCount = size(firstAxisGradient, 1);
secondRowCount = size(secondAxisGradient, 1);
segmentCount = size(firstAxisGradient, 2);
controlCount = sensitivity.ControlCount;
gradientBySegment = zeros( ...
    firstRowCount + secondRowCount, segmentCount, 2 * controlCount);
gradientBySegment(1:firstRowCount, :, 1:controlCount) = firstAxisGradient;
gradientBySegment(firstRowCount + 1:end, :, controlCount + 1:end) = ...
    secondAxisGradient;
matrix = reshape(gradientBySegment, [], 2 * controlCount);
end

function gradient = bernsteinViolationGradient(powerMap)
% Convert every coefficient sensitivity through the exact Bernstein basis.
% Positive and negative rows represent upper and lower c <= 0 inequalities.
coefficientCount = size(powerMap, 2);
segmentCount = size(powerMap, 1);
controlCount = size(powerMap, 3);
powerMatrix = reshape(permute(powerMap, [2 1 3]), ...
    coefficientCount, []);
bernsteinMatrix = hs3Engine.polynomial.convertPowerToBernstein(powerMatrix);
bernsteinMap = reshape(bernsteinMatrix, ...
    coefficientCount, segmentCount, controlCount);
gradient = cat(1, bernsteinMap, -bernsteinMap);
end

function matrix = monotonicityConstraintMatrix(sensitivity, constraintLayout)
% Differentiate continuous nonnegative progress along a certified direct line.
controlCount = sensitivity.ControlCount;
if ~isfield(constraintLayout, "MonotonicityDirection") || ...
        isempty(constraintLayout.MonotonicityDirection)
    matrix = zeros(0, 2 * controlCount);
    return;
end
powerMap = sensitivity.velocityPowerMap;
coefficientCount = size(powerMap, 2);
segmentCount = size(powerMap, 1);
powerMatrix = reshape(permute(powerMap, [2 1 3]), ...
    coefficientCount, []);
bernsteinMap = reshape( ...
    hs3Engine.polynomial.convertPowerToBernstein(powerMatrix), ...
    coefficientCount, segmentCount, controlCount);
tangent = constraintLayout.MonotonicityDirection(:).';
matrix = zeros(coefficientCount * segmentCount, 2 * controlCount);
matrix(:, 1:controlCount) = ...
    -tangent(1) * reshape(bernsteinMap, [], controlCount);
matrix(:, controlCount + 1:end) = ...
    -tangent(2) * reshape(bernsteinMap, [], controlCount);
end

function matrix = seedCorridorConstraintMatrix(sensitivity, corridor)
% Differentiate each six-coefficient exterior support projection.
% Projecting all six position coefficients onto an outward normal keeps the
% complete segment on the certified side of the supporting line.
controlCount = sensitivity.ControlCount;
recordCount = numel(corridor);
matrix = zeros(6 * recordCount, 2 * controlCount);
if recordCount == 0
    return;
end
segmentIndex = [corridor.SegmentIndex].';
normal = vertcat(corridor.Normal);
selectedPowerMap = sensitivity.positionPowerMap(segmentIndex, :, :);
powerMatrix = reshape(permute(selectedPowerMap, [2 1 3]), 6, []);
bernsteinMap = reshape(hs3Engine.polynomial.convertPowerToBernstein(powerMatrix), ...
    6, recordCount, controlCount);
azimuthMap = -reshape(normal(:, 1), 1, recordCount, 1) .* bernsteinMap;
elevationMap = -reshape(normal(:, 2), 1, recordCount, 1) .* bernsteinMap;
matrix(:, 1:controlCount) = reshape(azimuthMap, [], controlCount);
matrix(:, controlCount + 1:end) = reshape(elevationMap, [], controlCount);
end

function matrix = frozenCorridorConstraintMatrix( ...
        sensitivity, corridor, constraintLayout)
% Differentiate fixed-time obstacle supports over their stored sub-intervals.
if isempty(corridor)
    matrix = zeros(0, 2 * sensitivity.ControlCount);
    return;
end
if any(~[corridor.GeometryIsFixed])
    error("createConstraintMatrices:NonfixedCorridor", ...
        "Every fixed-time HS3 corridor association must freeze its geometry.");
end
matrix = corridorConstraintMatrix( ...
    sensitivity, corridor, vertcat(corridor.FixedNormal), constraintLayout);
end
