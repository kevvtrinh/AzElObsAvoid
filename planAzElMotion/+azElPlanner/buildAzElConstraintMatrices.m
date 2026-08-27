function [inequalityMatrix, equalityMatrix] = ...
        buildAzElConstraintMatrices( ...
        segmentCount, duration_s, allowAzimuthWrapping, ...
        seedCorridor, corridor, corridorTau, corridorNormal)
%% Section 0: Header & Readme
% SYNTAX
%   [inequalityMatrix, equalityMatrix] = ...
%       azElPlanner.buildAzElConstraintMatrices( ...
%       segmentCount, duration_s, allowAzimuthWrapping, ...
%       seedCorridor, corridor, corridorTau)
%   [inequalityMatrix, equalityMatrix] = ...
%       azElPlanner.buildAzElConstraintMatrices( ...
%       segmentCount, duration_s, allowAzimuthWrapping, ...
%       seedCorridor, corridor, corridorTau, corridorNormal)
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
%**************************************************************************
% OUTPUTS
%   - inequalityMatrix (M-by-D numeric), exact dc/djerk matrix.
%   - equalityMatrix (6-by-D numeric), terminal P/V/A Jacobian.
%**************************************************************************
% UNITS
%   - Rows retain the heterogeneous physical units of their constraints.
%**************************************************************************

%% Section 1: Assemble Ordered Constraint Blocks

sensitivity = hs3Internal.affineSensitivity( ...
    segmentCount, duration_s, corridorTau);
continuousMatrix = continuousBoundConstraintMatrix( ...
    sensitivity, allowAzimuthWrapping);
seedMatrix = seedCorridorConstraintMatrix(sensitivity, seedCorridor);
if nargin >= 7
    corridorMatrix = corridorConstraintMatrix( ...
        sensitivity, corridor, corridorNormal);
else
    corridorMatrix = frozenCorridorConstraintMatrix(sensitivity, corridor);
end
inequalityMatrix = [continuousMatrix; seedMatrix; corridorMatrix];
controlCount = sensitivity.ControlCount;
terminalMap = sensitivity.terminalStateMap;
equalityMatrix = zeros(6, 2 * controlCount);
equalityMatrix([1 3 5], 1:controlCount) = terminalMap;
equalityMatrix([2 4 6], controlCount + 1:end) = terminalMap;
end

function matrix = corridorConstraintMatrix(sensitivity, corridor, normal)
% Differentiate each sub-interval hull while geometry remains fixed in jerk.
controlCount = sensitivity.ControlCount;
recordCount = numel(corridor);
coefficientCount = size(sensitivity.positionPowerMap, 2);
if recordCount == 0
    matrix = zeros(0, 2 * controlCount);
    return;
end
useIntervalHull = [corridor.UseIntervalHull].';
rowCount = sum(1 + (coefficientCount - 1) * useIntervalHull);
matrix = zeros(rowCount, 2 * controlCount);
if size(normal, 1) ~= recordCount || size(normal, 2) ~= 2
    error("buildAzElConstraintMatrices:InvalidCorridorNormals", ...
        "corridorNormal must contain one two-axis row per association.");
end
effectiveTauEnd = [corridor.TauEnd];
effectiveTauEnd(~useIntervalHull.') = [corridor(~useIntervalHull).Tau];
[segmentIndex, hullMap] = hs3Internal.subintervalHullMap( ...
    [corridor.Tau], effectiveTauEnd, ...
    size(sensitivity.positionPowerMap, 1), coefficientCount);
selectedPowerMap = permute( ...
    sensitivity.positionPowerMap(segmentIndex, :, :), [2 3 1]);
hullByRecord = permute(pagemtimes(hullMap, selectedPowerMap), [1 3 2]);
nextRow = 1;
for associationIndex = 1:recordCount
    associationRowCount = 1 + ...
        (coefficientCount - 1) * useIntervalHull(associationIndex);
    matrixRows = nextRow:nextRow + associationRowCount - 1;
    nextRow = matrixRows(end) + 1;
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
coefficientCount = size(powerMap, 2);
segmentCount = size(powerMap, 1);
controlCount = size(powerMap, 3);
powerMatrix = reshape(permute(powerMap, [2 1 3]), ...
    coefficientCount, []);
bernsteinMatrix = hs3Internal.powerToBernstein(powerMatrix);
bernsteinMap = reshape(bernsteinMatrix, ...
    coefficientCount, segmentCount, controlCount);
gradient = cat(1, bernsteinMap, -bernsteinMap);
end

function matrix = seedCorridorConstraintMatrix(sensitivity, corridor)
% Differentiate each six-coefficient exterior support projection.
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
bernsteinMap = reshape(hs3Internal.powerToBernstein(powerMatrix), ...
    6, recordCount, controlCount);
azimuthMap = -reshape(normal(:, 1), 1, recordCount, 1) .* bernsteinMap;
elevationMap = -reshape(normal(:, 2), 1, recordCount, 1) .* bernsteinMap;
matrix(:, 1:controlCount) = reshape(azimuthMap, [], controlCount);
matrix(:, controlCount + 1:end) = reshape(elevationMap, [], controlCount);
end

function matrix = frozenCorridorConstraintMatrix(sensitivity, corridor)
% Differentiate fixed-time obstacle supports over their stored sub-intervals.
if isempty(corridor)
    matrix = zeros(0, 2 * sensitivity.ControlCount);
    return;
end
if any(~[corridor.GeometryIsFixed])
    error("buildAzElConstraintMatrices:NonfixedCorridor", ...
        "Every fixed-time HS3 corridor association must freeze its geometry.");
end
matrix = corridorConstraintMatrix( ...
    sensitivity, corridor, vertcat(corridor.FixedNormal));
end
