function [inequalityMatrix, equalityMatrix] = ...
        buildFixedHs3ConstraintMatrices( ...
        segmentCount, duration_s, allowAzimuthWrapping, ...
        seedCorridor, corridor, corridorTau, corridorNormal)
%% Section 0: Header & Readme
% SYNTAX
%   [inequalityMatrix, equalityMatrix] = ...
%       azElPlannerMethods.hs3.internal.motion.buildFixedHs3ConstraintMatrices( ...
%       segmentCount, duration_s, allowAzimuthWrapping, ...
%       seedCorridor, corridor, corridorTau)
%   [inequalityMatrix, equalityMatrix] = ...
%       azElPlannerMethods.hs3.internal.motion.buildFixedHs3ConstraintMatrices( ...
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

sensitivity = azElPlannerMethods.hs3.internal.motion.hs3AffineSensitivity( ...
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
% Differentiate current support rows while geometry remains fixed in jerk.
if isempty(corridor)
    matrix = zeros(0, 2 * sensitivity.ControlCount);
    return
end
if size(normal, 1) ~= numel(corridor) || size(normal, 2) ~= 2
    error("buildFixedHs3ConstraintMatrices:InvalidCorridorNormals", ...
        "corridorNormal must contain one two-axis row per association.");
end
controlIndex = [corridor.ControlIndex].';
positionMap = sensitivity.positionAtTauMap(controlIndex, :);
matrix = [-normal(:, 1) .* positionMap, ...
    -normal(:, 2) .* positionMap];
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
bernsteinMatrix = azElInternal.powerToBernstein(powerMatrix);
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
bernsteinMap = reshape(azElInternal.powerToBernstein(powerMatrix), ...
    6, recordCount, controlCount);
azimuthMap = -reshape(normal(:, 1), 1, recordCount, 1) .* bernsteinMap;
elevationMap = -reshape(normal(:, 2), 1, recordCount, 1) .* bernsteinMap;
matrix(:, 1:controlCount) = reshape(azimuthMap, [], controlCount);
matrix(:, controlCount + 1:end) = reshape(elevationMap, [], controlCount);
end

function matrix = frozenCorridorConstraintMatrix(sensitivity, corridor)
% Differentiate fixed-time obstacle supports at their stored control points.
controlCount = sensitivity.ControlCount;
matrix = zeros(numel(corridor), 2 * controlCount);
if isempty(corridor)
    return;
end
if any(~[corridor.GeometryIsFixed])
    error("buildFixedHs3ConstraintMatrices:NonfixedCorridor", ...
        "Every fixed-time HS3 corridor association must freeze its geometry.");
end
controlIndex = [corridor.ControlIndex].';
normal = vertcat(corridor.FixedNormal);
positionMap = sensitivity.positionAtTauMap(controlIndex, :);
matrix(:, 1:controlCount) = -normal(:, 1) .* positionMap;
matrix(:, controlCount + 1:end) = -normal(:, 2) .* positionMap;
end
