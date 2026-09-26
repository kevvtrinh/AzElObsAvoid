function [plane, exitFlag, solverOutput] = solveMaximumMarginLine(controlPoint_units, ...
    vertices_units, separationTarget_units, roundoffReserve_units, solverOptions)
%% Section 0: Header & Readme
% SYNTAX
%   [plane, exitFlag, solverOutput] = bmtpEngine.separation.solveMaximumMarginLine( ...
%       controlPoint_units, vertices_units, separationTarget_units, ...
%       roundoffReserve_units, solverOptions)
%**************************************************************************
% PURPOSE
%   - Ask the conic solver for a line between one Bezier segment and one
%     static convex obstacle. Its normal and offset may change linearly
%     during the segment. Verify the returned line over the whole segment.
%**************************************************************************
% INPUTS
%   - controlPoint_units (N-by-2 numeric array)
%       Bezier controls for one motion segment, in [x, y] rows.
%   - vertices_units (M-by-2 numeric array)
%       Vertices of one prepared static convex obstacle polygon.
%   - separationTarget_units (nonnegative numeric scalar)
%       Minimum required obstacle-side value at the line.
%   - roundoffReserve_units (nonnegative numeric scalar)
%       Extra curve-side margin checked after the solve for rounding.
%   - solverOptions (optim.options.Coneprog scalar)
%       Options for the conic solver.
%**************************************************************************
% OUTPUTS
%   - plane (scalar struct)
%       Active is true for a finite solver proposal. Verified is true only
%       if the separate line check proves the required separation.
%   - exitFlag (numeric scalar)
%       Conic solver status; it does not by itself prove clearance.
%   - solverOutput (scalar struct)
%       Conic solver diagnostics with TotalTime_s measured around the call.
%**************************************************************************
% UNITS
%   - Position, offsets, margin, target, and reserve are coordinate units;
%     normals are dimensionless.
%**************************************************************************

%% Section 1: Constrain The Obstacle And Curve To Opposite Sides

% Seven solver unknowns: [nx, ny] at both ends, an offset at both ends,
% and one upper bound on the curve-side value. Keep every obstacle vertex
% at or above the target and minimize the curve-side upper bound. A lower
% curve-side bound leaves more room between the obstacle and curve.

degree                = size(controlPoint_units, 1) - 1;
decisionVariableCount = 7;
offsetIndices         = 5:6;
maximumCurveSideIndex  = 7;
inequalityMatrix      = zeros(2 * size(vertices_units, 1) + degree + 2, decisionVariableCount);
inequalityBounds      = zeros(size(inequalityMatrix, 1), 1);
filledRowCount        = 0;

% For a fixed obstacle vertex, a line that changes linearly gives a
% line-side value that changes linearly too. If both endpoint values meet
% the obstacle target, every time between them meets it.
for endpointIndex = 0:1
    selectedRowIndices = filledRowCount + (1:size(vertices_units, 1));
    inequalityMatrix(selectedRowIndices, endpointIndex * 2 + (1:2)) = -vertices_units;
    inequalityMatrix(selectedRowIndices, offsetIndices(endpointIndex + 1)) = -1;
    inequalityBounds(selectedRowIndices) = -separationTarget_units;
    filledRowCount = selectedRowIndices(end);
end

% A degree-D position curve times a linear normal has degree D+1, with
% D+2 Bezier coefficients. Bound each coefficient, including the offset;
% then every point between the curve endpoints obeys the same bound.
endNormalWeights   = (0:degree + 1).' / (degree + 1);
startNormalWeights = 1 - endNormalWeights;

curveSideRows = zeros(degree + 2, decisionVariableCount);
curveSideRows(1:end - 1, 1:2)    = startNormalWeights(1:end - 1) .* controlPoint_units;
curveSideRows(2:end, 3:4)        = endNormalWeights(2:end) .* controlPoint_units;
curveSideRows(:, offsetIndices) = [startNormalWeights, endNormalWeights];

selectedRowIndices = filledRowCount + (1:degree + 2);
inequalityMatrix(selectedRowIndices, :)                    = curveSideRows;
inequalityMatrix(selectedRowIndices, maximumCurveSideIndex) = -1;

objectiveWeights = zeros(decisionVariableCount, 1);
objectiveWeights(maximumCurveSideIndex) = 1;

% Limit each endpoint normal to length 1. Otherwise multiplying every
% normal and offset by a large number could inflate the apparent gap.
% Linear interpolation keeps intermediate normal lengths at most 1 too.
emptyCone = secondordercone(zeros(2, decisionVariableCount), zeros(2, 1), ...
    zeros(decisionVariableCount, 1), -1);
normalLengthCones = repmat(emptyCone, 2, 1);
for endpointIndex = 0:1
    normalSelector = zeros(2, decisionVariableCount);
    normalSelector(:, endpointIndex * 2 + (1:2)) = eye(2);
    normalLengthCones(endpointIndex + 1) = secondordercone( ...
        normalSelector, zeros(2, 1), zeros(decisionVariableCount, 1), -1);
end

%% Section 2: Solve And Verify The Returned Line

% A finite proposal is only a candidate. The check below separately
% bounds the complete curve and obstacle; solver status alone is not proof.
solveTimer = tic;
[solverValues, ~, exitFlag, solverOutput] = coneprog( ...
    objectiveWeights, normalLengthCones, inequalityMatrix, inequalityBounds, ...
    [], [], [], [], solverOptions);
solverOutput.TotalTime_s = toc(solveTimer);
plane                   = bmtpEngine.separation.createEmptyPlane();
plane.ExitFlag           = exitFlag;
if isempty(solverValues) || any(~isfinite(solverValues))
    return
end

plane.Active       = true;
plane.Normal       = reshape(solverValues(1:4), 2, []).';
plane.Offset_units = solverValues(offsetIndices).';
plane = bmtpEngine.separation.verifySeparatingLine( ...
    plane, controlPoint_units, vertices_units, roundoffReserve_units, separationTarget_units);
end
