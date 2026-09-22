function [plane, exitFlag, solverOutput] = solveMaximumMarginLine(controlPoint_units, ...
    vertices_units, separationTarget_units, roundoffReserve_units, solverOptions)
%% Section 0: Header & Readme
% SYNTAX
%   [plane, exitFlag, solverOutput] = bmtpEngine.separation.solveMaximumMarginLine( ...
%       controlPoint_units, vertices_units, separationTarget_units, ...
%       roundoffReserve_units, solverOptions)
%**************************************************************************
% PURPOSE
%   - Find a line that changes linearly over one Bezier segment and separates
%     the curve from a static convex obstacle. Check the whole segment.
%**************************************************************************
% INPUTS
%   - controlPoint_units (N-by-2 numeric array)
%       Bezier control points for one trajectory segment.
%   - vertices_units (M-by-2 numeric array)
%       Vertices of one static convex obstacle region.
%   - separationTarget_units (nonnegative numeric scalar)
%       Required obstacle-side separation target.
%   - roundoffReserve_units (nonnegative numeric scalar)
%       Numerical reserve applied on the trajectory side.
%   - solverOptions (optim.options.Coneprog scalar)
%       Options for the conic solver.
%**************************************************************************
% OUTPUTS
%   - plane (scalar struct)
%       Candidate line. Verified is true only if all separation checks pass.
%   - exitFlag (numeric scalar)
%       Conic solver termination flag.
%   - solverOutput (scalar struct)
%       Conic solver diagnostics including measured total time.
%**************************************************************************
% UNITS
%   - Position, offsets, margin, target, and reserve are coordinate units;
%     normals are dimensionless.
%**************************************************************************

%% Section 1: Constrain The Obstacle And Curve To Opposite Sides

% Seven unknowns: two endpoint normals (four values), two endpoint offsets,
% and an upper bound on the curve-side value. Minimize that upper bound
% while each obstacle vertex stays at or above the separation target.

degree                = size(controlPoint_units, 1) - 1;
decisionVariableCount = 7;
offsetIndices         = 5:6;
maximumCurveSideIndex  = 7;
inequalityMatrix      = zeros(2 * size(vertices_units, 1) + degree + 2, decisionVariableCount);
inequalityBounds      = zeros(size(inequalityMatrix, 1), 1);
filledRowCount        = 0;

% The obstacle is static and the line changes linearly. Checking each vertex
% with the start/end lines also bounds its line value between those times.
for endpointIndex = 0:1
    selectedRowIndices = filledRowCount + (1:size(vertices_units, 1));
    inequalityMatrix(selectedRowIndices, endpointIndex * 2 + (1:2)) = -vertices_units;
    inequalityMatrix(selectedRowIndices, offsetIndices(endpointIndex + 1)) = -1;
    inequalityBounds(selectedRowIndices) = -separationTarget_units;
    filledRowCount = selectedRowIndices(end);
end

% Multiplying the degree-D curve by a degree-1 line normal gives degree D + 1.
% Bounding all D + 2 product coefficients bounds the whole curve-side value.
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

% Limit both normal lengths to 1 so rescaling a line cannot create an
% artificially larger gap. Interpolated normal lengths are then <= 1 too.
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

% A finite solver proposal still needs the continuous separation check.
% Its exit flag alone does not establish a safe gap.
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
