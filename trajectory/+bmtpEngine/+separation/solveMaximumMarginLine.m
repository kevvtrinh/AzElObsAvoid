function [plane, exitFlag, output] = solveMaximumMarginLine(controlPoint_units, ...
        vertices_units, target_units, roundoffReserve_units, options)
%% Section 0: Header & Readme
% SYNTAX
%   [plane, exitFlag, output] = bmtpEngine.separation.solveMaximumMarginLine( ...
%       controlPoint_units, vertices_units, target_units, roundoffReserve_units, options)
%**************************************************************************
% PURPOSE
%   - Solve and exactly verify a degree-one maximum-margin separating line
%     between one Bezier span and one convex exclusion region.
%**************************************************************************
% INPUTS
%   - controlPoint_units (N-by-2 numeric array)
%       Bezier control points for one motion span.
%   - vertices_units (M-by-2 numeric array)
%       Convex exclusion-region vertices.
%   - target_units (nonnegative numeric scalar)
%       Required obstacle-side separation target.
%   - roundoffReserve_units (nonnegative numeric scalar)
%       Numerical reserve applied on the trajectory side.
%   - options (optim.options.Coneprog scalar)
%       Options for the conic solver.
%**************************************************************************
% OUTPUTS
%   - plane (scalar struct)
%       Candidate separating line and its exact verification state.
%   - exitFlag (numeric scalar)
%       Conic solver termination flag.
%   - output (scalar struct)
%       Conic solver diagnostics including measured total time.
%**************************************************************************
% UNITS
%   - Position, offsets, margin, target, and reserve are coordinate units;
%     normals are dimensionless.
%**************************************************************************

%% Section 1: Assemble And Solve The Plane SOCP

degree        = size(controlPoint_units, 1) - 1;
variableCount = 7;
offsetIndex   = 5:6;
marginIndex   = 7;
A             = zeros(2 * size(vertices_units, 1) + degree + 2, variableCount);
b             = zeros(size(A, 1), 1);
rowCount      = 0;
for endpointIndex = 0:1
    targetRows = rowCount + (1:size(vertices_units, 1));
    A(targetRows, endpointIndex * 2 + (1:2)) = -vertices_units;
    A(targetRows, offsetIndex(endpointIndex + 1)) = -1;
    b(targetRows) = -target_units;
    rowCount      = targetRows(end);
end

beta  = (0:degree + 1).' / (degree + 1);
alpha = 1 - beta;
productRows = zeros(degree + 2, variableCount);
productRows(1:end - 1, 1:2) = alpha(1:end - 1) .* controlPoint_units;
productRows(2:end, 3:4)     = beta(2:end) .* controlPoint_units;
productRows(:, offsetIndex) = [alpha, beta];
targetRows                  = rowCount + (1:degree + 2);
A(targetRows, :)            = productRows;
A(targetRows, marginIndex)  = -1;
f                           = zeros(variableCount, 1);
f(marginIndex)              = 1;

emptyCone = secondordercone(zeros(2, variableCount), zeros(2, 1), ...
    zeros(variableCount, 1), -1);
cones = repmat(emptyCone, 2, 1);
for endpointIndex = 0:1
    coneA = zeros(2, variableCount);
    coneA(:, endpointIndex * 2 + (1:2)) = eye(2);
    cones(endpointIndex + 1) = secondordercone( ...
        coneA, zeros(2, 1), zeros(variableCount, 1), -1);
end

timer = tic;
[solution, ~, exitFlag, output] = coneprog(f, cones, A, b, [], [], [], [], options);
output.TotalTime_s = toc(timer);
plane              = bmtpEngine.separation.createEmptyPlane();
plane.ExitFlag     = exitFlag;
if isempty(solution) || any(~isfinite(solution))
    return
end

plane.Active       = true;
plane.Normal       = reshape(solution(1:4), 2, []).';
plane.Offset_units = solution(offsetIndex).';
plane = bmtpEngine.separation.verifySeparatingLine( ...
    plane, controlPoint_units, vertices_units, roundoffReserve_units, target_units);
end
