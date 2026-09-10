function [plane, exitFlag, output] = solveTimedSeparatingLine(controlPoint_units, vertices_units, target_units, reserve_units, options)
%% Section 0: Header & Readme
% SYNTAX
%   [plane, exitFlag, output] = bmtpEngine.solveTimedSeparatingLine( ...
%       controlPoint_units, vertices_units, target_units, reserve_units, options)
%
% PURPOSE
%   - Solve and directly verify one degree-one maximum-margin separating line
%     between a Bezier control hull and a convex region.
%
% INPUTS
%   - controlPoint_units (N-by-2 numeric array)
%       One Bezier span's control points.
%   - vertices_units (M-by-2 numeric array)
%       One convex exclusion-region boundary.
%   - target_units, reserve_units (nonnegative numeric scalars)
%       Obstacle-side target and trajectory-side numerical reserve.
%   - options (coneprog options)
%       Numerical solver controls.
%
% OUTPUTS
%   - plane (scalar struct)
%       Line normals, offsets, verified gap, and active state.
%   - exitFlag (numeric scalar)
%       Original coneprog exit flag.
%   - output (scalar struct, optional output)
%       Original coneprog diagnostics and measured solver time.
%
% UNITS
%   - Positions, offsets, targets, reserves, and gaps are coordinate units.
%

%% Section 1: Solve The Maximum-Margin Line

offsetIndex   = 5:6;
marginIndex   = 7;
variableCount = 7;
[A, b] = maximumMarginRows(controlPoint_units, vertices_units, target_units);
f = zeros(variableCount, 1);
f(marginIndex) = 1;
% Bound the two endpoint normals by the unit disk.
cones = [secondordercone([eye(2),zeros(2,5)],zeros(2,1),zeros(variableCount,1),-1); ...
    secondordercone([zeros(2),eye(2),zeros(2,3)],zeros(2,1),zeros(variableCount,1),-1)];
solverTimer = tic;
[x, ~, exitFlag, output] = coneprog(f, cones, A, b, [], [], [], [], options);
output.TotalTime_s = toc(solverTimer);
plane = emptyPlane();
plane.ExitFlag = exitFlag;
if isempty(x) || any(~isfinite(x))
    return;
end
[plane.Active, plane.Normal, plane.Offset_units] = deal(true, reshape(x(1:4), 2, []).', x(offsetIndex).');
plane = bmtpEngine.verifySeparatingLine(plane, controlPoint_units, vertices_units, reserve_units, target_units);
end

%% Section 2: Local Functions

function [A, b] = maximumMarginRows(controlPoint_units, vertices_units, target_units)
    % Build inequalities for the maximum-margin separating line.
    % Columns hold two endpoint normals, two offsets, and the margin.
    vertexCount = size(vertices_units,1);
    A = zeros(2*vertexCount+size(controlPoint_units,1)+1,7);
    b = zeros(size(A,1),1);
    A(1:vertexCount,[1,2,5]) = [-vertices_units,-ones(vertexCount,1)];
    A(vertexCount+(1:vertexCount),[3,4,6]) = [-vertices_units,-ones(vertexCount,1)];
    b(1:2*vertexCount) = -target_units;
    A(2*vertexCount+1:end,:) = variablePlaneRows(controlPoint_units,7);
    A(2*vertexCount+1:end,7) = -1;
end

function rows = variablePlaneRows(controlPoint_units, variableCount)
    % Multiply a variable separating line by fixed trajectory controls.
    degree = size(controlPoint_units, 1) - 1;
    % Exact degree-N by degree-one Bernstein product weights.
    beta  = (0:degree + 1).' / (degree + 1);
    alpha = 1 - beta;
    rows = zeros(degree + 2, variableCount);
    rows(1:end - 1, 1:2) = alpha(1:end - 1) .* controlPoint_units;
    rows(2:end, 3:4) = beta(2:end) .* controlPoint_units;
    rows(:, 5:6) = [alpha beta];
end

function plane = emptyPlane()
    % Initialize an inactive separating-plane record.
    plane = struct();
    plane.Active        = false;
    plane.Verified      = false;
    plane.ExitFlag      = NaN;
    plane.Normal        = zeros(2, 2);
    plane.Offset_units    = zeros(1, 2);
    plane.SignedGap_units = NaN;
end
