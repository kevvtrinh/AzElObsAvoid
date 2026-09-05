function [plane, exitFlag, output] = solveSeparatingLine( ...
        controlPoint_deg, vertices_deg, target_deg, reserve_deg, options)
%% Section 0: Header & Readme
% SYNTAX
%   [plane, exitFlag, output] = bmtpEngine.solveSeparatingLine( ...
%       controlPoint_deg, vertices_deg, target_deg, reserve_deg, options)
%**************************************************************************
% PURPOSE
%   - Solve and directly verify one degree-one maximum-margin separating line
%     between a Bezier control hull and a convex region.
%**************************************************************************
% INPUTS
%   - controlPoint_deg (N-by-2 numeric array)
%       One Bezier span's control points.
%   - vertices_deg (M-by-2 numeric array)
%       One convex exclusion-region boundary.
%   - target_deg, reserve_deg (nonnegative numeric scalars)
%       Obstacle-side target and trajectory-side numerical reserve.
%   - options (coneprog options)
%       Numerical solver controls.
%**************************************************************************
% OUTPUTS
%   - plane (scalar struct)
%       Line normals, offsets, verified gap, and active state.
%   - exitFlag (numeric scalar)
%       Fastcone acceptance or original coneprog recovery exit flag.
%   - output (scalar struct, optional output)
%       Executed method, certificates, elapsed time, and recovery diagnostics.
%**************************************************************************
% UNITS
%   - Positions, offsets, targets, reserves, and gaps are degrees.
%**************************************************************************

%% Section 1: Solve The Maximum-Margin Line

offsetIndex = 5:6;
marginIndex = 7;
variableCount = 7;
[A, b] = maximumMarginRows(controlPoint_deg, vertices_deg, target_deg);
f = zeros(variableCount, 1);
f(marginIndex) = 1;
emptyCone = secondordercone(zeros(2, variableCount), zeros(2, 1), ...
    zeros(variableCount, 1), -1);
cones = repmat(emptyCone, 2, 1);
for planeIndex = 0:1
    coneA = zeros(2, variableCount);
    coneA(:, planeIndex * 2 + (1:2)) = eye(2);
    cones(planeIndex + 1) = secondordercone( ...
        coneA, zeros(2, 1), zeros(variableCount, 1), -1);
end
[x, ~, exitFlag, output] = fastcone.solve( ...
    f, cones, A, b, [], [], [], [], options);
plane = emptyPlane();
plane.ExitFlag = exitFlag;
if isempty(x) || any(~isfinite(x))
    return;
end
[plane.Active, plane.Normal, plane.Offset_deg] = ...
    deal(true, reshape(x(1:4), 2, []).', x(offsetIndex).');
plane = bmtpEngine.verifySeparatingLine( ...
    plane, controlPoint_deg, vertices_deg, reserve_deg, target_deg);
end

%% Section 2: Local Functions

function [A, b] = maximumMarginRows( ...
        controlPoint_deg, vertices_deg, target_deg)
% Create linear inequalities for one maximum-margin line solve.
degree = size(controlPoint_deg, 1) - 1;
variableCount = 7;
offsetIndex = 5:6;
marginIndex = 7;
A = zeros(2 * size(vertices_deg, 1) + degree + 2, variableCount);
b = zeros(size(A, 1), 1);
rowIndex = 0;
for planeIndex = 0:1
    targets = rowIndex + (1:size(vertices_deg, 1));
    normal = planeIndex * 2 + (1:2);
    A(targets, normal) = -vertices_deg;
    A(targets, offsetIndex(planeIndex + 1)) = -1;
    b(targets) = -target_deg;
    rowIndex = targets(end);
end
objectiveRows = variablePlaneRows(controlPoint_deg, variableCount);
targets = rowIndex + (1:size(objectiveRows, 1));
A(targets, :) = objectiveRows;
A(targets, marginIndex) = -1;
end

function rows = variablePlaneRows(controlPoint_deg, variableCount)
% Expand a decision-valued line times one fixed trajectory control net.
degree = size(controlPoint_deg, 1) - 1;
[alpha, beta] = productWeights(degree);
rows = zeros(degree + 2, variableCount);
rows(1:end - 1, 1:2) = alpha(1:end - 1) .* controlPoint_deg;
rows(2:end, 3:4) = beta(2:end) .* controlPoint_deg;
rows(:, 5:6) = [alpha beta];
end

function [alpha, beta] = productWeights(degree)
% Return exact degree-N by degree-one Bernstein product weights.
beta = (0:degree + 1).' / (degree + 1);
alpha = 1 - beta;
end

function plane = emptyPlane()
% Define the stable inactive degree-one separating-line record.
plane = struct("Active", false, "Verified", false, "ExitFlag", NaN, ...
    "Normal", zeros(2, 2), "Offset_deg", zeros(1, 2), ...
    "SignedGap_deg", NaN);
end
