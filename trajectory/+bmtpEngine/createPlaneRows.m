function [rows, offset_units] = createPlaneRows(plane, degree, variableCount, segmentIndex)
%% Section 0: Header & Readme
% SYNTAX
%   [rows, offset_units] = ...
%       bmtpEngine.createPlaneRows(plane, degree, variableCount, segmentIndex)
%**************************************************************************
% PURPOSE
%   - Multiply fixed affine separating planes by variable Bezier controls.
%**************************************************************************
% INPUTS
%   - plane (scalar struct)
%       Separating-plane record with Normal, Offset_units, and TimeFraction.
%   - degree (integer scalar)
%       Polynomial degree.
%   - variableCount (integer scalar)
%       Decision-vector size.
%   - segmentIndex (integer scalar)
%       Target span index.
%**************************************************************************
% OUTPUTS
%   - rows (sparse matrix)
%       Degree-plus-two Bernstein product rows for the selected span.
%   - offset_units (numeric column)
%       Matching Bernstein offsets in the same constraint order.
%**************************************************************************
% UNITS
%   - Offsets are coordinate units; TimeFraction is dimensionless.
%**************************************************************************

%% Section 1: Form The Degree-Elevated Bernstein Product Rows
% Exact degree-N by degree-one Bernstein product weights.
beta  = (0:degree + 1).' / (degree + 1);
alpha = 1 - beta;

controlColumns = (segmentIndex - 1) * 2 * (degree + 1) + (1:2 * (degree + 1)).';
rowIndices     = [repelem((1:degree + 1).', 2); repelem((2:degree + 2).', 2)];
values         = [reshape((alpha(1:end - 1) * plane.Normal(1, :)).', [], 1); ...
    reshape((beta(2:end) * plane.Normal(2, :)).', [], 1)];
rows = sparse(rowIndices, [controlColumns; controlColumns], values, degree + 2, variableCount);

%% Section 2: Restrict To The Plane's Own Time Fraction
if ~isequal(plane.TimeFraction, [0, 1])
    % Restrict the unknown controls exactly before forming the existing
    % Bernstein plane product; keep every source interval as a constraint.
    restriction = bmtpEngine.restrictBezier(eye(degree + 1), plane.TimeFraction);
    rows(:, controlColumns) = rows(:, controlColumns) * kron(restriction, speye(2));
end
offset_units = alpha * plane.Offset_units(1) + beta * plane.Offset_units(2);
end
