function [lineConstraintRows, lineOffsetCoefficients_units] = createPlaneRows( ...
    plane, degree, decisionVariableCount, segmentIndex)
%% Section 0: Header & Readme
% SYNTAX
%   [lineConstraintRows, lineOffsetCoefficients_units] = ...
%       bmtpEngine.separation.createPlaneRows(plane, degree, decisionVariableCount, segmentIndex)
%**************************************************************************
% PURPOSE
%   - Build matrix rows that evaluate the curve's side of a separating line.
%     The line normal and offset can change linearly over its time interval.
%**************************************************************************
% INPUTS
%   - plane (scalar struct)
%       Separating-plane record with Normal, Offset_units, and TimeFraction.
%   - degree (integer scalar)
%       Polynomial degree.
%   - decisionVariableCount (integer scalar)
%       Number of unknown values in the solver vector.
%   - segmentIndex (integer scalar)
%       Curve segment to constrain.
%**************************************************************************
% OUTPUTS
%   - lineConstraintRows (sparse matrix)
%       Rows multiplying solver values to give normal x curve position.
%       There are degree + 2 Bernstein coefficients for this product.
%   - lineOffsetCoefficients_units (numeric column)
%       Add these offsets to the row results to get the full line-side values.
%**************************************************************************
% UNITS
%   - Offsets are coordinate units; TimeFraction is dimensionless.
%**************************************************************************

%% Section 1: Multiply The Curve By The Changing Line Normal

% A degree-D curve x a linear normal gives a degree-(D+1) polynomial.
% Its Bernstein coefficients use these start/end weights. Bounding every
% coefficient also bounds the line-side value throughout the interval.
endNormalWeights   = (0:degree + 1).' / (degree + 1);
startNormalWeights = 1 - endNormalWeights;

segmentCoordinateIndices = (segmentIndex - 1) * 2 * (degree + 1) + (1:2 * (degree + 1)).';
rowIndices               = [repelem((1:degree + 1).', 2); repelem((2:degree + 2).', 2)];
normalWeightsByCoordinate = [reshape((startNormalWeights(1:end - 1) * plane.Normal(1, :)).', [], 1); ...
    reshape((endNormalWeights(2:end) * plane.Normal(2, :)).', [], 1)];
lineConstraintRows = sparse(rowIndices, [segmentCoordinateIndices; segmentCoordinateIndices], ...
    normalWeightsByCoordinate, degree + 2, decisionVariableCount);

%% Section 2: Apply The Line Only To Its Selected Curve Portion

if ~isequal(plane.TimeFraction, [0, 1])
    % Applying curve restriction to an identity matrix gives the weights
    % for extracting this subcurve from the unknown controls. Duplicate the
    % map for interleaved x/y values, then apply it inside the line rows.
    subcurveControlMap = bmtpEngine.motion.restrictBezier(eye(degree + 1), plane.TimeFraction);
    lineConstraintRows(:, segmentCoordinateIndices) = ...
        lineConstraintRows(:, segmentCoordinateIndices) * kron(subcurveControlMap, speye(2));
end
lineOffsetCoefficients_units = startNormalWeights * plane.Offset_units(1) + ...
    endNormalWeights * plane.Offset_units(2);
end
