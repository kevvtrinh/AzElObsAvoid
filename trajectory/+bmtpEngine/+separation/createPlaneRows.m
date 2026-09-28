function [lineConstraintRows, lineOffsetCoefficients_units] = createPlaneRows( ...
    plane, degree, decisionVariableCount, segmentIndex)
%% Section 0: Header & Readme
% SYNTAX
%   [lineConstraintRows, lineOffsetCoefficients_units] = ...
%       bmtpEngine.separation.createPlaneRows(plane, degree, decisionVariableCount, segmentIndex)
%**************************************************************************
% PURPOSE
%   - Turn one separating line into solver rows for a motion segment. Each
%     row produces one Bezier coefficient of dot(normal, curve position).
%     The line normal and offset may change from interval start to end.
%**************************************************************************
% INPUTS
%   - plane (scalar struct)
%       Normal and Offset_units give the line at each interval end.
%       TimeFraction selects the portion of the motion segment it covers.
%   - degree (integer scalar)
%       Degree D of the segment's position curve.
%   - decisionVariableCount (integer scalar)
%       Total number of solver unknowns. Each curve control uses adjacent
%       x and y entries in that vector.
%   - segmentIndex (integer scalar)
%       One-based index of the curve segment whose controls fill the rows.
%**************************************************************************
% OUTPUTS
%   - lineConstraintRows ((D+2)-by-decisionVariableCount sparse matrix)
%       Multiply by the solver vector to get Bezier coefficients of
%       dot(normal, position), without the offset. A degree-D curve times
%       a linear normal has degree D+1, so it has D+2 coefficients here.
%   - lineOffsetCoefficients_units ((D+2)-by-1 numeric column)
%       Add to those row values to get dot(normal, position) + offset.
%**************************************************************************
% UNITS
%   - Row results and offsets are coordinate units. Normals and
%     TimeFraction are dimensionless.
%**************************************************************************

%% Section 1: Multiply The Curve By The Changing Line Normal

% A degree-D curve times a linearly changing normal has degree D+1.
% Before any subcurve restriction, each curve control contributes to two
% rows: one with the starting normal and the next with the ending normal.
% For D = 1, two curve controls produce three line-side coefficients.
% If all those coefficients obey a bound, the line-side value between
% them obeys it too because Bezier values stay within their controls.
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
    % Restrict an identity curve to get a map from full-segment controls
    % to the selected subcurve. Apply the same map to interleaved x/y
    % controls before checking the line on that subcurve.
    subcurveControlMap = bmtpEngine.motion.restrictBezier(eye(degree + 1), plane.TimeFraction);
    lineConstraintRows(:, segmentCoordinateIndices) = ...
        lineConstraintRows(:, segmentCoordinateIndices) * kron(subcurveControlMap, speye(2));
end
lineOffsetCoefficients_units = startNormalWeights * plane.Offset_units(1) + ...
    endNormalWeights * plane.Offset_units(2);
end
