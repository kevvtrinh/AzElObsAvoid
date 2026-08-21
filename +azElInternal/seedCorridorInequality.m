function inequality = seedCorridorInequality(polynomial, corridor)
%% Section 0: Header & Readme
% SYNTAX
%   inequality = azElInternal.seedCorridorInequality( ...
%       polynomial, corridor)
%**************************************************************************
% PURPOSE
%   - Enforce each complete trajectory segment outside its seed corridor.
%**************************************************************************
% INPUTS
%   - polynomial (scalar planner polynomial struct)
%       positionPower_deg contains segment-local ascending coefficients.
%   - corridor (structure array)
%       Records from azElInternal.buildSeedCorridor.
%**************************************************************************
% OUTPUTS
%   - inequality (numeric column vector)
%       Values are feasible when every value is less than or equal to zero.
%**************************************************************************
% UNITS
%   - Inequality values are degrees.
%**************************************************************************
%% Section 1: Convert Continuous Projection Bounds
if isempty(corridor)
    inequality = zeros(0, 1);
    return;
end
coefficientCount = size(polynomial.positionPower_deg, 3);
corridorCount = numel(corridor);
segmentIndex = [corridor.SegmentIndex].';
normal = vertcat(corridor.Normal);
selectedPower_deg = polynomial.positionPower_deg(segmentIndex, :, :);
azimuthPower_deg = reshape( ...
    selectedPower_deg(:, 1, :), corridorCount, coefficientCount);
elevationPower_deg = reshape( ...
    selectedPower_deg(:, 2, :), corridorCount, coefficientCount);
projectionPower_deg = normal(:, 1) .* azimuthPower_deg + ...
    normal(:, 2) .* elevationPower_deg;
projectionBernstein_deg = azElInternal.powerToBernstein( ...
    projectionPower_deg.');
offset_deg = [corridor.BoundaryOffset_deg] + [corridor.Clearance_deg];
inequalityMatrix = offset_deg - projectionBernstein_deg;
inequality = inequalityMatrix(:);
end
