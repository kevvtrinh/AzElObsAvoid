function inequality = seedCorridorInequality(polynomial, corridor)
%% Section 0: Header & Readme
% SYNTAX
%   inequality = azElPlannerMethods.corridor.internal.validation.seedCorridorInequality( ...
%       polynomial, corridor)
%**************************************************************************
% PURPOSE
%   - Convert continuous outside-corridor requirements into finite inequalities.
%     Bernstein coefficients bound a polynomial over the complete normalized
%     span, so satisfying every coefficient is stronger than checking samples.
%**************************************************************************
% INPUTS
%   - polynomial (scalar planner polynomial struct)
%       positionPower_deg contains segment-local ascending coefficients.
%   - corridor (structure array)
%       Records from azElPlannerMethods.corridor.internal.validation.buildSeedCorridor.
%**************************************************************************
% OUTPUTS
%   - inequality (numeric column vector)
%       Values are feasible when every value is less than or equal to zero.
%**************************************************************************
% UNITS
%   - Inequality values are degrees.
%**************************************************************************

%% Section 1: Convert Continuous Projection Bounds

% Project the two-axis trajectory onto each corridor normal. For a safe exterior
% span, every projected Bernstein control value must remain beyond the obstacle
% support offset plus its requested clearance.
if isempty(corridor)
    inequality = zeros(0, 1);
    return;
end
coefficientCount = size(polynomial.positionPower_deg, 3);
corridorCount = numel(corridor);
segmentIndex = [corridor.SegmentIndex].';
normal = vertcat(corridor.Normal);
selectedPower_deg = polynomial.positionPower_deg(segmentIndex, :, :);
azimuthPower_deg = reshape( selectedPower_deg(:, 1, :), corridorCount, coefficientCount);
elevationPower_deg = reshape( selectedPower_deg(:, 2, :), corridorCount, coefficientCount);
projectionPower_deg = normal(:, 1) .* azimuthPower_deg + normal(:, 2) .* elevationPower_deg;
projectionBernstein_deg = azElPlannerMethods.corridor.internal.powerToBernstein( projectionPower_deg.');
% Express feasibility as inequality <= 0 to match MATLAB optimizer conventions.
offset_deg = [corridor.BoundaryOffset_deg] + [corridor.Clearance_deg];
inequalityMatrix = offset_deg - projectionBernstein_deg;
inequality = inequalityMatrix(:);
end
