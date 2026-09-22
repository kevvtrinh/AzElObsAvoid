function variationCone = createVariationCone( ...
    jerkControlMap, segmentTime_s, limits, objectiveVariableIndex)
%% Section 0: Header & Readme
% SYNTAX
%   variationCone = bmtpEngine.optimization.createVariationCone( ...
%       jerkControlMap, segmentTime_s, limits, objectiveVariableIndex)
%**************************************************************************
% PURPOSE
%   - Give the solver a measure of how quickly jerk changes, so it can
%     prefer smoother motion. Snap is the rate of change of jerk.
%**************************************************************************
% INPUTS
%   - jerkControlMap (sparse matrix)
%       Maps solver values to three quadratic jerk controls per axis and segment.
%   - segmentTime_s (numeric column)
%       Duration of each motion segment.
%   - limits (scalar struct)
%       Per-axis jerk limits used to normalize the measure.
%   - objectiveVariableIndex (numeric scalar)
%       Solver variable that must be at least the calculated jerk-variation cost.
%**************************************************************************
% OUTPUTS
%   - variationCone (secondordercone object)
%       Vector-length constraint requiring cost <= objective variable.
%**************************************************************************
% UNITS
%   - Time is seconds; the objective measure is dimensionless.
%**************************************************************************

%% Section 1: Measure How Quickly Jerk Changes

% Quadratic jerk has linear snap. These two integration points and weights
% give the exact integral of snap^2, without sampling the whole segment.
% Divide each axis by its jerk limit so both axes use a comparable scale.
segmentCount         = numel(segmentTime_s);
weightedSnapMap      = sparse(4 * segmentCount, 6 * segmentCount);
integrationFractions = (1 + [-1, 1] / sqrt(3)) / 2;
for segmentIndex = 1:segmentCount
    segmentSnapWeights = sqrt(2 / segmentTime_s(segmentIndex)) * ...
        [-(1 - integrationFractions(:)), 1 - 2 * integrationFractions(:), integrationFractions(:)];
    weightedSnapMap((segmentIndex - 1) * 4 + (1:4), ...
        (segmentIndex - 1) * 6 + (1:6)) = ...
        kron(segmentSnapWeights, diag(1 ./ limits.maxJerk_units_s3));
end

%% Section 2: Convert The Total Cost Into A Solver Constraint

% For normalized jerk controls between -1 and 1, the largest integral of
% snap^2 is 16 / (3 x duration) per axis. Divide by the two-axis total
% across all segments so the cost is between 0 and 1 for these controls.
maximumVariationScale = sqrt((32 / 3) * sum(1 ./ segmentTime_s));
normalizedSnapMap     = weightedSnapMap * jerkControlMap / maximumVariationScale;
decisionVariableCount = size(jerkControlMap, 2);
% Let q = normalizedSnapMap x solverValues and z = the objective variable.
% The constraint norm([2 x q; z - 1]) <= z + 1 gives sum(q.^2) <= z.
% Minimizing z therefore minimizes the measured jerk variation.
leftSideMap = [2 * normalizedSnapMap; sparse(1, decisionVariableCount)] + ...
    sparse(4 * segmentCount + 1, objectiveVariableIndex, 1, ...
    4 * segmentCount + 1, decisionVariableCount);
rightSideWeights = sparse(objectiveVariableIndex, 1, 1, decisionVariableCount, 1);
variationCone = secondordercone( ...
    leftSideMap, [zeros(4 * segmentCount, 1); 1], rightSideWeights, -1);
end
