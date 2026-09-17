function cones = createVariationCone(jerkMap, times_s, limits, objectiveIndex)
%% Section 0: Header & Readme
% SYNTAX
%   cones = bmtpEngine.optimization.createVariationCone(jerkMap, times_s, limits, objectiveIndex)
%**************************************************************************
% PURPOSE
%   - Penalize jerk variation within the motion-generation solve.
%**************************************************************************
% INPUTS
%   - jerkMap (sparse matrix)
%       Physical quadratic-jerk map over the decision vector.
%   - times_s (numeric column)
%       Physical phase durations, one per span.
%   - limits (scalar struct)
%       Per-axis jerk limits used to normalize the measure.
%   - objectiveIndex (numeric scalar)
%       Epigraph variable for the complete integrated variation.
%**************************************************************************
% OUTPUTS
%   - cones (secondordercone array)
%       Cone bounding normalized integrated squared snap.
%**************************************************************************
% UNITS
%   - Time is seconds; the objective measure is dimensionless.
%**************************************************************************

%% Section 1: Integrate The Linear Snap Exactly
spanCount         = numel(times_s);
snapQuadratureMap = sparse(4 * spanCount, 6 * spanCount);
quadratureTau     = (1 + [-1, 1] / sqrt(3)) / 2;
for spanIndex = 1:spanCount
    localSnapWeights = sqrt(2 / times_s(spanIndex)) * ...
        [-(1 - quadratureTau(:)), 1 - 2 * quadratureTau(:), quadratureTau(:)];
    snapQuadratureMap((spanIndex - 1) * 4 + (1:4), ...
            (spanIndex - 1) * 6 + (1:6)) = ...
        kron(localSnapWeights, diag(1 ./ limits.maxJerk_units_s3));
end

%% Section 2: Bound The Complete Normalized Snap With One Cone
% For three jerk controls in [-1,1], the maximum integrated squared
% normalized snap is 16/(3*h) per axis. One quadratic epigraph for the
% concatenated samples is exactly equivalent to summing one epigraph per
% span, while avoiding redundant auxiliary variables and cone blocks.
normalizer    = sqrt((32 / 3) * sum(1 ./ times_s));
snapMap       = snapQuadratureMap * jerkMap / normalizer;
variableCount = size(jerkMap, 2);
coneLinearMap = [2 * snapMap; sparse(1, variableCount)] + ...
    sparse(4 * spanCount + 1, objectiveIndex, 1, ...
    4 * spanCount + 1, variableCount);
coneBoundVector = sparse(objectiveIndex, 1, 1, variableCount, 1);
cones = secondordercone( ...
    coneLinearMap, [zeros(4 * spanCount, 1); 1], coneBoundVector, -1);
end
