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
%   - objectiveIndex (numeric column)
%       One epigraph variable index per phase.
%**************************************************************************
% OUTPUTS
%   - cones (secondordercone array)
%       Local cones bounding normalized integrated squared snap.
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

%% Section 2: Bound Each Span's Normalized Snap With A Local Cone
% For three jerk controls in [-1,1], the maximum integrated squared
% normalized snap is 16/(3*h) per axis. This gives a bounded tie-break.
normalizer    = sqrt((32 / 3) * sum(1 ./ times_s));
snapMap       = snapQuadratureMap * jerkMap / normalizer;
variableCount = size(jerkMap, 2);
emptyCone     = secondordercone(sparse(5, variableCount), zeros(5, 1), ...
    sparse(variableCount, 1), 0);
cones         = repmat(emptyCone, spanCount, 1);
for spanIndex = 1:spanCount
    coneLinearMap = [2 * snapMap((spanIndex - 1) * 4 + (1:4), :); ...
        sparse(1, variableCount)];
    coneLinearMap(end, objectiveIndex(spanIndex)) = 1;
    coneBoundVector = sparse(variableCount, 1);
    coneBoundVector(objectiveIndex(spanIndex)) = 1;
    cones(spanIndex) = secondordercone( ...
        coneLinearMap, [zeros(4, 1); 1], coneBoundVector, -1);
end
end
