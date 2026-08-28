function [segmentIndex, hullMap] = createSubintervalBernsteinMap( ...
        tauStart, tauEnd, segmentCount, coefficientCount, segmentBreakTau)
%% Section 0: Header & Readme
% SYNTAX
%   [segmentIndex, hullMap] = hs3Engine.polynomial.createSubintervalBernsteinMap( ...
%       tauStart, tauEnd, segmentCount, coefficientCount)
%   [segmentIndex, hullMap] = hs3Engine.polynomial.createSubintervalBernsteinMap( ...
%       tauStart, tauEnd, segmentCount, coefficientCount, segmentBreakTau)
%**************************************************************************
% PURPOSE
%   - Restrict normalized polynomial intervals and return exact Bernstein
%     maps whose coefficients bound every value on each complete interval.
%**************************************************************************
% INPUTS
%   - tauStart, tauEnd (numeric vectors)
%       Inclusive normalized endpoints in [0,1]. Each pair is ordered. The
%       returned map clips its end to the segment that owns tauStart so the
%       established caller behavior remains unchanged at mesh boundaries.
%   - segmentCount (positive integer scalar), polynomial segment count.
%   - coefficientCount (positive integer scalar), powers per segment.
%   - segmentBreakTau ((N+1)-element vector, optional)
%       Strictly increasing normalized boundaries; [] selects a uniform mesh.
%**************************************************************************
% OUTPUTS
%   - segmentIndex (M-by-1 numeric), one-based owning segment per interval.
%   - hullMap (P-by-P-by-M numeric), restricted power-to-Bernstein maps.
%**************************************************************************
% UNITS
%   - Inputs and maps are dimensionless.
%   - Mapped values retain the source units.
%**************************************************************************

%% Section 1: Validate And Locate Every Interval

% Tau is normalized over the complete motion. Each polynomial uses a local
% coordinate from zero to one. Assign the final endpoint to the last segment.
% Callers verify that an interval does not cross a segment boundary.

validateattributes(tauStart, {'numeric'}, ...
    {'real', 'finite', 'vector'});
validateattributes(tauEnd, {'numeric'}, ...
    {'real', 'finite', 'vector'});
validateattributes(segmentCount, {'numeric'}, ...
    {'real', 'finite', 'scalar', 'integer', 'positive'});
validateattributes(coefficientCount, {'numeric'}, ...
    {'real', 'finite', 'scalar', 'integer', 'positive'});
tauStart = double(tauStart(:));
tauEnd = double(tauEnd(:));
if nargin < 5
    segmentBreakTau = [];
end
[segmentBreakTau, ~, isUniformMesh] = ...
    hs3Engine.polynomial.resolveSegmentMesh( ...
    segmentCount, 1, segmentBreakTau);
if numel(tauStart) ~= numel(tauEnd) || ...
        any(tauStart < 0 | tauStart > 1) || ...
        any(tauEnd < tauStart | tauEnd > 1)
    error("createSubintervalBernsteinMap:InvalidIntervals", ...
        "tauStart and tauEnd must be equal-length ordered vectors in [0,1].");
end
persistent cachedMap
cacheMatches = ~isempty(cachedMap) && ...
    cachedMap.SegmentCount == segmentCount && ...
    cachedMap.CoefficientCount == coefficientCount && ...
    isequal(cachedMap.SegmentBreakTau, segmentBreakTau) && ...
    isequal(cachedMap.TauStart, tauStart) && ...
    isequal(cachedMap.TauEnd, tauEnd);
if cacheMatches
    segmentIndex = cachedMap.SegmentIndex;
    hullMap = cachedMap.HullMap;
    return;
end
if isUniformMesh
    scaledStart = segmentCount * tauStart;
    segmentIndex = min(segmentCount, floor(scaledStart) + 1);
    localStart = min(1, max(0, scaledStart - segmentIndex + 1));
    localEnd = min(1, max(0, ...
        segmentCount * tauEnd - segmentIndex + 1));
else
    segmentIndex = discretize(tauStart, segmentBreakTau);
    segmentIndex(tauStart == 1) = segmentCount;
    segmentStartTau = segmentBreakTau(segmentIndex);
    segmentWidthTau = diff(segmentBreakTau);
    localStart = (tauStart - segmentStartTau) ./ ...
        segmentWidthTau(segmentIndex);
    localEnd = (tauEnd - segmentStartTau) ./ ...
        segmentWidthTau(segmentIndex);
    localStart = min(1, max(0, localStart));
    localEnd = min(1, max(0, localEnd));
end
localSpan = max(0, localEnd - localStart);

% The substitution tau_local = localStart + localSpan*u maps u in [0,1]
% onto the requested part of the segment. For a point constraint localSpan is
% zero, so every restricted coefficient reduces to the value at that point.

%% Section 2: Restrict The Power Basis And Convert To Bernstein

% Apply the binomial expansion of (localStart + localSpan*u)^k to obtain
% ascending powers of u, then change that restricted polynomial to Bernstein
% form. hullMap therefore performs both operations with one matrix multiply.

degree = coefficientCount - 1;
sourceExponent = 0:degree;
targetExponent = (0:degree).';
shiftExponent = sourceExponent - targetExponent;
binomialWeight = zeros(coefficientCount);
for targetIndex = 0:degree
    for sourceIndex = targetIndex:degree
        binomialWeight(targetIndex + 1, sourceIndex + 1) = ...
            nchoosek(sourceIndex, targetIndex);
    end
end
restriction = binomialWeight .* ...
    reshape(localStart, 1, 1, []) .^ max(shiftExponent, 0) .* ...
    reshape(localSpan, 1, 1, []) .^ targetExponent;
conversion = hs3Engine.polynomial.convertPowerToBernstein(eye(coefficientCount));
hullMap = reshape( ...
    conversion * reshape(restriction, coefficientCount, []), ...
    coefficientCount, coefficientCount, numel(tauStart));

% Publish one complete record so interruption cannot expose a key paired
% with partially replaced output arrays. One entry bounds persistent memory.
cachedMap = struct( ...
    "SegmentCount", segmentCount, ...
    "CoefficientCount", coefficientCount, ...
    "SegmentBreakTau", segmentBreakTau, ...
    "TauStart", tauStart, ...
    "TauEnd", tauEnd, ...
    "SegmentIndex", segmentIndex, ...
    "HullMap", hullMap);
end
