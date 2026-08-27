function [segmentIndex, hullMap] = createSubintervalBernsteinMap( ...
        tauStart, tauEnd, segmentCount, coefficientCount)
%% Section 0: Header & Readme
% SYNTAX
%   [segmentIndex, hullMap] = hs3Internal.polynomial.createSubintervalBernsteinMap( ...
%       tauStart, tauEnd, segmentCount, coefficientCount)
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

% local coordinate from zero to one. A start exactly on the final endpoint is
% assigned to the last segment. Callers verify that nonzero intervals do not
% cross a segment boundary before asking this routine for one segment hull.
% tau is normalized over the complete motion. Each polynomial uses a local
% coordinate from zero to one. Assign the final endpoint to the last segment.
% Callers verify that an interval does not cross a segment boundary.
% local coordinate from zero to one. A start exactly on the final endpoint is
% assigned to the last segment. Callers verify that nonzero intervals do not
% cross a segment boundary before asking this routine for one segment hull.

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
if numel(tauStart) ~= numel(tauEnd) || ...
        any(tauStart < 0 | tauStart > 1) || ...
        any(tauEnd < tauStart | tauEnd > 1)
    error("createSubintervalBernsteinMap:InvalidIntervals", ...
        "tauStart and tauEnd must be equal-length ordered vectors in [0,1].");
end
scaledStart = segmentCount * tauStart;
segmentIndex = min(segmentCount, floor(scaledStart) + 1);
localStart = min(1, max(0, scaledStart - segmentIndex + 1));
localEnd = min(1, max(0, segmentCount * tauEnd - segmentIndex + 1));
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
conversion = hs3Internal.polynomial.convertPowerToBernstein(eye(coefficientCount));
hullMap = reshape( ...
    conversion * reshape(restriction, coefficientCount, []), ...
    coefficientCount, coefficientCount, numel(tauStart));
end
