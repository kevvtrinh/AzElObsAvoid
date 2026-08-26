function [segmentIndex, hullMap] = subintervalHullMap( ...
        tauStart, tauEnd, segmentCount, coefficientCount)
%% Section 0: Header & Readme
% SYNTAX
%   [segmentIndex, hullMap] = azElInternal.subintervalHullMap( ...
%       tauStart, tauEnd, segmentCount, coefficientCount)
%**************************************************************************
% PURPOSE
%   - Map each normalized sub-interval onto its segment and one exact
%     power-to-Bernstein matrix whose coefficients bound the continuous
%     polynomial over that sub-interval alone.
%   - Sampling a polynomial at one instant bounds nothing between samples,
%     so obstacle constraints use these hulls instead of point values.
%**************************************************************************
% INPUTS
%   - tauStart, tauEnd (numeric vectors, 0 to 1)
%       Inclusive sub-interval endpoints in normalized trajectory time. Each
%       pair must lie inside one segment; tauEnd may equal tauStart.
%   - segmentCount (positive integer scalar)
%       Collocation segment count of the trajectory being bounded.
%   - coefficientCount (positive integer scalar)
%       Power-coefficient count per segment, so the degree is one less.
%**************************************************************************
% OUTPUTS
%   - segmentIndex (numeric column)
%       One-based segment owning each sub-interval.
%   - hullMap (coefficientCount-by-coefficientCount-by-N numeric)
%       hullMap(:, :, k) times the segment power coefficients returns the
%       Bernstein coefficients of the restricted polynomial. Their minimum
%       and maximum bound the polynomial on the whole sub-interval.
%**************************************************************************
% UNITS
%   - Inputs and outputs are unitless; the mapped coefficients keep the
%     physical units of the polynomial they are applied to.
%**************************************************************************

%% Section 1: Locate Each Sub-Interval Inside Its Segment

tauStart = double(tauStart(:));
tauEnd = double(tauEnd(:));
scaledStart = segmentCount * tauStart;
segmentIndex = min(segmentCount, floor(scaledStart) + 1);
localStart = min(1, max(0, scaledStart - segmentIndex + 1));
localEnd = min(1, max(0, segmentCount * tauEnd - segmentIndex + 1));
localSpan = max(0, localEnd - localStart);

%% Section 2: Restrict The Power Basis And Convert To Bernstein

% Substituting u = localStart + localSpan * s is exact and upper triangular,
% so restriction costs one small matrix per sub-interval.
degree = coefficientCount - 1;
sourceExponent = 0:degree;
targetExponent = (0:degree).';
shiftExponent = sourceExponent - targetExponent;
binomialWeight = zeros(coefficientCount);
for targetIndex = 0:degree
    binomialWeight(targetIndex + 1, targetIndex + 1:end) = ...
        arrayfun(@(source) nchoosek(source, targetIndex), ...
        targetIndex:degree);
end
restriction = binomialWeight .* ...
    reshape(localStart, 1, 1, []) .^ max(shiftExponent, 0) .* ...
    reshape(localSpan, 1, 1, []) .^ targetExponent;

% Bernstein coefficients of the restricted polynomial bound it on the whole
% sub-interval by the convex-hull property of the Bernstein basis.
conversion = azElInternal.powerToBernstein(eye(coefficientCount));
hullMap = reshape(conversion * reshape(restriction, coefficientCount, []), ...
    coefficientCount, coefficientCount, numel(tauStart));
end
