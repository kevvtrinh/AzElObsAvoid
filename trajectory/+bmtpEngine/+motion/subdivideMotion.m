function [controlPoint_units, segmentTime_s, powerCoefficients_units, parentSegmentIndex] = ...
    subdivideMotion(controlPoint_units, segmentTime_s, powerCoefficients_units, splitSegment, splitProgress)
%% Section 0: Header & Readme
% SYNTAX
%   [controls, durations, coefficients, parentSegmentIndex] = ...
%       bmtpEngine.motion.subdivideMotion(controls, durations, coefficients, splitSegment)
%   [controls, durations, coefficients, parentSegmentIndex] = ...
%       bmtpEngine.motion.subdivideMotion(controls, durations, coefficients, splitSegment, splitProgress)
%**************************************************************************
% PURPOSE
%   - Split selected segments without changing their curve or physical timing.
%     This operation does not impose endpoint states or correct curve joins.
%**************************************************************************
% INPUTS
%   - controlPoint_units (S-by-(D+1)-by-2 numeric array)
%       Composite Bezier controls.
%   - segmentTime_s (S-by-1 positive numeric vector)
%       Duration of each segment.
%   - powerCoefficients_units (S-by-2-by-(D+1) numeric array or empty)
%       Optional coefficients in ascending powers of segment fraction.
%       NaN axes remain unspecified; finite axes are split directly.
%   - splitSegment (S-by-1 logical vector)
%       True for each segment to divide into two pieces.
%   - splitProgress (S-by-1 numeric vector, optional)
%       Fraction strictly between 0 and 1 for each split; defaults to 0.5.
%       Inputs whose sizes do not match the segment count, or a selected
%       fraction outside (0, 1), throw an error.
%**************************************************************************
% OUTPUTS
%   - controlPoint_units, segmentTime_s, powerCoefficients_units
%       Subdivided controls, durations, and any supplied coefficients.
%   - parentSegmentIndex (numeric column)
%       Input segment that produced each output piece.
%**************************************************************************
% UNITS
%   - Position is coordinate units, time is seconds, and fractions are unitless.
%**************************************************************************

%% Section 1: Allocate The Selected Pieces

splitSegment = logical(splitSegment(:));
segmentCount = size(controlPoint_units, 1);
if nargin < 5
    splitProgress = repmat(0.5, segmentCount, 1);
end
splitProgress = splitProgress(:);
% Every per-segment input must name the same segments as the controls, or a
% short mask would silently drop the segments it does not mention.
coefficientsMatch = isempty(powerCoefficients_units) || ...
    isequal(size(powerCoefficients_units), [segmentCount, 2, size(controlPoint_units, 2)]);
if numel(splitSegment) ~= segmentCount || numel(segmentTime_s) ~= segmentCount || ...
        numel(splitProgress) ~= segmentCount || ~coefficientsMatch
    error('subdivideMotion:InvalidInput', ...
        'splitSegment, segmentTime_s, splitProgress and coefficients must match the segment count.');
end
validateattributes(segmentTime_s, {'numeric'}, {'real', 'finite', 'positive'});
selectedProgress = splitProgress(splitSegment);
if any(~isfinite(selectedProgress) | selectedProgress <= 0 | selectedProgress >= 1)
    error('subdivideMotion:InvalidSplitProgress', ...
        'Each selected split fraction must be finite and lie strictly between 0 and 1.');
end
outputCountBySegment     = 1 + double(splitSegment);
firstOutputSegmentIndex = 1 + [0; cumsum(outputCountBySegment(1:end - 1))];
parentSegmentIndex      = reshape(repelem((1:numel(splitSegment)).', outputCountBySegment), [], 1);
curveDegree             = size(controlPoint_units, 2) - 1;
subdividedControls_units = controlPoint_units(parentSegmentIndex, :, :);
subdividedSegmentTime_s  = reshape(segmentTime_s(parentSegmentIndex), [], 1);
subdividedPower_units    = powerCoefficients_units;
if ~isempty(powerCoefficients_units)
    subdividedPower_units = powerCoefficients_units(parentSegmentIndex, :, :);
end

%% Section 2: Restrict Each Curve And Divide Its Duration

for segmentIndex = find(splitSegment).'
    outputSegmentIndex = firstOutputSegmentIndex(segmentIndex);
    splitLocation      = splitProgress(segmentIndex);
    segmentControls_units = squeeze(controlPoint_units(segmentIndex, :, :));
    subdividedControls_units(outputSegmentIndex, :, :) = ...
        bmtpEngine.motion.restrictBezier(segmentControls_units, [0, splitLocation]);
    subdividedControls_units(outputSegmentIndex + 1, :, :) = ...
        bmtpEngine.motion.restrictBezier(segmentControls_units, [splitLocation, 1]);

    % Use subtraction for the remaining time so both pieces retain the
    % parent's duration. A split at 0.3 divides 10 seconds into 3 and 7.
    leftDuration_s   = segmentTime_s(segmentIndex) * splitLocation;
    pieceDurations_s = [leftDuration_s; segmentTime_s(segmentIndex) - leftDuration_s];
    % A fraction so small that a piece rounds to zero duration is refused:
    % that piece would have no time at all.
    if any(~(pieceDurations_s > 0) | ~isfinite(pieceDurations_s))
        error('subdivideMotion:InvalidSplitProgress', ...
            'A split must leave both pieces a positive finite duration.');
    end
    subdividedSegmentTime_s(outputSegmentIndex:outputSegmentIndex + 1) = pieceDurations_s;

    if isempty(powerCoefficients_units)
        continue
    end
    % For new fraction v, the original fraction is u = f x v on the left
    % and u = f + (1 - f) x v on the right. Expand the powers directly so
    % known zero coefficients survive without a conversion through controls.
    for powerIndex = 0:curveDegree
        subdividedPower_units(outputSegmentIndex, :, powerIndex + 1) = ...
            powerCoefficients_units(segmentIndex, :, powerIndex + 1) * splitLocation ^ powerIndex;
        rightCoefficient_units = zeros(1, 2);
        for sourcePowerIndex = powerIndex:curveDegree
            rightCoefficient_units = rightCoefficient_units + nchoosek(sourcePowerIndex, powerIndex) * ...
                powerCoefficients_units(segmentIndex, :, sourcePowerIndex + 1) * ...
                splitLocation ^ (sourcePowerIndex - powerIndex) * (1 - splitLocation) ^ powerIndex;
        end
        subdividedPower_units(outputSegmentIndex + 1, :, powerIndex + 1) = rightCoefficient_units;
    end
end

controlPoint_units      = subdividedControls_units;
segmentTime_s           = subdividedSegmentTime_s;
powerCoefficients_units = subdividedPower_units;
end
