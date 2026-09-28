function [controlPoint_units, segmentTime_s, powerCoefficients_units, parentSegmentIndex] = ...
    subdivideMotion(controlPoint_units, segmentTime_s, powerCoefficients_units, splitSegment, splitProgress)
%% Section 0: Header & Readme
% SYNTAX
%   [subdividedControls_units, subdividedSegmentTime_s, ...
%       subdividedPower_units, parentSegmentIndex] = ...
%       bmtpEngine.motion.subdivideMotion(controlPoint_units, segmentTime_s, ...
%       powerCoefficients_units, splitSegment)
%   [subdividedControls_units, subdividedSegmentTime_s, ...
%       subdividedPower_units, parentSegmentIndex] = ...
%       bmtpEngine.motion.subdivideMotion(controlPoint_units, segmentTime_s, ...
%       powerCoefficients_units, splitSegment, splitProgress)
%**************************************************************************
% PURPOSE
%   - Cut selected Bezier motion segments into two pieces. Together, the
%     pieces follow the same curve over the same total time. Other segments
%     are copied unchanged.
%   - This function does not correct endpoint states or joins between segments.
%**************************************************************************
% INPUTS
%   - controlPoint_units (S-by-(D+1)-by-2 numeric array)
%       S is the number of segments. Each segment has D+1 control points
%       for a degree-D Bezier curve. The last dimension holds the two
%       position axes in their supplied order.
%   - segmentTime_s (S-by-1 positive numeric vector)
%       Duration of each segment.
%   - powerCoefficients_units (S-by-2-by-(D+1) numeric array or empty)
%       Optional polynomial coefficients for each axis, ordered as constant,
%       u, u^2, and so on, where u runs from 0 to 1 within a segment.
%       Empty means none were supplied; NaN values remain unspecified.
%   - splitSegment (S-by-1 logical vector)
%       True for each segment to cut; false keeps one unchanged piece.
%   - splitProgress (S-by-1 numeric vector, optional)
%       Where to cut each selected segment, as a fraction from 0 to 1.
%       Defaults to 0.5. Only selected fractions are checked; each must
%       lie strictly between 0 and 1 and leave both pieces positive time.
%       All per-segment input lengths must match S, or the call throws an error.
%**************************************************************************
% OUTPUTS
%   - controlPoint_units, segmentTime_s, powerCoefficients_units
%       Data for the output pieces, in input order. A split produces its
%       earlier piece followed by its later piece.
%   - parentSegmentIndex (numeric column)
%       Input row that produced each output row. If only segment 2 of 3 is
%       split, this is [1; 2; 2; 3].
%**************************************************************************
% UNITS
%   - Positions use the supplied coordinate units; time is seconds.
%     Segment fractions are unitless.
%**************************************************************************

%% Section 1: Validate Inputs And Allocate Output Rows

splitSegment = logical(splitSegment(:));
segmentCount = size(controlPoint_units, 1);
if nargin < 5
    splitProgress = repmat(0.5, segmentCount, 1);
end
splitProgress = splitProgress(:);
% Every per-segment input must cover the same original rows. A short split
% mask could otherwise leave an original segment out of the output.
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
% Copy each input segment once, then reserve a second row for each split.
% Keep the source row so later checks can relate pieces to their originals.
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

% Each new Bezier piece gets its own fraction from 0 to 1. Restricting the
% original curve to [0, f] and [f, 1] gives the two matching control sets.
for segmentIndex = find(splitSegment).'
    outputSegmentIndex = firstOutputSegmentIndex(segmentIndex);
    splitLocation      = splitProgress(segmentIndex);
    segmentControls_units = squeeze(controlPoint_units(segmentIndex, :, :));
    subdividedControls_units(outputSegmentIndex, :, :) = ...
        bmtpEngine.motion.restrictBezier(segmentControls_units, [0, splitLocation]);
    subdividedControls_units(outputSegmentIndex + 1, :, :) = ...
        bmtpEngine.motion.restrictBezier(segmentControls_units, [splitLocation, 1]);

    % First time = f x original time. Subtract it for the second piece so
    % the two stored times still add to the original. At f = 0.3, a
    % 10-second segment becomes 3 seconds followed by 7 seconds.
    leftDuration_s   = segmentTime_s(segmentIndex) * splitLocation;
    pieceDurations_s = [leftDuration_s; segmentTime_s(segmentIndex) - leftDuration_s];
    % Reject a fraction so close to an end that rounding leaves a piece
    % with zero time; a zero-duration motion piece cannot be used.
    if any(~(pieceDurations_s > 0) | ~isfinite(pieceDurations_s))
        error('subdivideMotion:InvalidSplitProgress', ...
            'A split must leave both pieces a positive finite duration.');
    end
    subdividedSegmentTime_s(outputSegmentIndex:outputSegmentIndex + 1) = pieceDurations_s;

    if isempty(powerCoefficients_units)
        continue
    end
    % Let v run from 0 to 1 on a new piece. Its original fraction is
    % u = f x v on the first piece and u = f + (1 - f) x v on the second.
    % For f = 0.3, the second piece uses u = 0.3 + 0.7 x v. Expand the
    % supplied polynomial directly instead of rebuilding it from controls.
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
