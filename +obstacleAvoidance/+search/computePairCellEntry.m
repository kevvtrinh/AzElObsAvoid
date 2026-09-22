function [regionIndices, boxEntryFractions, boxExitFractions, activeStartTimes_s, activeEndTimes_s] = ...
    computePairCellEntry(segmentStart_units, segmentEnd_units, boxMinimum_units, ...
    boxMaximum_units, activeIntervals_s)
%% Section 0: Header & Readme
% SYNTAX
%   [regionIndices, boxEntryFractions, boxExitFractions, activeStartTimes_s, activeEndTimes_s] = ...
%       obstacleAvoidance.search.computePairCellEntry(segmentStart_units, segmentEnd_units, ...
%       boxMinimum_units, boxMaximum_units, activeIntervals_s)
%**************************************************************************
% PURPOSE
%   - Find obstacle-region boxes that a straight segment can meet, and the
%     fractions of the segment spent inside each box. Include rounding
%     allowance so possible collisions reach the full geometry check.
%   - Order candidates by the time each region becomes active.
%**************************************************************************
% INPUTS
%   - segmentStart_units, segmentEnd_units (1-by-2 numeric)
%       Segment endpoints.
%   - boxMinimum_units, boxMaximum_units (C-by-2 numeric)
%       Lower and upper [x y] box corners from createCellBoxes.
%   - activeIntervals_s (C-by-2 numeric)
%       [start end] times when each region is active.
%**************************************************************************
% OUTPUTS
%   - regionIndices (K-by-1 numeric)
%       Indices of regions whose boxes may meet the segment.
%   - boxEntryFractions, boxExitFractions (K-by-1 numeric)
%       Start/end fractions in [0 1] of the possible box overlap.
%   - activeStartTimes_s, activeEndTimes_s (K-by-1 numeric)
%       Active time interval for each selected region.
%**************************************************************************
% UNITS
%   - Positions use coordinate units; times use seconds. Segment fractions
%     are unitless: 0 = the start point and 1 = the end point.
%**************************************************************************

%% Section 1: Find The Segment Fractions Inside Each Box

% Intersect the x and y ranges. For x = 0 -> 10 and box x = [4 6],
% the x range gives fractions [0.4 0.6]; the y range can narrow this further.
regionCount         = size(boxMinimum_units, 1);
segmentVector_units = segmentEnd_units - segmentStart_units;
entryFractions      = zeros(regionCount, 1);
exitFractions       = ones(regionCount, 1);
segmentCouldMeetBox = true(regionCount, 1);
boxHasUnknownBounds = any(isnan(boxMinimum_units) | isnan(boxMaximum_units), 2);
for axisIndex = 1:2
    % With no movement on this axis, the entire segment has one coordinate.
    % That coordinate must lie inside the box's range.
    if segmentVector_units(axisIndex) == 0
        segmentCouldMeetBox = segmentCouldMeetBox & ( ...
            segmentStart_units(axisIndex) >= boxMinimum_units(:, axisIndex) & ...
            segmentStart_units(axisIndex) <= boxMaximum_units(:, axisIndex));
    else
        minimumBoundaryFraction = (boxMinimum_units(:, axisIndex) - ...
            segmentStart_units(axisIndex)) / segmentVector_units(axisIndex);
        maximumBoundaryFraction = (boxMaximum_units(:, axisIndex) - ...
            segmentStart_units(axisIndex)) / segmentVector_units(axisIndex);
        % Min/max handles either travel direction along the axis.
        entryFractions = max(entryFractions, min(minimumBoundaryFraction, maximumBoundaryFraction));
        exitFractions  = min(exitFractions, max(minimumBoundaryFraction, maximumBoundaryFraction));
    end
end

%% Section 2: Allow For Rounding And Keep Boxes With Unknown Bounds

fractionRoundingAllowance = 256 * eps(max(1, max(abs([entryFractions, exitFractions]), [], 2)));
fractionRoundingAllowance(~isfinite(fractionRoundingAllowance)) = 0;
segmentCouldMeetBox = segmentCouldMeetBox & entryFractions <= exitFractions + fractionRoundingAllowance & ...
    exitFractions >= -fractionRoundingAllowance & entryFractions <= 1 + fractionRoundingAllowance;

% Unknown bounds cannot prove that a segment is clear. Keep the region
% for the full collision check over the whole segment.
segmentCouldMeetBox(boxHasUnknownBounds) = true;
entryFractions(boxHasUnknownBounds)      = 0;
exitFractions(boxHasUnknownBounds)       = 1;

%% Section 3: Return Candidates In Active-Time Order

regionIndices = find(segmentCouldMeetBox);
% Break equal start-time ties by region index for repeatable ordering.
if ~isempty(regionIndices)
    [~, activationSortOrder] = sortrows( ...
        [activeIntervals_s(regionIndices, 1), regionIndices], [1, 2]);
    regionIndices = regionIndices(activationSortOrder);
end
boxEntryFractions  = max(0, entryFractions(regionIndices) - fractionRoundingAllowance(regionIndices));
boxExitFractions   = min(1, exitFractions(regionIndices) + fractionRoundingAllowance(regionIndices));
activeStartTimes_s = activeIntervals_s(regionIndices, 1);
activeEndTimes_s   = activeIntervals_s(regionIndices, 2);
end
