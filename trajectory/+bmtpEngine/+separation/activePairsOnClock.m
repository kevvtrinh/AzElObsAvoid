function regionActiveBySegment = activePairsOnClock(segmentBoundaryTime_s, coverage, regionCount)
%% Section 0: Header & Readme
% SYNTAX
%   regionActiveBySegment = bmtpEngine.separation.activePairsOnClock( ...
%       segmentBoundaryTime_s, coverage, regionCount)
%**************************************************************************
% PURPOSE
%   - For time-scoped regions, mark segment/region pairs whose shared
%     interval has positive length on the supplied absolute clock. For a
%     segment with positive rounded length, this equals the strict
%     start/end overlap test. Without time scoping every pair is active.
%   - A segment from 2^53 to 2^53 + 1 s rounds to [2^53, 2^53] in double
%     precision. It shares no time with any time-scoped region, even one
%     spanning it.
%**************************************************************************
% INPUTS
%   - segmentBoundaryTime_s ((S+1)-by-1 numeric vector)
%       Absolute segment boundary times after rounding.
%   - coverage (scalar struct)
%       Region activity intervals in ActiveTimeInterval_s when time scoped.
%   - regionCount (nonnegative integer scalar)
%       Number of prepared obstacle regions.
%**************************************************************************
% OUTPUTS
%   - regionActiveBySegment (S-by-R logical matrix)
%       For time-scoped regions, true only when the segment and region share
%       positive time after rounding. With no ActiveTimeInterval_s field,
%       every pair is active.
%**************************************************************************
% UNITS
%   - Boundary and activity times are absolute seconds.
%**************************************************************************

%% Section 1: Check Inputs And Find Positive Shared Time

validateattributes(segmentBoundaryTime_s, {'numeric'}, {'real', 'finite', 'column', 'nonempty'});
validateattributes(coverage, {'struct'}, {'scalar'});
validateattributes(regionCount, {'numeric'}, {'real', 'finite', 'scalar', 'integer', 'nonnegative'});
assert(numel(segmentBoundaryTime_s) >= 2, 'bmtpEngine:InvalidSegmentClock', ...
    'The segment clock must contain at least two boundaries.');

segmentCount = numel(segmentBoundaryTime_s) - 1;
if ~isfield(coverage, 'ActiveTimeInterval_s')
    regionActiveBySegment = true(segmentCount, regionCount);
    return
end

regionActiveIntervals_s = coverage.ActiveTimeInterval_s;
validateattributes(regionActiveIntervals_s, {'numeric'}, ...
    {'real', 'finite', 'size', [regionCount, 2]});
sharedStartTime_s = max(segmentBoundaryTime_s(1:end - 1), regionActiveIntervals_s(:, 1).');
sharedEndTime_s   = min(segmentBoundaryTime_s(2:end), regionActiveIntervals_s(:, 2).');
regionActiveBySegment = sharedStartTime_s < sharedEndTime_s;
end
