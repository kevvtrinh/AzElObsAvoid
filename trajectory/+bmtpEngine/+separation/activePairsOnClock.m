function regionActiveBySegment = activePairsOnClock(segmentBoundaryTime_s, coverage, regionCount)
%% Section 0: Header & Readme
% SYNTAX
%   regionActiveBySegment = bmtpEngine.separation.activePairsOnClock( ...
%       segmentBoundaryTime_s, coverage, regionCount)
%**************************************************************************
% PURPOSE
%   - Mark which motion segments overlap each obstacle's active time.
%     Overlap must last longer than an instant: touching at one boundary
%     does not count. Without active-time intervals, every pair is active.
%   - Use the supplied absolute times after floating-point rounding. For
%     example, 2^53 + 1 rounds to 2^53 in double precision, so a segment
%     from 2^53 to 2^53 + 1 has zero length on this clock.
%**************************************************************************
% INPUTS
%   - segmentBoundaryTime_s ((S+1)-by-1 numeric vector)
%       Ordered absolute boundaries of S motion segments, in seconds.
%   - coverage (scalar struct)
%       Optional ActiveTimeInterval_s holds one [start, end] time row per
%       region. If absent, every region is treated as always active.
%   - regionCount (nonnegative integer scalar)
%       Number of prepared obstacle regions.
%**************************************************************************
% OUTPUTS
%   - regionActiveBySegment (S-by-R logical matrix)
%       With active-time intervals, true only when a segment and region
%       share positive time. Without them, every pair is true, including a
%       segment whose rounded duration is zero. R equals regionCount.
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
% For each pair, the later start and earlier end delimit shared time.
% The strict comparison excludes intervals that only touch at an endpoint.
sharedStartTime_s = max(segmentBoundaryTime_s(1:end - 1), regionActiveIntervals_s(:, 1).');
sharedEndTime_s   = min(segmentBoundaryTime_s(2:end), regionActiveIntervals_s(:, 2).');
regionActiveBySegment = sharedStartTime_s < sharedEndTime_s;
end
