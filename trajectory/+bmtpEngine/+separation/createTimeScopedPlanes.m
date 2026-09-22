function [separatingPlanes, regionActiveBySegment, allRequiredLinesVerified, pairCounts] = createTimeScopedPlanes( ...
    referenceControlPoint_units, segmentTime_s, solverRequest, separationTarget_units, roundoffReserve_units)
%% Section 0: Header & Readme
% SYNTAX
%   [separatingPlanes, regionActiveBySegment, allRequiredLinesVerified, pairCounts] = ...
%       bmtpEngine.separation.createTimeScopedPlanes(referenceControlPoint_units, ...
%       segmentTime_s, solverRequest, separationTarget_units, roundoffReserve_units)
%**************************************************************************
% PURPOSE
%   - Find a separating line for every curve-segment/obstacle pair whose
%     time intervals overlap. Use only that shared part of the curve and
%     the obstacle's linearly moving vertices during the same interval.
%**************************************************************************
% INPUTS
%   - referenceControlPoint_units (S-by-(D+1)-by-2 numeric array)
%       Controls of the curve used to find the separating lines.
%   - segmentTime_s (positive numeric scalar or S-by-1 vector)
%       Duration of each segment; a scalar gives every segment the same duration.
%   - solverRequest (scalar struct)
%       Checked inputs and obstacle geometry with its active time intervals.
%   - separationTarget_units (finite numeric scalar)
%       Required distance from each obstacle to its separating line.
%   - roundoffReserve_units (finite numeric scalar)
%       Required trajectory-side separation reserve.
%**************************************************************************
% OUTPUTS
%   - separatingPlanes (S-by-R struct array)
%       Line records and their verification results, indexed by segment and region.
%   - regionActiveBySegment (S-by-R logical matrix)
%       True where a segment and obstacle interval share a positive duration.
%   - allRequiredLinesVerified (logical scalar)
%       False if any applicable pair lacks a verified line.
%   - pairCounts (scalar struct)
%       Number of applicable pairs and number with verified lines.
%**************************************************************************
% UNITS
%   - Position is coordinate units and time is seconds.
%**************************************************************************

%% Section 1: Find Overlapping Segment And Obstacle Times

segmentCount = size(referenceControlPoint_units, 1);
if isscalar(segmentTime_s)
    segmentTime_s = repmat(segmentTime_s, segmentCount, 1);
else
    segmentTime_s = double(segmentTime_s(:));
end
validateattributes(segmentTime_s, {'numeric'}, ...
    {'real', 'finite', 'positive', 'numel', segmentCount});
regionCount           = numel(solverRequest.Regions_units);
separatingPlanes      = repmat(bmtpEngine.separation.createEmptyPlane(), segmentCount, regionCount);
regionActiveBySegment = true(segmentCount, regionCount);
segmentBoundaryTime_s = solverRequest.InitialState.time_s + [0; cumsum(segmentTime_s)];
% Strict comparisons exclude pairs whose intervals only touch at one instant.
if isfield(solverRequest.Coverage, 'ActiveTimeInterval_s')
    obstacleActiveIntervals_s = solverRequest.Coverage.ActiveTimeInterval_s;
    regionActiveBySegment     = segmentBoundaryTime_s(1:end - 1) < obstacleActiveIntervals_s(:, 2).' & ...
        segmentBoundaryTime_s(2:end) > obstacleActiveIntervals_s(:, 1).';
else
    obstacleActiveIntervals_s = zeros(regionCount, 2);
end

%% Section 2: Build And Check Lines Over Each Shared Interval

% For a segment from 2 to 6 s and obstacle interval from 4 to 8 s,
% use only 4 to 6 s: curve fractions [0.5 1]. Check every applicable pair
% even if an earlier one fails, so the returned counts cover the full set.
allRequiredLinesVerified = true;
verifiedPairCount        = 0;
for segmentIndex = 1:segmentCount
    for regionIndex = reshape(find(regionActiveBySegment(segmentIndex, :)), 1, [])
        segmentControlPoint_units = squeeze(referenceControlPoint_units(segmentIndex, :, :));
        overlapInterval_s         = [];
        overlapFractions          = [0, 1];
        separatingLineGeometry    = [];
        if isfield(solverRequest.Coverage, 'ActiveTimeInterval_s')
            overlapInterval_s = [max(segmentBoundaryTime_s(segmentIndex), obstacleActiveIntervals_s(regionIndex, 1)), ...
                min(segmentBoundaryTime_s(segmentIndex + 1), obstacleActiveIntervals_s(regionIndex, 2))];
            overlapFractions = ...
                (overlapInterval_s - segmentBoundaryTime_s(segmentIndex)) / segmentTime_s(segmentIndex);
            overlapFractions          = max(0, min(1, overlapFractions));
            segmentControlPoint_units = bmtpEngine.motion.restrictBezier(segmentControlPoint_units, overlapFractions);
        elseif isfield(solverRequest, 'SeparatingLineGeometry')
            % Static obstacles can reuse their prepared edge geometry.
            separatingLineGeometry = solverRequest.SeparatingLineGeometry{regionIndex};
        end
        obstacleVertices_units = bmtpEngine.separation.regionOnInterval(solverRequest.Regions_units{regionIndex}, ...
            solverRequest.Coverage, regionIndex, overlapInterval_s);
        [plane, exitFlag] = bmtpEngine.separation.solveSeparatingLine( ...
            segmentControlPoint_units, obstacleVertices_units, separationTarget_units, ...
            roundoffReserve_units, separatingLineGeometry);
        plane.TimeFraction = overlapFractions;
        separatingPlanes(segmentIndex, regionIndex) = plane;
        if exitFlag <= 0 || ~plane.Active || ~plane.Verified
            allRequiredLinesVerified = false;
        else
            verifiedPairCount = verifiedPairCount + 1;
        end
    end
end
pairCounts = struct( ...
    'ActivePairCount',   nnz(regionActiveBySegment), ...
    'VerifiedPairCount', verifiedPairCount);
end
