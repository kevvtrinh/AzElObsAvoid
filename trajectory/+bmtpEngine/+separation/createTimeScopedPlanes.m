function [separatingPlanes, regionActiveBySegment, allRequiredLinesVerified, lineReport] = createTimeScopedPlanes( ...
    referenceControlPoint_units, segmentTime_s, solverRequest, separationTarget_units, roundoffReserve_units, lineUpdate)
%% Section 0: Header & Readme
% SYNTAX
%   [separatingPlanes, regionActiveBySegment, allRequiredLinesVerified, lineReport] = ...
%       bmtpEngine.separation.createTimeScopedPlanes(referenceControlPoint_units, ...
%       segmentTime_s, solverRequest, separationTarget_units, roundoffReserve_units)
%   [...] = bmtpEngine.separation.createTimeScopedPlanes(..., lineUpdate)
%**************************************************************************
% PURPOSE
%   - Find a separating line for every curve-segment/obstacle pair whose
%     time intervals overlap. Use only that shared part of the curve and
%     the obstacle's linearly moving vertices during the same interval.
%   - Optionally keep the direction of each pair's existing line and only
%     re-place and re-check it, which is cheaper than solving for a new one.
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
%   - lineUpdate (scalar struct, optional; default: solve every pair, check all)
%       Planes (S-by-R struct array): starting line records. Pairs this call
%       does not visit keep their record. Default: empty records.
%       VerifyExisting (logical): true keeps each pair's line direction from
%       Planes, places it against the obstacle, and checks it instead of
%       solving. Default false.
%       StopAtFirstFailure (logical): true returns after the first pair that
%       fails. When solving, failure means the solver returned no usable
%       line; when verifying, it means the line did not pass. Default false.
%**************************************************************************
% OUTPUTS
%   - separatingPlanes (S-by-R struct array)
%       Line records and their verification results, indexed by segment and region.
%   - regionActiveBySegment (S-by-R logical matrix)
%       For time-scoped regions, true where the shared time interval has
%       positive length after rounding; without time scoping, all true.
%   - allRequiredLinesVerified (logical scalar)
%       False if any applicable pair lacks a verified line.
%   - lineReport (scalar struct)
%       ActivePairCount, CheckedPairCount (active pairs checked or solved), VerifiedPairCount,
%       UnavailablePairCount (solver returned no usable line), VerifiedPairs
%       (S-by-R logical, true for verified and for inactive pairs), SocpCount
%       (numerical line solves), and SolverTime_s for those solves.
%**************************************************************************
% UNITS
%   - Position is coordinate units and time is seconds.
%**************************************************************************

%% Section 1: Read The Request And Find Overlapping Segment And Obstacle Times

segmentCount = size(referenceControlPoint_units, 1);
if isscalar(segmentTime_s)
    segmentTime_s = repmat(segmentTime_s, segmentCount, 1);
else
    segmentTime_s = double(segmentTime_s(:));
end
validateattributes(segmentTime_s, {'numeric'}, ...
    {'real', 'finite', 'positive', 'numel', segmentCount});
regionCount = numel(solverRequest.Regions_units);
if nargin < 6 || isempty(lineUpdate)
    lineUpdate = struct();
end
separatingPlanes = repmat(bmtpEngine.separation.createEmptyPlane(), segmentCount, regionCount);
if isfield(lineUpdate, 'Planes') && ~isempty(lineUpdate.Planes)
    validateattributes(lineUpdate.Planes, {'struct'}, {'size', [segmentCount, regionCount]});
    separatingPlanes = lineUpdate.Planes;
end
verifyExistingLines = isfield(lineUpdate, 'VerifyExisting') && lineUpdate.VerifyExisting;
stopAtFirstFailure  = isfield(lineUpdate, 'StopAtFirstFailure') && lineUpdate.StopAtFirstFailure;

segmentBoundaryTime_s = solverRequest.InitialState.time_s + [0; cumsum(segmentTime_s)];
regionActiveBySegment = bmtpEngine.separation.activePairsOnClock( ...
    segmentBoundaryTime_s, solverRequest.Coverage, regionCount);
if isfield(solverRequest.Coverage, 'ActiveTimeInterval_s')
    obstacleActiveIntervals_s = solverRequest.Coverage.ActiveTimeInterval_s;
else
    obstacleActiveIntervals_s = zeros(regionCount, 2);
end

%% Section 2: Build Or Check A Line Over Each Shared Interval

% For a segment from 2 to 6 s and obstacle interval from 4 to 8 s,
% use only 4 to 6 s: curve fractions [0.5 1]. For time-scoped regions,
% active means a positive shared window after rounding; static regions are
% active on every segment. Inactive pairs need no separating line.
verifiedPairs        = ~regionActiveBySegment;
checkedPairCount     = 0;
verifiedPairCount    = 0;
unavailablePairCount = 0;
socpCount            = 0;
solverTime_s         = 0;
stopRequested        = false;
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

        lineIsAvailable = true;
        if verifyExistingLines
            % Keep the existing direction, place the line against the
            % obstacle at both interval ends, then check the curve side.
            plane              = separatingPlanes(segmentIndex, regionIndex);
            lineNormal         = plane.Normal(1, :);
            plane.Offset_units = separationTarget_units - [min(obstacleVertices_units(:, :, 1) * lineNormal.'), ...
                min(obstacleVertices_units(:, :, end) * lineNormal.')];
            plane.TimeFraction = overlapFractions;
            plane              = bmtpEngine.separation.verifySeparatingLine( ...
                plane, segmentControlPoint_units, obstacleVertices_units, roundoffReserve_units, separationTarget_units);
        else
            [plane, exitFlag, lineOutput] = bmtpEngine.separation.solveSeparatingLine( ...
                segmentControlPoint_units, obstacleVertices_units, separationTarget_units, ...
                roundoffReserve_units, separatingLineGeometry);
            plane.TimeFraction  = overlapFractions;
            lineIsAvailable     = exitFlag > 0 && plane.Active;
            usedNumericalSolver = ~(isfield(lineOutput, 'IsAnalytic') && lineOutput.IsAnalytic);
            socpCount           = socpCount + usedNumericalSolver;
            if usedNumericalSolver && isfield(lineOutput, 'TotalTime_s')
                solverTime_s = solverTime_s + lineOutput.TotalTime_s;
            end
        end
        separatingPlanes(segmentIndex, regionIndex) = plane;
        verifiedPairs(segmentIndex, regionIndex)    = plane.Verified;
        checkedPairCount     = checkedPairCount + 1;
        verifiedPairCount    = verifiedPairCount + plane.Verified;
        unavailablePairCount = unavailablePairCount + ~lineIsAvailable;
        if stopAtFirstFailure && (~lineIsAvailable || (verifyExistingLines && ~plane.Verified))
            stopRequested = true;
            break
        end
    end
    if stopRequested
        break
    end
end
allRequiredLinesVerified = all(verifiedPairs, 'all');
lineReport = struct( ...
    'ActivePairCount',      nnz(regionActiveBySegment), ...
    'CheckedPairCount',     checkedPairCount, ...
    'VerifiedPairCount',    verifiedPairCount, ...
    'UnavailablePairCount', unavailablePairCount, ...
    'VerifiedPairs',        verifiedPairs, ...
    'SocpCount',            socpCount, ...
    'SolverTime_s',         solverTime_s);
end
