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
%   - Find or recheck a line between each motion segment and obstacle that
%     are active at the same time. Compare only their shared time window:
%     the matching piece of the curve and the obstacle at those times.
%   - An optional update reuses an existing line direction, avoiding a new
%     line solve when only its placement and proof need checking.
%**************************************************************************
% INPUTS
%   - referenceControlPoint_units (S-by-(D+1)-by-2 numeric array)
%       Bezier controls of S motion segments in x and y.
%   - segmentTime_s (positive numeric scalar or S-by-1 vector)
%       Duration of each segment; one scalar applies to all S segments.
%   - solverRequest (scalar struct)
%       Checked start time, prepared obstacle regions, optional active-time
%       intervals, and prepared static line geometry.
%   - separationTarget_units (finite numeric scalar)
%       Required obstacle-side margin at the line.
%   - roundoffReserve_units (finite numeric scalar)
%       Extra curve-side margin for numerical rounding.
%   - lineUpdate (scalar struct, optional; default: solve every pair, check all)
%       Planes (S-by-R struct array): starting line records. Pairs this call
%       does not visit keep their record. Default: empty records.
%       VerifyExisting (logical): true reuses Planes, places each line
%       against its obstacle, and checks it instead of solving again.
%       Default false.
%       StopAtFirstFailure (logical): true returns after the first pair that
%       fails. When solving, failure means the solver returned no usable
%       line; when verifying, it means the line did not pass. Default false.
%**************************************************************************
% OUTPUTS
%   - separatingPlanes (S-by-R struct array)
%       Line records and proof results, indexed by segment and region.
%   - regionActiveBySegment (S-by-R logical matrix)
%       True where a segment and timed region share positive time after
%       rounding. Without active-time intervals, every pair is true.
%   - allRequiredLinesVerified (logical scalar)
%       True only if every active pair has a verified line.
%   - lineReport (scalar struct)
%       ActivePairCount counts pairs needing a line. CheckedPairCount counts
%       pairs visited, VerifiedPairCount counts those proved separated, and
%       UnavailablePairCount counts solves with no usable line. VerifiedPairs
%       marks both verified and inactive pairs true. SocpCount counts
%       numerical line solves; SolverTime_s totals their reported time.
%**************************************************************************
% UNITS
%   - Positions and margins use coordinate units; durations and absolute
%     times use seconds; curve fractions are dimensionless.
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

% A segment from 2 to 6 s and region active from 4 to 8 s share 4 to 6 s,
% or curve fractions [0.5, 1]. A timed pair is active only for a positive
% shared window after rounding. Inactive pairs need no separating line.
% Regions without active-time intervals are checked on every segment.
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
            % Convert the shared absolute times to local curve fractions,
            % then keep exactly that Bezier subcurve for this pair.
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
            % Keep the saved line normals. Use the starting normal to set
            % offsets against the obstacle at both ends, then check the
            % resulting line against the curve and obstacle.
            plane              = separatingPlanes(segmentIndex, regionIndex);
            lineNormal         = plane.Normal(1, :);
            plane.Offset_units = separationTarget_units - [min(obstacleVertices_units(:, :, 1) * lineNormal.'), ...
                min(obstacleVertices_units(:, :, end) * lineNormal.')];
            plane.TimeFraction = overlapFractions;
            plane              = bmtpEngine.separation.verifySeparatingLine( ...
                plane, segmentControlPoint_units, obstacleVertices_units, roundoffReserve_units, separationTarget_units);
        else
            % Solve a fresh line. Count only numerical solves in SocpCount;
            % an analytic line does not use the numerical solver.
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
        % Stop once a required line is unavailable. When rechecking saved
        % lines, also stop if the current line fails its proof.
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
% Inactive pairs began as true, so this checks every line that was needed.
lineReport = struct( ...
    'ActivePairCount',      nnz(regionActiveBySegment), ...
    'CheckedPairCount',     checkedPairCount, ...
    'VerifiedPairCount',    verifiedPairCount, ...
    'UnavailablePairCount', unavailablePairCount, ...
    'VerifiedPairs',        verifiedPairs, ...
    'SocpCount',            socpCount, ...
    'SolverTime_s',         solverTime_s);
end
