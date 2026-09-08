function certificate = checkFinalMotion(request, warmStart, preparedMotion, roundoffReserve_units, obstacleTarget_units)
%% Section 0: Header & Readme
% SYNTAX
%   certificate = bmtpEngine.checkFinalMotion( ...
%       request, warmStart, preparedMotion, roundoffReserve_units, ...
%       obstacleTarget_units)
%
% PURPOSE
%   - Check every applicable final curve span against each supplied convex
%     obstacle region using direct separating-plane certificates.
%
% INPUTS
%   - request, warmStart, preparedMotion (scalar structs)
%       Checked request, region applicability, and final prepared curve.
%   - roundoffReserve_units, obstacleTarget_units (finite scalars)
%       Numerical reserve and required obstacle-side target in coordinate units.
%
% OUTPUTS
%   - certificate (scalar struct)
%       Pair coverage, separating planes, counts, and passing state.
%
% UNITS
%   - Position, gaps, and reserves are coordinate units.
%

%% Section 1: Check All Curve And Obstacle Pairs

% Each optimized segment becomes two output spans. Repeat the static
% all-region mask for both spans.
regionActiveBySegment = true(size(preparedMotion.CertifiedControlPoint_units,1),numel(request.Regions_units));
if isfield(request.Coverage,'ActiveTimeInterval_s')
    intervals_s = request.Coverage.ActiveTimeInterval_s;
    starts_s = request.InitialState.time_s+[0;cumsum(preparedMotion.SegmentTime_s(1:end-1))];
    ends_s = starts_s+preparedMotion.SegmentTime_s;
    regionActiveBySegment = starts_s < intervals_s(:,2).' & ends_s > intervals_s(:,1).';
end
spanBreaks_s = request.InitialState.time_s+[0;cumsum(preparedMotion.SegmentTime_s)];
certificate = checkAllCurveObstaclePairs(preparedMotion.CertifiedControlPoint_units, request.Regions_units, request.Coverage, regionActiveBySegment, roundoffReserve_units, obstacleTarget_units,spanBreaks_s);
end

%% Section 2: Local Functions

function certificate = checkAllCurveObstaclePairs(controlPoint_units, regions_units, coverage, regionActiveBySegment, reserve_units, target_units,spanBreaks_s)
    % Verify every applicable output-span and convex-exclusion-region pair.
    segmentCount   = size(controlPoint_units, 1);
    regionCount    = numel(regions_units);
    planes         = repmat(createEmptyPlane(), segmentCount, regionCount);
    verifiedCount  = 0;
    conicCount     = 0;
    analyticCount  = 0;
    conicSolver    = bmtpEngine.accumulateConicDiagnostics();
    minimumGap_units = Inf;
    % Process each segment while assembling the complete motion or interval result.
    for segmentIndex = 1:segmentCount
        trajectory_units = squeeze(controlPoint_units(segmentIndex, :, :));
        % Process each geometric region while constructing or checking the region topology.
        for regionIndex = 1:regionCount
            if ~regionActiveBySegment(segmentIndex, regionIndex)
                continue;
            end
            restricted_units = trajectory_units;
            if isfield(coverage,'ActiveTimeInterval_s')
                interval = (coverage.ActiveTimeInterval_s(regionIndex,:)-spanBreaks_s(segmentIndex))/diff(spanBreaks_s(segmentIndex:segmentIndex+1));
                restricted_units = bmtpEngine.restrictBezier(trajectory_units,max(0,min(1,interval)));
            end
            plane = bmtpEngine.solveSeparatingLine(restricted_units, regions_units{regionIndex}, target_units, reserve_units);
            analyticCount = analyticCount + 1;
            planes(segmentIndex, regionIndex) = plane;
            if plane.Verified
                verifiedCount  = verifiedCount + 1;
                minimumGap_units = min(minimumGap_units, plane.SignedGap_units);
            end
        end
    end
    allPairCount     = nnz(regionActiveBySegment);
    exactRegionCount = regionCount;
    if isfield(coverage, "ExactRegionCount")
        exactRegionCount = coverage.ExactRegionCount;
    end
    certificate = struct("Kind", "staticDegreeOne", ...
        "Passed", coverage.Passed && verifiedCount == allPairCount, ...
        "ExactRegionCount", exactRegionCount, ...
        "SolverRegionCount", regionCount, "Regions_units", {regions_units}, ...
        "Planes", planes, "RegionActiveBySegment", regionActiveBySegment, ...
        "RequiredGap_units", target_units + reserve_units, ...
        "RoundoffReserve_units", reserve_units, ...
        "MinimumSignedGap_units", minimumGap_units, ...
        "CoveragePassed", coverage.Passed, "Coverage", coverage, ...
        "AllPairCount", allPairCount, "VerifiedPairCount", verifiedCount, ...
        "ReusedPairCount", 0, "AnalyticPairCount", analyticCount, ...
        "ConicPairCount", conicCount, "ConicSolver", conicSolver);
end

function plane = createEmptyPlane()
    % Initialize an inactive separating-plane record.
    plane = struct();
    plane.Active        = false;
    plane.Verified      = false;
    plane.ExitFlag      = NaN;
    plane.Normal        = zeros(2, 2);
    plane.Offset_units    = zeros(1, 2);
    plane.SignedGap_units = NaN;
end
