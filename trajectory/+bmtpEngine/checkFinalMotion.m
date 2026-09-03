function certificate = checkFinalMotion( ...
        request, warmStart, preparedMotion, roundoffReserve_deg, ...
        obstacleTarget_deg)
%% Section 0: Header & Readme
% SYNTAX
%   certificate = bmtpEngine.checkFinalMotion( ...
%       request, warmStart, preparedMotion, roundoffReserve_deg, ...
%       obstacleTarget_deg)
%**************************************************************************
% PURPOSE
%   - Check every applicable final curve span against each supplied convex
%     obstacle region using direct separating-plane certificates.
%**************************************************************************
% INPUTS
%   - request, warmStart, preparedMotion (scalar structs)
%       Checked request, region applicability, and final prepared curve.
%   - roundoffReserve_deg, obstacleTarget_deg (finite scalars)
%       Numerical reserve and required obstacle-side target in degrees.
%**************************************************************************
% OUTPUTS
%   - certificate (scalar struct)
%       Pair coverage, separating planes, counts, and passing state.
%**************************************************************************
% UNITS
%   - Position, gaps, and reserves are degrees.
%**************************************************************************

%% Section 1: Check All Curve And Obstacle Pairs

% Splitting each optimized segment creates two output spans, so repeat the
% original applicability mask exactly once to preserve timed-region coverage.
regionActiveBySegment = repelem( ...
    warmStart.RegionActiveBySegment, 2, 1);
certificate = checkAllCurveObstaclePairs( ...
    preparedMotion.CertifiedControlPoint_deg, request.Regions_deg, ...
    request.Coverage, regionActiveBySegment, roundoffReserve_deg, ...
    obstacleTarget_deg, request.TightPlaneOptions);
end

%% Section 2: Local Functions

function certificate = checkAllCurveObstaclePairs( ...
        controlPoint_deg, regions_deg, coverage, ...
        regionActiveBySegment, reserve_deg, target_deg, solverOptions)
% Verify every applicable output-span and convex-exclusion-region pair.
segmentCount = size(controlPoint_deg, 1);
regionCount = numel(regions_deg);
planes = repmat(createEmptyPlane(), segmentCount, regionCount);
verifiedCount = 0;
conicCount = 0;
analyticCount = 0;
minimumGap_deg = Inf;
for segmentIndex = 1:segmentCount
    trajectory_deg = squeeze(controlPoint_deg(segmentIndex, :, :));
    for regionIndex = 1:regionCount
        if ~regionActiveBySegment(segmentIndex, regionIndex)
            continue;
        end
        plane = checkHullSeparationLine(trajectory_deg, ...
            regions_deg{regionIndex}, reserve_deg, target_deg);
        if plane.Verified
            analyticCount = analyticCount + 1;
        else
            [plane, ~] = bmtpEngine.solveSeparatingLine( ...
                trajectory_deg, regions_deg{regionIndex}, target_deg, ...
                reserve_deg, solverOptions);
            conicCount = conicCount + 1;
        end
        planes(segmentIndex, regionIndex) = plane;
        if plane.Verified
            verifiedCount = verifiedCount + 1;
            minimumGap_deg = min( ...
                minimumGap_deg, plane.SignedGap_deg);
        end
    end
end
allPairCount = nnz(regionActiveBySegment);
exactRegionCount = regionCount;
if isfield(coverage, "ExactRegionCount")
    exactRegionCount = coverage.ExactRegionCount;
end
certificateKind = "staticDegreeOne";
if isfield(coverage, "RegionActiveTauInterval")
    certificateKind = "timeCellDegreeOne";
end
certificate = struct("Kind", certificateKind, ...
    "Passed", coverage.Passed && verifiedCount == allPairCount, ...
    "ExactRegionCount", exactRegionCount, ...
    "SolverRegionCount", regionCount, "Regions_deg", {regions_deg}, ...
    "Planes", planes, "RegionActiveBySegment", regionActiveBySegment, ...
    "RequiredGap_deg", target_deg + reserve_deg, ...
    "RoundoffReserve_deg", reserve_deg, ...
    "MinimumSignedGap_deg", minimumGap_deg, ...
    "CoveragePassed", coverage.Passed, "Coverage", coverage, ...
    "AllPairCount", allPairCount, "VerifiedPairCount", verifiedCount, ...
    "ReusedPairCount", 0, "AnalyticPairCount", analyticCount, ...
    "ConicPairCount", conicCount);
end

function plane = checkHullSeparationLine( ...
        controlPoint_deg, vertices_deg, reserve_deg, target_deg)
% Prove disjoint convex hulls by separating axes; leave overlap to SOCP.
plane = createEmptyPlane();
edge_deg = vertices_deg([2:end 1], :) - vertices_deg;
controlPairs = nchoosek(1:size(controlPoint_deg, 1), 2);
edge_deg = [edge_deg; ...
    controlPoint_deg(controlPairs(:, 2), :) - ...
    controlPoint_deg(controlPairs(:, 1), :)];
edgeLength_deg = vecnorm(edge_deg, 2, 2);
edge_deg = edge_deg(edgeLength_deg > 0, :);
edgeLength_deg = edgeLength_deg(edgeLength_deg > 0);
if isempty(edge_deg)
    return;
end
normals = [-edge_deg(:, 2), edge_deg(:, 1)] ./ edgeLength_deg;
normals = [normals; -normals];
gaps_deg = min(vertices_deg * normals.', [], 1) - ...
    max(controlPoint_deg * normals.', [], 1);
[maximumGap_deg, normalIndex] = max(gaps_deg);
if maximumGap_deg < target_deg + reserve_deg
    return;
end
normal = normals(normalIndex, :);
[plane.Active, plane.Normal, plane.Offset_deg] = ...
    deal(true, repmat(normal, 2, 1), zeros(1, 2));
plane = bmtpEngine.verifySeparatingLine( ...
    plane, controlPoint_deg, vertices_deg, reserve_deg, target_deg);
end

function plane = createEmptyPlane()
% Define the stable inactive or verified degree-one plane record.
plane = struct("Active", false, "Verified", false, "ExitFlag", NaN, ...
    "Normal", zeros(2, 2), "Offset_deg", zeros(1, 2), ...
    "SignedGap_deg", NaN);
end
