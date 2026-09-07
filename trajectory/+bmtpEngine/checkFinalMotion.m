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

% Each optimized segment becomes two output spans.
% Repeat its timed-region mask for both spans.
regionActiveBySegment = repelem(warmStart.RegionActiveBySegment, 2, 1);
certificate           = checkAllCurveObstaclePairs(preparedMotion.CertifiedControlPoint_units, request.Regions_units, request.Coverage, regionActiveBySegment, roundoffReserve_units, obstacleTarget_units, request.TightPlaneOptions);
end

%% Section 2: Local Functions

function certificate = checkAllCurveObstaclePairs(controlPoint_units, regions_units, coverage, regionActiveBySegment, reserve_units, target_units, solverOptions)
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
            plane = checkHullSeparationLine(trajectory_units, regions_units{regionIndex}, reserve_units, target_units);
            if plane.Verified
                analyticCount = analyticCount + 1;
            else
                [plane, ~, output] = bmtpEngine.solveSeparatingLine(trajectory_units, regions_units{regionIndex}, target_units, reserve_units, solverOptions);
                conicCount  = conicCount + 1;
                conicSolver = bmtpEngine.accumulateConicDiagnostics(conicSolver, output);
            end
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
    certificateKind = "staticDegreeOne";
    if isfield(coverage, "RegionActiveTauInterval")
        certificateKind = "timeCellDegreeOne";
    end
    certificate = struct("Kind", certificateKind, ...
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

function plane = checkHullSeparationLine(controlPoint_units, vertices_units, reserve_units, target_units)
    % Prove disjoint convex hulls by separating axes; leave overlap to SOCP.
    plane        = createEmptyPlane();
    edge_units     = vertices_units([2:end 1], :) - vertices_units;
    controlPairs = nchoosek(1:size(controlPoint_units, 1), 2);
    edge_units     = [edge_units; ...
        controlPoint_units(controlPairs(:, 2), :) - controlPoint_units(controlPairs(:, 1), :)];
    edgeLength_units = vecnorm(edge_units, 2, 2);
    edge_units       = edge_units(edgeLength_units > 0, :);
    edgeLength_units = edgeLength_units(edgeLength_units > 0);
    if isempty(edge_units)
        return;
    end
    normals  = [-edge_units(:, 2), edge_units(:, 1)] ./ edgeLength_units;
    normals  = [normals; -normals];
    gaps_units = min(vertices_units * normals.', [], 1) - max(controlPoint_units * normals.', [], 1);
    [maximumGap_units, normalIndex] = max(gaps_units);
    if maximumGap_units < target_units + reserve_units
        return;
    end
    normal = normals(normalIndex, :);
    [plane.Active, plane.Normal, plane.Offset_units] = deal(true, repmat(normal, 2, 1), zeros(1, 2));
    plane = bmtpEngine.verifySeparatingLine(plane, controlPoint_units, vertices_units, reserve_units, target_units);
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
