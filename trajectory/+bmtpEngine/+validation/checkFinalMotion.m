function [proof, cache] = checkFinalMotion(request, preparedMotion, ...
        roundoffReserve_units, obstacleTarget_units, cache, stopOnFirstUnverified)
%% Section 0: Header & Readme
% SYNTAX
%   proof = bmtpEngine.validation.checkFinalMotion(request, preparedMotion, ...
%       roundoffReserve_units, obstacleTarget_units)
%   [proof, cache] = bmtpEngine.validation.checkFinalMotion(request, ...
%       preparedMotion, roundoffReserve_units, obstacleTarget_units, cache)
%   [proof, cache] = bmtpEngine.validation.checkFinalMotion(request, ...
%       preparedMotion, roundoffReserve_units, obstacleTarget_units, cache, ...
%       stopOnFirstUnverified)
%**************************************************************************
% PURPOSE
%   - Prove every applicable curve-span and convex-obstacle pair.
%**************************************************************************
% INPUTS
%   - request (scalar struct)
%       Checked motion request.
%   - preparedMotion (scalar struct)
%       Complete prepared curve.
%   - roundoffReserve_units (finite numeric scalar)
%       Numerical separation reserve.
%   - obstacleTarget_units (finite numeric scalar)
%       Required obstacle-side target.
%   - cache (scalar struct, optional; default [])
%       Matching checks from an earlier refinement in the same solve.
%   - stopOnFirstUnverified (logical scalar, optional; default false)
%       Stop after the first failed pair when rejecting a proposal.
%**************************************************************************
% OUTPUTS
%   - proof (scalar struct)
%       Pair proofs for the prepared motion; Passed is false when any
%       required clearance, workspace, dynamics, or continuity check fails.
%   - cache (scalar struct)
%       Reusable checks for a later refinement.
%**************************************************************************
% UNITS
%   - Position, gaps, targets, and reserves are coordinate units.
%**************************************************************************

%% Section 1: Check All Curve And Obstacle Pairs
if nargin < 5
    cache = [];
end
if nargin < 6
    stopOnFirstUnverified = false;
end

% Each optimized segment becomes two output spans. Repeat the static
% all-region mask for both spans.
regionActiveBySegment = true(size(preparedMotion.ProvenControlPoint_units, 1), ...
    numel(request.Regions_units));
spanBreaks_s      = request.InitialState.time_s + [0; cumsum(preparedMotion.SegmentTime_s)];
spanBreaks_s(end) = preparedMotion.FinalTime_s;
if isfield(request.Coverage, 'ActiveTimeInterval_s')
    intervals_s = request.Coverage.ActiveTimeInterval_s;
    starts_s    = spanBreaks_s(1:end - 1);
    ends_s      = spanBreaks_s(2:end);
    regionActiveBySegment = starts_s < intervals_s(:, 2).' & ends_s > intervals_s(:, 1).';
end
separatingLineGeometry = cell(numel(request.Regions_units), 1);
if isfield(request, 'SeparatingLineGeometry')
    separatingLineGeometry = request.SeparatingLineGeometry;
end
proof = checkAllCurveObstaclePairs(preparedMotion.ProvenControlPoint_units, ...
    request.Regions_units, request.Coverage, separatingLineGeometry, ...
    regionActiveBySegment, roundoffReserve_units, obstacleTarget_units, ...
    spanBreaks_s, cache, stopOnFirstUnverified);
if isfield(preparedMotion, 'ControlPoint_units')
    % Fixed physical boundary derivatives prevent post-solve dilation. Reject
    % an over-limit analytic proposal here so the shared optimizer can run.
    polynomial = bmtpEngine.motion.createPowerPolynomial(preparedMotion.ControlPoint_units, ...
        preparedMotion.SegmentTime_s, request.InitialState.time_s, preparedMotion.GivenPower_units, ...
        preparedMotion.FinalTime_s);
    derivativePowers = {polynomial.velocityPower_units_s, polynomial.accelerationPower_units_s2, ...
        polynomial.jerkPower_units_s3};
    derivativeBounds = [request.Limits.maxVelocity_units_s; request.Limits.maxAcceleration_units_s2; ...
        request.Limits.maxJerk_units_s3];
    workspaceBounds_units = [request.Limits.xInterval_units; ...
        request.Limits.yInterval_units];
    proof.WorkspacePassed = true;
    proof.DynamicsPassed  = true;
    for axisIndex = 1:2
        for segmentIndex = 1:polynomial.SegmentCount
            coefficients = reshape(polynomial.positionPower_units(segmentIndex, axisIndex, :), [], 1);
            proof.WorkspacePassed = proof.WorkspacePassed && ...
                bmtpEngine.validation.provePolynomialRange( ...
                coefficients, workspaceBounds_units(axisIndex, 1), workspaceBounds_units(axisIndex, 2), ...
                request.Options.ConstraintTolerance);
        end
    end
    for derivativeOrder = 1:3
        for axisIndex = 1:2
            for segmentIndex = 1:polynomial.SegmentCount
                coefficients = reshape(derivativePowers{derivativeOrder}(segmentIndex, axisIndex, :), [], 1);
                proof.DynamicsPassed = proof.DynamicsPassed && ...
                    bmtpEngine.validation.provePolynomialRange(coefficients, ...
                    -derivativeBounds(derivativeOrder, axisIndex), ...
                    derivativeBounds(derivativeOrder, axisIndex), ...
                    request.Options.ConstraintTolerance);
            end
        end
    end
    proof.ContinuityPassed = true;
    motionPowers = [{polynomial.positionPower_units}, derivativePowers];
    for derivativeOrder = 1:4
        values = motionPowers{derivativeOrder};
        residual = sum(values(1:end - 1, :, :), 3) - values(2:end, :, 1);
        proof.ContinuityPassed = proof.ContinuityPassed && ...
            all(abs(residual) <= request.Options.ConstraintTolerance, 'all');
    end
    proof.Passed = proof.Passed && proof.WorkspacePassed && ...
        proof.DynamicsPassed && proof.ContinuityPassed;
end
cache = struct( ...
    'Controls',     preparedMotion.ProvenControlPoint_units, ...
    'Breaks',       spanBreaks_s, ...
    'Target_units', obstacleTarget_units, ...
    'Proof',  proof);
end

%% Section 2: Local Functions
function proof = checkAllCurveObstaclePairs(controlPoint_units, regions_units, ...
        coverage, separatingLineGeometry, regionActiveBySegment, roundoffReserve_units, ...
        target_units, spanBreaks_s, cache, stopOnFirstUnverified)
    % Verify every applicable output-span and convex-exclusion-region pair.
    segmentCount        = size(controlPoint_units, 1);
    regionCount         = numel(regions_units);
    planes              = repmat(bmtpEngine.separation.createEmptyPlane(), segmentCount, regionCount);
    previousPlanes      = repmat(bmtpEngine.separation.createEmptyPlane(), 1, regionCount);
    verifiedCount       = 0;
    reusedCount         = 0;
    cachedCount         = 0;
    cachedGeometryCount = 0;
    minimumGap_units    = Inf;
    staticGeometry      = ~isfield(coverage, 'ActiveTimeInterval_s');

    % A subdivision leaves most spans unchanged. Their existing proofs
    % remain valid only for identical curve controls and supplied inputs.
    % Static collision geometry is independent of the span's clock; moving
    % geometry additionally requires the identical absolute time interval.
    cachedSpanIndexBySegment = zeros(segmentCount, 1);
    cacheMatchesRequest = ~isempty(cache) && ...
        isequaln(cache.Proof.Regions_units, regions_units) && ...
        isequaln(cache.Proof.Coverage, coverage) && ...
        cache.Proof.RoundoffReserve_units == roundoffReserve_units && ...
        cache.Target_units == target_units && ...
        size(cache.Controls, 2) == size(controlPoint_units, 2);
    if cacheMatchesRequest
        oldKeys = [cache.Breaks(1:end - 1), cache.Breaks(2:end), ...
            reshape(cache.Controls, size(cache.Controls, 1), [])];
        newKeys = [spanBreaks_s(1:end - 1), spanBreaks_s(2:end), ...
            reshape(controlPoint_units, segmentCount, [])];
        if staticGeometry
            oldKeys = oldKeys(:, 3:end);
            newKeys = newKeys(:, 3:end);
        end
        [~, cachedSpanIndexBySegment] = ismember(newKeys, oldKeys, 'rows');
    end
    if staticGeometry && regionCount > 0
        staticVertices_units = vertcat(regions_units{:});
        staticOwnerIndexByVertex = repelem( ...
            (1:regionCount).', cellfun(@(vertices) size(vertices, 1), regions_units));
        staticOwnerIndexByVertex = staticOwnerIndexByVertex(:);
    end

    pairWasRejected = false;
    for segmentIndex = 1:segmentCount
        trajectory_units = squeeze(controlPoint_units(segmentIndex, :, :));
        lastTimeFraction = [NaN, NaN];
        lastRestricted_units = zeros(0, 2);
        cachedSpanIndex   = cachedSpanIndexBySegment(segmentIndex);
        cachedSpanMatches = cachedSpanIndex > 0 && isequal( ...
            regionActiveBySegment(segmentIndex, :), ...
            cache.Proof.RegionActiveBySegment(cachedSpanIndex, :));
        if cachedSpanMatches
            activeRegions = regionActiveBySegment(segmentIndex, :);
            oldPlanes     = cache.Proof.Planes(cachedSpanIndex, :);
            if all([oldPlanes(activeRegions).Verified])
                planes(segmentIndex, :) = oldPlanes;
                previousPlanes          = oldPlanes;
                reusedCount             = reusedCount + nnz(activeRegions);
                cachedCount             = cachedCount + nnz(activeRegions);
                verifiedCount           = verifiedCount + nnz(activeRegions);
                if any(activeRegions)
                    minimumGap_units = min(minimumGap_units, ...
                        min([oldPlanes(activeRegions).SignedGap_units]));
                end
                continue;
            end
        end

        if staticGeometry && regionCount > 0 && segmentIndex > 1
            normalPages    = reshape([previousPlanes.Normal], 2, 2, regionCount);
            normals        = reshape(normalPages(1, :, :), 2, regionCount).';
            supports_units = accumarray(staticOwnerIndexByVertex, ...
                sum(staticVertices_units .* normals(staticOwnerIndexByVertex, :), 2), ...
                [regionCount, 1], @min);
            offsets                       = num2cell(repmat(target_units - supports_units, 1, 2), 2);
            [previousPlanes.Offset_units] = offsets{:};
            previousPlanes = bmtpEngine.separation.verifyStaticSeparatingLines( ...
                previousPlanes, trajectory_units, regions_units, roundoffReserve_units, target_units);
            verified               = [previousPlanes.Verified];
            planes(segmentIndex, :) = previousPlanes;
            reusedCount   = reusedCount + nnz(verified);
            verifiedCount = verifiedCount + nnz(verified);
            if any(verified)
                minimumGap_units = min(minimumGap_units, min([previousPlanes(verified).SignedGap_units]));
            end
        end

        for regionIndex = 1:regionCount
            if ~regionActiveBySegment(segmentIndex, regionIndex)
                continue;
            end
            if staticGeometry && segmentIndex > 1 && previousPlanes(regionIndex).Verified
                continue;
            end

            restricted_units = trajectory_units;
            interval_s       = spanBreaks_s(segmentIndex:segmentIndex + 1).';
            timeFraction     = [0, 1];
            if isfield(coverage, 'ActiveTimeInterval_s')
                activeInterval_s = (coverage.ActiveTimeInterval_s(regionIndex, :) - ...
                    spanBreaks_s(segmentIndex)) / ...
                    diff(spanBreaks_s(segmentIndex:segmentIndex + 1));
                timeFraction     = max(0, min(1, activeInterval_s));
                if isequal(timeFraction, lastTimeFraction)
                    restricted_units = lastRestricted_units;
                else
                    restricted_units       = bmtpEngine.motion.restrictBezier(trajectory_units, timeFraction);
                    lastTimeFraction       = timeFraction;
                    lastRestricted_units   = restricted_units;
                end
                interval_s       = [max(interval_s(1), coverage.ActiveTimeInterval_s(regionIndex, 1)), ...
                    min(interval_s(2), coverage.ActiveTimeInterval_s(regionIndex, 2))];
            end
            vertices_units = bmtpEngine.separation.regionOnInterval( ...
                regions_units{regionIndex}, coverage, regionIndex, interval_s);

            % A neighboring span's direction is only a proposal. Recompute
            % supports on this physical interval and verify the entire pair.
            plane = previousPlanes(regionIndex);
            if plane.Active && ~staticGeometry
                normal = plane.Normal(1, :);
                plane.Offset_units = target_units - [min(vertices_units(:, :, 1) * normal.'), ...
                    min(vertices_units(:, :, end) * normal.')];
                plane = bmtpEngine.separation.verifySeparatingLine( ...
                    plane, restricted_units, vertices_units, roundoffReserve_units, target_units);
            end
            if plane.Verified
                reusedCount = reusedCount + 1;
            else
                geometry = [];
                intervalMatchesFullActivity = staticGeometry || ...
                    isequal(interval_s, coverage.ActiveTimeInterval_s(regionIndex, :));
                if intervalMatchesFullActivity && ~isempty(separatingLineGeometry{regionIndex})
                    geometry            = separatingLineGeometry{regionIndex};
                    cachedGeometryCount = cachedGeometryCount + 1;
                end
                plane = bmtpEngine.separation.solveSeparatingLine( ...
                    restricted_units, vertices_units, target_units, roundoffReserve_units, geometry);
            end

            plane.TimeFraction = timeFraction;
            previousPlanes(regionIndex)       = plane;
            planes(segmentIndex, regionIndex) = plane;
            if plane.Verified
                verifiedCount    = verifiedCount + 1;
                minimumGap_units = min(minimumGap_units, plane.SignedGap_units);
            elseif stopOnFirstUnverified
                pairWasRejected = true;
                break;
            end
        end
        if pairWasRejected
            break;
        end
    end

    allPairCount     = nnz(regionActiveBySegment);
    exactRegionCount = regionCount;
    if isfield(coverage, "ExactRegionCount")
        exactRegionCount = coverage.ExactRegionCount;
    end
    regionCountConsistent = exactRegionCount == regionCount;
    timedMetadataComplete = ~isfield(coverage, "ActiveTimeInterval_s") || ...
        regionCount == 0 || (isfield(coverage, "EndRegions_units") && ...
        numel(coverage.EndRegions_units) == regionCount);
    coverageMetadataConsistent = regionCountConsistent && timedMetadataComplete;
    proof = struct( ...
        "Passed",                  verifiedCount == allPairCount && coverageMetadataConsistent, ...
        "CoverageMetadataConsistent", coverageMetadataConsistent, ...
        "RegionCountConsistent",   regionCountConsistent, ...
        "TimedMetadataComplete",   timedMetadataComplete, ...
        "ExactRegionCount",        exactRegionCount, ...
        "SolverRegionCount",       regionCount, ...
        "Regions_units",           {regions_units}, ...
        "Planes",                  planes, ...
        "RegionActiveBySegment",   regionActiveBySegment, ...
        "RequiredGap_units",       target_units + roundoffReserve_units, ...
        "RoundoffReserve_units",   roundoffReserve_units, ...
        "MinimumSignedGap_units",  minimumGap_units, ...
        "Coverage",                coverage, ...
        "AllPairCount",            allPairCount, ...
        "VerifiedPairCount",       verifiedCount, ...
        "ReusedPairCount",         reusedCount, ...
        "CachedPairCount",         cachedCount, ...
        "CachedGeometryPairCount", cachedGeometryCount);
end
