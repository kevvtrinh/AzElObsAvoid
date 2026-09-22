function [motionCheck, savedPairChecks] = checkFinalMotion(solverRequest, preparedMotion, ...
    roundoffReserve_units, separationTarget_units, savedPairChecks, stopOnFirstUnverified)
%% Section 0: Header & Readme
% SYNTAX
%   motionCheck = bmtpEngine.validation.checkFinalMotion(solverRequest, ...
%       preparedMotion, roundoffReserve_units, separationTarget_units)
%   [motionCheck, savedPairChecks] = bmtpEngine.validation.checkFinalMotion( ...
%       solverRequest, preparedMotion, roundoffReserve_units, ...
%       separationTarget_units, savedPairChecks, stopOnFirstUnverified)
%**************************************************************************
% PURPOSE
%   - Check separation for every curve segment and obstacle region whose
%     time intervals overlap. When full motion controls are present, also
%     check workspace limits, motion rates and continuity between segments.
%**************************************************************************
% INPUTS
%   - solverRequest (scalar struct)
%       Validated states, motion limits, options and prepared obstacle regions.
%   - preparedMotion (scalar struct)
%       Prepared curve controls, segment durations and final time.
%   - roundoffReserve_units (finite numeric scalar)
%       Numerical separation reserve.
%   - separationTarget_units (finite numeric scalar)
%       Required obstacle-side target.
%   - savedPairChecks (scalar struct or empty, optional; default [])
%       Earlier checked curves and their inputs, used only when they match.
%   - stopOnFirstUnverified (logical scalar, optional; default false)
%       Stop after the first failed pair when rejecting a proposal.
%**************************************************************************
% OUTPUTS
%   - motionCheck (scalar struct)
%       Separation results and checked lines. Passed is false when a required
%       clearance, workspace, motion-rate or continuity check fails.
%   - savedPairChecks (scalar struct)
%       Current curve checks and their inputs, ready for a later refinement.
%**************************************************************************
% UNITS
%   - Position, gaps, targets, and reserves are coordinate units.
%**************************************************************************

%% Section 1: Check Every Curve And Obstacle Pair That Overlaps In Time

if nargin < 5
    savedPairChecks = [];
end
if nargin < 6
    stopOnFirstUnverified = false;
end

% A static obstacle applies to every prepared segment. For moving regions,
% check only positive-duration overlap with each region's active interval.
% Use the prepared segment times, which may differ after curve subdivision.
regionActiveBySegment = true(size(preparedMotion.ProvenControlPoint_units, 1), ...
    numel(solverRequest.Regions_units));
segmentBoundaryTime_s = solverRequest.InitialState.time_s + [0; cumsum(preparedMotion.SegmentTime_s)];
segmentBoundaryTime_s(end) = preparedMotion.FinalTime_s;
if isfield(solverRequest.Coverage, 'ActiveTimeInterval_s')
    regionActiveIntervals_s = solverRequest.Coverage.ActiveTimeInterval_s;
    segmentStartTime_s      = segmentBoundaryTime_s(1:end - 1);
    segmentEndTime_s        = segmentBoundaryTime_s(2:end);
    regionActiveBySegment = segmentStartTime_s < regionActiveIntervals_s(:, 2).' & ...
        segmentEndTime_s > regionActiveIntervals_s(:, 1).';
end
separatingLineGeometry = cell(numel(solverRequest.Regions_units), 1);
if isfield(solverRequest, 'SeparatingLineGeometry')
    separatingLineGeometry = solverRequest.SeparatingLineGeometry;
end
motionCheck = checkAllCurveObstaclePairs(preparedMotion.ProvenControlPoint_units, ...
    solverRequest.Regions_units, solverRequest.Coverage, separatingLineGeometry, ...
    regionActiveBySegment, roundoffReserve_units, separationTarget_units, ...
    segmentBoundaryTime_s, savedPairChecks, stopOnFirstUnverified);

%% Section 2: Check Workspace Limits, Motion Rates And Segment Joins

if isfield(preparedMotion, 'ControlPoint_units')
    % Use the actual returned curve and durations for these checks. Simply
    % slowing it down would change supplied endpoint velocity/acceleration.
    % An over-limit direct proposal must fail so the optimizer can try.
    polynomial = bmtpEngine.motion.createPowerPolynomial(preparedMotion.ControlPoint_units, ...
        preparedMotion.SegmentTime_s, solverRequest.InitialState.time_s, preparedMotion.GivenPower_units, ...
        preparedMotion.FinalTime_s);
    motionRateCoefficients = {polynomial.velocityPower_units_s, polynomial.accelerationPower_units_s2, ...
        polynomial.jerkPower_units_s3};
    motionRateLimits = [solverRequest.Limits.maxVelocity_units_s; solverRequest.Limits.maxAcceleration_units_s2; ...
        solverRequest.Limits.maxJerk_units_s3];
    workspaceBounds_units = [solverRequest.Limits.xInterval_units; ...
        solverRequest.Limits.yInterval_units];

    % Each polynomial range check covers the full segment, including values
    % between output samples. Check x/y position first, then velocity,
    % acceleration and jerk against their separate limits.
    motionCheck.WorkspacePassed = true;
    motionCheck.DynamicsPassed  = true;
    for axisIndex = 1:2
        for segmentIndex = 1:polynomial.SegmentCount
            segmentCoefficients = reshape(polynomial.positionPower_units(segmentIndex, axisIndex, :), [], 1);
            motionCheck.WorkspacePassed = motionCheck.WorkspacePassed && ...
                bmtpEngine.validation.provePolynomialRange( ...
                segmentCoefficients, workspaceBounds_units(axisIndex, 1), workspaceBounds_units(axisIndex, 2), ...
                solverRequest.Options.ConstraintTolerance);
        end
    end
    for stateArrayIndex = 1:3
        for axisIndex = 1:2
            for segmentIndex = 1:polynomial.SegmentCount
                segmentCoefficients = reshape(motionRateCoefficients{stateArrayIndex}(segmentIndex, axisIndex, :), [], 1);
                motionCheck.DynamicsPassed = motionCheck.DynamicsPassed && ...
                    bmtpEngine.validation.provePolynomialRange(segmentCoefficients, ...
                    -motionRateLimits(stateArrayIndex, axisIndex), ...
                    motionRateLimits(stateArrayIndex, axisIndex), ...
                    solverRequest.Options.ConstraintTolerance);
            end
        end
    end

    % Adjacent segments must join with matching position, velocity,
    % acceleration and jerk. At fraction 1, sum the power coefficients;
    % at fraction 0, use the first coefficient. Compare those two values.
    motionCheck.ContinuityPassed = true;
    motionStateCoefficients = [{polynomial.positionPower_units}, motionRateCoefficients];
    for stateArrayIndex = 1:4
        stateCoefficients = motionStateCoefficients{stateArrayIndex};
        joinDifference    = sum(stateCoefficients(1:end - 1, :, :), 3) - stateCoefficients(2:end, :, 1);
        motionCheck.ContinuityPassed = motionCheck.ContinuityPassed && ...
            all(abs(joinDifference) <= solverRequest.Options.ConstraintTolerance, 'all');
    end
    motionCheck.Passed = motionCheck.Passed && motionCheck.WorkspacePassed && ...
        motionCheck.DynamicsPassed && motionCheck.ContinuityPassed;
end

% Save the checked controls and absolute times with the result so the next
% refinement can identify exactly which separation checks remain reusable.
savedPairChecks = struct( ...
    'Controls',     preparedMotion.ProvenControlPoint_units, ...
    'Breaks',       segmentBoundaryTime_s, ...
    'Target_units', separationTarget_units, ...
    'Proof',        motionCheck);
end

%% Section 3: Local Functions
function motionCheck = checkAllCurveObstaclePairs(controlPoint_units, regions_units, ...
        regionCoverage, separatingLineGeometry, regionActiveBySegment, roundoffReserve_units, ...
        separationTarget_units, segmentBoundaryTime_s, savedPairChecks, stopOnFirstUnverified)
    % Check each applicable segment/region pair and retain its verified line.
    % Saved results require identical inputs. A neighboring line direction
    % is only a proposal and must be checked again for the current curve.
    segmentCount            = size(controlPoint_units, 1);
    regionCount             = numel(regions_units);
    separatingPlanes        = repmat(bmtpEngine.separation.createEmptyPlane(), segmentCount, regionCount);
    previousSegmentPlanes   = repmat(bmtpEngine.separation.createEmptyPlane(), 1, regionCount);
    verifiedPairCount       = 0;
    reusedPairCount         = 0;
    cachedPairCount         = 0;
    cachedGeometryPairCount = 0;
    minimumGap_units        = Inf;
    obstaclesAreStatic      = ~isfield(regionCoverage, 'ActiveTimeInterval_s');

    % Splitting one segment can leave other segments unchanged. Reuse their
    % checked lines only when controls, regions and clearance settings match.
    % Moving regions also require the same absolute start/end times; static
    % geometry does not change when the same curve is run at another time.
    savedSegmentIndexByCurrentSegment = zeros(segmentCount, 1);
    savedChecksMatchRequest = ~isempty(savedPairChecks) && ...
        isequaln(savedPairChecks.Proof.Regions_units, regions_units) && ...
        isequaln(savedPairChecks.Proof.Coverage, regionCoverage) && ...
        savedPairChecks.Proof.RoundoffReserve_units == roundoffReserve_units && ...
        savedPairChecks.Target_units == separationTarget_units && ...
        size(savedPairChecks.Controls, 2) == size(controlPoint_units, 2);
    if savedChecksMatchRequest
        savedSegmentKeys = [savedPairChecks.Breaks(1:end - 1), savedPairChecks.Breaks(2:end), ...
            reshape(savedPairChecks.Controls, size(savedPairChecks.Controls, 1), [])];
        currentSegmentKeys = [segmentBoundaryTime_s(1:end - 1), segmentBoundaryTime_s(2:end), ...
            reshape(controlPoint_units, segmentCount, [])];
        if obstaclesAreStatic
            savedSegmentKeys   = savedSegmentKeys(:, 3:end);
            currentSegmentKeys = currentSegmentKeys(:, 3:end);
        end
        [~, savedSegmentIndexByCurrentSegment] = ismember(currentSegmentKeys, savedSegmentKeys, 'rows');
    end

    % Group static vertices once so later segments can recheck all previous
    % line directions together. Keep each vertex paired with its own region.
    if obstaclesAreStatic && regionCount > 0
        staticVertices_units      = vertcat(regions_units{:});
        staticRegionIndexByVertex = repelem( ...
            (1:regionCount).', cellfun(@(vertices) size(vertices, 1), regions_units));
        staticRegionIndexByVertex = staticRegionIndexByVertex(:);
    end

    pairWasRejected = false;
    for segmentIndex = 1:segmentCount
        segmentControlPoint_units   = squeeze(controlPoint_units(segmentIndex, :, :));
        savedOverlapFractions       = [NaN, NaN];
        savedSubcurveControls_units = zeros(0, 2);
        savedSegmentIndex           = savedSegmentIndexByCurrentSegment(segmentIndex);
        savedSegmentMatches         = savedSegmentIndex > 0 && isequal( ...
            regionActiveBySegment(segmentIndex, :), ...
            savedPairChecks.Proof.RegionActiveBySegment(savedSegmentIndex, :));

        % Reuse a complete saved segment only if its active-region list also
        % matches and every applicable line was verified.
        if savedSegmentMatches
            regionIsActive = regionActiveBySegment(segmentIndex, :);
            savedPlanes    = savedPairChecks.Proof.Planes(savedSegmentIndex, :);
            if all([savedPlanes(regionIsActive).Verified])
                separatingPlanes(segmentIndex, :) = savedPlanes;
                previousSegmentPlanes = savedPlanes;
                reusedPairCount       = reusedPairCount + nnz(regionIsActive);
                cachedPairCount       = cachedPairCount + nnz(regionIsActive);
                verifiedPairCount     = verifiedPairCount + nnz(regionIsActive);
                if any(regionIsActive)
                    minimumGap_units = min(minimumGap_units, ...
                        min([savedPlanes(regionIsActive).SignedGap_units]));
                end
                continue;
            end
        end

        % For a new curve segment, try the previous segment's directions
        % against every static region. Reposition and verify those lines
        % against this curve before reusing any of them.
        if obstaclesAreStatic && regionCount > 0 && segmentIndex > 1
            normalByEndpointAndRegion = reshape([previousSegmentPlanes.Normal], 2, 2, regionCount);
            startNormals             = reshape(normalByEndpointAndRegion(1, :, :), 2, regionCount).';
            obstacleSideBounds_units = accumarray(staticRegionIndexByVertex, ...
                sum(staticVertices_units .* startNormals(staticRegionIndexByVertex, :), 2), ...
                [regionCount, 1], @min);
            offsetCells = num2cell(repmat(separationTarget_units - obstacleSideBounds_units, 1, 2), 2);
            [previousSegmentPlanes.Offset_units] = offsetCells{:};
            previousSegmentPlanes = bmtpEngine.separation.verifyStaticSeparatingLines( ...
                previousSegmentPlanes, segmentControlPoint_units, regions_units, ...
                roundoffReserve_units, separationTarget_units);
            separationIsVerified = [previousSegmentPlanes.Verified];
            separatingPlanes(segmentIndex, :) = previousSegmentPlanes;
            reusedPairCount   = reusedPairCount + nnz(separationIsVerified);
            verifiedPairCount = verifiedPairCount + nnz(separationIsVerified);
            if any(separationIsVerified)
                minimumGap_units = min(minimumGap_units, ...
                    min([previousSegmentPlanes(separationIsVerified).SignedGap_units]));
            end
        end

        for regionIndex = 1:regionCount
            if ~regionActiveBySegment(segmentIndex, regionIndex)
                continue;
            end
            if obstaclesAreStatic && segmentIndex > 1 && previousSegmentPlanes(regionIndex).Verified
                continue;
            end

            % Restrict both the curve and region to their common times.
            % For example, a segment at 2-6 s and region active at 4-8 s
            % need a check over 4-6 s: curve fractions [0.5 1].
            subcurveControlPoint_units = segmentControlPoint_units;
            overlapInterval_s          = segmentBoundaryTime_s(segmentIndex:segmentIndex + 1).';
            overlapFractions           = [0, 1];
            if isfield(regionCoverage, 'ActiveTimeInterval_s')
                regionActiveFractions = (regionCoverage.ActiveTimeInterval_s(regionIndex, :) - ...
                    segmentBoundaryTime_s(segmentIndex)) / ...
                    diff(segmentBoundaryTime_s(segmentIndex:segmentIndex + 1));
                overlapFractions = max(0, min(1, regionActiveFractions));
                % Different regions can share the same interval; reuse that
                % curve restriction without changing the geometry checked.
                if isequal(overlapFractions, savedOverlapFractions)
                    subcurveControlPoint_units = savedSubcurveControls_units;
                else
                    subcurveControlPoint_units = bmtpEngine.motion.restrictBezier( ...
                        segmentControlPoint_units, overlapFractions);
                    savedOverlapFractions       = overlapFractions;
                    savedSubcurveControls_units = subcurveControlPoint_units;
                end
                overlapInterval_s = [max(overlapInterval_s(1), regionCoverage.ActiveTimeInterval_s(regionIndex, 1)), ...
                    min(overlapInterval_s(2), regionCoverage.ActiveTimeInterval_s(regionIndex, 2))];
            end
            vertices_units = bmtpEngine.separation.regionOnInterval( ...
                regions_units{regionIndex}, regionCoverage, regionIndex, overlapInterval_s);

            % Reuse a neighboring segment's direction only after recalculating
            % the obstacle bounds and checking the whole current interval.
            plane = previousSegmentPlanes(regionIndex);
            if plane.Active && ~obstaclesAreStatic
                lineNormal = plane.Normal(1, :);
                plane.Offset_units = separationTarget_units - [min(vertices_units(:, :, 1) * lineNormal.'), ...
                    min(vertices_units(:, :, end) * lineNormal.')];
                plane = bmtpEngine.separation.verifySeparatingLine( ...
                    plane, subcurveControlPoint_units, vertices_units, roundoffReserve_units, separationTarget_units);
            end
            if plane.Verified
                reusedPairCount = reusedPairCount + 1;
            else
                % A saved obstacle-edge calculation applies only to the
                % same complete activity interval. Otherwise build it from
                % the vertices restricted to this interval.
                reusableObstacleGeometry    = [];
                intervalMatchesFullActivity = obstaclesAreStatic || ...
                    isequal(overlapInterval_s, regionCoverage.ActiveTimeInterval_s(regionIndex, :));
                if intervalMatchesFullActivity && ~isempty(separatingLineGeometry{regionIndex})
                    reusableObstacleGeometry = separatingLineGeometry{regionIndex};
                    cachedGeometryPairCount  = cachedGeometryPairCount + 1;
                end
                plane = bmtpEngine.separation.solveSeparatingLine( ...
                    subcurveControlPoint_units, vertices_units, separationTarget_units, ...
                    roundoffReserve_units, reusableObstacleGeometry);
            end

            plane.TimeFraction                         = overlapFractions;
            previousSegmentPlanes(regionIndex)         = plane;
            separatingPlanes(segmentIndex, regionIndex) = plane;
            if plane.Verified
                verifiedPairCount = verifiedPairCount + 1;
                minimumGap_units  = min(minimumGap_units, plane.SignedGap_units);
            elseif stopOnFirstUnverified
                % A failed pair is enough to reject the candidate. Remaining
                % pairs stay unverified, so the overall result cannot pass.
                pairWasRejected = true;
                break;
            end
        end
        if pairWasRejected
            break;
        end
    end

    % Passing requires every applicable pair plus complete region metadata.
    % Matching counts alone would not reveal a missing moving-region endpoint.
    requiredPairCount = nnz(regionActiveBySegment);
    exactRegionCount  = regionCount;
    if isfield(regionCoverage, "ExactRegionCount")
        exactRegionCount = regionCoverage.ExactRegionCount;
    end
    regionCountConsistent = exactRegionCount == regionCount;
    timedMetadataComplete = ~isfield(regionCoverage, "ActiveTimeInterval_s") || ...
        regionCount == 0 || (isfield(regionCoverage, "EndRegions_units") && ...
        numel(regionCoverage.EndRegions_units) == regionCount);
    coverageMetadataConsistent = regionCountConsistent && timedMetadataComplete;

    motionCheck = struct( ...
        "Passed",                     verifiedPairCount == requiredPairCount && coverageMetadataConsistent, ...
        "CoverageMetadataConsistent", coverageMetadataConsistent, ...
        "RegionCountConsistent",      regionCountConsistent, ...
        "TimedMetadataComplete",      timedMetadataComplete, ...
        "ExactRegionCount",           exactRegionCount, ...
        "SolverRegionCount",          regionCount, ...
        "Regions_units",              {regions_units}, ...
        "Planes",                     separatingPlanes, ...
        "RegionActiveBySegment",      regionActiveBySegment, ...
        "RequiredGap_units",          separationTarget_units + roundoffReserve_units, ...
        "RoundoffReserve_units",      roundoffReserve_units, ...
        "MinimumSignedGap_units",     minimumGap_units, ...
        "Coverage",                   regionCoverage, ...
        "AllPairCount",               requiredPairCount, ...
        "VerifiedPairCount",          verifiedPairCount, ...
        "ReusedPairCount",            reusedPairCount, ...
        "CachedPairCount",            cachedPairCount, ...
        "CachedGeometryPairCount",    cachedGeometryPairCount);
end
