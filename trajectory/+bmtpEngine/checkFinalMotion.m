function [certificate,cache] = checkFinalMotion(request, warmStart, preparedMotion, roundoffReserve_units, obstacleTarget_units,cache)
%% Section 0: Header & Readme
% SYNTAX: certificate = bmtpEngine.checkFinalMotion( request, warmStart, preparedMotion,
%   roundoffReserve_units, obstacleTarget_units)
% PURPOSE: Check every applicable final curve span against each supplied convex obstacle region
%   using direct separating-plane certificates.
% INPUTS: request, warmStart, preparedMotion (scalar structs) Checked request, region applicability,
%   and final prepared curve. roundoffReserve_units, obstacleTarget_units (finite scalars) Numerical
%   reserve and required obstacle-side target in coordinate units. cache (optional opaque struct
%   returned by this function) Previous refinement's checks within the same solve. Reuse requires
%   exactly matching source geometry, coverage, controls, and tolerances.
% OUTPUTS: certificate (scalar struct) Pair coverage, separating planes, counts, and passing state.
%   cache: current checks for a later refinement; never a public input.
% UNITS: Position, gaps, and reserves are coordinate units.

%% Section 1: Check All Curve And Obstacle Pairs
if nargin<6, cache=[]; end
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
separatingLineGeometry=cell(numel(request.Regions_units),1);
if isfield(request,'SeparatingLineGeometry')
    separatingLineGeometry=request.SeparatingLineGeometry;
end
certificate = checkAllCurveObstaclePairs(preparedMotion.CertifiedControlPoint_units, ...
    request.Regions_units,request.Coverage,separatingLineGeometry, ...
    regionActiveBySegment,roundoffReserve_units,obstacleTarget_units,spanBreaks_s,cache);
if isfield(preparedMotion,'ControlPoint_units')
    % Fixed physical boundary derivatives prevent post-solve dilation. Reject
    % an over-limit analytic proposal here so the shared optimizer can run.
    polynomial = bmtpEngine.createPowerPolynomial(preparedMotion.ControlPoint_units, ...
        preparedMotion.SegmentTime_s,request.InitialState.time_s,preparedMotion.PrescribedPower_units);
    arrays = {polynomial.velocityPower_units_s,polynomial.accelerationPower_units_s2,polynomial.jerkPower_units_s3};
    bounds = [request.Limits.maxVelocity_units_s;request.Limits.maxAcceleration_units_s2;request.Limits.maxJerk_units_s3];
    certificate.DynamicsPassed = true;
    for order = 1:3
        for axis = 1:2
            for segment = 1:polynomial.SegmentCount
                coefficients = reshape(arrays{order}(segment,axis,:),[],1);
                certificate.DynamicsPassed = certificate.DynamicsPassed && ...
                    obstacleAvoidance.validation.certifyPolynomialRange(coefficients,-bounds(order,axis),bounds(order,axis),request.Options.ConstraintTolerance);
            end
        end
    end
    certificate.ContinuityPassed = true;
    allArrays = [{polynomial.positionPower_units},arrays];
    for order = 1:4
        values = allArrays{order};
        residual = sum(values(1:end-1,:,:),3)-values(2:end,:,1);
        certificate.ContinuityPassed = certificate.ContinuityPassed && ...
            all(abs(residual)<=request.Options.ConstraintTolerance,'all');
    end
    certificate.Passed = certificate.Passed && certificate.DynamicsPassed && certificate.ContinuityPassed;
end
cache=struct('Controls',preparedMotion.CertifiedControlPoint_units,'Breaks',spanBreaks_s, ...
    'Target_units',obstacleTarget_units,'Certificate',certificate);
end

%% Section 2: Local Functions
function certificate = checkAllCurveObstaclePairs(controlPoint_units, regions_units, coverage, separatingLineGeometry, regionActiveBySegment, reserve_units, target_units,spanBreaks_s,cache)
    % Verify every applicable output-span and convex-exclusion-region pair.
    segmentCount   = size(controlPoint_units, 1);
    regionCount    = numel(regions_units);
    planes         = repmat(createEmptyPlane(), segmentCount, regionCount);
    previousPlanes = repmat(createEmptyPlane(),1,regionCount);
    verifiedCount  = 0;
    conicCount     = 0;
    analyticCount  = 0;
    reusedCount    = 0;
    cachedCount    = 0;
    cachedGeometryCount = 0;
    conicSolver    = bmtpEngine.accumulateConicDiagnostics();
    minimumGap_units = Inf;
    staticGeometry = ~isfield(coverage,'ActiveTimeInterval_s');
    % A subdivision leaves most spans unchanged. Their existing certificates
    % remain valid only for identical curve controls and authoritative inputs.
    % Static collision geometry is independent of the span's clock; moving
    % geometry additionally requires the identical absolute time interval.
    cachedSpan=zeros(segmentCount,1);
    if ~isempty(cache) && isequaln(cache.Certificate.Regions_units,regions_units) && ...
            isequaln(cache.Certificate.Coverage,coverage) && ...
            cache.Certificate.RoundoffReserve_units==reserve_units && ...
            cache.Target_units==target_units && ...
            size(cache.Controls,2)==size(controlPoint_units,2)
        oldKeys=[cache.Breaks(1:end-1),cache.Breaks(2:end),reshape(cache.Controls,size(cache.Controls,1),[])];
        newKeys=[spanBreaks_s(1:end-1),spanBreaks_s(2:end),reshape(controlPoint_units,segmentCount,[])];
        if staticGeometry, oldKeys=oldKeys(:,3:end); newKeys=newKeys(:,3:end); end
        [~,cachedSpan]=ismember(newKeys,oldKeys,'rows');
    end
    if staticGeometry && regionCount>0
        staticVertices_units = vertcat(regions_units{:});
        staticOwners = repelem((1:regionCount).',cellfun(@(v)size(v,1),regions_units));
        staticOwners = staticOwners(:);
    end
    for segmentIndex = 1:segmentCount
        trajectory_units = squeeze(controlPoint_units(segmentIndex, :, :));
        oldSpan=cachedSpan(segmentIndex);
        if oldSpan>0 && isequal(regionActiveBySegment(segmentIndex,:),cache.Certificate.RegionActiveBySegment(oldSpan,:))
            active=regionActiveBySegment(segmentIndex,:);
            oldPlanes=cache.Certificate.Planes(oldSpan,:);
            if all([oldPlanes(active).Verified])
                planes(segmentIndex,:)=oldPlanes;
                previousPlanes=oldPlanes;
                reusedCount=reusedCount+nnz(active);
                cachedCount=cachedCount+nnz(active);
                verifiedCount=verifiedCount+nnz(active);
                if any(active), minimumGap_units=min(minimumGap_units,min([oldPlanes(active).SignedGap_units])); end
                continue;
            end
        end
        if staticGeometry && regionCount>0 && segmentIndex>1
            normalPages = reshape([previousPlanes.Normal],2,2,regionCount);
            normals = reshape(normalPages(1,:,:),2,regionCount).';
            supports_units = accumarray(staticOwners,sum(staticVertices_units.*normals(staticOwners,:),2),[regionCount,1],@min);
            offsets = num2cell(repmat(target_units-supports_units,1,2),2);
            [previousPlanes.Offset_units] = offsets{:};
            previousPlanes = bmtpEngine.verifyStaticSeparatingLines(previousPlanes,trajectory_units,regions_units,reserve_units,target_units);
            verified = [previousPlanes.Verified];
            planes(segmentIndex,:) = previousPlanes;
            reusedCount = reusedCount+nnz(verified);
            verifiedCount = verifiedCount+nnz(verified);
            if any(verified), minimumGap_units = min(minimumGap_units,min([previousPlanes(verified).SignedGap_units])); end
        end
        for regionIndex = 1:regionCount
            if ~regionActiveBySegment(segmentIndex, regionIndex)
                continue;
            end
            if staticGeometry && segmentIndex>1 && previousPlanes(regionIndex).Verified, continue; end
            restricted_units = trajectory_units;
            interval_s = spanBreaks_s(segmentIndex:segmentIndex+1).';
            if isfield(coverage,'ActiveTimeInterval_s')
                interval = (coverage.ActiveTimeInterval_s(regionIndex,:)-spanBreaks_s(segmentIndex))/diff(spanBreaks_s(segmentIndex:segmentIndex+1));
                restricted_units = bmtpEngine.restrictBezier(trajectory_units,max(0,min(1,interval)));
                interval_s = [max(interval_s(1),coverage.ActiveTimeInterval_s(regionIndex,1)), ...
                    min(interval_s(2),coverage.ActiveTimeInterval_s(regionIndex,2))];
            end
            vertices_units = bmtpEngine.regionOnInterval(regions_units{regionIndex},coverage,regionIndex,interval_s);
            % A neighboring span's direction is only a proposal. Recompute
            % supports on this physical interval and verify the entire pair.
            plane = previousPlanes(regionIndex);
            if plane.Active && ~staticGeometry
                normal = plane.Normal(1,:);
                plane.Offset_units = target_units-[min(vertices_units(:,:,1)*normal.'),min(vertices_units(:,:,end)*normal.')];
                plane = bmtpEngine.verifySeparatingLine(plane,restricted_units,vertices_units,reserve_units,target_units);
            end
            if plane.Verified
                reusedCount = reusedCount+1;
            else
                geometry=[];
                if (staticGeometry || isequal(interval_s,coverage.ActiveTimeInterval_s(regionIndex,:))) && ...
                        ~isempty(separatingLineGeometry{regionIndex})
                    geometry=separatingLineGeometry{regionIndex};
                    cachedGeometryCount=cachedGeometryCount+1;
                end
                plane = bmtpEngine.solveSeparatingLine(restricted_units,vertices_units, ...
                    target_units,reserve_units,geometry);
                analyticCount = analyticCount + 1;
            end
            previousPlanes(regionIndex) = plane;
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
        "ReusedPairCount", reusedCount, "CachedPairCount", cachedCount, "AnalyticPairCount", analyticCount, ...
        "CachedGeometryPairCount",cachedGeometryCount, ...
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
