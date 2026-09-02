function [candidate, diagnostics] = solve( ...
        seed, regions_deg, coverage, initialState, goalState, limits, options)
%% Section 0: Header & Readme
% SYNTAX
%   [candidate, diagnostics] = ...
%       bmtpEngine.solve( ...
%       seed, regions_deg, coverage, initialState, goalState, limits, options)
%**************************************************************************
% PURPOSE
%   - Optimize one topology seed with a composite Bezier trajectory,
%     third-order time-power cones, and alternating degree-one maximum-margin
%     separating planes.
%   - Use the compact degree-7 representation only when planner-supplied
%     coverage records conservative grouping of a complex exact outline.
%   - Accept motion only after direct Bernstein checks, every applicable
%     span/region plane certificate, and independent public validation.
%**************************************************************************
% INPUTS
%   - seed (scalar struct)
%       position_deg is N-by-2; tau strictly increases from zero to one.
%   - regions_deg (R-by-1 cell array)
%       Each cell contains one finite convex N-by-2 exclusion polygon.
%   - coverage (scalar struct)
%       Requires Passed. Optional RegionActiveTauInterval is R-by-2 and
%       limits each region to an absolute normalized motion-time interval.
%   - initialState, goalState (normalized scalar state structs)
%       Positions are fixed and endpoint velocity and acceleration are zero.
%   - limits (normalized scalar struct)
%       Workspace, velocity, acceleration, and jerk bounds.
%   - options (resolved scalar planner-options struct)
%       Goal-time policy, sampling interval, work limits, and tolerances.
%**************************************************************************
% OUTPUTS
%   - candidate (scalar struct)
%       Stable motion record. Expected infeasibility returns Success=false.
%   - diagnostics (scalar struct)
%       Solver, timing, coverage, motion, and plane-certificate evidence.
%**************************************************************************
% UNITS
%   - Position is degrees and time is seconds. Derivatives use deg/s,
%     deg/s^2, and deg/s^3. Polynomial powers use local normalized time.
%**************************************************************************

%% Section 1: Validate And Create The Exclusion Representation

totalTimer = tic;
usesConservativeGrouping = isfield(coverage, "ConservativeGrouping") && ...
    isstruct(coverage.ConservativeGrouping) && ...
    isfield(coverage.ConservativeGrouping, "Applied") && ...
    isequal(coverage.ConservativeGrouping.Applied, true);
usesTimedCells = isfield(coverage, "RegionActiveTauInterval");
if usesConservativeGrouping || usesTimedCells
    [degree, splitCount] = deal(7, 1);
else
    [degree, splitCount] = deal(16, 3);
end
validateKernelInputs(seed, regions_deg, coverage, ...
    initialState, goalState, limits, options);
originalSeedSegmentCount = size(seed.position_deg, 1) - 1;
% Preserve the former default 2-by-10 cap without exposing conic dimension.
maximumWarmSegmentCount = 20;
if usesTimedCells
    route_deg = createTimedWarmRoute( ...
        seed, coverage.TimedSegmentCount, maximumWarmSegmentCount);
else
    route_deg = limitWarmRouteSegments( ...
        double(seed.position_deg), maximumWarmSegmentCount);
    route_deg = splitRoute(route_deg, splitCount);
end
route_deg([1 end], :) = [initialState.position_deg; goalState.position_deg];
segmentCount = size(route_deg, 1) - 1;
regionActiveBySegment = createRegionActiveMask( ...
    segmentCount, numel(regions_deg), coverage);
candidate = createEmptyCandidate(seed, initialState, options);
diagnostics = createEmptyDiagnostics( degree, splitCount, segmentCount, numel(regions_deg));
diagnostics.OriginalSeedSegmentCount = originalSeedSegmentCount;
diagnostics.WarmRouteResampled = originalSeedSegmentCount > maximumWarmSegmentCount;
diagnostics.Coverage = coverage;
diagnostics.ApplicablePairCount = nnz(regionActiveBySegment);
motionHorizon_s = goalState.time_s - initialState.time_s;
travelSavingsRate_deg_s = double(optionalField( ...
    options, "MinimumTravelSavingsRate_deg_s", 10));
if motionHorizon_s <= 0
    error("bmtpEngine:InvalidGoalTime", ...
        "goalState.time_s must be greater than initialState.time_s.");
end
optimizationHorizon_s = motionHorizon_s;
regionMinimum_deg = zeros(numel(regions_deg), 2);
regionMaximum_deg = zeros(numel(regions_deg), 2);
for regionIndex = 1:numel(regions_deg)
    regionMinimum_deg(regionIndex, :) = min(regions_deg{regionIndex}, [], 1);
    regionMaximum_deg(regionIndex, :) = max(regions_deg{regionIndex}, [], 1);
end
[~, ~, roundoffReserve_deg] = bmtpEngine.createCoordinateTolerances( ...
    route_deg, limits.azimuthInterval_deg, ...
    limits.elevationInterval_deg, regions_deg);
normalNormLimit = 1 + 2 ^ 20 * eps;
obstacleTarget_deg = normalNormLimit * ...
    options.CollisionClearanceTolerance_deg + roundoffReserve_deg;
maximumTrajectoryIterations = 300;
trajectoryOptions = optimoptions("coneprog", "Display", "none", ...
    "MaxIterations", maximumTrajectoryIterations);
planeOptions = optimoptions("coneprog", "Display", "none");
tightPlaneOptions = optimoptions("coneprog", "Display", "none", ...
    "ConstraintTolerance", 1e-11, "OptimalityTolerance", 1e-11, "MaxIterations", 400);

%% Section 2: Alternate Time-Power And Maximum-Margin SOCPs

feasibleControl_deg = createWarmControl(route_deg, degree);
feasibleSegmentTime_s = requiredSegmentTime(feasibleControl_deg, limits);
diagnostics.WarmStartDuration_s = segmentCount * feasibleSegmentTime_s;
bestControl_deg = zeros(0, degree + 1, 2);
[bestSegmentTime_s, bestDuration_s] = deal(NaN, Inf);
diagnostics.RetainedBestTrialDuration_s = bestDuration_s;
taggedPairs = false(segmentCount, numel(regions_deg));
planes = repmat(emptyPlane(), segmentCount, numel(regions_deg));
solverMessage = "The biconvex iteration limit was reached.";
for iterationIndex = 1:35
    diagnostics.IterationCount = iterationIndex;
    usedRequestHorizon = optimizationHorizon_s == motionHorizon_s;
    [trialControl_deg, trialTime_s, exitFlag, output] = solveTrajectorySocp( ...
        segmentCount, degree, initialState.position_deg, goalState.position_deg, ...
        limits, planes, roundoffReserve_deg, optimizationHorizon_s, ...
        "earliestArrival", 0, ...
        feasibleSegmentTime_s, trajectoryOptions);
    diagnostics.TrajectorySocpCount = diagnostics.TrajectorySocpCount + 1;
    diagnostics.FinalTrajectoryExitFlag = exitFlag;
    if exitFlag <= 0 || isempty(trialControl_deg)
        % A request horizon constrains the returned motion, not the
        % intermediate iterate needed to establish separating planes.
        if exitFlag == -2 && isempty(bestControl_deg) && ...
                optimizationHorizon_s < diagnostics.WarmStartDuration_s
            optimizationHorizon_s = min(2 * optimizationHorizon_s, diagnostics.WarmStartDuration_s);
            continue;
        end
        solverMessage = "Trajectory SOCP failed: " + string(output.message);
        break;
    end
    collisionPairs = findSampledCollisionPairs( ...
        trialControl_deg, regions_deg, regionMinimum_deg, ...
        regionMaximum_deg, regionActiveBySegment, 1201);
    diagnostics.TrialDuration_s(iterationIndex) = ...
        segmentCount * trialTime_s;
    diagnostics.CollisionPairCountHistory(iterationIndex) = ...
        nnz(collisionPairs);
    diagnostics.TrialWasCollisionFree(iterationIndex) = ...
        ~any(collisionPairs, "all");
    diagnostics.FinalCollisionPairCount = nnz(collisionPairs);
    previousTaggedPairs = taggedPairs;
    newPairs = collisionPairs & ~taggedPairs;
    taggedPairs = taggedPairs | newPairs;
    if ~any(collisionPairs, "all")
        optimizationHorizon_s = motionHorizon_s;
        previousDuration_s = segmentCount * feasibleSegmentTime_s;
        feasibleControl_deg = trialControl_deg;
        feasibleSegmentTime_s = trialTime_s;
        duration_s = segmentCount * trialTime_s;
        retainedBestImprovement_s = bestDuration_s - duration_s;
        if duration_s < bestDuration_s
            bestControl_deg = trialControl_deg;
            bestSegmentTime_s = trialTime_s;
            bestDuration_s = duration_s;
            diagnostics.BestDuration_s = duration_s;
            diagnostics.RetainedBestTrialDuration_s = duration_s;
        end
        improvement_s = previousDuration_s - duration_s;
        if improvement_s >= 0 && improvement_s <= options.ArrivalTimeTolerance_s
            diagnostics.Converged = true;
            solverMessage = "The feasible arrival improvement reached tolerance.";
            break;
        end
        taggedPairSetUnchanged = isequal(taggedPairs, previousTaggedPairs);
        % Plane reuse is an internal continuation invariant, not a request
        % choice. Share the arrival tolerance used by convergence ownership.
        reusePlanes = retainedBestImprovement_s <= ...
            options.ArrivalTimeTolerance_s && taggedPairSetUnchanged;
        if reusePlanes
            diagnostics.PlaneReuseApplied = true;
            diagnostics.PlaneReuseCount = diagnostics.PlaneReuseCount + 1;
            if usedRequestHorizon
                diagnostics.Converged = true;
                solverMessage = "The next trajectory SOCP would be unchanged.";
                break;
            end
            continue;
        end
        planes(:) = emptyPlane();
        activePairs = taggedPairs;
    elseif any(newPairs, "all")
        activePairs = newPairs;
    else
        solverMessage = "A tagged pair crossed its retained separating plane.";
        break;
    end
    updateFailed = false;
    activePairIndices = reshape(find(activePairs), 1, []);
    for activeIndex = 1:numel(activePairIndices)
        pairIndex = activePairIndices(activeIndex);
        [segmentIndex, regionIndex] = ind2sub(size(activePairs), pairIndex);
        [plane, planeExitFlag] = solveMaximumMarginPlane( ...
            squeeze(feasibleControl_deg(segmentIndex, :, :)), ...
            regions_deg{regionIndex}, obstacleTarget_deg, ...
            roundoffReserve_deg, planeOptions);
        diagnostics.PlaneSocpCount = diagnostics.PlaneSocpCount + 1;
        if planeExitFlag <= 0 || ~plane.Active
            [diagnostics.FailedPlaneSegmentIndex, diagnostics.FailedPlaneRegionIndex, ...
                diagnostics.FailedPlane] = deal(segmentIndex, regionIndex, plane);
            solverMessage = "A maximum-margin plane solve failed.";
            updateFailed = true;
            break;
        end
        if ~plane.Verified
            % A boundary-touching visibility seed may have no positive gap.
            % Its solved separator is only a linearization that must move the
            % next iterate; final acceptance still requires direct proof.
            diagnostics.UnverifiedPlaneInitializationCount = ...
                diagnostics.UnverifiedPlaneInitializationCount + 1;
        end
        planes(segmentIndex, regionIndex) = plane;
    end
    if updateFailed
        break;
    end
end
[diagnostics.TaggedPairCount, diagnostics.SolverMessage] = ...
    deal(nnz(taggedPairs), solverMessage);
if isempty(bestControl_deg)
    [candidate, diagnostics] = finishFailure(candidate, diagnostics, totalTimer, ...
        "No optimized collision-free iterate was found. " + solverMessage, ...
        "noOptimizedFeasibleIterate", false);
    return;
end

% Establish a feasible homotopy with the proven time-minimizing kernel before
% asking for less travel. A path-length solve from an unconstrained chord can
% initialize a separating plane on the wrong side of a concave obstacle.
if options.GoalTimeMode ~= "earliestArrival"
    baseControl_deg = bestControl_deg;
    baseSegmentTime_s = bestSegmentTime_s;
    baseLength_deg = controlPolygonLength(baseControl_deg);
    selectedControl_deg = baseControl_deg;
    selectedSegmentTime_s = baseSegmentTime_s;
    selectedPlanes = planes;
    selectedLength_deg = baseLength_deg;
    selectedCost_deg = baseLength_deg + travelSavingsRate_deg_s * ...
        segmentCount * baseSegmentTime_s;
    trialSavingsRates_deg_s = travelSavingsRate_deg_s;
    if options.GoalTimeMode == "balancedArrival"
        trialSavingsRates_deg_s = travelSavingsRate_deg_s * [0.1 1 10];
    end
    travelRefinementAccepted = false;
    for portfolioIndex = 1:numel(trialSavingsRates_deg_s)
        travelPlanes = planes;
        trialRate_deg_s = trialSavingsRates_deg_s(portfolioIndex);
        for refinementIndex = 1:8
            [refinedControl_deg, refinedSegmentTime_s, travelExitFlag] = ...
                solveTrajectorySocp( ...
                segmentCount, degree, initialState.position_deg, ...
                goalState.position_deg, limits, travelPlanes, ...
                roundoffReserve_deg, motionHorizon_s, ...
                options.GoalTimeMode, trialRate_deg_s, ...
                baseSegmentTime_s, trajectoryOptions);
            if travelExitFlag <= 0 || isempty(refinedControl_deg)
                break;
            end
            refinedCollisionPairs = findSampledCollisionPairs( ...
                refinedControl_deg, regions_deg, regionMinimum_deg, ...
                regionMaximum_deg, regionActiveBySegment, 1201);
            if any(refinedCollisionPairs, "all")
                activeTravelPairs = reshape( ...
                    [travelPlanes.Active], size(travelPlanes));
                newPairs = refinedCollisionPairs & ~activeTravelPairs;
                newPairIndices = reshape(find(newPairs), 1, []);
                if isempty(newPairIndices)
                    break;
                end
                planeUpdateFailed = false;
                for newPairIndex = newPairIndices
                    [segmentIndex, regionIndex] = ind2sub( ...
                        size(newPairs), newPairIndex);
                    [travelPlane, planeExitFlag] = solveMaximumMarginPlane( ...
                        squeeze(baseControl_deg(segmentIndex, :, :)), ...
                        regions_deg{regionIndex}, obstacleTarget_deg, ...
                        roundoffReserve_deg, planeOptions);
                    if planeExitFlag <= 0 || ~travelPlane.Active
                        planeUpdateFailed = true;
                        break;
                    end
                    travelPlanes(segmentIndex, regionIndex) = travelPlane;
                end
                if planeUpdateFailed
                    break;
                end
                continue;
            end
            refinedLength_deg = controlPolygonLength(refinedControl_deg);
            refinedCost_deg = refinedLength_deg + ...
                travelSavingsRate_deg_s * ...
                segmentCount * refinedSegmentTime_s;
            if options.GoalTimeMode == "fixedArrival"
                refinementIsBetter = refinedLength_deg < selectedLength_deg;
            else
                refinementIsBetter = refinedCost_deg < selectedCost_deg;
            end
            if refinementIsBetter
                selectedControl_deg = refinedControl_deg;
                selectedSegmentTime_s = refinedSegmentTime_s;
                selectedPlanes = travelPlanes;
                selectedLength_deg = refinedLength_deg;
                selectedCost_deg = refinedCost_deg;
                travelRefinementAccepted = true;
            end
            break;
        end
    end
    if travelRefinementAccepted
        bestControl_deg = selectedControl_deg;
        bestSegmentTime_s = selectedSegmentTime_s;
        planes = selectedPlanes;
        taggedPairs = taggedPairs | reshape( ...
            [planes.Active], size(planes));
    end
    diagnostics.TaggedPairCount = nnz(taggedPairs);
end

%% Section 3: Project, Dilate, And Certify The Selected Motion

controlPoint_deg = bestControl_deg;
controlPoint_deg(1, 1:3, :) = reshape( repmat(initialState.position_deg, 3, 1), 1, 3, 2);
controlPoint_deg(end, end - 2:end, :) = reshape( repmat(goalState.position_deg, 3, 1), 1, 3, 2);
controlPoint_deg = subdivideMidpoint(controlPoint_deg);
segmentTime_s = bestSegmentTime_s / 2;
exportPolynomial = createPowerPolynomial(controlPoint_deg, 1, 0);
certifiedControlPoint_deg = ...
    powerToBernsteinControls(exportPolynomial.positionPower_deg);
requiredTime_s = max(requiredSegmentTime(controlPoint_deg, limits), ...
    requiredSegmentTime(certifiedControlPoint_deg, limits));
dilationScale = max(1, requiredTime_s / segmentTime_s) * (1 + 64 * eps);
segmentTime_s = segmentTime_s * dilationScale;
minimumDuration_s = size(controlPoint_deg, 1) * segmentTime_s;
isFixedArrival = options.GoalTimeMode == "fixedArrival";
if minimumDuration_s > motionHorizon_s + options.ConstraintTolerance
    reasons = ["timeWindowInfeasible", "fixedArrivalInfeasible"];
    messages = ["The certified motion exceeds the goal horizon.", ...
        "The certified minimum exceeds the fixed arrival."];
    diagnostics.EndpointProjectionApplied = true;
    diagnostics.DilationScale = dilationScale;
    [candidate, diagnostics] = finishFailure(candidate, diagnostics, totalTimer, ...
        messages(1 + isFixedArrival), reasons(1 + isFixedArrival), true);
    return;
elseif isFixedArrival
    fixedScale = motionHorizon_s / minimumDuration_s;
    segmentTime_s = segmentTime_s * fixedScale;
    dilationScale = dilationScale * fixedScale;
end
motion = motionCertificate(segmentTime_s, requiredTime_s);
diagnostics.EndpointProjectionApplied = true;
diagnostics.DilationScale = dilationScale;
diagnostics.MotionCertificate = motion;
certificate = certifyAllPlanes(certifiedControlPoint_deg, regions_deg, ...
    coverage, repelem(regionActiveBySegment, 2, 1), ...
    roundoffReserve_deg, obstacleTarget_deg, tightPlaneOptions);
diagnostics.PlaneCertificate = certificate;
candidate.PlaneCertificate = certificate;
candidate = exportMotion(candidate, controlPoint_deg, segmentTime_s, ...
    initialState.time_s, options.SampleTime_s, motion);
[candidate.OptimizerFeasible, candidate.ArrivalAtHorizon] = ...
    deal(true, isFixedArrival);
diagnostics.BestDuration_s = candidate.MotionDuration_s;
if ~certificate.Passed
    [candidate, diagnostics] = finishFailure(candidate, diagnostics, totalTimer, ...
        "The optimized motion requires independent collision validation.", ...
        "planeCertificateUnavailable", true);
    return;
end

%% Section 4: Finalize The Directly Certified Candidate

[candidate.Message, candidate.TerminationReason] = ...
    deal("A directly certified BMTP trajectory was found.", "goalReached");
[candidate.Success, diagnostics.Accepted] = deal(true);
[diagnostics.BestDuration_s, diagnostics.ElapsedTime_s] = ...
    deal(candidate.MotionDuration_s, toc(totalTimer));
candidate.SolverDiagnostics = diagnostics;
end

%% Section 5: Local Functions

function validateKernelInputs( ...
        seed, regions_deg, coverage, initialState, goalState, limits, options)
% Recheck only kernel-specific restrictions after public normalization.
if ~isstruct(seed) || ~isscalar(seed) || ~all(isfield(seed, {'position_deg', 'tau'}))
    error("bmtpEngine:InvalidSeed", ...
        "seed must be scalar and contain position_deg and tau.");
end
tau = double(seed.tau(:));
route_deg = double(seed.position_deg);
seedIsValid = size(route_deg, 2) == 2 && size(route_deg, 1) == numel(tau) && ...
    all(isfinite(route_deg), "all") && numel(tau) >= 2 && all(isfinite(tau)) && ...
    all(diff(tau) > 0) && abs(tau(1)) <= 32 * eps && abs(tau(end) - 1) <= 32 * eps;
if ~seedIsValid
    error("bmtpEngine:InvalidSeedTau", ...
        "seed.position_deg must be finite N-by-2 and tau must increase 0 to 1.");
end
regionsAreValid = iscell(regions_deg) && iscolumn(regions_deg);
for regionIndex = 1:numel(regions_deg)
    region_deg = regions_deg{regionIndex};
    regionsAreValid = regionsAreValid && isnumeric(region_deg) && ...
        size(region_deg, 2) == 2 && size(region_deg, 1) >= 3 && ...
        all(isfinite(region_deg), "all");
end
coverageIsValid = isstruct(coverage) && isscalar(coverage) && ...
    isfield(coverage, "Passed") && islogical(coverage.Passed) && ...
    isscalar(coverage.Passed);
if ~(regionsAreValid && coverageIsValid)
    error("bmtpEngine:InvalidExclusionRegions", ...
        "regions_deg must be a column cell array of finite N-by-2 polygons " + ...
        "and coverage must contain scalar logical Passed.");
end
if isfield(coverage, "RegionActiveTauInterval")
    activeInterval = double(coverage.RegionActiveTauInterval);
    intervalsAreValid = isnumeric(coverage.RegionActiveTauInterval) && ...
        isreal(coverage.RegionActiveTauInterval) && ...
        isequal(size(activeInterval), [numel(regions_deg), 2]) && ...
        all(isfinite(activeInterval), "all") && ...
        all(activeInterval(:, 1) >= 0) && ...
        all(activeInterval(:, 2) <= 1) && ...
        all(activeInterval(:, 2) > activeInterval(:, 1));
    if ~intervalsAreValid
        error("bmtpEngine:InvalidRegionActiveTauInterval", ...
            "coverage.RegionActiveTauInterval must be finite R-by-2 " + ...
            "intervals satisfying 0 <= start < finish <= 1.");
    end
    timedSegmentCountIsValid = isfield(coverage, "TimedSegmentCount") && ...
        isnumeric(coverage.TimedSegmentCount) && ...
        isreal(coverage.TimedSegmentCount) && ...
        isscalar(coverage.TimedSegmentCount) && ...
        isfinite(coverage.TimedSegmentCount) && ...
        coverage.TimedSegmentCount >= 1 && ...
        coverage.TimedSegmentCount == round(coverage.TimedSegmentCount);
    if ~timedSegmentCountIsValid
        error("bmtpEngine:InvalidTimedSegmentCount", ...
            "Timed coverage requires a positive integer TimedSegmentCount.");
    end
end
endpointDerivative = [initialState.velocity_deg_s, ...
    initialState.acceleration_deg_s2, goalState.velocity_deg_s, goalState.acceleration_deg_s2];
limitsMatrix = [limits.maxVelocity_deg_s; limits.maxAcceleration_deg_s2; limits.maxJerk_deg_s3];
requestIsSupported = max(abs(endpointDerivative)) <= options.ConstraintTolerance && ...
    any(string(options.GoalTimeMode) == ...
    ["balancedArrival", "earliestArrival", "fixedArrival"]) && ...
    ~options.AllowAzimuthWrapping && options.SampleTime_s > 0 && ...
    isequal(size(limitsMatrix), [3 2]) && all(isfinite(limitsMatrix), "all") && ...
    all(limitsMatrix > 0, "all") && (~isfield(goalState, "targetTime_s") || ...
    isempty(goalState.targetTime_s));
if ~requestIsSupported
    error("bmtpEngine:UnsupportedRequest", ...
        "The BMTP kernel requires a finite unwrapped rest-to-rest request.");
end
end

function activePairs = createRegionActiveMask( ...
        segmentCount, regionCount, coverage)
% Map equal-duration spans to caller-owned cells with positive-time overlap.
activePairs = true(segmentCount, regionCount);
if ~isfield(coverage, "RegionActiveTauInterval")
    return;
end
activeInterval = double(coverage.RegionActiveTauInterval);
segmentStartTau = (0:segmentCount - 1).' / segmentCount;
segmentFinishTau = (1:segmentCount).' / segmentCount;
activePairs = segmentStartTau < activeInterval(:, 2).' & ...
    segmentFinishTau > activeInterval(:, 1).';
end

function route_deg = createTimedWarmRoute( ...
        seed, requestedSegmentCount, maximumSegmentCount)
% Sample the timed seed on the equal-duration grid used by the optimizer.
segmentCount = min(round(double(requestedSegmentCount)), maximumSegmentCount);
queryTau = linspace(0, 1, segmentCount + 1).';
route_deg = interp1(double(seed.tau(:)), double(seed.position_deg), ...
    queryTau, "linear");
end

function route_deg = splitRoute(seedRoute_deg, splitCount)
% Split each authored edge uniformly without introducing route preference.
edgeCount = size(seedRoute_deg, 1) - 1;
route_deg = zeros(edgeCount * splitCount + 1, 2);
fractions = repmat((0:splitCount - 1).' / splitCount, edgeCount, 1);
edgeStart_deg = repelem(seedRoute_deg(1:end - 1, :), splitCount, 1);
edgeDelta_deg = repelem(diff(seedRoute_deg, 1, 1), splitCount, 1);
route_deg(1:end - 1, :) = edgeStart_deg + fractions .* edgeDelta_deg;
route_deg(end, :) = seedRoute_deg(end, :);
end

function route_deg = limitWarmRouteSegments(route_deg, maximumSegmentCount)
% Resample only oversized warm routes; final feasibility is independently checked.
segmentCount = size(route_deg, 1) - 1;
if segmentCount <= maximumSegmentCount
    return;
end
distance_deg = [0; cumsum(vecnorm(diff(route_deg, 1, 1), 2, 2))];
keepPoint = [true; diff(distance_deg) > 0];
distance_deg = distance_deg(keepPoint);
route_deg = route_deg(keepPoint, :);
if distance_deg(end) <= 0
    route_deg = repmat(route_deg(1, :), maximumSegmentCount + 1, 1);
    return;
end
queryDistance_deg = linspace(0, distance_deg(end), ...
    maximumSegmentCount + 1).';
route_deg = interp1(distance_deg, route_deg, queryDistance_deg, "linear");
end


function controlPoint_deg = createWarmControl(route_deg, degree)
% Create the route-shaped C3 rest-through-jerk warm control net.
segmentCount = size(route_deg, 1) - 1;
fraction = reshape(min(1, max(0, ((0:degree) - 2) / (degree - 4))), 1, [], 1);
start_deg = reshape(route_deg(1:end - 1, :), segmentCount, 1, 2);
finish_deg = reshape(route_deg(2:end, :), segmentCount, 1, 2);
controlPoint_deg = (1 - fraction) .* start_deg + fraction .* finish_deg;
end

function segmentTime_s = requiredSegmentTime(controlPoint_deg, limits)
% Bound one common segment time from exact derivative control coefficients.
degree = size(controlPoint_deg, 2) - 1;
limitValues = [limits.maxVelocity_deg_s; limits.maxAcceleration_deg_s2; limits.maxJerk_deg_s3];
segmentTime_s = 0;
for order = 1:3
    scale = factorial(degree) / factorial(degree - order);
    peak = squeeze(max(abs(scale * diff(controlPoint_deg, order, 2)), [], [1 2]));
    segmentTime_s = max(segmentTime_s, max( (peak(:).' ./ limitValues(order, :)) .^ (1 / order)));
end
segmentTime_s = max(segmentTime_s, eps);
end

function length_deg = controlPolygonLength(controlPoint_deg)
% Return the convex Bezier travel surrogate used by the secondary SOCP.
edge_deg = diff(controlPoint_deg, 1, 2);
length_deg = sum(vecnorm(edge_deg, 2, 3), "all");
end

function [controlPoint_deg, segmentTime_s, exitFlag, output] = ...
        solveTrajectorySocp( ...
        segmentCount, degree, start_deg, goal_deg, limits, planes, reserve_deg, ...
        maximumMotionDuration_s, goalTimeMode, travelSavingsRate_deg_s, ...
        referenceSegmentTime_s, options)
% Construct the trajectory SOCP for the current planes, timing, and objective.
controlCount = segmentCount * (degree + 1) * 2;
powerIndex = controlCount + (1:4);
travelBoundCount = (goalTimeMode ~= "earliestArrival") * segmentCount * degree;
travelBoundIndex = controlCount + 4 + (1:travelBoundCount);
variableCount = controlCount + 4 + travelBoundCount;
differenceCoefficients = {1, [-1 1], [1 -2 1], [-1 3 -3 1]};
baseInequalityCount = 4 * segmentCount * (3 * degree - 3);
activePlaneCount = nnz(reshape([planes.Active], size(planes)));
inequalityCount = baseInequalityCount + activePlaneCount * (degree + 2);
equalityCount = 13 + 8 * (segmentCount - 1);
A = spalloc(inequalityCount, variableCount, 6 * inequalityCount);
Aeq = spalloc(equalityCount, variableCount, 8 * equalityCount);
beq = zeros(equalityCount, 1);
lb = -Inf(variableCount, 1);
ub = Inf(variableCount, 1);
domain_deg = [limits.azimuthInterval_deg; limits.elevationInterval_deg];
lb(1:controlCount) = repmat( ...
    domain_deg(:, 1), segmentCount * (degree + 1), 1);
ub(1:controlCount) = repmat( ...
    domain_deg(:, 2), segmentCount * (degree + 1), 1);
lb(powerIndex) = 0;
lb(powerIndex(2)) = eps;
lb(travelBoundIndex) = 0;
equalityIndex = 0;
for axisIndex = 1:2
    equalityIndex = equalityIndex + 1;
    Aeq(equalityIndex, ...
        controlIndexOf(1, 0, axisIndex, degree)) = 1; %#ok<SPRIX>
    beq(equalityIndex) = start_deg(axisIndex);
    equalityIndex = equalityIndex + 1;
    Aeq(equalityIndex, controlIndexOf( ...
        segmentCount, degree, axisIndex, degree)) = 1; %#ok<SPRIX>
    beq(equalityIndex) = goal_deg(axisIndex);
    for endpointOrder = 1:2
        equalityIndex = equalityIndex + 1;
        indices = controlIndexOf( ...
            1, [endpointOrder 0], axisIndex, degree);
        Aeq(equalityIndex, indices) = [1 -1]; %#ok<SPRIX>
        equalityIndex = equalityIndex + 1;
        indices = controlIndexOf(segmentCount, ...
            [degree - endpointOrder degree], axisIndex, degree);
        Aeq(equalityIndex, indices) = [1 -1]; %#ok<SPRIX>
    end
end
for segmentIndex = 1:segmentCount - 1
    for order = 0:3
        coefficients = differenceCoefficients{order + 1};
        coefficientIndex = 0:order;
        for axisIndex = 1:2
            equalityIndex = equalityIndex + 1;
            left = controlIndexOf(segmentIndex, ...
                degree - order + coefficientIndex, axisIndex, degree);
            right = controlIndexOf( ...
                segmentIndex + 1, coefficientIndex, axisIndex, degree);
            Aeq(equalityIndex, left) = coefficients; %#ok<SPRIX>
            Aeq(equalityIndex, right) = ...
                Aeq(equalityIndex, right) - coefficients; %#ok<SPRIX>
        end
    end
end
equalityIndex = equalityIndex + 1;
Aeq(equalityIndex, powerIndex(1)) = 1;
beq(equalityIndex) = 1;
limitValues = [limits.maxVelocity_deg_s; ...
    limits.maxAcceleration_deg_s2; limits.maxJerk_deg_s3];
inequalityIndex = 0;
for segmentIndex = 1:segmentCount
    for order = 1:3
        coefficients = differenceCoefficients{order + 1};
        scale = factorial(degree) / factorial(degree - order);
        for derivativeIndex = 0:degree - order
            for axisIndex = 1:2
                inequalityIndex = inequalityIndex + 1;
                indices = controlIndexOf(segmentIndex, ...
                    derivativeIndex + (0:order), axisIndex, degree);
                A(inequalityIndex, indices) = ...
                    scale * coefficients; %#ok<SPRIX>
                A(inequalityIndex, powerIndex(order + 1)) = ...
                    -limitValues(order, axisIndex); %#ok<SPRIX>
                inequalityIndex = inequalityIndex + 1;
                A(inequalityIndex, indices) = ...
                    -scale * coefficients; %#ok<SPRIX>
                A(inequalityIndex, powerIndex(order + 1)) = ...
                    -limitValues(order, axisIndex); %#ok<SPRIX>
            end
        end
    end
end
cones = [createTimePowerCones(variableCount, powerIndex); ...
    createTravelBoundCones(variableCount, travelBoundIndex, ...
    segmentCount, degree)];
b = zeros(inequalityCount, 1);
f = zeros(variableCount, 1);
if goalTimeMode == "earliestArrival"
    f(powerIndex(4)) = 1;
else
    f(travelBoundIndex) = 1;
end
maximumSegmentTime_s = maximumMotionDuration_s / segmentCount;
timePowers_s = [1; maximumSegmentTime_s; ...
    maximumSegmentTime_s ^ 2; maximumSegmentTime_s ^ 3];
if goalTimeMode == "fixedArrival"
    lb(powerIndex) = timePowers_s;
    ub(powerIndex) = timePowers_s;
else
    ub(powerIndex) = timePowers_s;
end
if goalTimeMode == "balancedArrival"
    referenceSegmentTime_s = max(referenceSegmentTime_s, eps);
    f(powerIndex(4)) = travelSavingsRate_deg_s * segmentCount / ...
        (3 * referenceSegmentTime_s ^ 2);
end
inequalityIndex = baseInequalityCount;
for segmentIndex = 1:segmentCount
    for regionIndex = 1:size(planes, 2)
        plane = planes(segmentIndex, regionIndex);
        if ~plane.Active
            continue;
        end
        [rows, offset_deg] = fixedPlaneRows( plane, degree, variableCount, segmentIndex);
        targets = inequalityIndex + (1:size(rows, 1));
        A(targets, :) = rows; %#ok<SPRIX>
        b(targets) = -reserve_deg - offset_deg;
        inequalityIndex = targets(end);
    end
end
[x, ~, exitFlag, output] = coneprog( ...
    f, cones, A, b, Aeq, beq, lb, ub, options);
if exitFlag <= 0 || isempty(x) || any(~isfinite(x))
    controlPoint_deg = zeros(0, degree + 1, 2);
    segmentTime_s = NaN;
    return;
end
segmentTime_s = max(x(powerIndex(4)), 0) ^ (1 / 3);
controlPoint_deg = permute(reshape( x(1:controlCount), 2, degree + 1, segmentCount), [3 2 1]);
end

function soc = createTimePowerCones(variableCount, powerIndex)
% Create p0*p2>=p1^2 and p1*p3>=p2^2 as standard cones.
emptyCone = secondordercone(zeros(2, variableCount), zeros(2, 1), zeros(variableCount, 1), 0);
soc = repmat(emptyCone, 2, 1);
for coneIndex = 1:2
    coneA = zeros(2, variableCount);
    coneA(1, powerIndex(coneIndex + 1)) = 2;
    coneA(2, powerIndex(coneIndex)) = 1;
    coneA(2, powerIndex(coneIndex + 2)) = -1;
    coneD = zeros(variableCount, 1);
    coneD(powerIndex([coneIndex coneIndex + 2])) = 1;
    soc(coneIndex) = secondordercone(coneA, zeros(2, 1), coneD, 0);
end
end

function soc = createTravelBoundCones( ...
        variableCount, travelBoundIndex, segmentCount, degree)
% Bound every Bezier control edge so their sum is a convex length surrogate.
if isempty(travelBoundIndex)
    soc = repmat(secondordercone( ...
        zeros(2, variableCount), zeros(2, 1), ...
        zeros(variableCount, 1), 0), 0, 1);
    return;
end
soc = repmat(secondordercone( ...
    zeros(2, variableCount), zeros(2, 1), ...
    zeros(variableCount, 1), 0), numel(travelBoundIndex), 1);
boundIndex = 0;
for segmentIndex = 1:segmentCount
    for controlIndex = 0:degree - 1
        boundIndex = boundIndex + 1;
        coneA = zeros(2, variableCount);
        for axisIndex = 1:2
            firstIndex = controlIndexOf( ...
                segmentIndex, controlIndex, axisIndex, degree);
            secondIndex = controlIndexOf( ...
                segmentIndex, controlIndex + 1, axisIndex, degree);
            coneA(axisIndex, [firstIndex secondIndex]) = [-1 1];
        end
        coneC = zeros(variableCount, 1);
        coneC(travelBoundIndex(boundIndex)) = 1;
        soc(boundIndex) = secondordercone( ...
            coneA, zeros(2, 1), coneC, 0);
    end
end
end

function [plane, exitFlag] = solveMaximumMarginPlane( ...
        controlPoint_deg, vertices_deg, target_deg, reserve_deg, options)
% Solve and directly verify one degree-one maximum-margin plane.
offsetIndex = 5:6;
marginIndex = 7;
variableCount = 7;
[A, b] = maximumMarginRows(controlPoint_deg, vertices_deg, target_deg);
f = zeros(variableCount, 1);
f(marginIndex) = 1;
emptyCone = secondordercone(zeros(2, variableCount), zeros(2, 1), zeros(variableCount, 1), -1);
soc = repmat(emptyCone, 2, 1);
for planeIndex = 0:1
    coneA = zeros(2, variableCount);
    coneA(:, planeIndex * 2 + (1:2)) = eye(2);
    soc(planeIndex + 1) = secondordercone( coneA, zeros(2, 1), zeros(variableCount, 1), -1);
end
[x, ~, exitFlag] = coneprog( f, soc, A, b, [], [], [], [], options);
plane = emptyPlane();
plane.ExitFlag = exitFlag;
if isempty(x) || any(~isfinite(x))
    return;
end
[plane.Active, plane.Normal, plane.Offset_deg] = ...
    deal(true, reshape(x(1:4), 2, []).', x(offsetIndex).');
plane = verifyPlane(plane, controlPoint_deg, vertices_deg, reserve_deg, target_deg);
end

function [A, b] = maximumMarginRows(controlPoint_deg, vertices_deg, target_deg)
% Create the linear inequalities for one maximum-margin plane solve.
degree = size(controlPoint_deg, 1) - 1;
variableCount = 7;
offsetIndex = 5:6;
marginIndex = 7;
A = zeros(2 * size(vertices_deg, 1) + degree + 2, variableCount);
b = zeros(size(A, 1), 1);
rowIndex = 0;
for planeIndex = 0:1
    targets = rowIndex + (1:size(vertices_deg, 1));
    normal = planeIndex * 2 + (1:2);
    A(targets, normal) = -vertices_deg;
    A(targets, offsetIndex(planeIndex + 1)) = -1;
    b(targets) = -target_deg;
    rowIndex = targets(end);
end
objectiveRows = variablePlaneRows(controlPoint_deg, variableCount);
targets = rowIndex + (1:size(objectiveRows, 1));
A(targets, :) = objectiveRows;
A(targets, marginIndex) = -1;
end

function plane = verifyPlane(plane, control_deg, vertices_deg, reserve_deg, target_deg)
% Verify obstacle, trajectory, gap, and normal inequalities numerically.
minimumObstacleSide_deg = min( vertices_deg * plane.Normal.' + plane.Offset_deg, [], "all");
degree = size(control_deg, 1) - 1;
[alpha, beta] = productWeights(degree);
product_deg = alpha .* [sum(control_deg .* plane.Normal(1, :), 2); 0] + ...
    beta .* [0; sum(control_deg .* plane.Normal(2, :), 2)] + ...
    alpha * plane.Offset_deg(1) + beta * plane.Offset_deg(2);
[maximumTrajectorySide_deg, maximumNormalNorm] = ...
    deal(max(product_deg), max(vecnorm(plane.Normal, 2, 2)));
[minimumCorrection_deg, maximumCorrection_deg] = deal( ...
    target_deg - minimumObstacleSide_deg, -reserve_deg - maximumTrajectorySide_deg);
if minimumCorrection_deg <= maximumCorrection_deg
    scale_deg = bmtpEngine.createCoordinateTolerances( ...
        plane.Offset_deg, vertices_deg, control_deg);
    roundoff_deg = 16 * eps(scale_deg);
    [robustMinimum_deg, robustMaximum_deg] = deal( ...
        minimumCorrection_deg + roundoff_deg, maximumCorrection_deg - roundoff_deg);
    if robustMinimum_deg <= robustMaximum_deg
        correction_deg = min(max(0, robustMinimum_deg), robustMaximum_deg);
    else
        correction_deg = 0.5 * (minimumCorrection_deg + maximumCorrection_deg);
    end
    plane.Offset_deg = plane.Offset_deg + correction_deg;
    [minimumObstacleSide_deg, maximumTrajectorySide_deg] = deal( ...
        minimumObstacleSide_deg + correction_deg, maximumTrajectorySide_deg + correction_deg);
end
signedGap_deg = minimumObstacleSide_deg - maximumTrajectorySide_deg;
plane.SignedGap_deg = signedGap_deg;
normalNormLimit = 1 + 2 ^ 20 * eps;
clearanceTarget_deg = ...
    (target_deg - reserve_deg) / normalNormLimit;
certifiedClearance_deg = (signedGap_deg - 2 * reserve_deg) / ...
    max(maximumNormalNorm, realmin);
plane.Verified = minimumObstacleSide_deg >= target_deg && ...
    maximumTrajectorySide_deg <= -reserve_deg && signedGap_deg >= target_deg + reserve_deg && ...
    certifiedClearance_deg >= clearanceTarget_deg && ...
    maximumNormalNorm <= normalNormLimit;
end

function [rows, offset_deg] = fixedPlaneRows( plane, degree, variableCount, segmentIndex)
% Expand a fixed plane times decision-valued trajectory controls.
[alpha, beta] = productWeights(degree);
rows = spalloc(degree + 2, variableCount, 4 * (degree + 2));
% Each product row has bounded support, so direct sparse fills preserve that layout.
for productIndex = 1:degree + 2
    if alpha(productIndex) > 0
        indices = controlIndexOf(segmentIndex, productIndex - 1, 1:2, degree);
        rows(productIndex, indices) = ...
            alpha(productIndex) * plane.Normal(1, :); %#ok<SPRIX>
    end
    if beta(productIndex) > 0
        indices = controlIndexOf(segmentIndex, productIndex - 2, 1:2, degree);
        currentValues = full(rows(productIndex, indices));
        rows(productIndex, indices) = currentValues + ...
            beta(productIndex) * plane.Normal(2, :); %#ok<SPRIX>
    end
end
offset_deg = alpha * plane.Offset_deg(1) + beta * plane.Offset_deg(2);
end

function rows = variablePlaneRows(control_deg, variableCount)
% Expand a decision-valued plane times one fixed trajectory control net.
degree = size(control_deg, 1) - 1;
[alpha, beta] = productWeights(degree);
rows = zeros(degree + 2, variableCount);
rows(1:end - 1, 1:2) = alpha(1:end - 1) .* control_deg;
rows(2:end, 3:4) = beta(2:end) .* control_deg;
rows(:, 5:6) = [alpha beta];
end

function [alpha, beta] = productWeights(degree)
% Return exact degree-N by degree-one Bernstein product weights.
beta = (0:degree + 1).' / (degree + 1);
alpha = 1 - beta;
end

function collisionPairs = findSampledCollisionPairs( ...
        controlPoint_deg, regions_deg, regionMinimum_deg, ...
        regionMaximum_deg, regionActiveBySegment, sampleCount)
% Tag work densely; this broad phase is never an acceptance certificate.
segmentCount = size(controlPoint_deg, 1);
collisionPairs = false(segmentCount, numel(regions_deg));
tau = linspace(0, 1, sampleCount).';
for segmentIndex = 1:segmentCount
    position_deg = evaluateBezier( squeeze(controlPoint_deg(segmentIndex, :, :)), tau);
    sampleMinimum_deg = min(position_deg, [], 1);
    sampleMaximum_deg = max(position_deg, [], 1);
    overlaps = regionActiveBySegment(segmentIndex, :).' & ...
        regionMinimum_deg(:, 1) <= sampleMaximum_deg(1) & ...
        regionMaximum_deg(:, 1) >= sampleMinimum_deg(1) & ...
        regionMinimum_deg(:, 2) <= sampleMaximum_deg(2) & ...
        regionMaximum_deg(:, 2) >= sampleMinimum_deg(2);
    for regionIndex = reshape(find(overlaps), 1, [])
        vertices_deg = regions_deg{regionIndex};
        [inside, on] = inpolygon(position_deg(:, 1), ...
            position_deg(:, 2), vertices_deg(:, 1), vertices_deg(:, 2));
        collisionPairs(segmentIndex, regionIndex) = any(inside | on);
    end
end
end

function motion = motionCertificate(segmentTime_s, requiredTime_s)
% Record the exact derivative-control timing inequality used for dilation.
motion = struct("Passed", segmentTime_s >= requiredTime_s, ...
    "SegmentTime_s", segmentTime_s, "RequiredSegmentTime_s", requiredTime_s, ...
    "MaximumViolation", max(0, requiredTime_s - segmentTime_s));
end

function certificate = certifyAllPlanes(control_deg, regions_deg, ...
        coverage, regionActiveBySegment, reserve_deg, ...
        target_deg, solverOptions)
% Verify every applicable output-span and convex-exclusion-region pair.
segmentCount = size(control_deg, 1);
regionCount = numel(regions_deg);
planes = repmat(emptyPlane(), segmentCount, regionCount);
verifiedCount = 0;
conicCount = 0;
reusedCount = 0;
analyticCount = 0;
minimumGap_deg = Inf;
for segmentIndex = 1:segmentCount
    trajectory_deg = squeeze(control_deg(segmentIndex, :, :));
    for regionIndex = 1:regionCount
        if ~regionActiveBySegment(segmentIndex, regionIndex)
            continue;
        end
        plane = certifyHullSeparationPlane(trajectory_deg, ...
            regions_deg{regionIndex}, reserve_deg, target_deg);
        if plane.Verified
            analyticCount = analyticCount + 1;
        else
            [plane, ~] = solveMaximumMarginPlane(trajectory_deg, ...
                regions_deg{regionIndex}, target_deg, reserve_deg, ...
                solverOptions);
            conicCount = conicCount + 1;
        end
        planes(segmentIndex, regionIndex) = plane;
        if plane.Verified
            verifiedCount = verifiedCount + 1;
            minimumGap_deg = min(minimumGap_deg, plane.SignedGap_deg);
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
certificate = struct( "Kind", certificateKind, ...
    "Passed", coverage.Passed && verifiedCount == allPairCount, ...
    "ExactRegionCount", exactRegionCount, ...
    "SolverRegionCount", regionCount, "Regions_deg", {regions_deg}, ...
    "Planes", planes, "RegionActiveBySegment", regionActiveBySegment, ...
    "RequiredGap_deg", target_deg + reserve_deg, ...
    "RoundoffReserve_deg", reserve_deg, ...
    "MinimumSignedGap_deg", minimumGap_deg, ...
    "CoveragePassed", coverage.Passed, "Coverage", coverage, ...
    "AllPairCount", allPairCount, "VerifiedPairCount", verifiedCount, ...
    "ReusedPairCount", reusedCount, "AnalyticPairCount", analyticCount, ...
    "ConicPairCount", conicCount);
end

function plane = certifyHullSeparationPlane( ...
        control_deg, vertices_deg, reserve_deg, target_deg)
% Prove disjoint convex hulls by separating axes; leave overlap to SOCP.
plane = emptyPlane();
edge_deg = vertices_deg([2:end 1], :) - vertices_deg;
controlPairs = nchoosek(1:size(control_deg, 1), 2);
edge_deg = [edge_deg; ...
    control_deg(controlPairs(:, 2), :) - control_deg(controlPairs(:, 1), :)];
edgeLength_deg = vecnorm(edge_deg, 2, 2);
edge_deg = edge_deg(edgeLength_deg > 0, :);
edgeLength_deg = edgeLength_deg(edgeLength_deg > 0);
if isempty(edge_deg)
    return;
end
normals = [-edge_deg(:, 2), edge_deg(:, 1)] ./ edgeLength_deg;
normals = [normals; -normals];
gaps_deg = min(vertices_deg * normals.', [], 1) - ...
    max(control_deg * normals.', [], 1);
[maximumGap_deg, normalIndex] = max(gaps_deg);
if maximumGap_deg < target_deg + reserve_deg
    return;
end
normal = normals(normalIndex, :);
[plane.Active, plane.Normal, plane.Offset_deg] = ...
    deal(true, repmat(normal, 2, 1), zeros(1, 2));
plane = verifyPlane( ...
    plane, control_deg, vertices_deg, reserve_deg, target_deg);
end

function subdivided_deg = subdivideMidpoint(control_deg)
% Restrict every Bezier span to exact half intervals by de Casteljau averaging.
segmentCount = size(control_deg, 1);
degree = size(control_deg, 2) - 1;
subdivided_deg = zeros(2 * segmentCount, degree + 1, 2);
for segmentIndex = 1:segmentCount
    work_deg = squeeze(control_deg(segmentIndex, :, :));
    left_deg = zeros(degree + 1, 2);
    right_deg = zeros(degree + 1, 2);
    left_deg(1, :) = work_deg(1, :);
    right_deg(end, :) = work_deg(end, :);
    for levelIndex = 1:degree
        work_deg = (work_deg(1:end - 1, :) + work_deg(2:end, :)) / 2;
        left_deg(levelIndex + 1, :) = work_deg(1, :);
        right_deg(end - levelIndex, :) = work_deg(end, :);
    end
    subdivided_deg(2 * segmentIndex - 1, :, :) = left_deg;
    subdivided_deg(2 * segmentIndex, :, :) = right_deg;
end
end

function candidate = exportMotion(candidate, control_deg, segmentTime_s, ...
        initialTime_s, sampleTime_s, motion)
% Export one control net through the stable candidate motion fields.
polynomial = createPowerPolynomial(control_deg, segmentTime_s, initialTime_s);
sampled = samplePolynomial(polynomial, sampleTime_s);
[candidate.FinalTime_s, candidate.ArrivalTime_s] = deal(polynomial.FinalTime_s);
[candidate.MotionDuration_s, candidate.TrajectoryDuration_s] = ...
    deal(polynomial.FinalTime_s - initialTime_s);
candidate.MotionLength_deg = sum(vecnorm( diff(sampled.position_deg, 1, 1), 2, 2));
candidate.IntegratedSquaredJerk_deg2_s5 = integratedSquaredJerk(polynomial);
candidate.MaximumConstraintViolation = motion.MaximumViolation;
for name = ["time_s", "position_deg", "velocity_deg_s", ...
        "acceleration_deg_s2", "jerk_deg_s3"]
    candidate.(name) = sampled.(name);
end
candidate.Polynomial = polynomial;
end

function polynomial = createPowerPolynomial(control_deg, segmentTime_s, initialTime_s)
% Convert midpoint-restricted Bernstein segments to ascending powers.
segmentCount = size(control_deg, 1);
degree = size(control_deg, 2) - 1;
[powerIndex, bernsteinIndex] = ndgrid(0:degree);
valid = bernsteinIndex <= powerIndex;
conversion = zeros(degree + 1);
conversion(valid) = factorial(degree) * (-1) .^ ...
    (powerIndex(valid) - bernsteinIndex(valid)) ./ (factorial(bernsteinIndex(valid)) .* ...
    factorial(powerIndex(valid) - bernsteinIndex(valid)) .* factorial(degree - powerIndex(valid)));
bernsteinPages = permute(control_deg, [2 1 3]);
positionPower_deg = permute(pagemtimes(conversion, bernsteinPages), [2 3 1]);
positionPower_deg = stabilizePowerEndpoints(positionPower_deg, control_deg);
velocityPower_deg_s = positionPower_deg(:, :, 2:end) .* ...
    reshape(1:degree, 1, 1, []) / segmentTime_s;
accelerationPower_deg_s2 = velocityPower_deg_s(:, :, 2:end) .* ...
    reshape(1:degree - 1, 1, 1, []) / segmentTime_s;
jerkPower_deg_s3 = accelerationPower_deg_s2(:, :, 2:end) .* ...
    reshape(1:degree - 2, 1, 1, []) / segmentTime_s;
segmentStartTime_s = initialTime_s + (0:segmentCount - 1).' * segmentTime_s;
polynomial = struct( "Degree", degree, "SegmentCount", segmentCount, ...
    "SegmentStartTime_s", segmentStartTime_s, ...
    "SegmentDuration_s", repmat(segmentTime_s, segmentCount, 1), ...
    "SegmentBreakTau", (0:segmentCount).' / segmentCount, ...
    "FinalTime_s", initialTime_s + segmentCount * segmentTime_s, ...
    "positionPower_deg", positionPower_deg, "velocityPower_deg_s", velocityPower_deg_s, ...
    "accelerationPower_deg_s2", accelerationPower_deg_s2, ...
    "jerkPower_deg_s3", jerkPower_deg_s3, "TerminalState", struct("position_deg", ...
    squeeze(control_deg(end, end, :)).', "velocity_deg_s", [0 0], "acceleration_deg_s2", [0 0]));
end

function control_deg = powerToBernsteinControls(positionPower_deg)
% Reconstruct the exact control net represented by exported position powers.
degree = size(positionPower_deg, 3) - 1;
transform = zeros(degree + 1);
for bernsteinIndex = 0:degree
    for powerIndex = 0:bernsteinIndex
        transform(bernsteinIndex + 1, powerIndex + 1) = ...
            nchoosek(bernsteinIndex, powerIndex) / ...
            nchoosek(degree, powerIndex);
    end
end
powerPages = permute(positionPower_deg, [3 1 2]);
control_deg = permute(pagemtimes(transform, powerPages), [2 1 3]);
end

function power_deg = stabilizePowerEndpoints(power_deg, control_deg)
% Project last-four-power roundoff onto exact Bernstein C0-C3 endpoints.
degree = size(control_deg, 2) - 1;
segmentCount = size(control_deg, 1);
endPower = degree - 3:degree;
orders = (0:3).';
endMap = factorial(endPower) ./ factorial(endPower - orders);
target = zeros(segmentCount, 2, 4);
power_deg(:, :, 1) = reshape(control_deg(:, 1, :), segmentCount, 2);
target(:, :, 1) = reshape(control_deg(:, end, :), segmentCount, 2);
for order = 1:3
    difference = diff(control_deg, order, 2);
    scale = factorial(degree) / factorial(degree - order);
    power_deg(:, :, order + 1) = reshape(difference(:, 1, :), segmentCount, 2) * ...
        scale / factorial(order);
    target(:, :, order + 1) = reshape(difference(:, end, :), segmentCount, 2) * scale;
end
for projectionPass = 1:2
    current = zeros(segmentCount, 2, 4);
    for order = 0:3
        indices = order:degree;
        multipliers = reshape(factorial(indices) ./ factorial(indices - order), 1, 1, []);
        current(:, :, order + 1) = sum(power_deg(:, :, indices + 1) .* multipliers, 3);
    end
    residual = reshape(permute(target - current, [3 1 2]), 4, []);
    correction = permute(reshape(endMap \ residual, 4, segmentCount, 2), [2 3 1]);
    power_deg(:, :, endPower + 1) = power_deg(:, :, endPower + 1) + correction;
end
end

function sampled = samplePolynomial(polynomial, sampleTime_s)
% Sample through the same public polynomial evaluator used by validation.
initialTime_s = polynomial.SegmentStartTime_s(1);
duration_s = polynomial.FinalTime_s - initialTime_s;
segmentTime_s = polynomial.SegmentDuration_s(1);
relativeTime_s = unique([(0:sampleTime_s:duration_s).'; ...
    (0:polynomial.SegmentCount).' * segmentTime_s; duration_s]);
[time_s, position_deg, velocity_deg_s, acceleration_deg_s2, jerk_deg_s3] = ...
    bmtpEngine.evaluatePolynomial( ...
    polynomial, initialTime_s + relativeTime_s);
sampled = struct("time_s", time_s, "position_deg", position_deg, ...
    "velocity_deg_s", velocity_deg_s, "acceleration_deg_s2", acceleration_deg_s2, ...
    "jerk_deg_s3", jerk_deg_s3);
end

function cost_deg2_s5 = integratedSquaredJerk(polynomial)
% Integrate squared physical jerk exactly over every polynomial segment.
coefficients = permute(polynomial.jerkPower_deg_s3, [3 1 2]);
order = (1:size(coefficients, 1)).';
gram = 1 ./ (order + order.' - 1);
segmentCosts = sum(coefficients .* pagemtimes(gram, coefficients), 1);
cost_deg2_s5 = sum(polynomial.SegmentDuration_s(:) .* ...
    reshape(segmentCosts, polynomial.SegmentCount, 2), "all");
end

function position = evaluateBezier(control, tau)
% Evaluate samples through one vectorized de Casteljau recurrence.
degree = size(control, 1) - 1;
tau = reshape(double(tau), [], 1, 1);
work = repmat(reshape(control, 1, degree + 1, []), numel(tau), 1, 1);
for level = 1:degree
    work = (1 - tau) .* work(:, 1:end - 1, :) + tau .* work(:, 2:end, :);
end
position = reshape(work(:, 1, :), numel(tau), size(control, 2));
end

function index = controlIndexOf(segmentIndex, controlIndex, axisIndex, degree)
% Map trajectory control coordinates into the conic decision vector.
index = ((segmentIndex - 1) * (degree + 1) + controlIndex) * 2 + axisIndex;
end

function plane = emptyPlane()
% Define the stable inactive or verified degree-one plane record.
plane = struct("Active", false, "Verified", false, "ExitFlag", NaN, ...
    "Normal", zeros(2, 2), "Offset_deg", zeros(1, 2), "SignedGap_deg", NaN);
end

function candidate = createEmptyCandidate(seed, initialState, options)
% Define identical candidate fields for success and every failure path.
seedIndex = optionalField(seed, "Index", 0);
seedSource = string(optionalField(seed, "Source", ""));
corridorBoundary_deg = optionalField(seed, "CorridorBoundary_deg", zeros(0, 2));
[candidate, ~] = bmtpEngine.createMotionRecord( ...
    struct(), initialState, [], [], options.SampleTime_s, seedSource);
extra = struct("OptimizerFeasible", false, "ArrivalAtHorizon", false, ...
    "SeedIndex", seedIndex, "SeedSource", seedSource, "FinalTime_s", NaN, ...
    "MotionDuration_s", NaN, "MotionLength_deg", Inf, ...
    "IntegratedSquaredJerk_deg2_s5", Inf, "MaximumConstraintViolation", Inf, ...
    "SolverDiagnostics", struct());
for name = string(fieldnames(extra)).'
    candidate.(name) = extra.(name);
end
candidate.SeedCorridorBoundary_deg = corridorBoundary_deg;
candidate.Message = "The BMTP kernel was not run.";
end

function value = optionalField(record, name, defaultValue)
% Read one optional diagnostic field without duplicating fallback branches.
value = defaultValue;
if isfield(record, name)
    value = record.(name);
end
end

function diagnostics = createEmptyDiagnostics( degree, splitCount, segmentCount, regionCount)
% Define bounded solver, timing, and certificate evidence before solving.
diagnostics = struct( "Identifier", "bmtpStaticDegree" + string(degree), ...
    "ConstraintRepresentation", "thirdOrderTimePowerSocp", ...
    "Representation", "C3CompositeBezier", "Attempted", true, ...
    "Accepted", false, "Degree", degree, ...
    "SubspansPerSeedEdge", splitCount, "OriginalSeedSegmentCount", segmentCount, ...
    "WarmRouteResampled", false, "OptimizerSpanCount", segmentCount, ...
    "SegmentCount", 2 * segmentCount, "ExactRegionCount", regionCount, ...
    "IterationCount", 0, "Converged", false, ...
    "PlaneReuseApplied", false, "PlaneReuseCount", 0, ...
    "TaggedPairCount", 0, "ApplicablePairCount", segmentCount * regionCount, ...
    "TrajectorySocpCount", 0, "FinalCollisionPairCount", 0, ...
    "PlaneSocpCount", 0, "UnverifiedPlaneInitializationCount", 0, ...
    "FinalTrajectoryExitFlag", NaN, ...
    "FailedPlaneSegmentIndex", 0, "FailedPlaneRegionIndex", 0, ...
    "FailedPlane", emptyPlane(), "WarmStartDuration_s", NaN, ...
    "BestDuration_s", NaN, "RetainedBestTrialDuration_s", NaN, ...
    "EndpointProjectionApplied", false, ...
    "TrialDuration_s", NaN(35, 1), "TrialWasCollisionFree", false(35, 1), ...
    "CollisionPairCountHistory", NaN(35, 1), ...
    "DilationScale", NaN, "MotionCertificate", struct(), "Coverage", struct(), ...
    "PlaneCertificate", struct(), "SolverMessage", "", "ElapsedTime_s", 0);
end

function [candidate, diagnostics] = finishFailure( ...
        candidate, diagnostics, timer, message, reason, optimizerFeasible)
% Finalize one expected failure without manufacturing trajectory data.
[candidate.Message, candidate.TerminationReason, candidate.OptimizerFeasible] = ...
    deal(message, reason, optimizerFeasible);
[diagnostics.Accepted, diagnostics.ElapsedTime_s] = deal(false, toc(timer));
candidate.SolverDiagnostics = diagnostics;
end
