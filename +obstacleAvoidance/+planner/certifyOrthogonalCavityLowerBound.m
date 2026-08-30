function certificate = certifyOrthogonalCavityLowerBound( ...
        cavityDiagnostics, candidate, obstacles, initialState, goalState, limits, options)
%% Section 0: Header & Readme
% SYNTAX
%   certificate = ...
%       obstacleAvoidance.planner.certifyOrthogonalCavityLowerBound()
%   certificate = ...
%       obstacleAvoidance.planner.certifyOrthogonalCavityLowerBound( ...
%       cavityDiagnostics, candidate, obstacles, initialState, ...
%       goalState, limits, options)
%**************************************************************************
% PURPOSE
%   - Prove an all-route arrival lower bound for a bilateral orthogonal
%     cavity using guarded interior rectangles and triple-integrator losses.
%   - Report named rejection diagnostics outside the proven subfamily.
%**************************************************************************
% INPUTS
%   - cavityDiagnostics (scalar struct)
%       Output from createOrthogonalCavityMotion with one selected frame.
%   - candidate (scalar struct)
%       Constructed candidate whose polynomial final time supplies the upper.
%   - obstacles (canonical or prepared obstacle struct array)
%       Geometry must be static and active over the complete request horizon.
%   - initialState, goalState (normalized scalar structs)
%       Position is one-by-two; velocity and acceleration must be zero.
%   - limits (normalized scalar struct)
%       Positive per-axis velocity, acceleration, and jerk boxes are required.
%   - options (resolved scalar struct)
%       CollisionClearanceTolerance_deg supplies the Euclidean clearance.
%**************************************************************************
% OUTPUTS
%   - certificate (scalar struct)
%       Stable Passed/LowerBound_s/UpperGap_s record with predicate residuals.
%       Unsupported geometry returns Passed=false; invalid records throw.
%**************************************************************************
% UNITS
%   - Position and clearance are degrees. Time is seconds; derivatives use
%     degrees per corresponding power of seconds.
%**************************************************************************

%% Section 1: Validate The Candidate And Selected Frame

certificate = certificateTemplate();
if nargin == 0
    return;
end
if nargin ~= 7
    error("certifyOrthogonalCavityLowerBound:InvalidCall", ...
        "Use zero inputs or supply all seven documented certificate inputs.");
end
if ~isstruct(cavityDiagnostics) || ~isscalar(cavityDiagnostics) || ...
        ~isstruct(candidate) || ~isscalar(candidate)
    error("certifyOrthogonalCavityLowerBound:InvalidRecords", ...
        "Inputs must be scalar structures.");
end
try
    movingGoal = isfield(goalState, "targetTime_s") && ...
        ~isempty(goalState.targetTime_s) || ...
        isfield(goalState, "targetPosition_deg") && ...
        ~isempty(goalState.targetPosition_deg);
    requireProof(~movingGoal, "unsupportedMovingGoal");
    fields = {'OptimizerFeasible', 'MotionDuration_s', 'FinalTime_s', ...
        'ArrivalTime_s', 'time_s', 'Polynomial'};
    constructed = all(isfield(candidate, fields)) && candidate.OptimizerFeasible && ...
        ~isempty(candidate.time_s) && isstruct(candidate.Polynomial) && ...
        isfield(candidate.Polynomial, "FinalTime_s");
    requireProof(constructed, "candidateNotConstructed");
    finalTime_s = double(candidate.Polynomial.FinalTime_s);
    duration_s = finalTime_s - initialState.time_s;
    reported_s = double([candidate.MotionDuration_s, ...
        candidate.FinalTime_s - initialState.time_s, ...
        candidate.ArrivalTime_s - initialState.time_s, ...
        candidate.time_s(end) - initialState.time_s]);
    metadataResidual_s = max(abs(reported_s - duration_s));
    metadataTolerance_s = 2 ^ 16 * eps(max([1, abs(duration_s), abs(reported_s)]));
    certificate.UpperBound_s = duration_s;
    certificate.MetadataTimeResidual_s = metadataResidual_s;
    requireProof(isscalar(duration_s) && isfinite(duration_s) && duration_s > 0 && ...
        all(isfinite(reported_s)) && metadataResidual_s <= metadataTolerance_s, ...
        "candidateTimeMetadataMismatch");
    requireProof(finalTime_s <= goalState.time_s, "candidateExceedsGoalHorizon");
    frameIndex = double(cavityDiagnostics.SelectedFrameIndex);
    validIndex = isscalar(frameIndex) && isfinite(frameIndex) && ...
        frameIndex == fix(frameIndex) && frameIndex >= 1 && ...
        frameIndex <= numel(cavityDiagnostics.Frames);
    requireProof(validIndex, "selectedFrameMissing");
    frame = cavityDiagnostics.Frames(frameIndex);
    axisOrder = double(frame.AxisOrder(:).');
    axisSign = double(frame.AxisSign(:).');
    supports_deg = double(frame.Supports_deg(:).');
    validFrame = isequal(sort(axisOrder), [1 2]) && ...
        isequal(abs(axisSign), [1 1]) && numel(supports_deg) == 5 && ...
        all(isfinite(supports_deg));
    requireProof(validFrame, "selectedFrameInvalid");

%% Section 2: Validate One Original Static Shape

    % Source fields are authoritative; a stale preparation cache is not proof.
    obstacles = obstacleAvoidance.obstacles.prepareDynamic( ...
        obstacleAvoidance.obstacles.combineObstacles(obstacles));
    requireProof(isscalar(obstacles), "unsupportedMultipleObstacles");
    obstacle = obstacles(1);
    sourceTime_s = double(obstacle.time_s(:));
    horizonCovered = isscalar(sourceTime_s) || ...
        initialState.time_s >= sourceTime_s(1) && goalState.time_s <= sourceTime_s(end);
    requireProof(obstacle.InternalPreparation.IsTimeInvariant && horizonCovered, ...
        "unsupportedDynamicObstacle");
    occupiedShape = obstacle.InternalPreparation.StaticShape;
    requireProof(~isempty(occupiedShape.Vertices), "emptyProtectedUnion");
    p0_deg = initialState.position_deg(axisOrder) .* axisSign;
    pg_deg = goalState.position_deg(axisOrder) .* axisSign;
    derivatives = [initialState.velocity_deg_s, goalState.velocity_deg_s, ...
        initialState.acceleration_deg_s2, goalState.acceleration_deg_s2];
    requestPassed = max(abs(derivatives)) == 0 && ...
        ~options.AllowAzimuthWrapping && string(options.GoalTimeMode) == "earliestArrival";
    clearance_deg = double(options.CollisionClearanceTolerance_deg);
    finiteVertices_deg = occupiedShape.Vertices(isfinite(occupiedShape.Vertices));
    coordinateScale_deg = max([1; abs(finiteVertices_deg); abs(p0_deg(:)); ...
        abs(pg_deg(:)); abs(supports_deg(:)); abs(clearance_deg)]);
    coordinateGuard_deg = 2 ^ 18 * eps(coordinateScale_deg);
    clearanceInterval_deg = outwardInterval(clearance_deg - ...
        4 * coordinateGuard_deg, clearance_deg + 4 * coordinateGuard_deg, 8);
    requireProof(all(isfinite([coordinateScale_deg, coordinateGuard_deg, clearance_deg])) && ...
        clearanceInterval_deg(1, 1) > 0, "coordinateConditioningFailed");

%% Section 3: Prove The Bilateral Barrier Rectangles

    center_deg = mean([p0_deg(1), pg_deg(1)]);
    supports_deg = supports_deg + [8 -8 -8 8 -8] * coordinateGuard_deg;
    left_deg = supports_deg(1) - center_deg;
    right_deg = supports_deg(2) - center_deg;
    top_deg = supports_deg(3);
    bottom_deg = supports_deg(4);
    join_deg = supports_deg(5);
    geometryResidual_deg = min([left_deg, right_deg - left_deg, ...
        top_deg - join_deg, join_deg - bottom_deg]);
    certificate.PredicateResiduals.GeometryOrder_deg = geometryResidual_deg;
    geometryInterval_deg = outwardInterval(geometryResidual_deg, coordinateScale_deg, 16);
    requireProof(isfinite(geometryInterval_deg(1)) && geometryInterval_deg(1) > 0, ...
        "bilateralGeometryOrderFailed");
    bounds_deg = [center_deg + left_deg, center_deg + right_deg, join_deg, top_deg; ...
        center_deg - right_deg, center_deg - left_deg, join_deg, top_deg; ...
        center_deg - right_deg, center_deg + right_deg, bottom_deg, join_deg];
    contained = obstacleAvoidance.planner.certifyGuardedRectangleContainment( ...
        bounds_deg, occupiedShape, axisOrder, axisSign, [0, 0], ...
        coordinateGuard_deg, false).';
    certificate.ContainmentPassed = contained;
    containmentReasons = ["rightArmContainmentFailed", ...
        "leftArmContainmentFailed", "fullFloorContainmentFailed"];
    if ~all(contained)
        requireProof(false, containmentReasons(find(~contained, 1, "last")));
    end

%% Section 4: Evaluate The Event-Order Kinematic Lower Bound

    velocity = double(limits.maxVelocity_deg_s(axisOrder));
    acceleration = double(limits.maxAcceleration_deg_s2(axisOrder));
    jerk = double(limits.maxJerk_deg_s3(axisOrder));
    Vu = velocity(1); Au = acceleration(1); Ju = jerk(1);
    Vv = velocity(2); Av = acceleration(2); Jv = jerk(2);
    ratios_s = [Vu / Au, Au / Ju, Vv / Av, Av / Jv];
    safeRange = [8 * realmin("double") ^ 0.25, realmax("double") ^ 0.25 / 8];
    kinematics = [velocity, acceleration, jerk, ratios_s];
    conditioned = all(isfinite(kinematics)) && all(kinematics >= safeRange(1)) && ...
        all(kinematics <= safeRange(2));
    certificate.PredicateResiduals.KinematicConditioning = conditioned;
    requireProof(conditioned, "kinematicConditioningFailed");
    velocityBranch_s = ratios_s([1 3]) - ratios_s([2 4]);
    certificate.PredicateResiduals.VelocityBranchTime_s = velocityBranch_s;
    topLateral_deg = clearance_deg * 9 / 32;
    topVertical_deg = clearance_deg * 61 / 64;
    bottomVertical_deg = clearance_deg * 45 / 64;
    residuals_deg = [bottomVertical_deg - topLateral_deg, ...
        left_deg - topLateral_deg - abs(p0_deg(1) - center_deg), ...
        bottom_deg - bottomVertical_deg - pg_deg(2), p0_deg(2) - join_deg];
    certificate.PredicateResiduals.BottomCoverage_deg = residuals_deg(1);
    certificate.PredicateResiduals.InitialInner_deg = residuals_deg(2);
    certificate.PredicateResiduals.GoalBelowFloor_deg = residuals_deg(3);
    certificate.PredicateResiduals.StartAboveJoin_deg = residuals_deg(4);
    certificate.PredicateResiduals.RequestPassed = requestPassed;
    topDuration_s = (right_deg - left_deg + 2 * topLateral_deg) / Vu;
    speedDeficit = max(0, 2 * Vv - Av * topDuration_s) / 2;
    largeBranch_s = speedDeficit / Av - ratios_s(4) / 2;
    certificate.PredicateResiduals.LargeBranchTime_s = largeBranch_s;
    initialDistance_deg = Vv * (ratios_s(3) + ratios_s(4)) / 2;
    deficitDistance_deg = initialDistance_deg - Av * ratios_s(4) ^ 2 / 24;
    upDistance_deg = top_deg + topVertical_deg - p0_deg(2);
    downDistance_deg = top_deg + topVertical_deg - ...
        (bottom_deg - bottomVertical_deg);
    suffixDistance_deg = right_deg + bottomVertical_deg - abs(pg_deg(1) - center_deg);
    horizontalTransition_deg = Vu * (ratios_s(1) + ratios_s(2)) / 2;
    distanceResiduals_deg = [upDistance_deg - initialDistance_deg - deficitDistance_deg, ...
        downDistance_deg - deficitDistance_deg, suffixDistance_deg - horizontalTransition_deg];
    certificate.PredicateResiduals.Distance_deg = distanceResiduals_deg;
    transitionScale_s = max([ratios_s, topDuration_s, speedDeficit / Av]);
    distanceScale_deg = coordinateScale_deg + sum(abs([initialDistance_deg, ...
        deficitDistance_deg, horizontalTransition_deg]));
    predicates = [double(requestPassed) - 0.5, residuals_deg(4), ...
        min(velocityBranch_s), min(residuals_deg(1:3)), largeBranch_s, ...
        min(distanceResiduals_deg)];
    scales = [1, coordinateScale_deg, max(ratios_s), coordinateScale_deg, ...
        transitionScale_s, distanceScale_deg];
    predicateIntervals = outwardInterval(predicates, scales, 256);
    reasons = ["unsupportedRequest", "startNotAboveJoin", ...
        "unsupportedVelocityBranch", "eventOrderingPredicateFailed", ...
        "largeTransitionBranchNotProven", "transitionDistancePredicateFailed"];
    failed = find(predicateIntervals(:, 1) <= 0 | ...
        ~isfinite(predicateIntervals(:, 1)), 1);
    if ~isempty(failed)
        requireProof(false, reasons(failed));
    end
    terms_s = [(upDistance_deg + downDistance_deg) / Vv, ...
        (ratios_s(3) + ratios_s(4)) / 2, topDuration_s, ...
        (speedDeficit / Av) * (speedDeficit / Vv), ...
        ratios_s(4) * (ratios_s(4) / ratios_s(3)) / 12, ...
        suffixDistance_deg / Vu, ...
        (ratios_s(1) + ratios_s(2)) / 2];
    rawLower_s = sum(terms_s);
    timeScale_s = sum(abs(terms_s)) + distanceScale_deg / min([Vu, Vv]) + ...
        transitionScale_s;
    lowerInterval_s = outwardInterval(rawLower_s, timeScale_s, 512);
    lowerBound_s = max(0, lowerInterval_s(1));
    reserve_s = rawLower_s - lowerBound_s;
    upperGap_s = duration_s - lowerBound_s;
    certificate.RawLowerBound_s = rawLower_s;
    certificate.NumericalReserve_s = reserve_s;
    certificate.LowerBound_s = lowerBound_s;
    certificate.UpperGap_s = upperGap_s;
    requireProof(all(isfinite([rawLower_s, reserve_s, lowerBound_s, upperGap_s])), ...
        "nonfiniteCertificateState");
    requireProof(upperGap_s >= 0, "upperBelowCertifiedLower");
    certificate.Passed = true;
    certificate.Message = "Both exterior route classes have a certified lower bound.";
    certificate.TerminationReason = "allRoutesLowerBoundCertified";
catch proofFailure
    if string(proofFailure.identifier) ~= ...
            "certifyOrthogonalCavityLowerBound:ProofRejected"
        rethrow(proofFailure);
    end
    certificate = reject(certificate, string(proofFailure.message));
end
end

%% Section 5: Local Functions

function interval = outwardInterval(value, conditionScale, operationCount)
% Apply Higham's gamma_n bound plus endpoint-rounding reserve.
value = double(value(:));
scale = max(abs(value), abs(double(conditionScale(:))));
unitRoundoff = eps(1) / 2;
gamma = operationCount * unitRoundoff / (1 - operationCount * unitRoundoff);
padding = gamma * scale + 4 * eps(scale);
interval = [value - padding, value + padding];
interval(~isfinite(interval)) = NaN;
end

function requireProof(assertion, reason)
% Stop an ambiguous proof branch; the public function catches this sentinel.
if ~(isscalar(assertion) && islogical(assertion) && assertion)
    error("certifyOrthogonalCavityLowerBound:ProofRejected", "%s", reason);
end
end

function certificate = certificateTemplate()
% Define one stable success-or-rejection certificate record.
residuals = struct("GeometryOrder_deg", NaN, "KinematicConditioning", false, ...
    "VelocityBranchTime_s", NaN(1, 2), "BottomCoverage_deg", NaN, ...
    "InitialInner_deg", NaN, "GoalBelowFloor_deg", NaN, ...
    "StartAboveJoin_deg", NaN, "RequestPassed", false, ...
    "LargeBranchTime_s", NaN, "Distance_deg", NaN(1, 3));
certificate = struct("Passed", false, "Message", "Not run.", ...
    "TerminationReason", "notRun", "LowerBound_s", NaN, ...
    "RawLowerBound_s", NaN, "UpperBound_s", NaN, "UpperGap_s", NaN, ...
    "NumericalReserve_s", NaN, "MetadataTimeResidual_s", NaN, ...
    "ContainmentPassed", false(1, 3), "PredicateResiduals", residuals);
end

function certificate = reject(certificate, reason)
% Preserve a named expected rejection without implying public validation.
certificate.TerminationReason = reason;
certificate.Message = "Lower-bound certificate rejected: " + reason + ".";
end
