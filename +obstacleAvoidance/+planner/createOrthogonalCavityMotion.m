function [candidate, diagnostics] = createOrthogonalCavityMotion( ...
        seed, obstacles, initialState, goalState, limits, options)
%% Section 0: Header & Readme
% SYNTAX
%   [candidate, diagnostics] = ...
%       obstacleAvoidance.planner.createOrthogonalCavityMotion( ...
%       seed, obstacles, initialState, goalState, limits, options)
%**************************************************************************
% PURPOSE
%   - Detect an input-defined orthogonal concave cavity and create its exact
%     piecewise-constant-jerk rounded-corner tangent motion.
%   - Return explicit structural or motion rejection diagnostics; only the
%     independent public validator can mark a candidate successful.
%**************************************************************************
% INPUTS
%   - seed (scalar struct)
%       position_deg is an ordered N-by-2 topology route and tau is a
%       strictly increasing N-vector from zero to one.
%   - obstacles (canonical or prepared obstacle struct array)
%       Protected geometry must be static and active over the full horizon.
%   - initialState, goalState (normalized scalar structs)
%       Position, velocity, and acceleration are one-by-two. This restricted
%       kernel supports rest endpoints.
%   - limits (normalized scalar struct)
%       Rectangular workspace and positive per-axis velocity, acceleration,
%       and jerk limits are required.
%   - options (resolved scalar struct)
%       An unwrapped earliest-arrival request and the maintained sampling,
%       constraint, and collision-clearance tolerances are required.
%**************************************************************************
% OUTPUTS
%   - candidate (scalar struct)
%       Stable candidate record. A constructed polynomial includes a complete
%       static plane certificate but remains Success=false until its caller
%       runs the independent public validator.
%   - diagnostics (scalar struct)
%       Contains all eight signed/permuted frame outcomes, extracted supports,
%       containment residuals, certificate gaps, and the selected frame.
%**************************************************************************
% UNITS
%   - Position is degrees and time is seconds. Derivatives use deg/s,
%     deg/s^2, and deg/s^3. Histories are N-by-2.
%**************************************************************************

%% Section 1: Validate And Prepare The Restricted Request

if ~isstruct(seed) || ~isscalar(seed) || ~all(isfield(seed, {'position_deg', 'tau'}))
    error("createOrthogonalCavityMotion:InvalidSeed", "seed requires position_deg and tau.");
end
validateattributes(seed.position_deg, {'numeric'}, ...
    {'real', 'finite', '2d', 'ncols', 2, 'nrows', numel(seed.tau)});
validateattributes(seed.tau, {'numeric'}, {'real', 'finite', 'vector', 'increasing'});
candidate = candidateTemplate(initialState, options);
frameTemplate = struct("AxisOrder", [0 0], "AxisSign", [0 0], ...
    "TerminationReason", "notRun", "Supports_deg", NaN(1, 5), ...
    "ContainmentResidual_deg2", Inf, "MinimumPlaneGap_deg", NaN);
diagnostics = struct("Success", false, "TerminationReason", "notRun", ...
    "SelectedFrameIndex", 0, "Frames", repmat(frameTemplate, 8, 1));
endpointDerivative = [initialState.velocity_deg_s, goalState.velocity_deg_s, ...
    initialState.acceleration_deg_s2, goalState.acceleration_deg_s2];
unsupportedReason = "";
if max(abs(endpointDerivative)) > options.ConstraintTolerance
    unsupportedReason = "unsupportedEndpointDerivative";
elseif options.AllowAzimuthWrapping || string(options.GoalTimeMode) ~= "earliestArrival"
    unsupportedReason = "unsupportedGoalPolicy";
elseif any(limits.maxVelocity_deg_s < limits.maxAcceleration_deg_s2 .^ 2 ./ ...
        limits.maxJerk_deg_s3 - options.ConstraintTolerance)
    unsupportedReason = "unsupportedVelocityBeforeAccelerationBranch";
end
if unsupportedReason ~= ""
    [candidate, diagnostics] = reject(candidate, diagnostics, ...
        unsupportedReason, "The restricted cavity motion does not support this request.");
    return;
end
if isempty(obstacles) || ~isfield(obstacles, "InternalPreparation")
    obstacles = obstacleAvoidance.obstacles.combineObstacles(obstacles);
end
obstacles = obstacleAvoidance.obstacles.prepareDynamic(obstacles);
[hasStaticHorizon, occupiedShape] = ...
    obstacleAvoidance.obstacles.queryStaticHorizon( ...
    obstacles, initialState.time_s, goalState.time_s);
if ~hasStaticHorizon
    [candidate, diagnostics] = reject(candidate, diagnostics, ...
        "unsupportedDynamicObstacle", ...
        "Every obstacle must be static and active over the horizon.");
    return;
end
if isempty(occupiedShape.Vertices)
    [candidate, diagnostics] = reject(candidate, diagnostics, ...
        "structureNotDetected", "A cavity requires nonempty geometry.");
    return;
end
[~, geometryTolerance_deg, roundoffReserve_deg] = ...
    bmtpEngine.createCoordinateTolerances(occupiedShape.Vertices, ...
    seed.position_deg, initialState.position_deg, goalState.position_deg);

% Two roundoff reserves are required by the independent plane verifier. A
% third places the tangent strictly outside that threshold; subdivision then
% has one full reserve for cubic-hull over-approximation. Failure is reported,
% never hidden by changing the requested clearance or geometry.
clearanceRadius_deg = options.CollisionClearanceTolerance_deg + 3 * roundoffReserve_deg;

%% Section 2: Enumerate Signed And Permuted Frames

frameSpecifications = [1 2 -1 -1; 1 2 -1 1; 1 2 1 -1; 1 2 1 1; ...
    2 1 -1 -1; 2 1 -1 1; 2 1 1 -1; 2 1 1 1];
bestCandidate = candidate;
selectedFrameIndex = 0;
for frameIndex = 1:8
    frame = frameTemplate;
    axisOrder = frameSpecifications(frameIndex, 1:2);
    axisSign = frameSpecifications(frameIndex, 3:4);
    [frame.AxisOrder, frame.AxisSign] = deal(axisOrder, axisSign);
    seed_uv_deg = seed.position_deg(:, axisOrder) .* axisSign;
    initial_uv_deg = initialState.position_deg(:, axisOrder) .* axisSign;
    goal_uv_deg = goalState.position_deg(:, axisOrder) .* axisSign;
    vertices_uv_deg = occupiedShape.Vertices(:, axisOrder) .* axisSign;
    shape_uv = polyshape(vertices_uv_deg(:, 1), vertices_uv_deg(:, 2), "Simplify", false);
    [detector, reason] = detectCavity(seed_uv_deg, shape_uv, ...
        initial_uv_deg, goal_uv_deg, geometryTolerance_deg, clearanceRadius_deg);
    frame.TerminationReason = reason;
    frame.Supports_deg = [detector.Left_deg, detector.Right_deg, ...
        detector.Top_deg, detector.Bottom_deg, detector.Join_deg];
    frame.ContainmentResidual_deg2 = detector.Residual_deg2;
    if detector.Passed
        frameRadius_deg = clearanceRadius_deg;
        for certificateAttemptIndex = 1:2
            [trial, reason] = createSchedule(detector, axisOrder, axisSign, ...
                frameRadius_deg, initialState, goalState, limits, options);
            frame.TerminationReason = reason;
            if ~isempty(fieldnames(trial.Polynomial))
                certificate = createPlaneCertificate(trial.Polynomial, occupiedShape, options);
                trial.PlaneCertificate = certificate;
                frame.MinimumPlaneGap_deg = certificate.MinimumSignedGap_deg;
                if certificate.Passed
                    frame.TerminationReason = "candidateCertified";
                    if selectedFrameIndex == 0 || trial.ArrivalTime_s < bestCandidate.ArrivalTime_s
                        bestCandidate = trial;
                        selectedFrameIndex = frameIndex;
                    end
                    break;
                end
                frame.TerminationReason = "planeCertificateFailed";
                gapDeficit_deg = certificate.RequiredGap_deg - certificate.MinimumSignedGap_deg;
                frameRadius_deg = frameRadius_deg + max(0, gapDeficit_deg) + ...
                    (certificate.RequiredGap_deg - options.CollisionClearanceTolerance_deg) / 32;
            end
        end
    end
    diagnostics.Frames(frameIndex) = frame;
end

%% Section 3: Select And Independently Validate One Candidate

if selectedFrameIndex == 0
    [candidate, diagnostics] = reject(candidate, diagnostics, "noCertifiedCavityCandidate", ...
        "No frame produced a certified motion; inspect Frames for the cause.");
    return;
end
candidate = bestCandidate;
diagnostics.SelectedFrameIndex = selectedFrameIndex;
candidate.OptimizerFeasible = true;
[candidate.TerminationReason, diagnostics.TerminationReason] = ...
    deal("candidateCertifiedPendingValidation");
end

%% Section 4: Local Functions

function [detector, reason] = detectCavity(seed_deg, occupiedShape, ...
        initial_deg, goal_deg, geometryTolerance_deg, clearance_deg)
% Extract an exposed top support and prove contained side/floor rectangles.
detector = struct("Passed", false, "Left_deg", NaN, "Right_deg", NaN, ...
    "Top_deg", NaN, "Bottom_deg", NaN, "Join_deg", NaN, "Residual_deg2", Inf);
reason = "seedPatternMissing";
delta_deg = diff(seed_deg, 1, 1);
edgeLength_deg = vecnorm(delta_deg, 2, 2);
directionTolerance_deg = max(geometryTolerance_deg, 1e-8 * max(1, max(edgeLength_deg)));
priorMaximum_deg = cummax(seed_deg(:, 2));
futureMinimumFirst_deg = flip(cummin(flip(seed_deg(:, 1))));
futureMinimumSecond_deg = flip(cummin(flip(seed_deg(:, 2))));
edgeHeight_deg = mean([seed_deg(1:end - 1, 2), seed_deg(2:end, 2)], 2);
seedPattern = delta_deg(:, 1) > directionTolerance_deg & ...
    abs(delta_deg(:, 2)) <= directionTolerance_deg & ...
    priorMaximum_deg(2:end) >= initial_deg(2) + directionTolerance_deg & ...
    futureMinimumSecond_deg(2:end) <= edgeHeight_deg - directionTolerance_deg & ...
    futureMinimumFirst_deg(2:end) <= seed_deg(2:end, 1) - directionTolerance_deg;
candidateIndices = find(seedPattern);
if isempty(candidateIndices)
    return;
end
[~, localIndex] = max(delta_deg(candidateIndices, 1));
topSeedIndex = candidateIndices(localIndex);
[edgeStart_deg, edgeEnd_deg] = ...
    obstacleAvoidance.geometry.boundaryToEdges(occupiedShape, geometryTolerance_deg);
edgeDelta_deg = edgeEnd_deg - edgeStart_deg;
horizontal = abs(edgeDelta_deg(:, 2)) <= geometryTolerance_deg & ...
    abs(edgeDelta_deg(:, 1)) > geometryTolerance_deg;
seedMinimum_deg = min(seed_deg(topSeedIndex:topSeedIndex + 1, 1));
seedMaximum_deg = max(seed_deg(topSeedIndex:topSeedIndex + 1, 1));
seedHeight_deg = mean(seed_deg(topSeedIndex:topSeedIndex + 1, 2));
probe_deg = max(100 * geometryTolerance_deg, 1e-7 * max(1, max(edgeLength_deg)));
faceIndices = find(horizontal);
faceMinimum_deg = min(edgeStart_deg(faceIndices, 1), edgeEnd_deg(faceIndices, 1));
faceMaximum_deg = max(edgeStart_deg(faceIndices, 1), edgeEnd_deg(faceIndices, 1));
faceHeight_deg = mean([edgeStart_deg(faceIndices, 2), edgeEnd_deg(faceIndices, 2)], 2);
midpoint_deg = (edgeStart_deg(faceIndices, :) + edgeEnd_deg(faceIndices, :)) / 2;
overlap_deg = min(faceMaximum_deg, seedMaximum_deg) - max(faceMinimum_deg, seedMinimum_deg);
isTop = isinterior(occupiedShape, midpoint_deg(:, 1), midpoint_deg(:, 2) - probe_deg) & ...
    ~isinterior(occupiedShape, midpoint_deg(:, 1), midpoint_deg(:, 2) + probe_deg) & ...
    seedHeight_deg >= faceHeight_deg - geometryTolerance_deg;
overlap_deg(~isTop) = -Inf;
[bestOverlap_deg, localIndex] = max(overlap_deg);
if isempty(bestOverlap_deg) || bestOverlap_deg <= geometryTolerance_deg
    reason = "topSupportNotFound";
    return;
end
faceIndex = faceIndices(localIndex);
left_deg = min(edgeStart_deg(faceIndex, 1), edgeEnd_deg(faceIndex, 1));
right_deg = max(edgeStart_deg(faceIndex, 1), edgeEnd_deg(faceIndex, 1));
top_deg = mean([edgeStart_deg(faceIndex, 2), edgeEnd_deg(faceIndex, 2)]);
vertices_deg = occupiedShape.Vertices(all(isfinite(occupiedShape.Vertices), 2), :);
bottom_deg = min(vertices_deg(:, 2));
extent_deg = min(right_deg - left_deg, top_deg - bottom_deg);
inset_deg = max(10 * clearance_deg, 1e-7 * extent_deg);
inner = [left_deg + inset_deg, right_deg - inset_deg, bottom_deg + inset_deg, top_deg - inset_deg];
floorLeft_deg = min(initial_deg(1), goal_deg(1)) + inset_deg;
levels_deg = unique(vertices_deg(:, 2));
levels_deg = levels_deg(levels_deg > inner(3) & levels_deg < inner(4));
areaReserve_deg2 = 2 ^ 22 * eps(max(1, area(occupiedShape)));
join_deg = NaN;
residual_deg2 = Inf;
for level_deg = reshape(levels_deg, 1, [])
    if floorLeft_deg >= inner(2)
        continue;
    end
    side = polyshape(inner([1 2 2 1]), [level_deg level_deg inner(4) inner(4)]);
    floor = polyshape([floorLeft_deg inner(2) inner(2) floorLeft_deg], ...
        [inner(3) inner(3) level_deg level_deg]);
    trialResidual_deg2 = area(subtract(side, occupiedShape)) + area(subtract(floor, occupiedShape));
    if trialResidual_deg2 < residual_deg2
        residual_deg2 = trialResidual_deg2;
        join_deg = level_deg;
    end
end
if ~isfinite(join_deg) || residual_deg2 > areaReserve_deg2
    reason = "cavityContainmentFailed";
    return;
end
ordered = initial_deg(1) < inner(1) && initial_deg(2) > join_deg && ...
    initial_deg(2) < inner(4) && goal_deg(1) < inner(2) && goal_deg(2) < inner(3);
if ~ordered
    reason = "endpointCavityOrderingFailed";
    return;
end
detector = struct("Passed", true, "Left_deg", left_deg, "Right_deg", right_deg, ...
    "Top_deg", top_deg, "Bottom_deg", bottom_deg, "Join_deg", join_deg, ...
    "Residual_deg2", residual_deg2);
reason = "structureDetected";
end

function [candidate, reason] = createSchedule(detector, axisOrder, ...
        axisSign, radius_deg, initialState, goalState, limits, options)
% Construct the saturated tangent event word and exact cubic polynomial.
candidate = candidateTemplate(initialState, options);
reason = "closedFormPredicateFailed";
p0 = initialState.position_deg(:, axisOrder) .* axisSign;
pg = goalState.position_deg(:, axisOrder) .* axisSign;
V = double(limits.maxVelocity_deg_s(axisOrder));
A = double(limits.maxAcceleration_deg_s2(axisOrder));
J = double(limits.maxJerk_deg_s3(axisOrder));
Vu = V(1); Au = A(1); Ju = J(1);
Vv = V(2); Av = A(2); Jv = J(2);
baseTopTime_s = (detector.Right_deg - detector.Left_deg) / Vu;
tangentEquation = @(x) x / sqrt(1 - x ^ 2) - ...
    Av * (baseTopTime_s + 2 * radius_deg * x / Vu) / (2 * Vu);
topFraction = fzero(tangentEquation, [0, 1 - sqrt(eps)]);
topLateral_deg = radius_deg * topFraction;
topVertical_deg = radius_deg * sqrt(1 - topFraction ^ 2);
bottomLateral_deg = radius_deg * Vv / hypot(Vu, Vv);
bottomVertical_deg = radius_deg * Vu / hypot(Vu, Vv);
left_deg = detector.Left_deg - topLateral_deg;
rightTop_deg = detector.Right_deg + topLateral_deg;
rightBottom_deg = detector.Right_deg + bottomLateral_deg;
high_deg = detector.Top_deg + topVertical_deg;
low_deg = detector.Bottom_deg - bottomVertical_deg;
tauU_s = Au / Ju; holdU_s = Vu / Au - tauU_s;
transitionU_s = 2 * tauU_s + holdU_s;
distanceU_deg = Vu * transitionU_s / 2;
tauV_s = Av / Jv; holdV_s = Vv / Av - tauV_s;
transitionV_s = 2 * tauV_s + holdV_s;
distanceV_deg = Vv * transitionV_s / 2;
topTime_s = (rightTop_deg - left_deg) / Vu;
verticalHold_s = (Vv - Av * tauV_s / 2 - Av * topTime_s / 2) / Av;
rampDistance_deg = Vv * tauV_s - Jv * tauV_s ^ 3 / 6;
holdDistance_deg = (Vv - Av * tauV_s / 2) * verticalHold_s - Av * verticalHold_s ^ 2 / 2;
cruiseUp_s = (high_deg - p0(2) - distanceV_deg - rampDistance_deg - holdDistance_deg) / Vv;
timeAtLeft_s = transitionV_s + cruiseUp_s + tauV_s + verticalHold_s;
cruiseToLeft_s = (left_deg - p0(1) - distanceU_deg) / Vu;
startDelay_s = timeAtLeft_s - transitionU_s - cruiseToLeft_s;
cruiseDown_s = (high_deg - low_deg - holdDistance_deg - rampDistance_deg) / Vv;
descentTime_s = verticalHold_s + tauV_s + cruiseDown_s;
reversalHold_s = 2 * Vu / Au - tauU_s;
paddingTotal_s = descentTime_s - 2 * tauU_s - reversalHold_s;
paddingDifference_s = (rightBottom_deg - rightTop_deg) / Vu;
positivePadding_s = (paddingTotal_s + paddingDifference_s) / 2;
negativePadding_s = (paddingTotal_s - paddingDifference_s) / 2;
cruiseToGoal_s = (rightBottom_deg - pg(1) - distanceU_deg) / Vu;
horizontalSuffix_s = cruiseToGoal_s + transitionU_s;
verticalStop_deg = low_deg - distanceV_deg;
if pg(2) <= verticalStop_deg
    extraCruise_s = (verticalStop_deg - pg(2)) / Vv;
    returnDuration_s = zeros(0, 1);
    returnJerk_deg_s3 = zeros(0, 1);
else
    extraCruise_s = 0;
    returnDistance_deg = pg(2) - verticalStop_deg;
    returnHold_s = max(0, 0.5 * (sqrt(tauV_s ^ 2 + 4 * returnDistance_deg / Av) - 3 * tauV_s));
    returnTau_s = tauV_s * (returnHold_s > 0) + ...
        nthroot(returnDistance_deg / (2 * Jv), 3) * (returnHold_s == 0);
    if Jv * returnTau_s * (returnTau_s + returnHold_s) > Vv
        reason = "unsupportedReturnProfileBranch";
        return;
    end
    returnDuration_s = [returnTau_s; returnHold_s; returnTau_s; 0; ...
        returnTau_s; returnHold_s; returnTau_s];
    returnJerk_deg_s3 = Jv * [1; 0; -1; 0; -1; 0; 1];
end
goalWait_s = horizontalSuffix_s - extraCruise_s - transitionV_s - sum(returnDuration_s);
predicate = [verticalHold_s, cruiseUp_s, cruiseToLeft_s, startDelay_s, ...
    cruiseDown_s, positivePadding_s, negativePadding_s, cruiseToGoal_s, goalWait_s];
predicateTolerance_s = 1e-11 * max([1, abs(predicate)]);
if any(predicate < -predicateTolerance_s)
    return;
end
predicate(abs(predicate) <= predicateTolerance_s) = 0;
[verticalHold_s, cruiseUp_s, cruiseToLeft_s, startDelay_s, cruiseDown_s, ...
    positivePadding_s, negativePadding_s, cruiseToGoal_s, goalWait_s] = ...
    deal(predicate(1), predicate(2), predicate(3), predicate(4), predicate(5), ...
    predicate(6), predicate(7), predicate(8), predicate(9));
uDuration_s = [startDelay_s; tauU_s; holdU_s; tauU_s; cruiseToLeft_s; ...
    topTime_s; positivePadding_s; tauU_s; reversalHold_s; tauU_s; ...
    negativePadding_s; cruiseToGoal_s; tauU_s; holdU_s; tauU_s];
uJerk_deg_s3 = [0; Ju; 0; -Ju; 0; 0; 0; -Ju; 0; Ju; 0; 0; Ju; 0; -Ju];
vDuration_s = [tauV_s; holdV_s; tauV_s; cruiseUp_s; tauV_s; ...
    verticalHold_s; topTime_s; verticalHold_s; tauV_s; cruiseDown_s; ...
    extraCruise_s; tauV_s; holdV_s; tauV_s; returnDuration_s; goalWait_s];
vJerk_deg_s3 = [Jv; 0; -Jv; 0; -Jv; 0; 0; 0; Jv; 0; 0; Jv; 0; -Jv; returnJerk_deg_s3; 0];
uBreak_s = [0; cumsum(uDuration_s)];
vBreak_s = [0; cumsum(vDuration_s)];
finalDuration_s = uBreak_s(end);
if abs(vBreak_s(end) - finalDuration_s) > 1e-9 * max(1, finalDuration_s)
    reason = "axisDurationMismatch";
    return;
end
breakTime_s = unique([uBreak_s; vBreak_s; finalDuration_s]);
breakTime_s = breakTime_s([true; diff(breakTime_s) > 1e-12]);
for subdivisionIndex = 1:4
    breakTime_s = sort([breakTime_s; mean([breakTime_s(1:end - 1), breakTime_s(2:end)], 2)]);
end
midpoint_s = mean([breakTime_s(1:end - 1), breakTime_s(2:end)], 2);
uPhaseIndex = 1 + sum(midpoint_s >= uBreak_s(2:end).', 2);
vPhaseIndex = 1 + sum(midpoint_s >= vBreak_s(2:end).', 2);
segmentJerk_deg_s3 = zeros(numel(midpoint_s), 2);
segmentJerk_deg_s3(:, axisOrder(1)) = axisSign(1) * uJerk_deg_s3(uPhaseIndex);
segmentJerk_deg_s3(:, axisOrder(2)) = axisSign(2) * vJerk_deg_s3(vPhaseIndex);
[candidate, terminalState] = ...
    bmtpEngine.createMotionRecord(candidate, ...
    initialState, breakTime_s, segmentJerk_deg_s3, ...
    options.SampleTime_s, "orthogonalCavityClosedForm");
terminalResidual = max(abs([terminalState.position_deg - goalState.position_deg, ...
    terminalState.velocity_deg_s - goalState.velocity_deg_s, ...
    terminalState.acceleration_deg_s2 - goalState.acceleration_deg_s2]));
if terminalResidual > max(options.ConstraintTolerance, 1e-9)
    reason = "terminalResidualFailed";
    return;
end
if initialState.time_s + finalDuration_s > goalState.time_s + options.ConstraintTolerance
    reason = "timeWindowInfeasible";
    return;
end
candidate.MaximumConstraintViolation = terminalResidual;
reason = "closedFormCandidate";
end

function certificate = createPlaneCertificate(polynomial, occupiedShape, options)
% Create complete cubic-hull/convex-region SAT plane witnesses.
regions = obstacleAvoidance.geometry.convexPolygonRegions(occupiedShape);
segmentCount = polynomial.SegmentCount;
regionCount = numel(regions);
emptyPlane = struct("Active", false, "Normal", zeros(2, 2), ...
    "Offset_deg", zeros(1, 2), "SignedGap_deg", NaN, ...
    "Verified", false, "ExitFlag", NaN);
planes = repmat(emptyPlane, segmentCount, regionCount);
regions_deg = cell(regionCount, 1);
powerToBernstein = [1 0 0 0; 1 1/3 0 0; 1 2/3 1/3 0; 1 1 1 1];
controls_deg = zeros(segmentCount, 4, 2);
for segmentIndex = 1:segmentCount
    power = reshape(polynomial.positionPower_deg(segmentIndex, :, :), 2, 4).';
    controls_deg(segmentIndex, :, :) = reshape(powerToBernstein * power, 1, 4, 2);
end
[~, ~, roundoffReserve_deg] = bmtpEngine.createCoordinateTolerances( ...
    occupiedShape.Vertices, controls_deg);
normalNormLimit = 1 + 2 ^ 20 * eps;
requiredGap_deg = normalNormLimit * ...
    options.CollisionClearanceTolerance_deg + 2 * roundoffReserve_deg;
minimumSignedGap_deg = Inf;
passed = regionCount > 0;
pairIndices = [1 2; 1 3; 1 4; 2 3; 2 4; 3 4];
for segmentIndex = 1:segmentCount
    trajectory_deg = squeeze(controls_deg(segmentIndex, :, :));
    for regionIndex = 1:regionCount
        vertices_deg = regions(regionIndex).Vertices;
        vertices_deg = vertices_deg(all(isfinite(vertices_deg), 2), :);
        regions_deg{regionIndex} = vertices_deg;
        obstacleEdges_deg = diff([vertices_deg; vertices_deg(1, :)], 1, 1);
        trajectoryEdges_deg = trajectory_deg(pairIndices(:, 2), :) - ...
            trajectory_deg(pairIndices(:, 1), :);
        edges_deg = [obstacleEdges_deg; trajectoryEdges_deg];
        axes = [-edges_deg(:, 2), edges_deg(:, 1)];
        axisNorm = vecnorm(axes, 2, 2);
        axes = axes(axisNorm > 0, :) ./ axisNorm(axisNorm > 0);
        obstacleProjection_deg = vertices_deg * axes.';
        trajectoryProjection_deg = trajectory_deg * axes.';
        forwardGap_deg = min(obstacleProjection_deg) - max(trajectoryProjection_deg);
        reverseGap_deg = min(trajectoryProjection_deg) - max(obstacleProjection_deg);
        [bestGap_deg, signedAxisIndex] = max([forwardGap_deg, reverseGap_deg]);
        axisCount = size(axes, 1);
        direction = 1 - 2 * (signedAxisIndex > axisCount);
        bestNormal = direction * axes(mod(signedAxisIndex - 1, axisCount) + 1, :);
        maximumTrajectory_deg = max(trajectory_deg * bestNormal.');
        minimumObstacle_deg = min(vertices_deg * bestNormal.');
        offset_deg = -0.5 * (maximumTrajectory_deg + minimumObstacle_deg);
        maximumNormalNorm = norm(bestNormal);
        certifiedClearance_deg = ...
            (bestGap_deg - 2 * roundoffReserve_deg) / ...
            max(maximumNormalNorm, realmin);
        pairPassed = certifiedClearance_deg >= ...
            options.CollisionClearanceTolerance_deg && ...
            maximumNormalNorm <= normalNormLimit && ...
            minimumObstacle_deg + offset_deg >= roundoffReserve_deg && ...
            maximumTrajectory_deg + offset_deg <= -roundoffReserve_deg;
        planes(segmentIndex, regionIndex) = struct( ...
            "Active", true, "Normal", repmat(bestNormal, 2, 1), ...
            "Offset_deg", [offset_deg offset_deg], ...
            "SignedGap_deg", bestGap_deg, "Verified", pairPassed, ...
            "ExitFlag", NaN);
        minimumSignedGap_deg = min(minimumSignedGap_deg, bestGap_deg);
        passed = passed && pairPassed && bestGap_deg >= requiredGap_deg;
    end
end
allPairCount = segmentCount * regionCount;
verifiedPairCount = nnz(reshape([planes.Verified], [], 1));
coverage = struct("Passed", regionCount > 0, ...
    "ExactRegionCount", regionCount, "SolverRegionCount", regionCount);
certificate = struct("Kind", "staticDegreeOne", ...
    "Passed", passed && verifiedPairCount == allPairCount, ...
    "ExactRegionCount", regionCount, "SolverRegionCount", regionCount, ...
    "Regions_deg", {regions_deg}, "Planes", planes, ...
    "RequiredGap_deg", requiredGap_deg, ...
    "RoundoffReserve_deg", roundoffReserve_deg, ...
    "MinimumSignedGap_deg", minimumSignedGap_deg, ...
    "CoveragePassed", coverage.Passed, "Coverage", coverage, ...
    "AllPairCount", allPairCount, ...
    "VerifiedPairCount", verifiedPairCount, "ReusedPairCount", 0, ...
    "AnalyticPairCount", verifiedPairCount, "ConicPairCount", 0);
end

function candidate = candidateTemplate(initialState, options)
% Define the stable candidate shape consumed by the public validator.
candidate = bmtpEngine.createMotionRecord( ...
    struct(), initialState, [], [], options.SampleTime_s, ...
    "orthogonalCavityClosedForm");
candidate.Message = "Candidate awaiting independent validation.";
candidate.MotionLength_deg = Inf;
candidate.IntegratedSquaredJerk_deg2_s5 = Inf;
end

function [candidate, diagnostics] = reject(candidate, diagnostics, reason, message)
% Preserve one expected rejection in both stable output records.
[candidate.TerminationReason, diagnostics.TerminationReason] = deal(reason);
candidate.Message = message;
end
