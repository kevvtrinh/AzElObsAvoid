function [certificate, preparedObstacles] = certifyTimedOpeningRequestLowerBound( ...
        openingDiagnostics, obstacles, initialState, goalState, limits, options)
%% Section 0: Header & Readme
% SYNTAX
%   certificate = ...
%       obstacleAvoidance.planner.certifyTimedOpeningRequestLowerBound()
%   certificate = ...
%       obstacleAvoidance.planner.certifyTimedOpeningRequestLowerBound( ...
%       openingDiagnostics, obstacles, initialState, goalState, limits, options)
%**************************************************************************
% PURPOSE
%   - Certify a request-wide physical duration lower bound for a stationary
%     orthogonal cavity that opens at one exact obstacle source time.
%**************************************************************************
% INPUTS
%   - openingDiagnostics (scalar struct)
%       Validated timed-opening metadata. Supplying NaN for the event time,
%       blocking face, and clearance requests authoritative source derivation.
%   - obstacles (canonical or prepared obstacle struct array)
%       Complete protected history; its preparation cache is never trusted.
%   - initialState, goalState (normalized scalar structs)
%       Fixed axis-aligned rest-to-rest request and finite planning horizon.
%   - limits (normalized scalar struct)
%       Positive componentwise velocity, acceleration, and jerk limits.
%   - options (resolved scalar struct)
%       Earliest-arrival, unwrapped motion, and collision-clearance policy.
%**************************************************************************
% OUTPUTS
%   - certificate (scalar struct)
%       Stable success-or-rejection record. LowerBound_s is a duration, not
%       an absolute time. Invalid record structure throws; an unsupported or
%       geometrically ambiguous request returns Passed = false.
%   - preparedObstacles (canonical prepared obstacle struct array)
%       Reusable obstacle data after authoritative source preparation.
%**************************************************************************
% UNITS
%   - Position and clearance are degrees. Time is seconds. Derivatives use
%     degrees per second and corresponding higher powers.
%**************************************************************************

%% Section 1: Validate The Fixed Rest Request

certificate = certificateTemplate();
preparedObstacles = [];
if nargin == 0
    return;
end
if nargin ~= 6
    error("certifyTimedOpeningRequestLowerBound:InputCount", ...
        "Use either the zero-input template call or all six documented inputs.");
end
recordsValid = isstruct(openingDiagnostics) && isscalar(openingDiagnostics) && ...
    isstruct(initialState) && isscalar(initialState) && ...
    isstruct(goalState) && isscalar(goalState) && isstruct(limits) && ...
    isscalar(limits) && isstruct(options) && isscalar(options);
diagnosticFields = {'EventTime_s', 'BlockingFaceProgress_deg', ...
    'ClearanceTolerance_deg', 'EventCompatibleLowerBound_s'};
stateFields = {'time_s', 'position_deg', 'velocity_deg_s', 'acceleration_deg_s2'};
limitFields = {'maxVelocity_deg_s', 'maxAcceleration_deg_s2', 'maxJerk_deg_s3'};
optionFields = {'GoalTimeMode', 'AllowAzimuthWrapping', ...
    'CollisionClearanceTolerance_deg'};
fieldsValid = recordsValid && all(isfield(openingDiagnostics, diagnosticFields)) && ...
    all(isfield(initialState, stateFields)) && all(isfield(goalState, stateFields)) && ...
    all(isfield(limits, limitFields)) && all(isfield(options, optionFields));
if ~fieldsValid
    error("certifyTimedOpeningRequestLowerBound:InvalidRecords", ...
        "Normalized timed-opening inputs and diagnostics are required.");
end
try
metadata = double([openingDiagnostics.EventTime_s, ...
    openingDiagnostics.BlockingFaceProgress_deg, ...
    openingDiagnostics.ClearanceTolerance_deg]);
deriveMetadata = all(isnan(metadata));
requireProof(deriveMetadata || all(isfinite(metadata)), "eventMetadataMismatch");
hasMovingGoal = isfield(goalState, "targetTime_s") && ~isempty(goalState.targetTime_s) || ...
    isfield(goalState, "targetPosition_deg") && ~isempty(goalState.targetPosition_deg);
derivatives = [initialState.velocity_deg_s, goalState.velocity_deg_s, ...
    initialState.acceleration_deg_s2, goalState.acceleration_deg_s2];
requireProof(~hasMovingGoal, "unsupportedMovingGoal");
requireProof(string(options.GoalTimeMode) == "earliestArrival" && ...
    ~options.AllowAzimuthWrapping, "unsupportedRequestPolicy");
requireProof(all(isfinite(derivatives)) && all(derivatives == 0), ...
    "unsupportedEndpointState");

%% Section 2: Recover The Exact Source-Time Event

preparedObstacles = obstacleAvoidance.obstacles.prepareDynamic( ...
    obstacleAvoidance.obstacles.combineObstacles(obstacles));
requireProof(isscalar(preparedObstacles), "unsupportedObstacleCount");
obstacle = preparedObstacles(1);
time_s = double(obstacle.time_s(:));
preparation = obstacle.InternalPreparation;
mismatchIndex = find(~preparation.MatchingTopology);
requireProof(isscalar(mismatchIndex), "singleOpeningEventNotFound");
mismatchIndex = mismatchIndex(1);
eventSourceIndex = mismatchIndex + 1;
eventTime_s = time_s(eventSourceIndex);
certificate.EventTime_s = eventTime_s;
certificate.EventSourceIndex = eventSourceIndex;
historyCovered = time_s(1) <= initialState.time_s && time_s(end) >= goalState.time_s && ...
    initialState.time_s < eventTime_s && eventTime_s < goalState.time_s;
otherIntervals = [1:mismatchIndex - 1, mismatchIndex + 1:numel(preparation.MatchingTopology)];
stationary = all(preparation.MatchingTopology(otherIntervals)) && ...
    all(preparation.IntervalSpeedBound_deg_s(otherIntervals) == 0);
requireProof(historyCovered, "obstacleHistoryTruncated");
requireProof(stationary, "nonstationaryObstacleHistory");
[~, eventGeometry] = obstacleAvoidance.obstacles.shapeAtTime(obstacle, eventTime_s, true);
exactEvent = eventGeometry.LowerSampleIndex == eventSourceIndex && ...
    eventGeometry.UpperSampleIndex == eventSourceIndex;
certificate.ExactEventUsesUpperSource = exactEvent;
metadataMatches = deriveMetadata || openingDiagnostics.EventTime_s == eventTime_s && ...
    openingDiagnostics.ClearanceTolerance_deg == options.CollisionClearanceTolerance_deg;
requireProof(exactEvent, "exactEventSourceSemanticsFailed");
requireProof(metadataMatches, "eventMetadataMismatch");

%% Section 3: Create Guarded Authoritative Witnesses

source_deg = [initialState.position_deg(:); goalState.position_deg(:); ...
    preparation.HistoryBounds_deg(:)];
worldScale_deg = bmtpEngine.createCoordinateTolerances(source_deg);
guard_deg = 2 ^ 18 * eps(worldScale_deg);
clearance_deg = double(options.CollisionClearanceTolerance_deg);
certificate.CoordinateGuard_deg = guard_deg;
certificate.ClearanceTolerance_deg = clearance_deg;
% The dyadic guard localizes predicates; gamma_n below, rather than this
% spatial guard, encloses each stated floating expression.
requireProof(outward(clearance_deg - 32 * guard_deg, ...
    abs(clearance_deg) + 32 * guard_deg, 1) > 0, "coordinateConditioningFailed");
delta_deg = double(goalState.position_deg - initialState.position_deg);
motionAxis = find(abs(delta_deg) > 32 * guard_deg);
requireProof(isscalar(motionAxis), "unsupportedEndpointGeometry");
motionAxis = motionAxis(1);
lateralAxis = 3 - motionAxis;
motionSign = sign(delta_deg(motionAxis));
totalProgress_deg = motionSign * delta_deg(motionAxis);
requireProof(abs(delta_deg(lateralAxis)) <= guard_deg && ...
    totalProgress_deg > 32 * guard_deg, "unsupportedEndpointGeometry");
axisOrder = [lateralAxis, motionAxis];
axisSign = [1, motionSign];
origin_uv_deg = initialState.position_deg(axisOrder) .* axisSign;
original_deg = [obstacle.originalAz_deg{mismatchIndex}(:), ...
    obstacle.originalEl_deg{mismatchIndex}(:)];
[~, originalPassed, original_uv_deg] = ...
    obstacleAvoidance.planner.certifyGuardedRectangleContainment( ...
    zeros(0, 4), original_deg, axisOrder, axisSign, origin_uv_deg, guard_deg, true);
certificate.OriginalOrthogonalRingPassed = originalPassed;
uSupport_deg = unique(original_uv_deg(:, 1));
pSupport_deg = unique(original_uv_deg(:, 2));
supportedCavity = originalPassed && size(original_uv_deg, 1) == 8 && ...
    numel(uSupport_deg) == 4 && numel(pSupport_deg) == 3 && ...
    min(diff(uSupport_deg)) > 32 * guard_deg && min(diff(pSupport_deg)) > 32 * guard_deg;
requireProof(originalPassed, "originalRingNotGuardedlySimple");
requireProof(supportedCavity, "unsupportedOriginalOrthogonalCavity");
[~, closedPassed, closed_uv_deg] = ...
    obstacleAvoidance.planner.certifyGuardedRectangleContainment( ...
    zeros(0, 4), preparation.SampleShapes{mismatchIndex}, axisOrder, axisSign, ...
    origin_uv_deg, guard_deg, false);
requireProof(closedPassed, "authoritativeBoundaryNotGuardedlySimple");
edgeEnd_uv_deg = closed_uv_deg([2:end 1], :);
face = closed_uv_deg(:, 2) == edgeEnd_uv_deg(:, 2) & ...
    min(closed_uv_deg(:, 1), edgeEnd_uv_deg(:, 1)) < -32 * guard_deg & ...
    max(closed_uv_deg(:, 1), edgeEnd_uv_deg(:, 1)) > 32 * guard_deg & ...
    closed_uv_deg(:, 2) > 32 * guard_deg & ...
    closed_uv_deg(:, 2) < totalProgress_deg - 32 * guard_deg;
faceProgress_deg = sort(closed_uv_deg(face, 2));
requireProof(~isempty(faceProgress_deg), "protectedBlockingFaceNotFound");
blockingFace_deg = faceProgress_deg(1);
certificate.BlockingFaceProgress_deg = blockingFace_deg;
if ~deriveMetadata
    faceResidual_deg = 16 * guard_deg - ...
        abs(double(openingDiagnostics.BlockingFaceProgress_deg) - blockingFace_deg);
    faceScale_deg = 16 * guard_deg + ...
        abs(openingDiagnostics.BlockingFaceProgress_deg) + abs(blockingFace_deg);
    requireProof(outward(faceResidual_deg, faceScale_deg, 2) > 0, ...
        "blockingFaceMetadataMismatch");
end
inward_deg = 8 * guard_deg;
bottomTop_deg = blockingFace_deg + inward_deg;
witness_uv_deg = [uSupport_deg(1) + inward_deg, uSupport_deg(2) - inward_deg, ...
    pSupport_deg(1) + inward_deg, pSupport_deg(3) - inward_deg; ...
    uSupport_deg(3) + inward_deg, uSupport_deg(4) - inward_deg, ...
    pSupport_deg(1) + inward_deg, pSupport_deg(3) - inward_deg; ...
    uSupport_deg(1) + 12 * guard_deg, uSupport_deg(4) - 12 * guard_deg, ...
    bottomTop_deg, pSupport_deg(2) - inward_deg];
certificate.WitnessBounds_uv_deg = witness_uv_deg;
geometryResidual_deg = min([witness_uv_deg(:, 2) - witness_uv_deg(:, 1); ...
    witness_uv_deg(:, 4) - witness_uv_deg(:, 3); -witness_uv_deg(1, 2); ...
    witness_uv_deg(2, 1); -witness_uv_deg(1, 3); bottomTop_deg; ...
    totalProgress_deg - witness_uv_deg(3, 4); ...
    witness_uv_deg(1, 2) - witness_uv_deg(3, 1); ...
    witness_uv_deg(3, 2) - witness_uv_deg(2, 1)]);
requireProof(outward(geometryResidual_deg, 2 * worldScale_deg, 1) > 0, ...
    "witnessGeometryOrderFailed");

%% Section 4: Prove Occupancy Over Every Pre-Event Interval

[sampleContained, intervalContained, sampleSimple, intervalSimple] = ...
    deal(false(mismatchIndex, 1));
intervalShapes = preparation.SampleShapes(1:mismatchIndex);
intervalShapes{end} = preparation.IntervalUnionShapes{mismatchIndex};
% Zero source speed makes every earlier matching interval equal its sample.
% shapeAtTime uses the authoritative conservative union on the sole mismatched
% interval until the exact upper-source event checked above.
for intervalIndex = 1:mismatchIndex
    [sampleContainment, sampleSimple(intervalIndex)] = ...
        obstacleAvoidance.planner.certifyGuardedRectangleContainment( ...
        witness_uv_deg, preparation.SampleShapes{intervalIndex}, axisOrder, ...
        axisSign, origin_uv_deg, guard_deg, false);
    [intervalContainment, intervalSimple(intervalIndex)] = ...
        obstacleAvoidance.planner.certifyGuardedRectangleContainment( ...
        witness_uv_deg, intervalShapes{intervalIndex}, axisOrder, axisSign, ...
        origin_uv_deg, guard_deg, false);
    sampleContained(intervalIndex) = all(sampleContainment);
    intervalContained(intervalIndex) = all(intervalContainment);
end
certificate.SourceSampleContainmentPassed = sampleContained;
certificate.PreEventIntervalContainmentPassed = intervalContained;
certificate.SourceSampleSimpleRingPassed = sampleSimple;
certificate.PreEventIntervalSimpleRingPassed = intervalSimple;
requireProof(all(sampleSimple) && all(intervalSimple), ...
    "authoritativeBoundaryNotGuardedlySimple");
requireProof(all(sampleContained) && all(intervalContained), ...
    "witnessContainmentFailed");

%% Section 5: Bound Both Exhaustive Trajectory Classes

V = double(limits.maxVelocity_deg_s(motionAxis));
A = double(limits.maxAcceleration_deg_s2(motionAxis));
J = double(limits.maxJerk_deg_s3(motionAxis));
ratios_s = [V / A, A / J];
safeRange = [8 * realmin("double") ^ 0.25, realmax("double") ^ 0.25 / 8];
kinematics = [V, A, J, ratios_s];
conditioned = all(isfinite(kinematics)) && all(kinematics >= safeRange(1)) && ...
    all(kinematics <= safeRange(2));
requireProof(conditioned, "kinematicConditioningFailed");
% Switching time has two divisions and one subtraction. Distance switching
% adds the two terminal-brake products, for five operations, then subtracts.
switchTime_s = ratios_s(1) - ratios_s(2);
certificate.SwitchingTimeResidual_s = switchTime_s;
requireProof(outward(switchTime_s, sum(abs(ratios_s)), 3) > 0, ...
    "switchingTimeBranchAmbiguous");
maximumEvent_deg = -outward(-(bottomTop_deg - clearance_deg), ...
    abs(bottomTop_deg) + abs(clearance_deg), 1);
certificate.MaximumEventProgress_deg = maximumEvent_deg;
remaining_deg = outward(totalProgress_deg - maximumEvent_deg, ...
    abs(totalProgress_deg) + abs(maximumEvent_deg), 1);
brakingTime_s = sum(ratios_s);
brakingTimeLower_s = outward(brakingTime_s, sum(abs(ratios_s)), 3);
brakingDistanceUpper_deg = -outward(-0.5 * V * brakingTime_s, ...
    0.5 * abs(V * brakingTime_s), 5);
switchDistance_deg = remaining_deg - brakingDistanceUpper_deg;
certificate.SwitchingDistanceResidual_deg = switchDistance_deg;
distanceCondition = outward(switchDistance_deg, abs(remaining_deg) + ...
    abs(brakingDistanceUpper_deg), 1) > 0;
requireProof(distanceCondition, "switchingDistanceBranchAmbiguous");
% Backward from fixed terminal rest, the jerk/acceleration-saturated brake is
% the pointwise largest recoverable velocity. Its area deficit below V is
% V*(V/A + A/J)/2, so every event-side suffix is at least
% L/V + (V/A + A/J)/2 regardless of event velocity and acceleration.
rawEvent_s = eventTime_s - initialState.time_s + remaining_deg / V + ...
    0.5 * brakingTimeLower_s;
eventLower_s = outward(rawEvent_s, abs(eventTime_s) + abs(initialState.time_s) + ...
    abs(remaining_deg / V) + 0.5 * abs(brakingTimeLower_s), 5);
exteriorVariation_deg = outward(totalProgress_deg - 2 * witness_uv_deg(1, 3), ...
    abs(totalProgress_deg) + 2 * abs(witness_uv_deg(1, 3)), 2);
% A route not on the cavity side at the event crossed the guarded exterior
% witness first. Componentwise velocity total variation is therefore at least
% totalProgress_deg - 2*topExcursion_deg, giving variation/V seconds.
rawExterior_s = exteriorVariation_deg / V;
exteriorLower_s = outward(rawExterior_s, abs(exteriorVariation_deg / V), 1);
lowerBound_s = min(eventLower_s, exteriorLower_s);
finiteBounds = all(isfinite([rawEvent_s, eventLower_s, rawExterior_s, ...
    exteriorLower_s, lowerBound_s]));
requireProof(finiteBounds, "nonfiniteCertificateState");
certificate.EventSideLowerBound_s = eventLower_s;
certificate.ExteriorLowerBound_s = exteriorLower_s;
certificate.LowerBound_s = lowerBound_s;
certificate.NumericalReserve_s = max(rawEvent_s - eventLower_s, ...
    rawExterior_s - exteriorLower_s);
activeClasses = ["eventSide", "preEventExterior"];
certificate.ActiveLowerBoundClass = activeClasses(1 + (exteriorLower_s < eventLower_s));
certificate.Passed = true;
certificate.Message = "All trajectories satisfy the event-side or pre-event exterior bound.";
certificate.TerminationReason = "allRoutesLowerBoundCertified";
catch proofFailure
    if string(proofFailure.identifier) ~= "certifyTimedOpeningRequestLowerBound:ProofRejected"
        rethrow(proofFailure);
    end
    certificate = reject(certificate, string(proofFailure.message));
end
end

%% Section 6: Local Functions

function lower = outward(value, absoluteConditionSum, operationCount)
% Return the lower endpoint from Higham's gamma_n bound for declared operations.
operationProduct = operationCount * eps(1) / 2;
gamma = operationProduct / (1 - operationProduct);
scale = max(abs(value), absoluteConditionSum);
padding = gamma * scale + 4 * eps(max(scale, realmin("double")));
lower = value - padding;
lower(~isfinite(lower)) = NaN;
end

function requireProof(assertion, reason)
% Stop an ambiguous proof branch; the public function catches this sentinel.
if ~(isscalar(assertion) && islogical(assertion) && assertion)
    error("certifyTimedOpeningRequestLowerBound:ProofRejected", "%s", reason);
end
end

function certificate = certificateTemplate()
% Define stable proof and fail-closed rejection fields.
certificate = struct("Passed", false, "Message", "Not run.", ...
    "TerminationReason", "notRun", "LowerBound_s", NaN, ...
    "EventSideLowerBound_s", NaN, "ExteriorLowerBound_s", NaN, ...
    "ActiveLowerBoundClass", "notEstablished", "EventTime_s", NaN, ...
    "EventSourceIndex", 0, "ExactEventUsesUpperSource", false, ...
    "BlockingFaceProgress_deg", NaN, "ClearanceTolerance_deg", NaN, ...
    "MaximumEventProgress_deg", NaN, ...
    "SwitchingTimeResidual_s", NaN, "SwitchingDistanceResidual_deg", NaN, ...
    "CoordinateGuard_deg", NaN, "NumericalReserve_s", NaN, ...
    "WitnessBounds_uv_deg", NaN(3, 4), "OriginalOrthogonalRingPassed", false, ...
    "SourceSampleContainmentPassed", false(0, 1), ...
    "PreEventIntervalContainmentPassed", false(0, 1), ...
    "SourceSampleSimpleRingPassed", false(0, 1), ...
    "PreEventIntervalSimpleRingPassed", false(0, 1));
end

function certificate = reject(certificate, reason)
% Preserve a named expected rejection without implying candidate invalidity.
certificate.TerminationReason = string(reason);
certificate.Message = "Request-wide opening certificate rejected: " + reason + ".";
end
