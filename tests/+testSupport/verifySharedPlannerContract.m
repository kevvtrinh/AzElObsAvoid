function verifySharedPlannerContract(testCase, method, contractName)
%% Section 0: Header & Readme
% SYNTAX
%   testSupport.verifySharedPlannerContract(testCase, method, contractName)
%**************************************************************************
% PURPOSE
%   - Execute one behavior-identical planner contract against a selected
%     method while retaining method-specific tests in their owning suites.
%**************************************************************************
% INPUTS
%   - testCase (matlab.unittest.FunctionTestCase)
%       Active function-based test case with shared fixtures in TestData.
%   - method (string scalar)
%       Either "corridor" or "hs3".
%   - contractName (string scalar)
%       Name of one registered shared contract.
%**************************************************************************
% OUTPUTS
%   - None. Verification failures are reported through testCase.
%**************************************************************************
% UNITS
%   - Individual contracts document physical units through field suffixes.
%**************************************************************************

%% Section 1: Resolve Method Adapter

method = string(method);
switch method
    case "corridor"
        adapter = struct( ...
            "FixedOptions", @() planAzElMotion());
    case "hs3"
        adapter = struct( ...
            "FixedOptions", @() planAzElMotion("hs3"));
    otherwise
        error("testSupport:UnknownPlannerMethod", ...
            "Shared planner contracts support corridor or hs3.");
end

%% Section 2: Run Named Contract

contracts = struct( ...
    "testAzimuthWrappingChangesThePhysicalRequest", ...
    @testAzimuthWrappingChangesThePhysicalRequest, ...
    "testAzimuthWrappingRejectsUnmodeledPeriodicGeometry", ...
    @testAzimuthWrappingRejectsUnmodeledPeriodicGeometry, ...
    "testBetweenNodeCollisionFailsValidation", @testBetweenNodeCollisionFailsValidation, ...
    "testBetweenNodeVelocityViolationFailsValidation", ...
    @testBetweenNodeVelocityViolationFailsValidation, ...
    "testConstantJerkPolynomialPassesIndependentDynamics", ...
    @testConstantJerkPolynomialPassesIndependentDynamics, ...
    "testDeformingObstacleUsesThePlannerPath", @testDeformingObstacleUsesThePlannerPath, ...
    "testDenseSweptEnvelopeIsConservativeAndProtectsEndpoints", ...
    @testDenseSweptEnvelopeIsConservativeAndProtectsEndpoints, ...
    "testDeterministicRepeatedRun", @testDeterministicRepeatedRun, ...
    "testEarliestGoalIsNotRejectedByHorizonOccupancy", ...
    @testEarliestGoalIsNotRejectedByHorizonOccupancy, ...
    "testEarlyPlannerFailureKeepsValidationFieldOrder", ...
    @testEarlyPlannerFailureKeepsValidationFieldOrder, ...
    "testInterceptWrapperRequiresTwoTargetSamples", ...
    @testInterceptWrapperRequiresTwoTargetSamples, ...
    "testInterceptWrapperTextOptionsMustBeScalar", @testInterceptWrapperTextOptionsMustBeScalar, ...
    "testMovingGoalHistoryRequiresTwoSamples", @testMovingGoalHistoryRequiresTwoSamples, ...
    "testMovingGoalInterpolationMethodMustBeScalar", ...
    @testMovingGoalInterpolationMethodMustBeScalar, ...
    "testObstacleActivationAtTerminalTimeFailsValidation", ...
    @testObstacleActivationAtTerminalTimeFailsValidation, ...
    "testOldWorkspaceOptionGivesMigrationError", @testOldWorkspaceOptionGivesMigrationError, ...
    "testRemovedPlanningTimeOptionGivesMigrationError", ...
    @testRemovedPlanningTimeOptionGivesMigrationError, ...
    "testSafetyMarginIsAppliedExactlyOnce", @testSafetyMarginIsAppliedExactlyOnce, ...
    "testShiftedPolynomialTimeCannotHideInitialTime", ...
    @testShiftedPolynomialTimeCannotHideInitialTime, ...
    "testStaticObstacleProducesOppositeSideSeeds", @testStaticObstacleProducesOppositeSideSeeds, ...
    "testTopologyChangeUsesAStationaryConservativeUnion", ...
    @testTopologyChangeUsesAStationaryConservativeUnion, ...
    "testTranslatedHistoryReusesExactProtectedShape", ...
    @testTranslatedHistoryReusesExactProtectedShape, ...
    "testUnrelatedPolynomialCannotValidateSampledHistory", ...
    @testUnrelatedPolynomialCannotValidateSampledHistory, ...
    "testWorkspaceIntervalsBelongToLimits", @testWorkspaceIntervalsBelongToLimits);
contractName = char(string(contractName));
if ~isfield(contracts, contractName)
    error("testSupport:UnknownPlannerContract", ...
        "Unknown shared planner contract '%s'.", contractName);
end
contract = contracts.(contractName);
contract(testCase, adapter);
end

%% Section 3: Local Contract Functions

function testAzimuthWrappingChangesThePhysicalRequest(testCase, adapter)
% Verify wrapping selects the short move and disabled wrapping keeps the long move.
initialState = testCase.TestData.Fixtures.State(0, [179 0], [0 0], [0 0]);
goalState = testCase.TestData.Fixtures.State(8, [-179 0], [0 0], [0 0]);
limits = testCase.TestData.Fixtures.PhysicalLimits([1 1], [1 1], [2 2]);
options = adapter.FixedOptions();
options.AllowAzimuthWrapping = false;
longResult = planAzElMotion([], initialState, goalState, limits, options);
options.AllowAzimuthWrapping = true;
shortResult = planAzElMotion([], initialState, goalState, limits, options);
verifyFalse(testCase, longResult.Success);
verifyTrue(testCase, shortResult.Success, shortResult.Message);
verifyEqual(testCase, shortResult.position_deg(end, 1), 181, "AbsTol", 1e-6);
verifyTrue(testCase, shortResult.Validation.AzimuthWrapPolicySatisfied);
end

function testAzimuthWrappingRejectsUnmodeledPeriodicGeometry(testCase, adapter)
% Verify wrapped obstacle requests cannot return a false physical success.
initialState = testCase.TestData.Fixtures.State(0, [179 0], [0 0], [0 0]);
goalState = testCase.TestData.Fixtures.State(8, [-179 0], [0 0], [0 0]);
limits = testCase.TestData.Fixtures.PhysicalLimits([1 1], [1 1], [2 2]);
options = adapter.FixedOptions();
options.AllowAzimuthWrapping = true;
obstacle = testCase.TestData.Fixtures.RectangleObstacle([0 8], [-180.5 -179.5 -1 1], 0);
verifyError(testCase, @() planAzElMotion( ...
    obstacle, initialState, goalState, limits, options), ...
    "planAzElMotion:UnsupportedWrappedGeometry");
end

function testBetweenNodeCollisionFailsValidation(testCase, adapter)
% Verify a collision at the segment midpoint cannot hide between samples.
initialState = testCase.TestData.Fixtures.State(0, [-1 0], [2 0], [0 0]);
goalState = testCase.TestData.Fixtures.State(1, [1 0], [2 0], [0 0]);
limits = testCase.TestData.Fixtures.PhysicalLimits([3 3], [1 1], [1 1]);
trajectory = testCase.TestData.Fixtures.LinearTrajectory(initialState, goalState);
obstacle = testCase.TestData.Fixtures.RectangleObstacle([0 1], [-0.1 0.1 -1 1], 0);
validation = validateAzElTrajectory( ...
    trajectory, obstacle, initialState, goalState, limits, ...
    adapter.FixedOptions());
verifyFalse(testCase, validation.Passed);
verifyFalse(testCase, validation.CollisionFree);
verifyLessThanOrEqual(testCase, validation.MinimumClearance_deg, 0);
end

function testBetweenNodeVelocityViolationFailsValidation(testCase, adapter)
% Verify continuous extrema detect an interior peak with clear samples.
initialState = testCase.TestData.Fixtures.State(0, [0 0], [0 0], [4 0]);
goalState = testCase.TestData.Fixtures.State(1, [2 / 3 0], [0 0], [-4 0]);
limits = testCase.TestData.Fixtures.PhysicalLimits([0.8 1], [5 5], [9 9]);
trajectory = testCase.TestData.Fixtures.InteriorVelocityPeakTrajectory();
validation = validateAzElTrajectory( ...
    trajectory, [], initialState, goalState, limits, ...
    adapter.FixedOptions());
verifyFalse(testCase, validation.Passed);
verifyFalse(testCase, validation.VelocityWithinLimits);
verifyEqual(testCase, max(abs(trajectory.velocity_deg_s), [], "all"), 0);
end

function testConstantJerkPolynomialPassesIndependentDynamics(testCase, adapter)
% Verify the third-order chain against analytic constant-jerk motion.
duration_s = 2;
initialState = testCase.TestData.Fixtures.State(0, [0 0], [0 0], [0 0]);
goalState = testCase.TestData.Fixtures.State(duration_s, [4 / 3 0], [2 0], [2 0]);
limits = testCase.TestData.Fixtures.PhysicalLimits([3 3], [3 3], [2 2]);
trajectory = testCase.TestData.Fixtures.ConstantJerkTrajectory(duration_s);
options = adapter.FixedOptions();
validation = validateAzElTrajectory( trajectory, [], initialState, goalState, limits, options);
verifyTrue(testCase, validation.Passed, validation.Message);
verifyLessThanOrEqual(testCase, validation.MaximumDynamicsResidual, 1e-12);
end

function testDeformingObstacleUsesThePlannerPath(testCase, adapter)
% Verify a deforming protected polygon uses the maintained planner.
time_s = [0; 8];
first_deg = [-1 4; 1 4; 1 6; -1 6];
second_deg = [-2 4.5; 2 4.5; 2 5.5; -2 5.5];
obstacle = makeAzElObstacleData( ...
    "deforming", time_s, ...
    {first_deg(:, 1); second_deg(:, 1)}, ...
    {first_deg(:, 2); second_deg(:, 2)}, 0.1);
initialState = testCase.TestData.Fixtures.State(0, [0 0], [0 0], [0 0]);
goalState = testCase.TestData.Fixtures.State(8, [4 0], [0 0], [0 0]);
limits = testCase.TestData.Fixtures.PhysicalLimits([2 2], [1 1], [2 2]);
result = planAzElMotion( obstacle, initialState, goalState, limits, adapter.FixedOptions());
verifyTrue(testCase, result.Success, result.Message);
verifyTrue(testCase, result.Validation.Passed, result.Validation.Message);
verifyTrue(testCase, result.Validation.CollisionFree);
end

function testDenseSweptEnvelopeIsConservativeAndProtectsEndpoints(testCase, ~)
% Verify the dense seed fallback bounds history and rejects endpoint capture.
obstacle = testCase.TestData.Fixtures.RectangleObstacle([0 20], [-1 1 -2 2], 0);
sampleTimes_s = (0:5:20).';
endpointPosition_deg = [-5 0; 5 0];
[envelopeShape, usedEnvelope] = azElInternal.denseSweptEnvelope( ...
    obstacle, sampleTimes_s, endpointPosition_deg, 10);
verifyTrue(testCase, usedEnvelope);
verifyEqual(testCase, min(envelopeShape.Vertices, [], 1), [-1 -2], "AbsTol", 1e-5);
verifyEqual(testCase, max(envelopeShape.Vertices, [], 1), [1 2], "AbsTol", 1e-5);
[capturingShape, usedCapturingEnvelope] = azElInternal.denseSweptEnvelope( ...
    obstacle, sampleTimes_s, [0 0; 5 0], 10);
verifyFalse(testCase, usedCapturingEnvelope);
verifyEmpty(testCase, capturingShape.Vertices);
triangle = makeAzElObstacleData( "triangle", [0; 20], [-4; 4; 0], [-3; -3; 4], 0);
[coarseShape, usedCoarseShape] = azElInternal.denseSweptEnvelope( ...
    triangle, sampleTimes_s, [-8 0; 8 0], 10);
verifyTrue(testCase, usedCoarseShape);
verifyLessThan(testCase, area(coarseShape), 50);
guardedShape = polybuffer(coarseShape, 1e-9);
verifyTrue(testCase, all(isinterior( guardedShape, [-4; 4; 0], [-3; -3; 4])));
secondTriangle = makeAzElObstacleData( "second triangle", [0; 20], [6; 8; 7], [-3; -3; 4], 0);
[manyObstacleShape, usedManyObstacleEnvelope] = azElInternal.denseSweptEnvelope( ...
    [triangle; secondTriangle], linspace(0, 20, 2000), [-8 8; 12 8], 10000);
verifyTrue(testCase, usedManyObstacleEnvelope);
verifyNotEmpty(testCase, manyObstacleShape.Vertices);
end

function testDeterministicRepeatedRun(testCase, adapter)
% Verify identical fixed inputs return identical seed order and trajectory.
initialState = testCase.TestData.Fixtures.State(0, [0 0], [0 0], [0 0]);
goalState = testCase.TestData.Fixtures.State(6, [3 1], [0 0], [0 0]);
limits = testCase.TestData.Fixtures.PhysicalLimits([2 2], [1 1], [2 2]);
options = adapter.FixedOptions();
first = planAzElMotion([], initialState, goalState, limits, options);
second = planAzElMotion([], initialState, goalState, limits, options);
verifyEqual(testCase, first.Success, second.Success);
verifyEqual(testCase, [first.Seeds.Source], [second.Seeds.Source]);
verifyEqual(testCase, first.time_s, second.time_s, "AbsTol", 1e-12);
verifyEqual(testCase, first.position_deg, second.position_deg, "AbsTol", 1e-9);
end

function testEarliestGoalIsNotRejectedByHorizonOccupancy(testCase, adapter)
% Verify a later blocked goal does not reject a valid earlier arrival.
source_deg = [-0.5 -0.5; 0.5 -0.5; 0.5 0.5; -0.5 0.5];
initialObstacle_deg = source_deg + [20 20];
finalObstacle_deg = source_deg + [5 0];
obstacle = makeAzElObstacleData( ...
    "late goal blocker", [0; 10], ...
    {initialObstacle_deg(:, 1); finalObstacle_deg(:, 1)}, ...
    {initialObstacle_deg(:, 2); finalObstacle_deg(:, 2)}, 0);
initialState = testCase.TestData.Fixtures.State(0, [0 0], [0 0], [0 0]);
goalState = testCase.TestData.Fixtures.State(10, [5 0], [0 0], [0 0]);
limits = testCase.TestData.Fixtures.PhysicalLimits([2 2], [1 1], [2 2]);
options = adapter.FixedOptions();
options.GoalTimeMode = "earliestArrival";
result = planAzElMotion( obstacle, initialState, goalState, limits, options);
verifyTrue(testCase, result.Success, result.Message);
verifyLessThan(testCase, result.ArrivalTime_s, goalState.time_s);
verifyTrue(testCase, result.Validation.CollisionFree);
end

function testEarlyPlannerFailureKeepsValidationFieldOrder(testCase, adapter)
% Verify endpoint failure and success use one public validation schema.
initialState = testCase.TestData.Fixtures.State(0, [0 0], [0 0], [0 0]);
goalState = testCase.TestData.Fixtures.State(8, [4 0], [0 0], [0 0]);
limits = testCase.TestData.Fixtures.PhysicalLimits([2 2], [1 1], [2 2]);
success = planAzElMotion( [], initialState, goalState, limits, adapter.FixedOptions());
blockingObstacle = testCase.TestData.Fixtures.RectangleObstacle([0 8], [-1 1 -1 1], 0);
failure = planAzElMotion( ...
    blockingObstacle, initialState, goalState, limits, ...
    adapter.FixedOptions());
verifyTrue(testCase, success.Validation.Passed);
verifyFalse(testCase, failure.Success);
verifyEqual(testCase, failure.TerminationReason, "endpointBlocked");
verifyEqual(testCase, fieldnames(failure.Validation), fieldnames(success.Validation));
end

function testInterceptWrapperRequiresTwoTargetSamples(testCase, ~)
% Verify the wrapper rejects a one-sample target at its public boundary.
targetMotion = struct("time_s", 10, "position_deg", [1 0]);
initialState = testCase.TestData.Fixtures.State(0, [0 0], [0 0], [0 0]);
limits = testCase.TestData.Fixtures.PhysicalLimits([2 2], [1 1], [2 2]);
verifyError(testCase, @() planAzElMovingTargetIntercept( ...
    initialState, targetMotion, limits, struct()), ...
    "planAzElMovingTargetIntercept:TargetHistoryTooShort");
end

function testInterceptWrapperTextOptionsMustBeScalar(testCase, ~)
% Verify the moving-target wrapper rejects ambiguous text arrays.
targetMotion = struct( ...
    "time_s", [0; 10], ...
    "position_deg", [1 0; 2 0], ...
    "InterpolationMethod", "linear");
initialState = testCase.TestData.Fixtures.State(0, [0 0], [0 0], [0 0]);
limits = testCase.TestData.Fixtures.PhysicalLimits([2 2], [1 1], [2 2]);
options = struct("InterceptMode", ["earliest" "specifiedTime"]);
verifyError(testCase, @() planAzElMovingTargetIntercept( ...
    initialState, targetMotion, limits, options), "planAzElMovingTargetIntercept:InvalidMode");
targetMotion.InterpolationMethod = ["linear" "pchip"];
verifyError(testCase, @() planAzElMovingTargetIntercept( ...
    initialState, targetMotion, limits, struct()), ...
    "planAzElMovingTargetIntercept:InvalidInterpolation");
end

function testMovingGoalHistoryRequiresTwoSamples(testCase, adapter)
% Verify one target sample fails with one actionable public error.
initialState = testCase.TestData.Fixtures.State(0, [0 0], [0 0], [0 0]);
goalState = testCase.TestData.Fixtures.State(10, [1 0], [0 0], [0 0]);
goalState.targetTime_s = 10;
goalState.targetPosition_deg = [1 0];
limits = testCase.TestData.Fixtures.PhysicalLimits([2 2], [1 1], [2 2]);
verifyError(testCase, @() planAzElMotion( ...
    [], initialState, goalState, limits, adapter.FixedOptions()), ...
    "planAzElMotion:MovingGoalHistoryTooShort");
end

function testMovingGoalInterpolationMethodMustBeScalar(testCase, adapter)
% Verify a text array cannot pass as one interpolation method.
initialState = testCase.TestData.Fixtures.State(0, [0 0], [0 0], [0 0]);
goalState = testCase.TestData.Fixtures.State(10, [5 0], [0 0], [0 0]);
goalState.targetTime_s = [0; 10];
goalState.targetPosition_deg = [4 0; 5 0];
goalState.InterpolationMethod = ["linear" "pchip"];
limits = testCase.TestData.Fixtures.PhysicalLimits([2 2], [1 1], [2 2]);
verifyError(testCase, @() planAzElMotion( ...
    [], initialState, goalState, limits, adapter.FixedOptions()), ...
    "planAzElMotion:InvalidGoalInterpolation");
end

function testObstacleActivationAtTerminalTimeFailsValidation(testCase, adapter)
% Verify collision at the first active event endpoint cannot be missed.
initialState = testCase.TestData.Fixtures.State(0, [0 0], [0.1 0], [0 0]);
goalState = testCase.TestData.Fixtures.State(10, [1 0], [0.1 0], [0 0]);
limits = testCase.TestData.Fixtures.PhysicalLimits([1 1], [1 1], [1 1]);
trajectory = testCase.TestData.Fixtures.LinearTrajectory(initialState, goalState);
obstacle = testCase.TestData.Fixtures.RectangleObstacle([10 20], [0.5 1.5 -0.5 0.5], 0);
validation = validateAzElTrajectory( ...
    trajectory, obstacle, initialState, goalState, limits, ...
    adapter.FixedOptions());
verifyFalse(testCase, validation.Passed);
verifyFalse(testCase, validation.CollisionFree);
verifyLessThanOrEqual(testCase, validation.MinimumClearance_deg, 0);
end

function testOldWorkspaceOptionGivesMigrationError(testCase, adapter)
% Verify old workspace option names identify their new limits location.
initialState = testCase.TestData.Fixtures.State(0, [0 0], [0 0], [0 0]);
goalState = testCase.TestData.Fixtures.State(6, [2 0], [0 0], [0 0]);
limits = testCase.TestData.Fixtures.PhysicalLimits([2 2], [1 1], [2 2]);
options = adapter.FixedOptions();
options.ElevationInterval_deg = [-5 5];
verifyError(testCase, @() planAzElMotion( ...
    [], initialState, goalState, limits, options), "planAzElMotion:WorkspaceLimitMoved");
end

function testRemovedPlanningTimeOptionGivesMigrationError(testCase, adapter)
% Verify the removed planner timeout gives an actionable public error.
initialState = testCase.TestData.Fixtures.State(0, [0 0], [0 0], [0 0]);
goalState = testCase.TestData.Fixtures.State(8, [4 0], [0 0], [0 0]);
limits = testCase.TestData.Fixtures.PhysicalLimits([2 2], [1 1], [2 2]);
options = adapter.FixedOptions();
options.MaximumPlanningTime_s = 1;
verifyError(testCase, @() planAzElMotion( ...
    [], initialState, goalState, limits, options), "planAzElMotion:RemovedMaximumPlanningTime");
end

function testSafetyMarginIsAppliedExactlyOnce(testCase, ~)
% Verify absolute reconstruction from original geometry is idempotent.
source_deg = [-1 -1; 1 -1; 1 1; -1 1];
obstacle = makeAzElObstacleData( "margin", [0; 1], source_deg(:, 1), source_deg(:, 2), 0.2);
reinflated = makeAzElObstacleData(obstacle, 0.2);
verifyEqual(testCase, reinflated.az_deg, obstacle.az_deg, "AbsTol", 1e-12);
verifyEqual(testCase, reinflated.el_deg, obstacle.el_deg, "AbsTol", 1e-12);
verifyEqual(testCase, reinflated.safetyMargin_deg, 0.2);
end

function testShiftedPolynomialTimeCannotHideInitialTime(testCase, adapter)
% Verify matching shifted samples and coefficients still honor initial time.
duration_s = 2;
initialState = testCase.TestData.Fixtures.State(0, [0 0], [0 0], [0 0]);
goalState = testCase.TestData.Fixtures.State(10, [4 / 3 0], [2 0], [2 0]);
limits = testCase.TestData.Fixtures.PhysicalLimits([3 3], [3 3], [2 2]);
trajectory = testCase.TestData.Fixtures.ConstantJerkTrajectory(duration_s);
trajectory.time_s = trajectory.time_s + 1;
trajectory.Polynomial.SegmentStartTime_s = trajectory.Polynomial.SegmentStartTime_s + 1;
trajectory.Polynomial.FinalTime_s = trajectory.Polynomial.FinalTime_s + 1;
options = adapter.FixedOptions();
options.GoalTimeMode = "earliestArrival";
validation = validateAzElTrajectory( trajectory, [], initialState, goalState, limits, options);
verifyFalse(testCase, validation.Passed);
verifyFalse(testCase, validation.PolynomialInitialTimeMatched);
verifyTrue(testCase, validation.PolynomialHistoryConsistent);
end

function testStaticObstacleProducesOppositeSideSeeds(testCase, adapter)
% Verify bounded input-driven seed diversity and validated selection.
obstacle = testCase.TestData.Fixtures.RectangleObstacle([0 20], [-1 1 -2 2], 0.2);
initialState = testCase.TestData.Fixtures.State(0, [-5 0], [0 0], [0 0]);
goalState = testCase.TestData.Fixtures.State(12, [5 0], [0 0], [0 0]);
limits = testCase.TestData.Fixtures.PhysicalLimits([2 2], [1 1], [2 2]);
options = adapter.FixedOptions();
options.MaximumSeedCount = 3;
options.DirectSeedOnly = false;
result = planAzElMotion( obstacle, initialState, goalState, limits, options);
verifyGreaterThanOrEqual(testCase, numel(result.Seeds), 3);
verifyTrue(testCase, any([result.Seeds.Source] == "visibilityGraph"));
verifyEqual(testCase, result.SearchDiagnostics.Grid.GraphType, "timeExpandedVisibilityGraph");
verifyGreaterThan(testCase, result.SearchDiagnostics.Grid.VisibilityEdgeCount, 0);
verifyLessThan(testCase, ...
    result.SearchDiagnostics.Grid.VisibilityCandidatePairCount, ...
    result.SearchDiagnostics.Grid.NodeCount * (result.SearchDiagnostics.Grid.NodeCount - 1) / 2);
verifyEqual(testCase, size(result.SearchDiagnostics.Grid.AcceptedEdges_deg, 2), 4);
verifyEqual(testCase, size(result.SearchDiagnostics.Grid.RejectedEdges_deg, 2), 4);
verifyTrue(testCase, all(isfinite( result.SearchDiagnostics.Grid.AcceptedEdges_deg), "all"));
verifyEqual(testCase, size(result.SearchDiagnostics.Grid.FrontierNodes_deg, 2), 2);
verifyTrue(testCase, result.SearchDiagnostics.Grid.Coverage.ExactSpatialProposalUsed);
verifyFalse(testCase, result.SearchDiagnostics.Grid.Coverage.ReducedSpatialProposalUsed);
verifyTrue(testCase, result.SearchDiagnostics.Grid.Coverage.CompletenessLost);
verifyEqual(testCase, ...
    result.SearchDiagnostics.Grid.Coverage.CompletenessLossReason, ...
    "boundedSeedNodeAndTimeSearch");
verifySize(testCase, result.SearchDiagnostics.Grid.HomologyRepresentative_deg, [1 2]);
verifyGreaterThanOrEqual(testCase, result.SearchDiagnostics.Grid.HomologyClassCount, 2);
signatureCount = size(unique( ...
    result.SearchDiagnostics.Grid.HomologyClassSignatures, "rows"), 1);
verifyGreaterThanOrEqual(testCase, signatureCount, 2);
verifyFalse(testCase, result.SearchDiagnostics.Grid.HomologySearchTruncated);
minimumElevations_deg = zeros(numel(result.Seeds), 1);
maximumElevations_deg = zeros(numel(result.Seeds), 1);

% Measure every generated seed's elevation range to confirm both detour classes exist.
for seedIndex = 1:numel(result.Seeds)
    minimumElevations_deg(seedIndex) = min( result.Seeds(seedIndex).position_deg(:, 2));
    maximumElevations_deg(seedIndex) = max( result.Seeds(seedIndex).position_deg(:, 2));
end
verifyLessThan(testCase, min(minimumElevations_deg), -2);
verifyGreaterThan(testCase, max(maximumElevations_deg), 2);
verifyTrue(testCase, result.Success, result.Message);
verifyTrue(testCase, result.Validation.CollisionFree);
validated = find([result.SeedSummaries.ValidationPassed]);
if result.Success
    arrival_s = [result.SeedSummaries(validated).ArrivalTime_s];
    verifyLessThanOrEqual(testCase, ...
        result.SeedSummaries(result.SelectedSeedIndex).ArrivalTime_s, ...
        min(arrival_s) + options.ArrivalTimeTolerance_s);
end
end

function testTopologyChangeUsesAStationaryConservativeUnion(testCase, ~)
% Verify a topology-change interval has constant conservative geometry.
closed_deg = [-2 -1; 2 -1; 2 1; -2 1];
left_deg = [-2 -1; -0.5 -1; -0.5 1; -2 1];
right_deg = [0.5 -1; 2 -1; 2 1; 0.5 1];
open_deg = [left_deg; NaN NaN; right_deg];
obstacle = makeAzElObstacleData( ...
    "opening", [0; 2], {closed_deg(:, 1); open_deg(:, 1)}, {closed_deg(:, 2); open_deg(:, 2)}, 0);
[shape, geometry] = azElInternal.obstacles.shapeAtTime(obstacle, 1);
verifyFalse(testCase, geometry.TopologyIsInterpolated);
verifyEqual(testCase, geometry.VertexSpeedBound_deg_s, 0);
verifyTrue(testCase, isinterior(shape, 0, 0));
end

function testTranslatedHistoryReusesExactProtectedShape(testCase, ~)
% Verify rigid obstacle motion preserves one translated protected boundary.
source_deg = [-1 -1; 1 -1; 1 1; -1 1];
translation_deg = [3 2];
azimuth_deg = {source_deg(:, 1); source_deg(:, 1) + translation_deg(1)};
elevation_deg = {source_deg(:, 2); source_deg(:, 2) + translation_deg(2)};
obstacle = makeAzElObstacleData( "translated", [0; 1], azimuth_deg, elevation_deg, 0.2);
verifyEqual(testCase, obstacle.az_deg{2}, obstacle.az_deg{1} + translation_deg(1), "AbsTol", 1e-12);
verifyEqual(testCase, obstacle.el_deg{2}, obstacle.el_deg{1} + translation_deg(2), "AbsTol", 1e-12);
end

function testUnrelatedPolynomialCannotValidateSampledHistory(testCase, adapter)
% Verify valid samples cannot hide unrelated polynomial coefficients.
duration_s = 2;
initialState = testCase.TestData.Fixtures.State(0, [0 0], [0 0], [0 0]);
goalState = testCase.TestData.Fixtures.State(duration_s, [4 / 3 0], [2 0], [2 0]);
limits = testCase.TestData.Fixtures.PhysicalLimits([3 3], [3 3], [2 2]);
trajectory = testCase.TestData.Fixtures.ConstantJerkTrajectory(duration_s);
trajectory.Polynomial.positionPower_deg(:) = 0;
trajectory.Polynomial.velocityPower_deg_s(:) = 0;
trajectory.Polynomial.accelerationPower_deg_s2(:) = 0;
trajectory.Polynomial.jerkPower_deg_s3(:) = 0;
validation = validateAzElTrajectory( ...
    trajectory, [], initialState, goalState, limits, ...
    adapter.FixedOptions());
verifyFalse(testCase, validation.Passed);
verifyTrue(testCase, validation.PolynomialSchemaValid);
verifyFalse(testCase, validation.PolynomialEndpointStatesMatched);
verifyFalse(testCase, validation.PolynomialHistoryConsistent);
end

function testWorkspaceIntervalsBelongToLimits(testCase, adapter)
% Verify omitted and explicit workspace intervals use the limits contract.
initialState = testCase.TestData.Fixtures.State(0, [0 0], [0 0], [0 0]);
goalState = testCase.TestData.Fixtures.State(6, [2 0], [0 0], [0 0]);
limits = rmfield( ...
    testCase.TestData.Fixtures.PhysicalLimits([2 2], [1 1], [2 2]), ...
    ["azimuthInterval_deg", "elevationInterval_deg"]);
result = planAzElMotion([], initialState, goalState, limits, adapter.FixedOptions());
verifyEqual(testCase, result.Inputs.limits.azimuthInterval_deg, [-180 180]);
verifyEqual(testCase, result.Inputs.limits.elevationInterval_deg, [-90 90]);
limits.azimuthInterval_deg = [-12 14];
limits.elevationInterval_deg = [-5 6];
result = planAzElMotion([], initialState, goalState, limits, adapter.FixedOptions());
verifyEqual(testCase, result.Inputs.limits.azimuthInterval_deg, [-12 14]);
verifyEqual(testCase, result.Inputs.limits.elevationInterval_deg, [-5 6]);
end
