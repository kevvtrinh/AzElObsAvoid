function verifySharedPlannerRequirement(testCase, requirementName)
%% Section 0: Header & Readme
% SYNTAX
%   testSupport.verifySharedPlannerRequirement(testCase, requirementName)
%**************************************************************************
% PURPOSE
%   - Run one reusable behavior check against the maintained obstacle planner.
%**************************************************************************
% INPUTS
%   - testCase (matlab.unittest.FunctionTestCase)
%       Active function-based test case with shared fixtures in TestData.
%   - requirementName (string scalar)
%       Name of one registered shared behavior check.
%**************************************************************************
% OUTPUTS
%   - None. Verification failures are reported through testCase.
%**************************************************************************
% UNITS
%   - Individual requirements document physical units through field suffixes.
%**************************************************************************

%% Section 1: Create The Planner Adapter

% The adapter gives each shared check one way to call the maintained planner.
% This separates tested behavior from package names and entry-point code.

adapter = struct("FixedOptions", @() obstacleAvoidance.planTrajectory());

%% Section 2: Run Named Behavior Check

% Select one check by its registered name. An unknown name is a test setup
% error. A failed selected check names the physical behavior to investigate.

requirements = struct("testXWrappingChangesThePhysicalRequest", ...
    @testXWrappingChangesThePhysicalRequest, ...
    "testXWrappingRejectsUnmodeledPeriodicGeometry", ...
    @testXWrappingRejectsUnmodeledPeriodicGeometry, ...
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
requirementName = char(string(requirementName));
if ~isfield(requirements, requirementName)
    error("testSupport:UnknownPlannerRequirement", "Unknown shared planner requirement '%s'.", requirementName);
end
requirement = requirements.(requirementName);
requirement(testCase, adapter);
end

%% Section 3: Local Behavior Check Functions

function testXWrappingChangesThePhysicalRequest(testCase, adapter)
    % Verify wrapping selects the short move and disabled wrapping keeps the long move.
    initialState = testCase.TestData.Fixtures.State(0, [179 0], [0 0], [0 0]);
    goalState    = testCase.TestData.Fixtures.State(8, [-179 0], [0 0], [0 0]);
    limits       = testCase.TestData.Fixtures.PhysicalLimits([1 1], [1 1], [2 2]);
    options      = adapter.FixedOptions();
    options.WrapX = false;
    longResult = obstacleAvoidance.planTrajectory([], initialState, goalState, limits, options);
    options.WrapX = true;
    shortResult = obstacleAvoidance.planTrajectory([], initialState, goalState, limits, options);
    verifyFalse(testCase, longResult.Success);
    verifyTrue(testCase, shortResult.Success, shortResult.Message);
    verifyEqual(testCase, shortResult.position_units(end, 1), 181, "AbsTol", 1e-6);
    verifyTrue(testCase, shortResult.Validation.WrapPolicySatisfied);
end

function testXWrappingRejectsUnmodeledPeriodicGeometry(testCase, adapter)
    % Verify wrapped obstacle requests cannot return a false physical success.
    initialState = testCase.TestData.Fixtures.State(0, [179 0], [0 0], [0 0]);
    goalState    = testCase.TestData.Fixtures.State(8, [-179 0], [0 0], [0 0]);
    limits       = testCase.TestData.Fixtures.PhysicalLimits([1 1], [1 1], [2 2]);
    options      = adapter.FixedOptions();
    options.WrapX = true;
    obstacle = testCase.TestData.Fixtures.RectangleObstacle([0 8], [-180.5 -179.5 -1 1], 0);
    verifyError(testCase, @() obstacleAvoidance.planTrajectory(obstacle, initialState, goalState, limits, options), "planTrajectory:UnsupportedWrappedGeometry");
end

function testBetweenNodeCollisionFailsValidation(testCase, adapter)
    % Verify a collision at the segment midpoint cannot hide between samples.
    initialState = testCase.TestData.Fixtures.State(0, [-1 0], [2 0], [0 0]);
    goalState    = testCase.TestData.Fixtures.State(1, [1 0], [2 0], [0 0]);
    limits       = testCase.TestData.Fixtures.PhysicalLimits([3 3], [1 1], [1 1]);
    trajectory   = testCase.TestData.Fixtures.LinearTrajectory(initialState, goalState);
    obstacle     = testCase.TestData.Fixtures.RectangleObstacle([0 1], [-0.1 0.1 -1 1], 0);
    validation   = obstacleAvoidance.validateTrajectory(trajectory, obstacle, initialState, goalState, limits, adapter.FixedOptions());
    verifyFalse(testCase, validation.Passed);
    verifyFalse(testCase, validation.CollisionFree);
    verifyLessThanOrEqual(testCase, validation.MinimumClearance_units, 0);
end

function testBetweenNodeVelocityViolationFailsValidation(testCase, adapter)
    % Verify continuous extrema detect an interior peak with clear samples.
    initialState = testCase.TestData.Fixtures.State(0, [0 0], [0 0], [4 0]);
    goalState    = testCase.TestData.Fixtures.State(1, [2 / 3 0], [0 0], [-4 0]);
    limits       = testCase.TestData.Fixtures.PhysicalLimits([0.8 1], [5 5], [9 9]);
    trajectory   = testCase.TestData.Fixtures.InteriorVelocityPeakTrajectory();
    validation   = obstacleAvoidance.validateTrajectory(trajectory, [], initialState, goalState, limits, adapter.FixedOptions());
    verifyFalse(testCase, validation.Passed);
    verifyFalse(testCase, validation.VelocityWithinLimits);
    verifyEqual(testCase, max(abs(trajectory.velocity_units_s), [], "all"), 0);
end

function testConstantJerkPolynomialPassesIndependentDynamics(testCase, adapter)
    % Verify the third-order chain against analytic constant-jerk motion.
    duration_s   = 2;
    initialState = testCase.TestData.Fixtures.State(0, [0 0], [0 0], [0 0]);
    goalState    = testCase.TestData.Fixtures.State(duration_s, [4 / 3 0], [2 0], [2 0]);
    limits       = testCase.TestData.Fixtures.PhysicalLimits([3 3], [3 3], [2 2]);
    trajectory   = testCase.TestData.Fixtures.ConstantJerkTrajectory(duration_s);
    options      = adapter.FixedOptions();
    validation   = obstacleAvoidance.validateTrajectory(trajectory, [], initialState, goalState, limits, options);
    verifyTrue(testCase, validation.Passed, validation.Message);
    verifyLessThanOrEqual(testCase, validation.MaximumDynamicsResidual, 1e-12);
end

function testDeformingObstacleUsesThePlannerPath(testCase, adapter)
    % Verify a deforming protected polygon uses the maintained planner.
    time_s       = [0; 8];
    first_units    = [-1 4; 1 4; 1 6; -1 6];
    second_units   = [-2 4.5; 2 4.5; 2 5.5; -2 5.5];
    obstacle     = obstacleAvoidance.obstacles.createObstacle("deforming", time_s, {first_units(:, 1); second_units(:, 1)}, {first_units(:, 2); second_units(:, 2)}, 0.1);
    initialState = testCase.TestData.Fixtures.State(0, [0 0], [0 0], [0 0]);
    goalState    = testCase.TestData.Fixtures.State(8, [4 0], [0 0], [0 0]);
    limits       = testCase.TestData.Fixtures.PhysicalLimits([2 2], [1 1], [2 2]);
    result       = obstacleAvoidance.planTrajectory(obstacle, initialState, goalState, limits, adapter.FixedOptions());
    verifyTrue(testCase, result.Success, result.Message);
    verifyTrue(testCase, result.Validation.Passed, result.Validation.Message);
    verifyTrue(testCase, result.Validation.CollisionFree);
end

function testDenseSweptEnvelopeIsConservativeAndProtectsEndpoints(testCase, ~)
    % Verify the dense seed fallback bounds history and rejects endpoint capture.
    obstacle             = testCase.TestData.Fixtures.RectangleObstacle([0 20], [-1 1 -2 2], 0);
    sampleTimes_s        = (0:5:20).';
    endpointPosition_units = [-5 0; 5 0];
    [envelopeShape, usedEnvelope] = obstacleAvoidance.search.denseSweptEnvelope(obstacle, sampleTimes_s, endpointPosition_units, 10);
    verifyTrue(testCase, usedEnvelope);
    verifyEqual(testCase, min(envelopeShape.Vertices, [], 1), [-1 -2], "AbsTol", 1e-5);
    verifyEqual(testCase, max(envelopeShape.Vertices, [], 1), [1 2], "AbsTol", 1e-5);
    [capturingShape, usedCapturingEnvelope] = obstacleAvoidance.search.denseSweptEnvelope(obstacle, sampleTimes_s, [0 0; 5 0], 10);
    verifyFalse(testCase, usedCapturingEnvelope);
    verifyEmpty(testCase, capturingShape.Vertices);
    triangle = obstacleAvoidance.obstacles.createObstacle("triangle", [0; 20], [-4; 4; 0], [-3; -3; 4], 0);
    [coarseShape, usedCoarseShape] = obstacleAvoidance.search.denseSweptEnvelope(triangle, sampleTimes_s, [-8 0; 8 0], 10);
    verifyTrue(testCase, usedCoarseShape);
    verifyLessThan(testCase, area(coarseShape), 50);
    guardedShape = polybuffer(coarseShape, 1e-9);
    verifyTrue(testCase, all(isinterior(guardedShape, [-4; 4; 0], [-3; -3; 4])));
    secondTriangle = obstacleAvoidance.obstacles.createObstacle("second triangle", [0; 20], [6; 8; 7], [-3; -3; 4], 0);
    [manyObstacleShape, usedManyObstacleEnvelope] = obstacleAvoidance.search.denseSweptEnvelope([triangle; secondTriangle], linspace(0, 20, 2000), [-8 8; 12 8], 10000);
    verifyTrue(testCase, usedManyObstacleEnvelope);
    verifyNotEmpty(testCase, manyObstacleShape.Vertices);
    verifyFalse(testCase, isinterior(manyObstacleShape, 5, 0));
end

function testDeterministicRepeatedRun(testCase, adapter)
    % Verify identical fixed inputs return identical seed order and trajectory.
    initialState = testCase.TestData.Fixtures.State(0, [0 0], [0 0], [0 0]);
    goalState    = testCase.TestData.Fixtures.State(6, [3 1], [0 0], [0 0]);
    limits       = testCase.TestData.Fixtures.PhysicalLimits([2 2], [1 1], [2 2]);
    options      = adapter.FixedOptions();
    [first, firstDiagnosis]   = obstacleAvoidance.planTrajectory([], initialState, goalState, limits, options);
    [second, secondDiagnosis] = obstacleAvoidance.planTrajectory([], initialState, goalState, limits, options);
    verifyEqual(testCase, first.Success, second.Success);
    verifyEqual(testCase, [firstDiagnosis.Routes.Source], [secondDiagnosis.Routes.Source]);
    verifyEqual(testCase, first.time_s, second.time_s, "AbsTol", 1e-12);
    verifyEqual(testCase, first.position_units, second.position_units, "AbsTol", 1e-9);
end

function testEarliestGoalIsNotRejectedByHorizonOccupancy(testCase, adapter)
    % Verify a later blocked goal does not reject a valid earlier arrival.
    source_units          = [-0.5 -0.5; 0.5 -0.5; 0.5 0.5; -0.5 0.5];
    initialObstacle_units = source_units + [20 20];
    finalObstacle_units   = source_units + [5 0];
    obstacle            = obstacleAvoidance.obstacles.createObstacle("late goal blocker", [0; 10], {initialObstacle_units(:, 1); finalObstacle_units(:, 1)}, {initialObstacle_units(:, 2); finalObstacle_units(:, 2)}, 0);
    initialState        = testCase.TestData.Fixtures.State(0, [0 0], [0 0], [0 0]);
    goalState           = testCase.TestData.Fixtures.State(10, [5 0], [0 0], [0 0]);
    limits              = testCase.TestData.Fixtures.PhysicalLimits([2 2], [1 1], [2 2]);
    options             = adapter.FixedOptions();
    options.GoalTimeMode = "earliestArrival";
    result = obstacleAvoidance.planTrajectory(obstacle, initialState, goalState, limits, options);
    verifyTrue(testCase, result.Success, result.Message);
    verifyLessThan(testCase, result.ArrivalTime_s, goalState.time_s);
    verifyTrue(testCase, result.Validation.CollisionFree);
end

function testEarlyPlannerFailureKeepsValidationFieldOrder(testCase, adapter)
    % Check that endpoint failure and success use the same validation fields.
    initialState     = testCase.TestData.Fixtures.State(0, [0 0], [0 0], [0 0]);
    goalState        = testCase.TestData.Fixtures.State(8, [4 0], [0 0], [0 0]);
    limits           = testCase.TestData.Fixtures.PhysicalLimits([2 2], [1 1], [2 2]);
    success          = obstacleAvoidance.planTrajectory([], initialState, goalState, limits, adapter.FixedOptions());
    blockingObstacle = testCase.TestData.Fixtures.RectangleObstacle([0 8], [-1 1 -1 1], 0);
    failure          = obstacleAvoidance.planTrajectory(blockingObstacle, initialState, goalState, limits, adapter.FixedOptions());
    verifyTrue(testCase, success.Validation.Passed);
    verifyFalse(testCase, failure.Success);
    verifyEqual(testCase, failure.TerminationReason, "endpointBlocked");
    verifyEqual(testCase, fieldnames(failure.Validation), fieldnames(success.Validation));
end

function testInterceptWrapperRequiresTwoTargetSamples(testCase, ~)
    % Verify the wrapper rejects a one-sample target at its public boundary.
    targetMotion = struct();
    targetMotion.time_s       = 10;
    targetMotion.position_units = [1 0];
    initialState = testCase.TestData.Fixtures.State(0, [0 0], [0 0], [0 0]);
    limits       = testCase.TestData.Fixtures.PhysicalLimits([2 2], [1 1], [2 2]);
    verifyError(testCase, @() obstacleAvoidance.planMovingTargetIntercept(initialState, targetMotion, limits, struct()), "planMovingTargetIntercept:TargetHistoryTooShort");
end

function testInterceptWrapperTextOptionsMustBeScalar(testCase, ~)
    % Verify the moving-target wrapper rejects ambiguous text arrays.
    targetMotion = struct();
    targetMotion.time_s              = [0; 10];
    targetMotion.position_units        = [1 0; 2 0];
    targetMotion.InterpolationMethod = "linear";
    initialState = testCase.TestData.Fixtures.State(0, [0 0], [0 0], [0 0]);
    limits       = testCase.TestData.Fixtures.PhysicalLimits([2 2], [1 1], [2 2]);
    options      = struct("InterceptMode", ["earliest" "specifiedTime"]);
    verifyError(testCase, @() obstacleAvoidance.planMovingTargetIntercept(initialState, targetMotion, limits, options), "planMovingTargetIntercept:InvalidMode");
    targetMotion.InterpolationMethod = ["linear" "pchip"];
    verifyError(testCase, @() obstacleAvoidance.planMovingTargetIntercept(initialState, targetMotion, limits, struct()), "planMovingTargetIntercept:InvalidInterpolation");
end

function testMovingGoalHistoryRequiresTwoSamples(testCase, adapter)
    % Verify one target sample fails with one actionable public error.
    initialState = testCase.TestData.Fixtures.State(0, [0 0], [0 0], [0 0]);
    goalState    = testCase.TestData.Fixtures.State(10, [1 0], [0 0], [0 0]);
    goalState.targetTime_s       = 10;
    goalState.targetPosition_units = [1 0];
    limits = testCase.TestData.Fixtures.PhysicalLimits([2 2], [1 1], [2 2]);
    verifyError(testCase, @() obstacleAvoidance.planTrajectory([], initialState, goalState, limits, adapter.FixedOptions()), "planTrajectory:MovingGoalHistoryTooShort");
end

function testMovingGoalInterpolationMethodMustBeScalar(testCase, adapter)
    % Verify a text array cannot pass as one interpolation method.
    initialState = testCase.TestData.Fixtures.State(0, [0 0], [0 0], [0 0]);
    goalState    = testCase.TestData.Fixtures.State(10, [5 0], [0 0], [0 0]);
    goalState.targetTime_s        = [0; 10];
    goalState.targetPosition_units  = [4 0; 5 0];
    goalState.InterpolationMethod = ["linear" "pchip"];
    limits = testCase.TestData.Fixtures.PhysicalLimits([2 2], [1 1], [2 2]);
    verifyError(testCase, @() obstacleAvoidance.planTrajectory([], initialState, goalState, limits, adapter.FixedOptions()), "planTrajectory:InvalidGoalInterpolation");
end

function testObstacleActivationAtTerminalTimeFailsValidation(testCase, adapter)
    % Verify collision at the first active event endpoint cannot be missed.
    initialState = testCase.TestData.Fixtures.State(0, [0 0], [0.1 0], [0 0]);
    goalState    = testCase.TestData.Fixtures.State(10, [1 0], [0.1 0], [0 0]);
    limits       = testCase.TestData.Fixtures.PhysicalLimits([1 1], [1 1], [1 1]);
    trajectory   = testCase.TestData.Fixtures.LinearTrajectory(initialState, goalState);
    obstacle     = testCase.TestData.Fixtures.RectangleObstacle([10 20], [0.5 1.5 -0.5 0.5], 0);
    validation   = obstacleAvoidance.validateTrajectory(trajectory, obstacle, initialState, goalState, limits, adapter.FixedOptions());
    verifyFalse(testCase, validation.Passed);
    verifyFalse(testCase, validation.CollisionFree);
    verifyLessThanOrEqual(testCase, validation.MinimumClearance_units, 0);
end

function testOldWorkspaceOptionGivesMigrationError(testCase, adapter)
    % Verify old workspace option names identify their new limits location.
    initialState = testCase.TestData.Fixtures.State(0, [0 0], [0 0], [0 0]);
    goalState    = testCase.TestData.Fixtures.State(6, [2 0], [0 0], [0 0]);
    limits       = testCase.TestData.Fixtures.PhysicalLimits([2 2], [1 1], [2 2]);
    options      = adapter.FixedOptions();
    options.YInterval_units = [-5 5];
    verifyError(testCase, @() obstacleAvoidance.planTrajectory([], initialState, goalState, limits, options), "planTrajectory:WorkspaceLimitMoved");
end

function testRemovedPlanningTimeOptionGivesMigrationError(testCase, adapter)
    % Verify the removed planner timeout gives an actionable public error.
    initialState = testCase.TestData.Fixtures.State(0, [0 0], [0 0], [0 0]);
    goalState    = testCase.TestData.Fixtures.State(8, [4 0], [0 0], [0 0]);
    limits       = testCase.TestData.Fixtures.PhysicalLimits([2 2], [1 1], [2 2]);
    options      = adapter.FixedOptions();
    options.MaximumPlanningTime_s = 1;
    verifyWarning(testCase, @() obstacleAvoidance.planTrajectory([], initialState, goalState, limits, options), "planTrajectory:UnknownOptions");
end

function testSafetyMarginIsAppliedExactlyOnce(testCase, ~)
    % Verify absolute reconstruction from original geometry is idempotent.
    source_units = [-1 -1; 1 -1; 1 1; -1 1];
    obstacle   = obstacleAvoidance.obstacles.createObstacle("margin", [0; 1], source_units(:, 1), source_units(:, 2), 0.2);
    reinflated = obstacleAvoidance.obstacles.createObstacle(obstacle, 0.2);
    verifyEqual(testCase, reinflated.x_units, obstacle.x_units, "AbsTol", 1e-12);
    verifyEqual(testCase, reinflated.y_units, obstacle.y_units, "AbsTol", 1e-12);
    verifyEqual(testCase, reinflated.safetyMargin_units, 0.2);
end

function testShiftedPolynomialTimeCannotHideInitialTime(testCase, adapter)
    % Verify matching shifted samples and coefficients still honor initial time.
    duration_s   = 2;
    initialState = testCase.TestData.Fixtures.State(0, [0 0], [0 0], [0 0]);
    goalState    = testCase.TestData.Fixtures.State(10, [4 / 3 0], [2 0], [2 0]);
    limits       = testCase.TestData.Fixtures.PhysicalLimits([3 3], [3 3], [2 2]);
    trajectory   = testCase.TestData.Fixtures.ConstantJerkTrajectory(duration_s);
    trajectory.time_s                        = trajectory.time_s + 1;
    trajectory.Polynomial.SegmentStartTime_s = trajectory.Polynomial.SegmentStartTime_s + 1;
    trajectory.Polynomial.FinalTime_s        = trajectory.Polynomial.FinalTime_s + 1;
    options = adapter.FixedOptions();
    options.GoalTimeMode = "earliestArrival";
    validation = obstacleAvoidance.validateTrajectory(trajectory, [], initialState, goalState, limits, options);
    verifyFalse(testCase, validation.Passed);
    verifyFalse(testCase, validation.PolynomialInitialTimeMatched);
    verifyTrue(testCase, validation.PolynomialHistoryConsistent);
end

function testStaticObstacleProducesOppositeSideSeeds(testCase, adapter)
    % Verify bounded input-driven seed diversity and validated selection.
    obstacle     = testCase.TestData.Fixtures.RectangleObstacle([0 20], [-1 1 -2 2], 0.2);
    initialState = testCase.TestData.Fixtures.State(0, [-5 0], [0 0], [0 0]);
    goalState    = testCase.TestData.Fixtures.State(12, [5 0], [0 0], [0 0]);
    limits       = testCase.TestData.Fixtures.PhysicalLimits([2 2], [1 1], [2 2]);
    options      = adapter.FixedOptions();
    options.MaximumSeedCount = 3;
    [result, resultDiagnosis] = obstacleAvoidance.planTrajectory(obstacle, initialState, goalState, limits, options);
    verifyGreaterThanOrEqual(testCase, numel(resultDiagnosis.Routes), 3);
    verifyTrue(testCase, any([resultDiagnosis.Routes.Source] == "visibilityGraph"));
    verifyEqual(testCase, resultDiagnosis.Search.GraphType, "visibilityGraph");
    verifyGreaterThan(testCase, resultDiagnosis.Search.VisibilityEdgeCount, 0);
    verifyLessThan(testCase, resultDiagnosis.Search.VisibilityCandidatePairCount, resultDiagnosis.Search.NodeCount * (resultDiagnosis.Search.NodeCount - 1) / 2);
    verifyEqual(testCase, size(resultDiagnosis.Search.AcceptedEdges_units, 2), 4);
    verifyEqual(testCase, size(resultDiagnosis.Search.RejectedEdges_units, 2), 4);
    verifyTrue(testCase, all(isfinite(resultDiagnosis.Search.AcceptedEdges_units), "all"));
    verifyEqual(testCase, size(resultDiagnosis.Search.FrontierNodes_units, 2), 2);
    verifyTrue(testCase, resultDiagnosis.SearchCoverage.ExactSpatialProposalUsed);
    verifyFalse(testCase, resultDiagnosis.SearchCoverage.ReducedSpatialProposalUsed);
    verifyTrue(testCase, resultDiagnosis.SearchCoverage.CompletenessLost);
    verifyEqual(testCase, resultDiagnosis.SearchCoverage.CompletenessLossReason, "boundedSeedNodeAndTimeSearch");
    verifySize(testCase, resultDiagnosis.Search.RouteClassRepresentative_units, [1 2]);
    verifyGreaterThanOrEqual(testCase, resultDiagnosis.Search.RouteClassCount, 2);
    signatureCount = size(unique(resultDiagnosis.Search.RouteClassSignatures, "rows"), 1);
    verifyGreaterThanOrEqual(testCase, signatureCount, 2);
    verifyFalse(testCase, resultDiagnosis.Search.RouteClassSearchTruncated);
    minimumYs_units = zeros(numel(resultDiagnosis.Routes), 1);
    maximumYs_units = zeros(numel(resultDiagnosis.Routes), 1);

    % Measure every generated seed's y range to confirm both detour classes exist.
    for seedIndex = 1:numel(resultDiagnosis.Routes)
        minimumYs_units(seedIndex) = min(resultDiagnosis.Routes(seedIndex).position_units(:, 2));
        maximumYs_units(seedIndex) = max(resultDiagnosis.Routes(seedIndex).position_units(:, 2));
    end
    verifyLessThan(testCase, min(minimumYs_units), -2);
    verifyGreaterThan(testCase, max(maximumYs_units), 2);
    verifyTrue(testCase, result.Success, result.Message);
    verifyTrue(testCase, result.Validation.CollisionFree);
    validated = find([resultDiagnosis.Attempts.ValidationPassed]);
    if result.Success
        selectedSummary = resultDiagnosis.Attempts(resultDiagnosis.SelectedAttemptIndex);
        if options.GoalTimeMode == "fixedArrival"
            motionLength_units = [resultDiagnosis.Attempts(validated).MotionLength_units];
            verifyLessThanOrEqual(testCase, selectedSummary.MotionLength_units, min(motionLength_units) + 1e-9);
        else
            arrival_s = [resultDiagnosis.Attempts(validated).ArrivalTime_s];
            verifyLessThanOrEqual(testCase, selectedSummary.ArrivalTime_s, min(arrival_s) + options.ArrivalTimeTolerance_s);
        end
    end
end

function testTopologyChangeUsesAStationaryConservativeUnion(testCase, ~)
    % Verify a topology-change interval has constant conservative geometry.
    closed_units = [-2 -1; 2 -1; 2 1; -2 1];
    left_units   = [-2 -1; -0.5 -1; -0.5 1; -2 1];
    right_units  = [0.5 -1; 2 -1; 2 1; 0.5 1];
    open_units   = [left_units; NaN NaN; right_units];
    obstacle   = obstacleAvoidance.obstacles.createObstacle("opening", [0; 2], {closed_units(:, 1); open_units(:, 1)}, {closed_units(:, 2); open_units(:, 2)}, 0);
    [shape, geometry] = obstacleAvoidance.obstacles.shapeAtTime(obstacle, 1);
    verifyFalse(testCase, geometry.TopologyIsInterpolated);
    verifyEqual(testCase, geometry.VertexSpeedBound_units_s, 0);
    verifyTrue(testCase, isinterior(shape, 0, 0));
end

function testTranslatedHistoryReusesExactProtectedShape(testCase, ~)
    % Verify rigid obstacle motion preserves one translated protected boundary.
    source_units      = [-1 -1; 1 -1; 1 1; -1 1];
    translation_units = [3 2];
    x_units     = {source_units(:, 1); source_units(:, 1) + translation_units(1)};
    y_units   = {source_units(:, 2); source_units(:, 2) + translation_units(2)};
    obstacle        = obstacleAvoidance.obstacles.createObstacle("translated", [0; 1], x_units, y_units, 0.2);
    verifyEqual(testCase, obstacle.x_units{2}, obstacle.x_units{1} + translation_units(1), "AbsTol", 1e-12);
    verifyEqual(testCase, obstacle.y_units{2}, obstacle.y_units{1} + translation_units(2), "AbsTol", 1e-12);
end

function testUnrelatedPolynomialCannotValidateSampledHistory(testCase, adapter)
    % Verify valid samples cannot hide unrelated polynomial coefficients.
    duration_s   = 2;
    initialState = testCase.TestData.Fixtures.State(0, [0 0], [0 0], [0 0]);
    goalState    = testCase.TestData.Fixtures.State(duration_s, [4 / 3 0], [2 0], [2 0]);
    limits       = testCase.TestData.Fixtures.PhysicalLimits([3 3], [3 3], [2 2]);
    trajectory   = testCase.TestData.Fixtures.ConstantJerkTrajectory(duration_s);
    trajectory.Polynomial.positionPower_units(:) = 0;
    trajectory.Polynomial.velocityPower_units_s(:) = 0;
    trajectory.Polynomial.accelerationPower_units_s2(:) = 0;
    trajectory.Polynomial.jerkPower_units_s3(:) = 0;
    validation = obstacleAvoidance.validateTrajectory(trajectory, [], initialState, goalState, limits, adapter.FixedOptions());
    verifyFalse(testCase, validation.Passed);
    verifyTrue(testCase, validation.PolynomialFormatValid);
    verifyFalse(testCase, validation.PolynomialEndpointStatesMatched);
    verifyFalse(testCase, validation.PolynomialHistoryConsistent);
end

function testWorkspaceIntervalsBelongToLimits(testCase, adapter)
    % Check that omitted and explicit workspace intervals use the limits input.
    initialState = testCase.TestData.Fixtures.State(0, [0 0], [0 0], [0 0]);
    goalState    = testCase.TestData.Fixtures.State(6, [2 0], [0 0], [0 0]);
    limits       = rmfield(testCase.TestData.Fixtures.PhysicalLimits([2 2], [1 1], [2 2]), ["xInterval_units", "yInterval_units"]);
    [result, resultDiagnosis] = obstacleAvoidance.planTrajectory([], initialState, goalState, limits, adapter.FixedOptions());
    verifyEqual(testCase, result.Inputs.limits.xInterval_units, [-180 180]);
    verifyEqual(testCase, result.Inputs.limits.yInterval_units, [-90 90]);
    limits.xInterval_units   = [-12 14];
    limits.yInterval_units = [-5 6];
    [result, resultDiagnosis] = obstacleAvoidance.planTrajectory([], initialState, goalState, limits, adapter.FixedOptions());
    verifyEqual(testCase, result.Inputs.limits.xInterval_units, [-12 14]);
    verifyEqual(testCase, result.Inputs.limits.yInterval_units, [-5 6]);
end
