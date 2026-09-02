function tests = testPlannerContract
%% Section 0: Header & Readme
% SYNTAX
%   tests = testPlannerContract
%**************************************************************************
% PURPOSE
%   - Verify the maintained public planner contract without legacy-engine
%     names, private entry points, or implementation-specific solver counts.
%**************************************************************************
% INPUTS
%   - None.
%**************************************************************************
% OUTPUTS
%   - tests (matlab.unittest function test array)
%       Deterministic public success, failure, validation, and diagnostic tests.
%**************************************************************************
% UNITS
%   - Position is degrees; time is seconds; derivatives use deg/s powers.
%**************************************************************************
tests = functiontests(localfunctions);
end

function setupOnce(~)
% Add the public planner and independent BMTP engine to the MATLAB path.
repositoryRoot = fileparts(fileparts(mfilename("fullpath")));
addpath(repositoryRoot, fullfile(repositoryRoot, "trajectory"));
end

function testPlanningRequestOwnsResolvedNormalizedInputs(testCase)
% Keep option resolution and physical-input normalization in one stage.
initialState = struct("time_s", 0, "position_deg", [0 0]);
goalState = struct("time_s", 12, "position_deg", [4 2]);
request = obstacleAvoidance.input.createPlanningRequest( ...
    [], initialState, goalState, physicalLimits(), ...
    struct("MaximumSeedCount", 2));

verifyEqual(testCase, fieldnames(request), ...
    {'obstacles'; 'initialState'; 'goalState'; 'limits'; 'options'});
verifyEqual(testCase, request.options.MaximumSeedCount, 2);
verifyEqual(testCase, request.initialState.velocity_deg_s, [0 0]);
verifyEqual(testCase, request.initialState.acceleration_deg_s2, [0 0]);
verifyEqual(testCase, request.goalState.velocity_deg_s, [0 0]);
verifyEqual(testCase, request.goalState.acceleration_deg_s2, [0 0]);
end

function testPlanningSceneOwnsPreparedHistoriesAndHorizon(testCase)
% Expose prepared obstacle details and one shared horizon classification.
obstacleTime_s = [0; 10];
staticObstacle = obstacleAvoidance.obstacles.createObstacle( ...
    "static", obstacleTime_s, [-1 1 1 -1], [-1 -1 1 1], 0.1);
movingStart_deg = [4 -1; 6 -1; 6 1; 4 1];
movingEnd_deg = movingStart_deg + [1 0];
movingObstacle = obstacleAvoidance.obstacles.createObstacle( ...
    "moving", obstacleTime_s, ...
    {movingStart_deg(:, 1); movingEnd_deg(:, 1)}, ...
    {movingStart_deg(:, 2); movingEnd_deg(:, 2)}, 0.1);
request = obstacleAvoidance.input.createPlanningRequest( ...
    obstacleAvoidance.obstacles.combineObstacles( ...
    staticObstacle, movingObstacle), ...
    restState(0, [-4 0]), restState(10, [8 0]), ...
    physicalLimits(), plannerOptions("earliestArrival"));
scene = obstacleAvoidance.obstacles.preparePlanningScene(request);

verifyEqual(testCase, scene.startTime_s, 0);
verifyEqual(testCase, scene.endTime_s, 10);
verifyFalse(testCase, scene.isStaticHorizon);
verifyEqual(testCase, numel(scene.preparedObstacles), 2);
verifyEqual(testCase, [scene.obstacleDetails.SampleCount], [2 2]);
verifyTrue(testCase, scene.obstacleDetails(1).IsTimeInvariant);
verifyFalse(testCase, scene.obstacleDetails(2).IsTimeInvariant);
verifyEqual(testCase, scene.obstacleDetails(2).IntervalGeometryMethod, ...
    "linearCorrespondingVertices");

proposal = obstacleAvoidance.search.createProposalGeometry(scene, request);
verifyEqual(testCase, proposal.start_deg, [-4 0]);
verifyEqual(testCase, proposal.goal_deg, [8 0]);
verifyEqual(testCase, proposal.sampleTimes_s, ...
    linspace(0, 10, 9).');
maximumVerticesPerObstacle = zeros(1, numel(scene.preparedObstacles));
for obstacleIndex = 1:numel(scene.preparedObstacles)
    maximumVerticesPerObstacle(obstacleIndex) = max(cellfun( ...
        @numel, scene.preparedObstacles(obstacleIndex).az_deg));
end
expectedVertexWork = numel(proposal.sampleTimes_s) * ...
    sum(maximumVerticesPerObstacle);
verifyEqual(testCase, proposal.estimatedVertexWork, expectedVertexWork);
verifyEqual(testCase, proposal.representation, "sampledObstacleUnion");
verifyFalse(testCase, proposal.usedDenseEnvelope);
verifyEqual(testCase, proposal.sampledShapeCount, 18);
verifyEqual(testCase, size(proposal.edgeStart_deg), ...
    size(proposal.edgeEnd_deg));
end

function testObstacleFreeEarliestMotionPassesPublicValidation(testCase)
% Require a finite rest-to-rest direct motion inside the supplied horizon.
initialState = restState(0, [0 0]);
goalState = restState(20, [4 2]);
result = obstacleAvoidance.planTrajectory( ...
    [], initialState, goalState, physicalLimits(), plannerOptions("earliestArrival"));

verifyTrue(testCase, result.Success, result.Message);
verifyTrue(testCase, result.Validation.Passed, result.Validation.Message);
verifyTrue(testCase, result.Validation.CollisionFree);
verifyTrue(testCase, result.Validation.VelocityWithinLimits);
verifyTrue(testCase, result.Validation.AccelerationWithinLimits);
verifyTrue(testCase, result.Validation.JerkWithinLimits);
verifyEqual(testCase, result.TerminationReason, "goalReached");
verifyEqual(testCase, result.SeedSummaries(1).SeedSource, ...
    "directRestToRest");
verifyFalse(testCase, result.SearchDiagnostics.SeedEarlyExit.Applied);
verifyEqual(testCase, result.SearchDiagnostics.SeedEarlyExit.Reason, ...
    "notApplicableExactPath");
verifyLessThan(testCase, result.TrajectoryDuration_s, ...
    goalState.time_s - initialState.time_s);
end

function testBernsteinOutlierIsNotAnExactRangeRejection(testCase)
% Require subdivision because one control can exceed the polynomial range.
safePower = [0; 4; -4];
within = obstacleAvoidance.validation.certifyPolynomialRange( ...
    safePower, -0.1, 1.1, 0);
verifyTrue(testCase, within);
end

function testBernsteinSubdivisionFindsInteriorViolation(testCase)
% Reject a true interior peak even though both endpoints satisfy the bounds.
violatingPower = [0; 4.4; -4.4];
within = obstacleAvoidance.validation.certifyPolynomialRange( ...
    violatingPower, -0.1, 0.5, 0);
verifyFalse(testCase, within);
end

function testStationaryFallbackAcceptsNondyadicTangent(testCase)
% Keep exact stationary-point resolution for a boundary tangent at tau=1/3.
tangentPower = [8 / 9; 2 / 3; -1];
within = obstacleAvoidance.validation.certifyPolynomialRange( ...
    tangentPower, -0.1, 1, 1e-12);
verifyTrue(testCase, within);
end

function testSuccessAndEndpointFailureShareResultShape(testCase)
% Preserve every public field on expected failure as well as success.
initialState = restState(0, [-2 0]);
goalState = restState(10, [2 0]);
limits = physicalLimits();
options = plannerOptions("earliestArrival");
success = obstacleAvoidance.planTrajectory( ...
    [], initialState, goalState, limits, options);
blocking = obstacleAvoidance.obstacles.createObstacle( ...
    "blocked start", [0; 10], [-3 -1 -1 -3], [-1 -1 1 1], 0);
failure = obstacleAvoidance.planTrajectory( ...
    blocking, initialState, goalState, limits, options);

verifyTrue(testCase, success.Success);
verifyFalse(testCase, failure.Success);
verifyEqual(testCase, failure.TerminationReason, "endpointBlocked");
verifyEqual(testCase, fieldnames(failure), fieldnames(success));
verifyEqual(testCase, fieldnames(failure.Validation), ...
    fieldnames(success.Validation));
verifyEqual(testCase, fieldnames(failure.PlaneCertificate), ...
    fieldnames(success.PlaneCertificate));
end

function testStaticDetourReturnsCertifiedCollisionFreeMotion(testCase)
% Exercise topology generation, BMTP, and independent static-plane replay.
obstacle = obstacleAvoidance.obstacles.createObstacle( ...
    "center box", [0; 30], [-1 1 1 -1], [-1 -1 1 1], 0.2);
initialState = restState(0, [-4 0]);
goalState = restState(30, [4 0]);
result = obstacleAvoidance.planTrajectory( ...
    obstacle, initialState, goalState, physicalLimits(), ...
    plannerOptions("earliestArrival"));

verifyTrue(testCase, result.Success, result.Message);
verifyTrue(testCase, result.Validation.Passed, result.Validation.Message);
verifyTrue(testCase, result.Validation.CollisionFree);
verifyTrue(testCase, result.Validation.CollisionResolved);
verifyGreaterThan(testCase, result.Validation.MinimumClearance_deg, 0);
verifyGreaterThan(testCase, ...
    sum(vecnorm(diff(result.position_deg, 1, 1), 2, 2)), 8);
verifyFalse(testCase, result.SearchDiagnostics.SeedEarlyExit.Applied);
end

function testBalancedArrivalRefinesObjectiveRelevantDirectWait(testCase)
% Remove avoidable wait when time contributes to the balanced objective.
obstacleTime_s = [0; 6; 6.5; 12];
barrierCenterElevation_deg = [0; 0; 8; 8];
sourcePosition_deg = [-0.2 -3; 0.2 -3; 0.2 3; -0.2 3];
azimuthBySlice_deg = cell(numel(obstacleTime_s), 1);
elevationBySlice_deg = cell(numel(obstacleTime_s), 1);
for sampleIndex = 1:numel(obstacleTime_s)
    translatedPosition_deg = sourcePosition_deg + ...
        [0 barrierCenterElevation_deg(sampleIndex)];
    azimuthBySlice_deg{sampleIndex} = translatedPosition_deg(:, 1);
    elevationBySlice_deg{sampleIndex} = translatedPosition_deg(:, 2);
end
obstacle = obstacleAvoidance.obstacles.createObstacle( ...
    "balanced direct wait", obstacleTime_s, ...
    azimuthBySlice_deg, elevationBySlice_deg, 0.1);
initialState = restState(0, [-5 0]);
goalState = restState(12, [5 0]);
limits = physicalLimits();
limits.azimuthInterval_deg = [-6 6];
limits.elevationInterval_deg = [-3 3];
options = struct( ...
    "GoalTimeMode", "balancedArrival", ...
    "MaximumSeedCount", 5, ...
    "SampleTime_s", 0.05);

result = obstacleAvoidance.planTrajectory( ...
    obstacle, initialState, goalState, limits, options);
directWaitIndex = find( ...
    string({result.Seeds.Source}) == "directWait", 1);
verifyTrue(testCase, result.Success, result.Message);
verifyNotEmpty(testCase, directWaitIndex);
diagnostics = result.SeedSummaries(directWaitIndex).SolverDiagnostics;
verifyGreaterThan(testCase, diagnostics.RefinementCount, 0);
verifyLessThan(testCase, diagnostics.FinalWaitTime_s, ...
    diagnostics.InitialWaitTime_s);
end

function testPlaneReuseSummaryAndRetainedBestTrial(testCase)
% Preserve behavior-bearing reuse summary and best-trial evidence.
repositoryRoot = fileparts(fileparts(mfilename("fullpath")));
addpath(fullfile(repositoryRoot, "examples"));
overrides = struct( ...
    "CollisionClearanceTolerance_deg", 1e-4, ...
    "MaximumSeedCount", 2, ...
    "FigureVisible", "off", "PlotOutputs", false, ...
    "ShowAnimation", false, "ShowKinematicPlot", false);
result = exampleTargetExitsObstacle(overrides);

verifyTrue(testCase, result.Success, result.Message);
verifyTrue(testCase, result.Validation.Passed, result.Validation.Message);
diagnostics = result.SeedSummaries(result.SelectedSeedIndex).SolverDiagnostics;
verifyFalse(testCase, any(startsWith( ...
    string(fieldnames(diagnostics)), "TravelRefinement")));
verifyTrue(testCase, diagnostics.PlaneReuseApplied);
verifyGreaterThan(testCase, diagnostics.PlaneReuseCount, 0);
verifyTrue(testCase, diagnostics.Converged);
verifyEqual(testCase, diagnostics.SolverMessage, ...
    "The next trajectory SOCP would be unchanged.");
verifyGreaterThan(testCase, diagnostics.CollisionPairCountHistory(1), 0);
collisionFreeTrials_s = diagnostics.TrialDuration_s( ...
    diagnostics.TrialWasCollisionFree);
verifyEqual(testCase, diagnostics.RetainedBestTrialDuration_s, ...
    min(collisionFreeTrials_s), "AbsTol", 1e-12);
end

function testReflectedProgressAxisRanksValidatedFamiliesByLength(testCase)
% Rank two validated fixed-clock families under negative first-axis progress.
obstacleTime_s = [0; 30];
firstObstacle = obstacleAvoidance.obstacles.createObstacle( ...
    "first floating barrier", obstacleTime_s, ...
    [4.4 5.6 5.6 4.4], [-0.45 -0.45 0.45 0.45], 0.1);
secondObstacle = obstacleAvoidance.obstacles.createObstacle( ...
    "second floating barrier", obstacleTime_s, ...
    [-5.6 -4.4 -4.4 -5.6], [-0.45 -0.45 0.45 0.45], 0.1);
obstacles = obstacleAvoidance.obstacles.combineObstacles( ...
    firstObstacle, secondObstacle);
initialState = restState(0, [10 0]);
goalState = restState(30, [-10 0]);
limits = physicalLimits();
limits.azimuthInterval_deg = [-12 12];
limits.elevationInterval_deg = [-4 4];
result = obstacleAvoidance.planTrajectory( ...
    obstacles, initialState, goalState, limits, ...
    plannerOptions("earliestArrival"));

verifyTrue(testCase, result.Success, result.Message);
verifyTrue(testCase, result.Validation.Passed, result.Validation.Message);
verifyTrue(testCase, result.Validation.CollisionFree);
verifyTrue(testCase, result.Validation.CollisionResolved);
diagnostics = result.SearchDiagnostics.FixedClockExcursion;
verifyTrue(testCase, diagnostics.Success, diagnostics.Message);
verifyEqual(testCase, diagnostics.SelectedMode, "singleAmplitude");
verifyTrue(testCase, diagnostics.ProgressPolynomial.Success, ...
    diagnostics.ProgressPolynomial.Message);
verifyEqual(testCase, ...
    diagnostics.ProgressPolynomial.ProgressAxisIndex, 1);
verifyEqual(testCase, ...
    diagnostics.ProgressPolynomial.LateralAxisIndex, 2);
verifyEqual(testCase, result.TrajectoryDuration_s, 12.5, ...
    "AbsTol", 1e-9);
verifyEqual(testCase, result.SelectedSeed_deg, result.position_deg);
verifyEqual(testCase, result.Seeds(1).Length_deg, ...
    sum(vecnorm(diff(result.position_deg), 2, 2)), "AbsTol", 1e-12);
verifyLessThan(testCase, result.Seeds(1).Length_deg, ...
    diagnostics.ProgressPolynomial.SelectedMotionLength_deg, ...
    "The planner did not retain the shorter validated fixed-clock family.");
end

function testNearStartBarrierUsesShorterOneSidedExactClockFamily(testCase)
% A local static obstruction must not force an alternating full-path tail.
obstacle = obstacleAvoidance.obstacles.createObstacle( ...
    "offset rectangle", [0; 30], [-9 -6 -6 -9], [0 0 2.8 2.8], 0.1);
initialState = restState(0, [-10 3.2]);
goalState = restState(30, [10 -1]);
limits = physicalLimits();
limits.azimuthInterval_deg = [-20 20];
limits.elevationInterval_deg = [-10 10];
result = obstacleAvoidance.planTrajectory( ...
    obstacle, initialState, goalState, limits, ...
    plannerOptions("earliestArrival"));

verifyTrue(testCase, result.Success, result.Message);
verifyTrue(testCase, result.Validation.Passed, result.Validation.Message);
progressReport = result.SearchDiagnostics.FixedClockExcursion.ProgressPolynomial;
verifyTrue(testCase, startsWith( ...
    progressReport.SelectedBasis, "oneSidedBeta_"));
alternatingIndex = find([progressReport.BasisReports.Name] == ...
    "alternatingDegreeFive", 1, "first");
verifyNotEmpty(testCase, alternatingIndex);
verifyLessThan(testCase, sum(vecnorm(diff(result.position_deg), 2, 2)), ...
    progressReport.BasisReports(alternatingIndex).SelectedMotionLength_deg);
progress = (result.position_deg(:, 1) - initialState.position_deg(1)) / ...
    (goalState.position_deg(1) - initialState.position_deg(1));
directElevation_deg = initialState.position_deg(2) + ...
    (goalState.position_deg(2) - initialState.position_deg(2)) * progress;
verifyGreaterThanOrEqual(testCase, ...
    min(result.position_deg(:, 2) - directElevation_deg), -1e-9);
end

function testBalancedDefaultReportsTradeoffAndUtilization(testCase)
% Expose the declared selection cost and measured envelope use on success.
initialState = restState(0, [0 0]);
goalState = restState(20, [4 2]);
result = obstacleAvoidance.planTrajectory( ...
    [], initialState, goalState, physicalLimits());

verifyTrue(testCase, result.Success, result.Message);
summary = result.SeedSummaries(result.SelectedSeedIndex);
expectedCost_deg = summary.MotionLength_deg + ...
    result.Options.MinimumTravelSavingsRate_deg_s * ...
    summary.MotionDuration_s;
verifyEqual(testCase, summary.TravelTimeTradeoffCost_deg, ...
    expectedCost_deg, "AbsTol", 1e-10);
verifyGreaterThan(testCase, summary.KinematicUtilization, 0);
verifyLessThanOrEqual(testCase, summary.KinematicUtilization, 1 + 1e-6);
verifyEqual(testCase, ...
    result.SearchDiagnostics.SelectionPolicy.JerkRole, ...
    "hardConstraintOnly");
end

function testFixedArrivalBelowPhysicalMinimumReturnsFailure(testCase)
% An infeasible clock is an expected result rather than a thrown error.
initialState = restState(0, [0 0]);
goalState = restState(0.25, [1 0]);
limits = physicalLimits();
result = obstacleAvoidance.planTrajectory( ...
    [], initialState, goalState, limits, plannerOptions("fixedArrival"));

verifyFalse(testCase, result.Success);
verifyNotEmpty(testCase, result.Message);
verifyTrue(testCase, any(result.TerminationReason == ...
    ["noValidatedSeed", "fixedArrivalInfeasible", "timeWindowInfeasible"]));
verifyEmpty(testCase, result.time_s);
verifyTrue(testCase, isfield(result.SearchDiagnostics, "StageTiming"));
end

function testUnsupportedEndpointDerivativesThrowNamedError(testCase)
% The compact BMTP scope rejects non-rest endpoints explicitly.
initialState = restState(0, [0 0]);
initialState.velocity_deg_s = [0.1 0];
goalState = restState(10, [2 0]);
request = @() obstacleAvoidance.planTrajectory( ...
    [], initialState, goalState, physicalLimits(), ...
    plannerOptions("earliestArrival"));
verifyError(testCase, request, "bmtpEngine:UnsupportedRequest");
end

function testNoPathReturnsRecognizedDiagnostics(testCase)
% A full-height wall must fail with retained search evidence.
wall = obstacleAvoidance.obstacles.createObstacle( ...
    "full-height wall", [0; 20], [-0.5 0.5 0.5 -0.5], ...
    [-90 -90 90 90], 0);
initialState = restState(0, [-5 0]);
goalState = restState(12, [5 0]);
limits = physicalLimits();
limits.elevationInterval_deg = [-10 10];
options = plannerOptions("earliestArrival");
options.MaximumSeedCount = 3;
result = obstacleAvoidance.planTrajectory( ...
    wall, initialState, goalState, limits, options);

verifyFalse(testCase, result.Success);
verifyEqual(testCase, result.TerminationReason, "noValidatedSeed");
verifyEmpty(testCase, result.time_s);
verifyTrue(testCase, isfield(result.SearchDiagnostics.Grid, ...
    "ExpandedCount"));
verifyGreaterThanOrEqual(testCase, ...
    result.SearchDiagnostics.AttemptedSeedCount, 1);
verifyFalse(testCase, result.SearchDiagnostics.SeedEarlyExit.Applied);
verifyEqual(testCase, result.SearchDiagnostics.SeedEarlyExit.Reason, ...
    "lowerBoundNotReached");
end

function testRepeatedDirectRequestIsDeterministic(testCase)
% Identical input must reproduce every selected motion sample exactly.
initialState = restState(0, [-1 0.5]);
goalState = restState(12, [3 -1]);
options = plannerOptions("earliestArrival");
first = obstacleAvoidance.planTrajectory( ...
    [], initialState, goalState, physicalLimits(), options);
second = obstacleAvoidance.planTrajectory( ...
    [], initialState, goalState, physicalLimits(), options);

verifyEqual(testCase, second.TerminationReason, first.TerminationReason);
verifyEqual(testCase, second.time_s, first.time_s);
verifyEqual(testCase, second.position_deg, first.position_deg);
verifyEqual(testCase, second.velocity_deg_s, first.velocity_deg_s);
verifyEqual(testCase, second.acceleration_deg_s2, ...
    first.acceleration_deg_s2);
verifyEqual(testCase, second.jerk_deg_s3, first.jerk_deg_s3);
end

function testInterceptValidatesInitialStateBeforeFieldAccess(testCase)
% Use the shared identified state error for malformed public input.
targetMotion = struct("time_s", [0; 1], ...
    "position_deg", [0 0; 1 0]);
verifyError(testCase, @() obstacleAvoidance.planMovingTargetIntercept( ...
    struct(), targetMotion, physicalLimits(), struct()), ...
    "planTrajectory:InvalidState");
end

function testInterceptEchoesTheFixedArrivalModeItUses(testCase)
% Intercept trials select one terminal time, regardless of the caller mode.
targetMotion = struct("time_s", [0; 5], ...
    "position_deg", [0 0; 4 0]);
interceptOptions = struct("InterceptMode", "specifiedTime", ...
    "SpecifiedInterceptTime_s", 5, ...
    "PlannerOptions", plannerOptions("earliestArrival"));
result = obstacleAvoidance.planMovingTargetIntercept( ...
    restState(0, [0 0]), targetMotion, physicalLimits(), interceptOptions);

verifyTrue(testCase, result.Success, result.Message);
verifyEqual(testCase, result.Options.GoalTimeMode, "fixedArrival");
verifyEqual(testCase, result.Intercept.Options.PlannerOptions.GoalTimeMode, ...
    "fixedArrival");
end

function state = restState(time_s, position_deg)
% Create one two-axis rest endpoint.
state = struct("time_s", time_s, "position_deg", position_deg, ...
    "velocity_deg_s", [0 0], "acceleration_deg_s2", [0 0]);
end

function limits = physicalLimits()
% Create neutral two-axis bounds used by the public-contract cases.
limits = struct("maxVelocity_deg_s", [2 2], ...
    "maxAcceleration_deg_s2", [1 1], ...
    "maxJerk_deg_s3", [2 2], ...
    "azimuthInterval_deg", [-180 180], ...
    "elevationInterval_deg", [-90 90]);
end

function options = plannerOptions(goalTimeMode)
% Resolve only behavior-level public choices for deterministic tests.
options = struct("GoalTimeMode", goalTimeMode, ...
    "SampleTime_s", 0.05);
end
