function tests = testArchitectureBoundaries
%% Section 0: Header & Readme
% SYNTAX
%   tests = testArchitectureBoundaries
%**************************************************************************
% PURPOSE
%   - Protect one-way ownership between planning and both motion engines.
%**************************************************************************
% INPUTS
%   - None.
%**************************************************************************
% OUTPUTS
%   - tests (matlab.unittest function test array)
%**************************************************************************
% UNITS
%   - Not applicable.
%**************************************************************************
tests = functiontests(localfunctions);
end

function setupOnce(testCase)
% Locate both production roots once for source-layout and dependency checks.
repositoryRoot = fileparts(fileparts(mfilename("fullpath")));
trajectoryRoot = fullfile(repositoryRoot, "trajectory");
addpath(repositoryRoot, trajectoryRoot);
testCase.TestData.RepositoryRoot = repositoryRoot;
testCase.TestData.ProductRoot = fullfile(repositoryRoot, "+obstacleAvoidance");
testCase.TestData.TrajectoryRoot = trajectoryRoot;
testCase.TestData.EngineRoot = fullfile(trajectoryRoot, "+bmtpEngine");
testCase.TestData.RuckigRoot = fullfile(trajectoryRoot, "+ruckigEngine");
end

function testObstacleAvoidancePackagesMatchResponsibilities(testCase)
% Require one shallow package for each planner-owned responsibility.
productRoot = testCase.TestData.ProductRoot;
packageRecords = dir(fullfile(productRoot, "+*"));
actualNames = sort(string({packageRecords([packageRecords.isdir]).name}));
expectedNames = sort([ ...
    "+geometry", "+input", "+obstacles", "+planner", ...
    "+plotting", "+search", "+validation"]);
verifyEqual(testCase, actualNames, expectedNames);
publicSources = sort(string({dir(fullfile(productRoot, "*.m")).name}));
expectedSources = sort(["planTrajectory.m", ...
    "planMovingTargetIntercept.m", "validateTrajectory.m"]);
verifyEqual(testCase, publicSources, expectedSources);
plottingRoot = fullfile(productRoot, "+plotting");
plottingSources = dir(fullfile(plottingRoot, "*.m"));
for sourceIndex = 1:numel(plottingSources)
    sourcePath = fullfile(plottingSources(sourceIndex).folder, ...
        plottingSources(sourceIndex).name);
    sourceText = string(fileread(sourcePath));
    forbiddenCall = "obstacleAvoidance.(planner|search)\.[A-Za-z0-9_]+\s*\(";
    verifyEmpty(testCase, regexp(sourceText, forbiddenCall, "once"), ...
        sprintf("Plotting source reruns planner work: %s", sourcePath));
end
searchRoot = fullfile(productRoot, "+search");
searchSources = dir(fullfile(searchRoot, "*.m"));
for sourceIndex = 1:numel(searchSources)
    sourcePath = fullfile(searchSources(sourceIndex).folder, ...
        searchSources(sourceIndex).name);
    sourceText = string(fileread(sourcePath));
    verifyFalse(testCase, contains(sourceText, "obstacleAvoidance.planner."), ...
        sprintf("Search source depends on planner adapters: %s", sourcePath));
end
end

function testTrajectoryRootContainsEnginePackagesAndRuckigFacade(testCase)
% Keep both engines packaged while the remaining Ruckig facade is explicit.
trajectoryRoot = testCase.TestData.TrajectoryRoot;
actualRootSources = string({dir(fullfile(trajectoryRoot, "*.m")).name});
verifyEqual(testCase, sort(actualRootSources), "planTrajRuckig.m");
packageRecords = dir(fullfile(trajectoryRoot, "+*"));
actualNames = sort(string({packageRecords([packageRecords.isdir]).name}));
verifyEqual(testCase, actualNames, sort(["+bmtpEngine", "+ruckigEngine"]));
actualSources = sort(string({dir(fullfile( ...
    testCase.TestData.EngineRoot, "*.m")).name}));
expectedSources = sort([ ...
    "createConstantJerkPowerCoefficients.m", ...
    "createDelayedMotion.m", "createDirectMotion.m", ...
    "createCoordinateTolerances.m", "createMotionRecord.m", ...
    "createMotionOutput.m", "createOffsetSplineMotion.m", ...
    "createPowerPolynomial.m", ...
    "createProgressPolynomialMotion.m", "createSolveRequest.m", ...
    "createWarmStart.m", "evaluatePolynomial.m", ...
    "checkFinalMotion.m", "findRequiredSegmentTime.m", ...
    "findSampledObstacleOverlaps.m", ...
    "maximumRestToRestDistance.m", "solve.m", ...
    "prepareFinalMotion.m", "refineTravel.m", ...
    "solveAlternatingTrajectory.m", "solveSeparatingLine.m", ...
    "solveTrajectoryStep.m", "verifySeparatingLine.m"]);
verifyEqual(testCase, actualSources, expectedSources);
end

function testRuckigEngineHasNoObstaclePlannerDependency(testCase)
% Keep exact switching equations independent of obstacle-route ownership.
sourceRecords = dir(fullfile(testCase.TestData.RuckigRoot, "**", "*.m"));
for sourceIndex = 1:numel(sourceRecords)
    sourcePath = fullfile(sourceRecords(sourceIndex).folder, ...
        sourceRecords(sourceIndex).name);
    sourceText = lower(string(fileread(sourcePath)));
    forbiddenPattern = ...
        "obstacleavoidance|bmtp|fmincon|coneprog|quadprog|optimoptions";
    verifyEmpty(testCase, regexp(sourceText, forbiddenPattern, "once"), ...
        sprintf("Ruckig source crosses its engine boundary: %s", sourcePath));
end
end

function testBmtpEngineHasNoObstaclePlannerDependency(testCase)
% Enforce the one-way dependency from obstacle avoidance into trajectory math.
sourceRecords = dir(fullfile(testCase.TestData.EngineRoot, "**", "*.m"));
for sourceIndex = 1:numel(sourceRecords)
    sourcePath = fullfile(sourceRecords(sourceIndex).folder, ...
        sourceRecords(sourceIndex).name);
    sourceText = string(fileread(sourcePath));
    verifyFalse(testCase, contains(sourceText, "obstacleAvoidance."), ...
        sprintf("BMTP source depends on obstacle planning: %s", sourcePath));
end
end

function testNumericalSolverCallsRemainInsideEngine(testCase)
% Prevent obstacle packages from absorbing the BMTP conic optimizer again.
sourceRecords = dir(fullfile(testCase.TestData.ProductRoot, "**", "*.m"));
solverPattern = "(?<![A-Za-z0-9_])" + ...
    "(coneprog|fmincon|quadprog|optimoptions)\s*\(";
offendingPaths = strings(0, 1);
for sourceIndex = 1:numel(sourceRecords)
    sourcePath = fullfile(sourceRecords(sourceIndex).folder, ...
        sourceRecords(sourceIndex).name);
    if ~isempty(regexp(string(fileread(sourcePath)), solverPattern, "once"))
        offendingPaths(end + 1, 1) = string(sourcePath); %#ok<AGROW>
    end
end
verifyEmpty(testCase, offendingPaths, sprintf( ...
    "Numerical solver call outside BMTP engine: %s", ...
    strjoin(offendingPaths, ", ")));
end

function testPlannerUsesOnlyNamedEngineEntries(testCase)
% Verify obstacle-aware adapters call the engine rather than duplicating it.
plannerRoot = fullfile(testCase.TestData.ProductRoot, "+planner");
adapterText = string(fileread(fullfile( ...
    plannerRoot, "solveBmtpTrajectory.m")));
ruckigAdapterText = string(fileread(fullfile( ...
    plannerRoot, "createRuckigWaypointMotion.m")));
plannerText = string(fileread(fullfile( ...
    testCase.TestData.ProductRoot, "planTrajectory.m")));
exactStageText = string(fileread(fullfile( ...
    plannerRoot, "solveExactCandidates.m")));
fixedClockText = string(fileread(fullfile( ...
    plannerRoot, "createFixedClockLateralExcursion.m")));
clearanceCheckText = string(fileread(fullfile( ...
    testCase.TestData.ProductRoot, "+validation", ...
    "checkObstacleClearance.m")));
verifyTrue(testCase, contains(adapterText, "bmtpEngine.solve("));
verifyTrue(testCase, contains(ruckigAdapterText, "ruckigEngine.solve("));
verifyTrue(testCase, contains(plannerText, ...
    "obstacleAvoidance.planner.solveExactCandidates("));
verifyTrue(testCase, contains(exactStageText, ...
    "bmtpEngine.createDirectMotion("));
verifyTrue(testCase, contains(exactStageText, ...
    "obstacleAvoidance.planner.createRuckigWaypointMotion("));
verifyTrue(testCase, contains(fixedClockText, ...
    "bmtpEngine.createOffsetSplineMotion("));
verifyTrue(testCase, contains(fixedClockText, ...
    "bmtpEngine.createProgressPolynomialMotion("));
verifyTrue(testCase, contains(clearanceCheckText, ...
    "bmtpEngine.evaluatePolynomial("));
stageCalls = [ ...
    "createPlanningRequest(", "preparePlanningScene(", ...
    "checkPlannerEndpoints(", "solveExactCandidates(", ...
    "createProposalGeometry(", "createVisibilityGraph(", ...
    "searchRoutes(", "createSeeds(", "solveSeeds(", ...
    "selectValidatedCandidate(", "createPlannerResult("];
plannerExecutable = extractAfter(plannerText, ...
    "%% Section 3: Create The Request And Prepare The Scene");
stageLocations = zeros(size(stageCalls));
for stageIndex = 1:numel(stageCalls)
    location = strfind(plannerExecutable, stageCalls(stageIndex));
    verifyNotEmpty(testCase, location, sprintf( ...
        "Planner trunk omits required stage: %s", stageCalls(stageIndex)));
    if stageCalls(stageIndex) == "solveExactCandidates("
        % The exact-candidate stage also supports a zero-input stable template;
        % its final occurrence is the request-dependent stage call.
        stageLocations(stageIndex) = location(end);
    else
        % Primary stages precede any recovery that deliberately resumes search.
        stageLocations(stageIndex) = location(1);
    end
end
verifyGreaterThan(testCase, diff(stageLocations), 0, ...
    "Planner stage calls must remain in production execution order.");
routeLocations = strfind(plannerExecutable, "searchRoutes(");
timedRecoveryLocations = strfind(plannerExecutable, "searchTimedRoute(");
seedLocations = strfind(plannerExecutable, "createSeeds(");
verifyEqual(testCase, numel(routeLocations), 1, ...
    "The planner must run the spatial route search exactly once.");
verifyEqual(testCase, numel(timedRecoveryLocations), 1, ...
    "The planner must own one recovery-only timed search.");
verifyEqual(testCase, numel(seedLocations), 3, ...
    "The planner must own two exclusive primary and one recovered conversion.");
verifyLessThan(testCase, seedLocations(1:2), ...
    stageLocations(stageCalls == "solveSeeds("), ...
    "Both mutually exclusive primary seed conversions must precede solving.");
verifyGreaterThan(testCase, timedRecoveryLocations, ...
    stageLocations(stageCalls == "solveSeeds("), ...
    "Deferred timed search must follow rejection of primary seeds.");
verifyGreaterThan(testCase, seedLocations(3), timedRecoveryLocations, ...
    "Recovered seed conversion must follow deferred timed search.");
verifyLessThan(testCase, seedLocations(3), ...
    stageLocations(stageCalls == "selectValidatedCandidate("), ...
    "Every recovered seed must exist before final candidate selection.");
end

function testScenarioSpecificOrthogonalPlannersRemainAbsent(testCase)
% Prevent restoration of benchmark-shaped cavity and opening constructors.
plannerRoot = fullfile(testCase.TestData.ProductRoot, "+planner");
removedSources = [ ...
    "createOrthogonalCavityMotion.m", ...
    "certifyOrthogonalCavityLowerBound.m", ...
    "createTimedOrthogonalOpeningMotion.m", ...
    "certifyTimedOpeningRequestLowerBound.m", ...
    "certifyGuardedRectangleContainment.m", ...
    "evaluateArrivalCertificatePortfolio.m"];
for sourceIndex = 1:numel(removedSources)
    verifyFalse(testCase, isfile(fullfile(plannerRoot, ...
        removedSources(sourceIndex))));
end
plannerText = lower(string(fileread(fullfile( ...
    testCase.TestData.ProductRoot, "planTrajectory.m"))));
verifyFalse(testCase, contains(plannerText, "orthogonal"));
verifyFalse(testCase, contains(plannerText, "cavity"));
end
