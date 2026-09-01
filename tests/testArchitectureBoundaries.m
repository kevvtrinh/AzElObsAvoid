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

function testHighLevelProductionRootsAreExplicit(testCase)
% Keep the public planner and dimension-neutral trajectory engine visible.
repositoryRoot = testCase.TestData.RepositoryRoot;
packageRecords = dir(fullfile(repositoryRoot, "+*"));
packageNames = string({packageRecords([packageRecords.isdir]).name});
verifyEqual(testCase, sort(packageNames), "+obstacleAvoidance");
verifyTrue(testCase, isfolder(testCase.TestData.TrajectoryRoot));
verifyEmpty(testCase, dir(fullfile(repositoryRoot, "*.m")));
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
end

function testTrajectoryRootContainsEnginePackagesAndRuckigFacade(testCase)
% Keep engines and conic ownership packaged while Ruckig facade is explicit.
trajectoryRoot = testCase.TestData.TrajectoryRoot;
actualRootSources = string({dir(fullfile(trajectoryRoot, "*.m")).name});
verifyEqual(testCase, sort(actualRootSources), "planTrajRuckig.m");
packageRecords = dir(fullfile(trajectoryRoot, "+*"));
actualNames = sort(string({packageRecords([packageRecords.isdir]).name}));
verifyEqual(testCase, actualNames, ...
    sort(["+bmtpEngine", "+conicSolver", "+ruckigEngine"]));
actualSources = sort(string({dir(fullfile( ...
    testCase.TestData.EngineRoot, "*.m")).name}));
expectedSources = sort([ ...
    "createConstantJerkPowerCoefficients.m", ...
    "createDelayedMotion.m", "createDirectMotion.m", ...
    "createCoordinateTolerances.m", "createMotionRecord.m", ...
    "createOffsetSplineMotion.m", ...
    "createProgressPolynomialMotion.m", "evaluatePolynomial.m", ...
    "maximumRestToRestDistance.m", "solve.m"]);
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
    plannerRoot, "planCorridorQuintic.m")));
fixedClockText = string(fileread(fullfile( ...
    plannerRoot, "createFixedClockLateralExcursion.m")));
validatorText = string(fileread(fullfile( ...
    testCase.TestData.ProductRoot, "validateTrajectory.m")));
verifyTrue(testCase, contains(adapterText, "bmtpEngine.solve("));
verifyTrue(testCase, contains(ruckigAdapterText, "ruckigEngine.solve("));
verifyTrue(testCase, contains(plannerText, "bmtpEngine.createDirectMotion("));
verifyTrue(testCase, contains(plannerText, ...
    "obstacleAvoidance.planner.createRuckigWaypointMotion("));
verifyTrue(testCase, contains(fixedClockText, ...
    "bmtpEngine.createOffsetSplineMotion("));
verifyTrue(testCase, contains(fixedClockText, ...
    "bmtpEngine.createProgressPolynomialMotion("));
verifyTrue(testCase, contains(validatorText, ...
    "bmtpEngine.evaluatePolynomial("));
end

function testProductionFunctionBasenamesAreUnique(testCase)
% Keep stack traces and repository searches unambiguous across both roots.
sourceRecords = [ ...
    dir(fullfile(testCase.TestData.ProductRoot, "**", "*.m")); ...
    dir(fullfile(testCase.TestData.EngineRoot, "**", "*.m"))];
sourceNames = lower(string({sourceRecords.name}));
[uniqueNames, ~, nameGroup] = unique(sourceNames);
nameCounts = accumarray(nameGroup(:), 1);
duplicateNames = uniqueNames(nameCounts > 1);
verifyEmpty(testCase, duplicateNames, sprintf( ...
    "Duplicate production function basenames: %s", ...
    strjoin(duplicateNames, ", ")));
end

function testDeletedHs3EngineTreeRemainsAbsent(testCase)
% Do not restore the deleted HS3 implementation beside the compact engines.
trajectoryRoot = testCase.TestData.TrajectoryRoot;
verifyFalse(testCase, isfolder(fullfile(trajectoryRoot, "+hs3Engine")));
verifyFalse(testCase, isfile(fullfile(trajectoryRoot, "planTrajHs3.m")));
verifyTrue(testCase, isfolder(fullfile(trajectoryRoot, "+ruckigEngine")));
verifyFalse(testCase, isfile(fullfile(trajectoryRoot, "planTrajBmtp.m")));
verifyTrue(testCase, isfile(fullfile(trajectoryRoot, "planTrajRuckig.m")));
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
    plannerRoot, "planCorridorQuintic.m"))));
verifyFalse(testCase, contains(plannerText, "orthogonal"));
verifyFalse(testCase, contains(plannerText, "cavity"));
end
