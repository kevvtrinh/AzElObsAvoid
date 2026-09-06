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

function testPlannerStagesUseOnlyTheirInputs(testCase)
% Internal stages must not accept unused inputs to imitate the public API.
stages = ["obstacles.preparePlanningScene", "search.createRouteSearchGeometry", ...
    "search.createVisibilityGraph", "search.createPathGuesses", "search.searchRoutes", ...
    "planner.tryDirectAndFixedTimeMotions", "planner.solvePathGuess", ...
    "planner.solveDynamicPathGuess", "planner.tryAdditionalPathGuesses"];
for index = 1:numel(stages)
    name = "obstacleAvoidance." + stages(index);
    issues = checkcode(which(name), '-id');
    verifyFalse(testCase, any(strcmp({issues.id}, 'INUSD')), name);
end
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

% Planner behavior and recovery are exercised by the contract, engine, and
% timed-topology tests; internal filenames and source call order are not an API.

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
