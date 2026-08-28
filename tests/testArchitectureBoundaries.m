function tests = testArchitectureBoundaries
%% Section 0: Header & Readme
% SYNTAX
%   tests = testArchitectureBoundaries
%**************************************************************************
% PURPOSE
%   - Protect obstacle avoidance and the two independent trajectory engines.
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
% Locate the repository once for all source-layout checks. These tests inspect
% file ownership and dependency direction. A failure usually means that code
% moved into the wrong package or gained a higher-level dependency.
% Record the two production roots and add their public path parents.
repositoryRoot = fileparts(fileparts(mfilename("fullpath")));
productRoot = fullfile(repositoryRoot, "+obstacleAvoidance");
trajectoryRoot = fullfile(repositoryRoot, "trajectory");
hs3Root = fullfile(trajectoryRoot, "+hs3Engine");
addpath(repositoryRoot);
addpath(trajectoryRoot);
testCase.TestData.RepositoryRoot = repositoryRoot;
testCase.TestData.ProductRoot = productRoot;
testCase.TestData.Hs3Root = hs3Root;
testCase.TestData.TrajectoryRoot = trajectoryRoot;
end

function testRootContainsOnlyHighLevelProductionRoots(testCase)
% Keep obstacle avoidance and trajectory engines as visible product roots.
repositoryRoot = testCase.TestData.RepositoryRoot;
directoryRecords = dir(fullfile(repositoryRoot, "+*"));
actualNames = sort(string({directoryRecords([directoryRecords.isdir]).name}));
verifyEqual(testCase, actualNames, "+obstacleAvoidance");
verifyEmpty(testCase, dir(fullfile(repositoryRoot, "*.m")), ...
    "Production MATLAB functions must not accumulate at the root.");
verifyFalse(testCase, isfolder(fullfile(repositoryRoot, "hs3")));
verifyTrue(testCase, isfolder(fullfile(repositoryRoot, "trajectory")));
end

function testObstacleAvoidancePackagesMatchHighLevelArchitecture(testCase)
% Verify the product contains six responsibility-oriented subpackages.
productRoot = testCase.TestData.ProductRoot;
directoryRecords = dir(fullfile(productRoot, "+*"));
actualNames = sort(string({directoryRecords([directoryRecords.isdir]).name}));
expectedNames = sort([ ...
    "+geometry", "+input", "+obstacles", ...
    "+planner", "+plotting", "+search"]);
verifyEqual(testCase, actualNames, expectedNames);
publicSources = sort(string({dir(fullfile(productRoot, "*.m")).name}));
expectedSources = sort(["planTrajectory.m", ...
    "planMovingTargetIntercept.m", "validateTrajectory.m"]);
verifyEqual(testCase, publicSources, expectedSources);
end

function testHs3ImplementationLivesInsideTrajectoryEngine(testCase)
% Protect the complete collocation implementation under its engine package.
hs3Root = testCase.TestData.Hs3Root;
actualSources = sort(string({dir(fullfile(hs3Root, "*.m")).name}));
expectedSources = sort([ ...
    "defaultOptions.m", "optimize.m", "solve.m", ...
    "solveFixedTime.m", "solveFreeTime.m", "validate.m"]);
verifyEqual(testCase, actualSources, expectedSources);
verifyTrue(testCase, isfolder(fullfile(hs3Root, "+constraints")));
verifyTrue(testCase, isfolder(fullfile(hs3Root, "+polynomial")));
end

function testTrajectoryRootExposesNamedPlanningEntries(testCase)
% Verify the trajectory root exposes only the two named public entry points.
trajectoryRoot = testCase.TestData.TrajectoryRoot;
sourceRecords = dir(fullfile(trajectoryRoot, "*.m"));
actualSources = sort(string({sourceRecords.name}));
verifyEqual(testCase, actualSources, ...
    sort(["planTrajHs3.m", "planTrajRuckig.m"]));
packageRecords = dir(fullfile(trajectoryRoot, "+*"));
actualPackages = sort(string({packageRecords([packageRecords.isdir]).name}));
expectedPackages = sort(["+ruckigEngine", "+hs3Engine"]);
verifyEqual(testCase, actualPackages, expectedPackages);
end

function testProductionPackageNestingMatchesArchitecture(testCase)
% Allow one product namespace level and cohesive engine-owned HS3 packages.
productRoot = testCase.TestData.ProductRoot;
hs3Root = testCase.TestData.Hs3Root;
productPackages = dir(fullfile(productRoot, "+*"));
for packageIndex = 1:numel(productPackages)
    packagePath = fullfile(productPackages(packageIndex).folder, ...
        productPackages(packageIndex).name);
    nestedRecords = dir(fullfile(packagePath, "**", "*"));
    nestedRecords = nestedRecords([nestedRecords.isdir]);
    nestedNames = string({nestedRecords.name});
    nestedNames = nestedNames(nestedNames ~= "." & nestedNames ~= "..");
    verifyEmpty(testCase, nestedNames, sprintf( ...
        "Production package %s contains nested directories.", ...
        productPackages(packageIndex).name));
end

internalPackages = dir(fullfile(hs3Root, "+*"));
actualNames = sort(string({internalPackages([internalPackages.isdir]).name}));
expectedNames = sort(["+constraints", "+polynomial"]);
verifyEqual(testCase, actualNames, expectedNames);
for packageIndex = 1:numel(internalPackages)
    packagePath = fullfile(internalPackages(packageIndex).folder, ...
        internalPackages(packageIndex).name);
    nestedRecords = dir(fullfile(packagePath, "**", "*"));
    nestedRecords = nestedRecords([nestedRecords.isdir]);
    nestedNames = string({nestedRecords.name});
    nestedNames = nestedNames(nestedNames ~= "." & nestedNames ~= "..");
    verifyEmpty(testCase, nestedNames, sprintf( ...
        "HS3 internal package %s contains deeper directories.", ...
        internalPackages(packageIndex).name));
end
end

function testProductionFunctionBasenamesAreUnique(testCase)
% Duplicate base names make stack traces and searches ambiguous. This check
% reports the repeated name so a developer can rename or consolidate its owner.
% Verify one production function basename owns each responsibility.
productRoot = testCase.TestData.ProductRoot;
hs3Root = testCase.TestData.Hs3Root;
sourceRecords = [dir(fullfile(productRoot, "**", "*.m")); ...
    dir(fullfile(hs3Root, "**", "*.m"))];
sourceNames = string({sourceRecords.name});
[uniqueNames, ~, nameGroup] = unique(lower(sourceNames));
nameCounts = accumarray(nameGroup(:), 1);
duplicateNames = uniqueNames(nameCounts > 1);
verifyEmpty(testCase, duplicateNames, sprintf( ...
    "Duplicate production function basenames: %s", ...
    strjoin(duplicateNames, ", ")));
end

function testNumericalSolversAreOwnedByHs3(testCase)
% Keep numerical solver calls in HS3. A failure identifies planner code that
% bypasses the shared motion engine.
% Verify numerical optimizer calls cannot leak into avoidance packages.
productRoot = testCase.TestData.ProductRoot;
packageRecords = dir(fullfile(productRoot, "+*"));
offendingFiles = strings(0, 1);
solverPattern = "(?<![A-Za-z0-9_])(fmincon|quadprog|optimoptions)\s*\(";
sourceRecords = dir(fullfile(productRoot, "*.m"));
for sourceIndex = 1:numel(sourceRecords)
    sourcePath = fullfile(sourceRecords(sourceIndex).folder, ...
        sourceRecords(sourceIndex).name);
    sourceText = string(fileread(sourcePath));
    if ~isempty(regexp(sourceText, solverPattern, "once"))
        offendingFiles(end + 1, 1) = string(sourcePath); %#ok<AGROW>
    end
end
for packageIndex = 1:numel(packageRecords)
    sourceRecords = dir(fullfile( ...
        productRoot, packageRecords(packageIndex).name, "*.m"));
    for sourceIndex = 1:numel(sourceRecords)
        sourcePath = fullfile(sourceRecords(sourceIndex).folder, ...
            sourceRecords(sourceIndex).name);
        sourceText = string(fileread(sourcePath));
        if ~isempty(regexp(sourceText, solverPattern, "once"))
            offendingFiles(end + 1, 1) = string(sourcePath); %#ok<AGROW>
        end
    end
end
verifyEmpty(testCase, offendingFiles, sprintf( ...
    "Numerical solver calls outside the HS3 product: %s", ...
    strjoin(offendingFiles, ", ")));
end

function testObstaclePlannerOwnsEngineRouting(testCase)
% Verify obstacle context selects Ruckig while constrained routes use HS3.
productRoot = testCase.TestData.ProductRoot;
plannerText = string(fileread(fullfile( ...
    productRoot, "+planner", "plan.m")));
solverText = string(fileread(fullfile( ...
    productRoot, "+planner", "solveRouteCandidate.m")));
evaluatorText = string(fileread(fullfile( ...
    productRoot, "+planner", "evaluatePlannerPolynomial.m")));
verifyNotEmpty(testCase, regexp( ...
    plannerText, "planTrajRuckig\s*\(", "once"));
verifyNotEmpty(testCase, regexp( ...
    solverText, "hs3Engine\.optimize\s*\(", "once"));
verifyNotEmpty(testCase, regexp( ...
    solverText, ...
    "hs3Engine\.polynomial\.createTrajectoryPolynomial\s*\(", "once"));
verifyNotEmpty(testCase, regexp( ...
    evaluatorText, ...
    "hs3Engine\.polynomial\.evaluateTrajectoryPolynomial\s*\(", "once"));
end

function testNamedTrajectoryEntriesPreserveEngineContracts(testCase)
% Verify the new public names preserve zero-input defaults and one simple solve.
verifyEqual(testCase, planTrajHs3(), hs3Engine.defaultOptions());
verifyEqual(testCase, planTrajRuckig(), ruckigEngine.defaultOptions());

initialState = struct( ...
    "time", 0, "position", 0, "velocity", 0, "acceleration", 0);
terminalState = struct( ...
    "position", 1, "velocity", 0, "acceleration", 0, "maximumTime", 5);
limits = struct( ...
    "maximumVelocity", 2, ...
    "maximumAcceleration", 2, ...
    "maximumJerk", 4);
result = planTrajRuckig(initialState, terminalState, limits);
verifyTrue(testCase, result.Success, result.Message);
end

function testHs3SourceIsAzElAgnostic(testCase)
% Verify the engine source contains no Az/El domain dependency.
hs3Root = testCase.TestData.Hs3Root;
sourceRecords = dir(fullfile(hs3Root, "**", "*.m"));
for sourceIndex = 1:numel(sourceRecords)
    sourcePath = fullfile(sourceRecords(sourceIndex).folder, ...
        sourceRecords(sourceIndex).name);
    sourceText = lower(string(fileread(sourcePath)));
    forbiddenPattern = ...
        "azel|azimuth|elevation|obstacle|visibility|topology|" + ...
        "corridor|seedroute|plotazel|planner";
    verifyEmpty(testCase, regexp(sourceText, forbiddenPattern, "once"), ...
        sprintf("HS3 source contains domain language: %s", sourcePath));
end
end

function testRuckigEngineHasNoHs3OptimizerOrPlannerDependency(testCase)
% Keep exact switching ownership independent of HS3 and obstacle planning.
ruckigRoot = fullfile( ...
    testCase.TestData.TrajectoryRoot, "+ruckigEngine");
sourceRecords = dir(fullfile(ruckigRoot, "**", "*.m"));
for sourceIndex = 1:numel(sourceRecords)
    sourcePath = fullfile(sourceRecords(sourceIndex).folder, ...
        sourceRecords(sourceIndex).name);
    sourceText = lower(string(fileread(sourcePath)));
    forbiddenPattern = ...
        "hs3|fmincon|quadprog|optimoptions|obstacle|visibility|planner";
    verifyEmpty(testCase, regexp(sourceText, forbiddenPattern, "once"), ...
        sprintf("Ruckig-derived source crosses its boundary: %s", sourcePath));
end
end

function testHs3ContainsNoSwitchingShortcut(testCase)
% Verify pure collocation cannot regain the extracted switching entry points.
hs3Root = testCase.TestData.Hs3Root;
sourceRecords = dir(fullfile(hs3Root, "**", "*.m"));
for sourceIndex = 1:numel(sourceRecords)
    sourcePath = fullfile(sourceRecords(sourceIndex).folder, ...
        sourceRecords(sourceIndex).name);
    sourceText = string(fileread(sourcePath));
    forbiddenPattern = ...
        "createMinimumTimeAxisProfile|createFixedTimeAxisProfile|" + ...
        "createRestToRestJerkProfile|createSynchronizedJerkProfile|" + ...
        "evaluateAxisSwitchingProfile";
    verifyEmpty(testCase, regexp(sourceText, forbiddenPattern, "once"), ...
        sprintf("HS3 source contains an extracted switching call: %s", ...
        sourcePath));
end
end

function testHs3DependenciesPointTowardPolynomialCore(testCase)
% Prevent polynomial or constraint code from depending on higher layers.
hs3Root = testCase.TestData.Hs3Root;
polynomialSources = dir(fullfile(hs3Root, "+polynomial", "*.m"));
constraintSources = dir(fullfile(hs3Root, "+constraints", "*.m"));
verifySourcesExclude(testCase, polynomialSources, ...
    ["hs3Engine.constraints.", "hs3Engine.optimize("]);
verifySourcesExclude(testCase, constraintSources, ...
    "hs3Engine.optimize(");
end

function testLegacyNestedPackagesAreAbsent(testCase)
% Verify removed ownership trees do not return as compatibility copies.
productRoot = testCase.TestData.ProductRoot;
repositoryRoot = testCase.TestData.RepositoryRoot;
verifyFalse(testCase, isfolder(fullfile(repositoryRoot, "planAzElMotion")));
verifyFalse(testCase, isfolder(fullfile(repositoryRoot, "hs3")));
verifyFalse(testCase, isfolder(fullfile(productRoot, "+azElInternal")));
verifyFalse(testCase, isfolder(fullfile( ...
    productRoot, "+azElPlannerMethods")));
end

function verifySourcesExclude(testCase, sourceRecords, forbiddenText)
% Verify a source collection contains none of the forbidden dependencies.
for sourceIndex = 1:numel(sourceRecords)
    sourcePath = fullfile(sourceRecords(sourceIndex).folder, ...
        sourceRecords(sourceIndex).name);
    sourceText = string(fileread(sourcePath));
    for forbiddenIndex = 1:numel(forbiddenText)
        verifyFalse(testCase, contains( ...
            sourceText, forbiddenText(forbiddenIndex)), sprintf( ...
            "Forbidden dependency %s in %s.", ...
            forbiddenText(forbiddenIndex), sourcePath));
    end
end
end
