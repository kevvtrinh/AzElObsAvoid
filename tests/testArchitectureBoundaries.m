function tests = testArchitectureBoundaries
%% Section 0: Header & Readme
% SYNTAX
%   tests = testArchitectureBoundaries
%**************************************************************************
% PURPOSE
%   - Protect the obstacle-avoidance namespace and neutral HS3 boundary.
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
% Record the two production roots and add their public path parents.
repositoryRoot = fileparts(fileparts(mfilename("fullpath")));
productRoot = fullfile(repositoryRoot, "+obstacleAvoidance");
hs3Root = fullfile(repositoryRoot, "hs3");
addpath(repositoryRoot);
addpath(hs3Root);
testCase.TestData.RepositoryRoot = repositoryRoot;
testCase.TestData.ProductRoot = productRoot;
testCase.TestData.Hs3Root = hs3Root;
end

function testRootContainsOnlyTwoProductionRoots(testCase)
% Keep one obstacle-avoidance package and one neutral HS3 product.
repositoryRoot = testCase.TestData.RepositoryRoot;
directoryRecords = dir(fullfile(repositoryRoot, "+*"));
actualNames = sort(string({directoryRecords([directoryRecords.isdir]).name}));
verifyEqual(testCase, actualNames, "+obstacleAvoidance");
verifyEmpty(testCase, dir(fullfile(repositoryRoot, "*.m")), ...
    "Production MATLAB functions must not accumulate at the root.");
verifyTrue(testCase, isfolder(fullfile(repositoryRoot, "hs3")));
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

function testHs3HasOnePublicEntryAndOneInternalPackage(testCase)
% Protect the normal HS3 folder and its single public function.
hs3Root = testCase.TestData.Hs3Root;
sourceRecords = dir(fullfile(hs3Root, "*.m"));
verifyEqual(testCase, string({sourceRecords.name}), "solveTrajHS3.m");
verifyTrue(testCase, isfolder(fullfile(hs3Root, "+hs3Internal")));
end

function testProductionPackageNestingMatchesArchitecture(testCase)
% Allow one product namespace level and three cohesive HS3 subpackages.
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

internalRoot = fullfile(hs3Root, "+hs3Internal");
internalPackages = dir(fullfile(internalRoot, "+*"));
actualNames = sort(string({internalPackages([internalPackages.isdir]).name}));
expectedNames = sort(["+constraints", "+polynomial", "+solver"]);
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

function testObstacleAvoidanceRoutesThroughNeutralHs3Engine(testCase)
% Verify the adapter delegates optimization and polynomial mechanics.
productRoot = testCase.TestData.ProductRoot;
solverText = string(fileread(fullfile( ...
    productRoot, "+planner", "solveRouteCandidate.m")));
evaluatorText = string(fileread(fullfile( ...
    productRoot, "+planner", "evaluatePlannerPolynomial.m")));
verifyNotEmpty(testCase, regexp( ...
    solverText, "hs3Internal\.solver\.optimize\s*\(", "once"));
verifyNotEmpty(testCase, regexp( ...
    solverText, ...
    "hs3Internal\.polynomial\.createTrajectoryPolynomial\s*\(", "once"));
verifyNotEmpty(testCase, regexp( ...
    evaluatorText, ...
    "hs3Internal\.polynomial\.evaluateTrajectoryPolynomial\s*\(", "once"));
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

function testHs3InternalDependenciesPointTowardPolynomialCore(testCase)
% Prevent polynomial or constraint code from depending on higher layers.
internalRoot = fullfile(testCase.TestData.Hs3Root, "+hs3Internal");
polynomialSources = dir(fullfile(internalRoot, "+polynomial", "*.m"));
constraintSources = dir(fullfile(internalRoot, "+constraints", "*.m"));
verifySourcesExclude(testCase, polynomialSources, ...
    ["hs3Internal.constraints.", "hs3Internal.solver."]);
verifySourcesExclude(testCase, constraintSources, ...
    "hs3Internal.solver.");
end

function testLegacyNestedPackagesAreAbsent(testCase)
% Verify removed ownership trees do not return as compatibility copies.
productRoot = testCase.TestData.ProductRoot;
repositoryRoot = testCase.TestData.RepositoryRoot;
verifyFalse(testCase, isfolder(fullfile(repositoryRoot, "planAzElMotion")));
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
