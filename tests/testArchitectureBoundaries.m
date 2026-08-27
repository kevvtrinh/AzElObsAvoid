function tests = testArchitectureBoundaries
%% Section 0: Header & Readme
% SYNTAX
%   tests = testArchitectureBoundaries
%**************************************************************************
% PURPOSE
%   - Protect the single-folder Az/El product and neutral HS3 boundary.
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
% Record and add the two product folders for direct test execution.
repositoryRoot = fileparts(fileparts(mfilename("fullpath")));
productRoot = fullfile(repositoryRoot, "planAzElMotion");
hs3Root = fullfile(repositoryRoot, "hs3");
addpath(productRoot);
addpath(hs3Root);
testCase.TestData.RepositoryRoot = repositoryRoot;
testCase.TestData.ProductRoot = productRoot;
testCase.TestData.Hs3Root = hs3Root;
end

function testRootContainsOnlyTwoProductionFolders(testCase)
% Keep production source out of the repository root.
repositoryRoot = testCase.TestData.RepositoryRoot;
directoryRecords = dir(fullfile(repositoryRoot, "+*"));
actualNames = sort(string({directoryRecords([directoryRecords.isdir]).name}));
verifyEmpty(testCase, actualNames);
verifyEmpty(testCase, dir(fullfile(repositoryRoot, "*.m")), ...
    "Production MATLAB functions must not accumulate at the root.");
verifyTrue(testCase, isfolder(fullfile(repositoryRoot, "planAzElMotion")));
verifyTrue(testCase, isfolder(fullfile(repositoryRoot, "hs3")));
end

function testAzElPackagesMatchHighLevelArchitecture(testCase)
% Verify the product contains the six one-level Az/El packages.
productRoot = testCase.TestData.ProductRoot;
directoryRecords = dir(fullfile(productRoot, "+*"));
actualNames = sort(string({directoryRecords([directoryRecords.isdir]).name}));
expectedNames = sort([ ...
    "+azElGeometry", "+azElInput", "+azElObstacles", ...
    "+azElPlanner", "+azElPlotting", "+azElSearch"]);
verifyEqual(testCase, actualNames, expectedNames);
publicSources = sort(string({dir(fullfile(productRoot, "*.m")).name}));
expectedSources = sort(["planAzElMotion.m", ...
    "planAzElMovingTargetIntercept.m", "validateAzElTrajectory.m"]);
verifyEqual(testCase, publicSources, expectedSources);
end

function testHs3HasOnePublicEntryAndOneInternalPackage(testCase)
% Protect the normal HS3 folder and its single public function.
hs3Root = testCase.TestData.Hs3Root;
sourceRecords = dir(fullfile(hs3Root, "*.m"));
verifyEqual(testCase, string({sourceRecords.name}), "solveTrajHS3.m");
verifyTrue(testCase, isfolder(fullfile(hs3Root, "+hs3Internal")));
end

function testProductionPackagesAreNotNested(testCase)
% Verify package ownership does not regress to nested implementation trees.
productRoot = testCase.TestData.ProductRoot;
hs3Root = testCase.TestData.Hs3Root;
packageRecords = [dir(fullfile(productRoot, "+*")); ...
    dir(fullfile(hs3Root, "+*"))];
for packageIndex = 1:numel(packageRecords)
    packagePath = fullfile(packageRecords(packageIndex).folder, ...
        packageRecords(packageIndex).name);
    nestedRecords = dir(fullfile(packagePath, "**", "*"));
    nestedRecords = nestedRecords([nestedRecords.isdir]);
    nestedNames = string({nestedRecords.name});
    nestedNames = nestedNames(nestedNames ~= "." & nestedNames ~= "..");
    verifyEmpty(testCase, nestedNames, sprintf( ...
        "Production package %s contains nested directories.", ...
        packageRecords(packageIndex).name));
end
end

function testProductionFunctionBasenamesAreUnique(testCase)
% Verify one production function basename owns each responsibility.
productRoot = testCase.TestData.ProductRoot;
hs3Root = testCase.TestData.Hs3Root;
sourceRecords = [dir(fullfile(productRoot, "*.m")); ...
    dir(fullfile(hs3Root, "*.m"))];
packageRecords = [dir(fullfile(productRoot, "+*")); ...
    dir(fullfile(hs3Root, "+*"))];
for packageIndex = 1:numel(packageRecords)
    packageSources = dir(fullfile(packageRecords(packageIndex).folder, ...
        packageRecords(packageIndex).name, "*.m"));
    sourceRecords = [sourceRecords; packageSources]; %#ok<AGROW>
end
sourceNames = string({sourceRecords.name});
[uniqueNames, ~, nameGroup] = unique(lower(sourceNames));
nameCounts = accumarray(nameGroup(:), 1);
duplicateNames = uniqueNames(nameCounts > 1);
verifyEmpty(testCase, duplicateNames, sprintf( ...
    "Duplicate production function basenames: %s", ...
    strjoin(duplicateNames, ", ")));
end

function testNumericalSolversAreOwnedByHs3(testCase)
% Verify numerical optimizer calls cannot leak into Az/El packages.
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

function testAzElSolverRoutesThroughNeutralHs3Engine(testCase)
% Verify the adapter delegates optimization and polynomial mechanics.
productRoot = testCase.TestData.ProductRoot;
solverText = string(fileread(fullfile( ...
    productRoot, "+azElPlanner", "solveSeed.m")));
evaluatorText = string(fileread(fullfile( ...
    productRoot, "+azElPlanner", "evaluateAzElPolynomial.m")));
verifyNotEmpty(testCase, regexp( ...
    solverText, "hs3Internal\.optimize\s*\(", "once"));
verifyNotEmpty(testCase, regexp( ...
    solverText, "hs3Internal\.reconstructPolynomial\s*\(", "once"));
verifyNotEmpty(testCase, regexp( ...
    evaluatorText, "hs3Internal\.evaluatePolynomial\s*\(", "once"));
end

function testHs3SourceIsAzElAgnostic(testCase)
% Verify the engine source contains no Az/El domain dependency.
hs3Root = testCase.TestData.Hs3Root;
sourceRecords = [dir(fullfile(hs3Root, "*.m")); ...
    dir(fullfile(hs3Root, "+hs3Internal", "*.m"))];
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

function testLegacyNestedPackagesAreAbsent(testCase)
% Verify removed ownership trees do not return as compatibility copies.
productRoot = testCase.TestData.ProductRoot;
verifyFalse(testCase, isfolder(fullfile(productRoot, "+azElInternal")));
verifyFalse(testCase, isfolder(fullfile( ...
    productRoot, "+azElPlannerMethods")));
end
