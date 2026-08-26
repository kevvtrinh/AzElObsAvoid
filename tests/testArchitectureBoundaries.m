function tests = testArchitectureBoundaries
%% Section 0: Header & Readme
% SYNTAX
%   tests = testArchitectureBoundaries
%**************************************************************************
% PURPOSE
%   - Protect the flat production architecture and neutral HS3 boundary.
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
% Record and add the repository root for direct test execution.
repositoryRoot = fileparts(fileparts(mfilename("fullpath")));
addpath(repositoryRoot);
testCase.TestData.RepositoryRoot = repositoryRoot;
end

function testProductionPackagesMatchHighLevelArchitecture(testCase)
% Verify production package names remain a small one-level architecture.
repositoryRoot = testCase.TestData.RepositoryRoot;
directoryRecords = dir(fullfile(repositoryRoot, "+*"));
actualNames = sort(string({directoryRecords([directoryRecords.isdir]).name}));
expectedNames = sort([ ...
    "+azElGeometry", "+azElInput", "+azElObstacles", ...
    "+azElPlanner", "+azElPlotting", "+azElSearch", "+hs3"]);
verifyEqual(testCase, actualNames, expectedNames);
end

function testProductionPackagesAreNotNested(testCase)
% Verify package ownership does not regress to nested implementation trees.
repositoryRoot = testCase.TestData.RepositoryRoot;
packageRecords = dir(fullfile(repositoryRoot, "+*"));
for packageIndex = 1:numel(packageRecords)
    packagePath = fullfile(repositoryRoot, packageRecords(packageIndex).name);
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
repositoryRoot = testCase.TestData.RepositoryRoot;
sourceRecords = dir(fullfile(repositoryRoot, "*.m"));
packageRecords = dir(fullfile(repositoryRoot, "+*"));
for packageIndex = 1:numel(packageRecords)
    packageSources = dir(fullfile( ...
        repositoryRoot, packageRecords(packageIndex).name, "*.m"));
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
repositoryRoot = testCase.TestData.RepositoryRoot;
packageRecords = dir(fullfile(repositoryRoot, "+*"));
offendingFiles = strings(0, 1);
solverPattern = "(?<![A-Za-z0-9_])(fmincon|quadprog|optimoptions)\s*\(";
for packageIndex = 1:numel(packageRecords)
    if packageRecords(packageIndex).name == "+hs3"
        continue;
    end
    sourceRecords = dir(fullfile( ...
        repositoryRoot, packageRecords(packageIndex).name, "*.m"));
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
    "Numerical solver calls outside +hs3: %s", ...
    strjoin(offendingFiles, ", ")));
end

function testAzElSolverRoutesThroughNeutralHs3Engine(testCase)
% Verify the adapter delegates optimization and polynomial mechanics.
repositoryRoot = testCase.TestData.RepositoryRoot;
solverText = string(fileread(fullfile( ...
    repositoryRoot, "+azElPlanner", "solveSeed.m")));
evaluatorText = string(fileread(fullfile( ...
    repositoryRoot, "+azElPlanner", "evaluateAzElPolynomial.m")));
verifyNotEmpty(testCase, regexp(solverText, "hs3\.optimize\s*\(", "once"));
verifyNotEmpty(testCase, regexp( ...
    solverText, "hs3\.reconstructPolynomial\s*\(", "once"));
verifyNotEmpty(testCase, regexp( ...
    evaluatorText, "hs3\.evaluatePolynomial\s*\(", "once"));
end

function testHs3SourceIsAzElAgnostic(testCase)
% Verify the frozen engine source contains no planner-domain dependency.
repositoryRoot = testCase.TestData.RepositoryRoot;
sourceRecords = dir(fullfile(repositoryRoot, "+hs3", "*.m"));
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
repositoryRoot = testCase.TestData.RepositoryRoot;
verifyFalse(testCase, isfolder(fullfile(repositoryRoot, "+azElInternal")));
verifyFalse(testCase, isfolder(fullfile( ...
    repositoryRoot, "+azElPlannerMethods")));
end
