function tests = testObstacleInfrastructure
%% Section 0: Header & Readme
% SYNTAX
%   tests = testObstacleInfrastructure
%**************************************************************************
% PURPOSE
%   - Freeze canonical obstacle combination, normalization, and query
%     behavior while duplicated method implementations are consolidated.
%**************************************************************************
% INPUTS
%   - None.
%**************************************************************************
% OUTPUTS
%   - tests (matlab.unittest function test array)
%**************************************************************************
% UNITS
%   - Boundary coordinates are degrees and time is seconds.
%**************************************************************************
tests = functiontests(localfunctions);
end

function setupOnce(testCase)
% Add the repository root for path-based test execution.
repositoryRoot = fileparts(fileparts(mfilename("fullpath")));
addpath(repositoryRoot);
testCase.TestData.RepositoryRoot = repositoryRoot;
end

function testCombinePreservesCanonicalOrderAndSchema(testCase)
% Verify every supported container form has one canonical interpretation.
firstObstacle = rectangleObstacle("first", [0; 4], [-2 0 -1 1]);
secondObstacle = rectangleObstacle("second", [0; 4], [1 3 -2 2]);
nestedInputs = {[], {firstObstacle, {[], secondObstacle}}};

rootResult = combineAzElObstacles(nestedInputs);
verifyEqual(testCase, size(rootResult), [2 1]);
verifyEqual(testCase, [rootResult.targetName].', ["first"; "second"]);

rootEmpty = combineAzElObstacles();
verifyEqual(testCase, combineAzElObstacles([]), rootEmpty);
verifyEqual(testCase, fieldnames(rootEmpty), fieldnames(rootResult));
end

function testUnifiedOwnerNormalizesCanonicalValues(testCase)
% Verify the unified owner establishes stable values, shapes, and fields.
inputData = normalizationFixture();
rootResult = makeAzElObstacleData(inputData);
verifyEqual(testCase, rootResult.time_s, [0; 2]);
verifyEqual(testCase, rootResult.status, ["visible"; "visible"]);
verifyEqual(testCase, size(rootResult.az_deg), [2 1]);
verifyEqual(testCase, rootResult.safetyMargin_deg, 0.25);
end

function testNormalizeDiagnosticsAreEquivalent(testCase)
% Verify malformed inputs preserve the established public error identifiers.
base = normalizationFixture();
cases = cell(0, 2);
cases(end + 1, :) = {@() struct(), ...
    "normalizeAzElTimeObstacleData:InvalidInput"};
cases(end + 1, :) = {@() setField(base, "targetName", ""), ...
    "normalizeAzElTimeObstacleData:InvalidTargetName"};
cases(end + 1, :) = {@() setField(base, "time_s", [0 0]), ...
    "normalizeAzElTimeObstacleData:InvalidTime"};
cases(end + 1, :) = {@() setField(base, "az_deg", {[0 1 1 0]}), ...
    "normalizeAzElTimeObstacleData:InvalidBoundary"};
cases(end + 1, :) = {@() removeField(base, "originalEl_deg"), ...
    "normalizeAzElTimeObstacleData:IncompleteOriginalBoundary"};
cases(end + 1, :) = {@() setField(base, "status", ["a" "b" "c"]), ...
    "normalizeAzElTimeObstacleData:StatusSizeMismatch"};
cases(end + 1, :) = {@() setBoundaryPair(base, [0 1 NaN 2], ...
    [0 1 0 2]), ...
    "normalizeAzElTimeObstacleData:UnpairedNonfiniteBoundary"};
cases(end + 1, :) = {@() setBoundaryPair(base, 0, 0), ...
    "normalizeAzElTimeObstacleData:BoundaryRingTooShort"};

for caseIndex = 1:size(cases, 1)
    inputData = cases{caseIndex, 1}();
    expectedIdentifier = cases{caseIndex, 2};
    actualIdentifier = captureErrorIdentifier( ...
        @makeAzElObstacleData, inputData);
    verifyEqual(testCase, actualIdentifier, expectedIdentifier);
end
end

function testTwoVertexWarningIsEquivalent(testCase)
% Verify degenerate-region removal emits one stable warning.
inputData = normalizationFixture();
inputData.az_deg{1} = [0; 1; NaN; -2; 2; 2; -2];
inputData.el_deg{1} = [0; 1; NaN; -1; -1; 1; 1];
inputData.originalAz_deg = inputData.az_deg;
inputData.originalEl_deg = inputData.el_deg;
lastwarn("");
output = makeAzElObstacleData(inputData);
[~, warningIdentifier] = lastwarn();
verifyEqual(testCase, string(warningIdentifier), ...
    "normalizeAzElTimeObstacleData:RemovedTwoVertexRegions");
verifyEqual(testCase, numel(output.az_deg{1}), 4);
end

function testSharedQueryPreservesGeometryAndCompatibility(testCase)
% Verify shared moving, multi-ring, broadcast, and diagnostic query behavior.
movingObstacle = movingMultiRingObstacle();
staticObstacle = rectangleObstacle("static", [0; 4], [-0.5 0.5 -0.5 0.5]);
obstacles = combineAzElObstacles(movingObstacle, staticObstacle);
azimuth_deg = [-3 0 3; -2 NaN 2];
elevation_deg = [0 0 0; 1 0 -1];
time_s = [0 0 0; 2 2 4];
options = struct( ...
    "BoundaryIsOccupied", false, ...
    "ClearanceTolerance_deg", 1e-10);

[sharedOccupied, sharedBlocker, sharedDetails] = queryAzElTimeObstacle( ...
    obstacles, azimuth_deg, elevation_deg, time_s, options);
verifyClass(testCase, sharedBlocker, "uint32");
verifyEqual(testCase, size(sharedOccupied), size(azimuth_deg));
verifyEqual(testCase, size(sharedDetails.MinimumClearance_deg), ...
    size(azimuth_deg));

fastOccupied = queryAzElTimeObstacle( ...
    obstacles, azimuth_deg, elevation_deg, time_s, options);
verifyEqual(testCase, fastOccupied, sharedOccupied);

hs3Options = options;
hs3Options.PlannerMethod = "hs3";
[hs3Occupied, hs3Blocker, hs3Details] = queryAzElTimeObstacle( ...
    obstacles, azimuth_deg, elevation_deg, time_s, hs3Options);
verifyEqual(testCase, hs3Occupied, sharedOccupied);
verifyEqual(testCase, hs3Blocker, sharedBlocker);
verifyEqual(testCase, hs3Details.MinimumClearance_deg, ...
    sharedDetails.MinimumClearance_deg, "AbsTol", 1e-12);
verifyEqual(testCase, hs3Details.Options.PlannerMethod, "hs3");

referenceTime = datetime(2026, 1, 1, "TimeZone", "UTC");
datetimeOptions = options;
datetimeOptions.ReferenceTime = referenceTime;
datetimeOccupied = queryAzElTimeObstacle( ...
    obstacles, azimuth_deg, elevation_deg, ...
    referenceTime + seconds(time_s), datetimeOptions);
verifyEqual(testCase, datetimeOccupied, sharedOccupied);
end

function testShapeQueryReportsOrderedBoundaryProperties(testCase)
% Compare the lightweight ordered-boundary record with polyshape evidence.
convexObstacle = rectangleObstacle("convex", [0; 4], [-2 2 -1 1]);
[convexShape, convexGeometry] = ...
    azElObstacles.shapeAtTime(convexObstacle, 2);
verifyTrue(testCase, convexGeometry.HasOrderedSingleRegion);
verifyTrue(testCase, convexGeometry.IsConvex);
vertices_deg = [convexGeometry.azimuth_deg, convexGeometry.elevation_deg];
edgeDelta_deg = vertices_deg(2, :) - vertices_deg(1, :);
leftNormal = [-edgeDelta_deg(2), edgeDelta_deg(1)] / norm(edgeDelta_deg);
probe_deg = 0.5 * sum(vertices_deg(1:2, :), 1) + 1e-6 * leftNormal;
referenceOutwardSign = 1 - 2 * isinterior( ...
    convexShape, probe_deg(1), probe_deg(2));
verifyEqual(testCase, convexGeometry.OutwardSign, referenceOutwardSign);

concaveAzimuth_deg = [0; 2; 2; 1; 1; 0];
concaveElevation_deg = [0; 0; 1; 1; 2; 2];
concaveObstacle = makeAzElObstacleData( ...
    "concave", [0; 4], concaveAzimuth_deg, concaveElevation_deg, 0);
[~, concaveGeometry] = ...
    azElObstacles.shapeAtTime(concaveObstacle, 2, true);
verifyTrue(testCase, concaveGeometry.HasOrderedSingleRegion);
verifyFalse(testCase, concaveGeometry.IsConvex);
multiRegionObstacle = movingMultiRingObstacle();
[~, multiRegionGeometry] = ...
    azElObstacles.shapeAtTime(multiRegionObstacle, 1, true);
verifyFalse(testCase, multiRegionGeometry.HasOrderedSingleRegion);
verifyFalse(testCase, multiRegionGeometry.IsConvex);
verifyTrue(testCase, isnan(multiRegionGeometry.OutwardSign));
end

function obstacle = normalizationFixture()
% Construct one raw moving, multi-ring record requiring normalization.
azimuthSlices_deg = {[-3 -1 -1 -3 NaN 1 3 3 1], ...
    [-2 0 0 -2 NaN 2 4 4 2]};
elevationSlices_deg = {[-1 -1 1 1 NaN -1 -1 1 1], ...
    [-1 -1 1 1 NaN -1 -1 1 1]};
obstacle = struct( ...
    "targetName", "normalization fixture", ...
    "time_s", [0 2], ...
    "az_deg", {azimuthSlices_deg}, ...
    "el_deg", {elevationSlices_deg}, ...
    "originalAz_deg", {azimuthSlices_deg}, ...
    "originalEl_deg", {elevationSlices_deg}, ...
    "safetyMargin_deg", 0.25, ...
    "status", "visible");
end

function obstacle = movingMultiRingObstacle()
% Construct two translating protected regions in one canonical obstacle.
inputData = normalizationFixture();
inputData.targetName = "moving multi-ring";
inputData.safetyMargin_deg = 0;
obstacle = makeAzElObstacleData(inputData);
end

function obstacle = rectangleObstacle(name, time_s, bounds_deg)
% Construct one canonical static rectangle.
azimuth_deg = bounds_deg([1 2 2 1]).';
elevation_deg = bounds_deg([3 3 4 4]).';
obstacle = makeAzElObstacleData( ...
    name, time_s, azimuth_deg, elevation_deg, 0);
end

function value = setField(value, fieldName, fieldValue)
% Return a fixture with one field replaced.
value.(fieldName) = fieldValue;
end

function value = removeField(value, fieldName)
% Return a fixture with one field removed.
value = rmfield(value, fieldName);
end

function value = setBoundaryPair(value, azimuth_deg, elevation_deg)
% Replace the first protected slice with a requested malformed boundary.
value.az_deg{1} = azimuth_deg;
value.el_deg{1} = elevation_deg;
end

function identifier = captureErrorIdentifier(normalizer, inputData)
% Return the identifier produced for one invalid normalization request.
identifier = "";
try
    normalizer(inputData);
catch exception
    identifier = string(exception.identifier);
end
end
