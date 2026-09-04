function tests = testObstacleHistoryContract
%% Section 0: Header & Readme
% SYNTAX
%   tests = testObstacleHistoryContract
%**************************************************************************
% PURPOSE
%   - Lock the documented obstacle-history interpolation and activity model.
%   - Exercise representation changes and conservative fallback geometry.
%**************************************************************************
% INPUTS
%   - None.
%**************************************************************************
% OUTPUTS
%   - tests (MATLAB function-test array)
%       Deterministic obstacle-history contract regressions.
%**************************************************************************
% UNITS
%   - Position is degrees and time is seconds.
%**************************************************************************
tests = functiontests(localfunctions);
end

function setupOnce(~)
% Add production and trajectory packages used by obstacle preparation.
repositoryRoot = fileparts(fileparts(mfilename("fullpath")));
addpath(repositoryRoot);
addpath(fullfile(repositoryRoot, "trajectory"));
end

function testEquivalentRingOrderDoesNotCreateMotion(testCase)
% Treat cyclic shifts and reversed traversal as the same occupied polygon.
base_deg = [-2 -1; 2 -1; 2 1; -2 1];
shifted_deg = circshift(base_deg, 2, 1);
reversed_deg = flipud(base_deg);
obstacle = obstacleAvoidance.obstacles.createObstacle( ...
    "equivalent ring order", [0; 1; 2], ...
    {base_deg(:, 1); shifted_deg(:, 1); reversed_deg(:, 1)}, ...
    {base_deg(:, 2); shifted_deg(:, 2); reversed_deg(:, 2)}, 0);
referenceShape = polyshape(base_deg, "Simplify", false);

for queryTime_s = [0.5 1.5]
    [shape, geometry] = obstacleAvoidance.obstacles.shapeAtTime( ...
        obstacle, queryTime_s);
    verifyEqual(testCase, area(subtract(shape, referenceShape)), 0, ...
        "AbsTol", 1e-12);
    verifyEqual(testCase, area(subtract(referenceShape, shape)), 0, ...
        "AbsTol", 1e-12);
    verifyEqual(testCase, geometry.VertexSpeedBound_deg_s, 0, ...
        "AbsTol", 1e-12);
end
end

function testRotatingRectangleUsesLinearVertexMotion(testCase)
% Define sampled rotation as linear corresponding-vertex motion, not rigid arc.
lower_deg = [-2 -1; 2 -1; 2 1; -2 1];
angle_rad = deg2rad(20);
rotation = [cos(angle_rad), -sin(angle_rad); ...
    sin(angle_rad), cos(angle_rad)];
upper_deg = lower_deg * rotation.';
obstacle = obstacleAvoidance.obstacles.createObstacle( ...
    "sampled rotating rectangle", [0; 2], ...
    {lower_deg(:, 1); upper_deg(:, 1)}, ...
    {lower_deg(:, 2); upper_deg(:, 2)}, 0);
[~, geometry] = obstacleAvoidance.obstacles.shapeAtTime(obstacle, 1, true);

verifyEqual(testCase, ...
    [geometry.azimuth_deg, geometry.elevation_deg], ...
    0.5 * (lower_deg + upper_deg), "AbsTol", 1e-12);
verifyTrue(testCase, geometry.TopologyIsInterpolated);
end

function testUnprovenCorrespondenceContainsSweptGap(testCase)
% Conservative fallback must cover the gap between separated endpoint shapes.
left_deg = [-4 -1; -2 -1; -2 1; -4 1];
rightLower_deg = [2 -1; 4 -1; 4 1; 2 1];
rightUpper_deg = [2 2; 3 2; 3 3; 2 3];
right_deg = [rightLower_deg; NaN NaN; rightUpper_deg];
obstacle = obstacleAvoidance.obstacles.createObstacle( ...
    "unproven correspondence", [0; 2], ...
    {left_deg(:, 1); right_deg(:, 1)}, ...
    {left_deg(:, 2); right_deg(:, 2)}, 0);

[occupied, ~, details] = ...
    obstacleAvoidance.obstacles.queryObstacleOccupancyAtTime( ...
    obstacle, 0, 0, 1);
[~, geometry] = obstacleAvoidance.obstacles.shapeAtTime(obstacle, 1);
verifyTrue(testCase, occupied, ...
    "The fallback omitted the between-sample swept gap.");
verifyLessThanOrEqual(testCase, details.MinimumClearance_deg, 0);
verifyFalse(testCase, geometry.TopologyIsInterpolated);
end

function testNestedTopologyChangePreservesLargerConcavity(testCase)
% Use the larger exact occupied set when one endpoint is wholly nested.
largerL_deg = [0 0; 2 0; 2 1; 1 1; 1 2; 0 2];
smallerSquare_deg = [0 0; 1 0; 1 1; 0 1];
obstacle = obstacleAvoidance.obstacles.createObstacle( ...
    "nested topology transition", [0; 2], ...
    {largerL_deg(:, 1); smallerSquare_deg(:, 1)}, ...
    {largerL_deg(:, 2); smallerSquare_deg(:, 2)}, 0);

[~, geometry] = obstacleAvoidance.obstacles.shapeAtTime(obstacle, 1);
occupied = obstacleAvoidance.obstacles.queryObstacleOccupancyAtTime( ...
    obstacle, [0.5 1.5], [0.5 1.5], 1);
verifyEqual(testCase, geometry.GeometryModel, ...
    "conservativeNestedEndpointUnion");
verifyEqual(testCase, occupied, [true false]);
end

function testVisibilityStatusIsConservativeMetadata(testCase)
% A non-visible status label must not deactivate supplied physical geometry.
boundary_deg = [-1 -1; 1 -1; 1 1; -1 1];
obstacle = obstacleAvoidance.obstacles.createObstacle( ...
    "status metadata", [0; 1], boundary_deg(:, 1), ...
    boundary_deg(:, 2), 0);
obstacle.status = ["visible"; "hidden"];
obstacle = obstacleAvoidance.obstacles.createObstacle(obstacle);
occupied = obstacleAvoidance.obstacles.queryObstacleOccupancyAtTime( ...
    obstacle, 0, 0, 0.75);

verifyTrue(testCase, occupied);
end

function testStaticQueryPreservesActivitySpan(testCase)
% Collapse equal shapes without extending or shortening obstacle activity.
boundary_deg = [-1 -1; 1 -1; 1 1; -1 1];
finiteHistory = obstacleAvoidance.obstacles.createObstacle( ...
    "finite static history", [1; 3], boundary_deg(:, 1), ...
    boundary_deg(:, 2), 0);
finiteOccupied = ...
    obstacleAvoidance.obstacles.queryObstacleOccupancyAtTime( ...
    finiteHistory, zeros(1, 3), zeros(1, 3), [0 2 4]);
verifyEqual(testCase, finiteOccupied, [false true false]);

singleSample = obstacleAvoidance.obstacles.createObstacle( ...
    "single static sample", 2, boundary_deg(:, 1), boundary_deg(:, 2), 0);
singleOccupied = obstacleAvoidance.obstacles.queryObstacleOccupancyAtTime( ...
    singleSample, zeros(1, 3), zeros(1, 3), [0 2 4]);
verifyEqual(testCase, singleOccupied, [true true true]);
end
