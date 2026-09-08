function tests = testStaticRegionGrouping
%% Section 0: Header & Readme
% SYNTAX
%   tests = testStaticRegionGrouping
%**************************************************************************
% PURPOSE
%   - Verify conservative solver grouping on a complex non-geographic outline.
%   - Require independent validation against the original obstacle geometry.
%**************************************************************************
% INPUTS
%   - None.
%**************************************************************************
% OUTPUTS
%   - tests (matlab.unittest function test array)
%       Deterministic static-region grouping tests.
%**************************************************************************
% UNITS
%   - Position is coordinate units and time is seconds.
%**************************************************************************
tests = functiontests(localfunctions);
end

function setupOnce(~)
    % Add the obstacle planner and independent trajectory engine to the path.
    repositoryRoot = fileparts(fileparts(mfilename("fullpath")));
    addpath(repositoryRoot, fullfile(repositoryRoot, "trajectory"));
end

function testComplexCombUsesEightConservativeGroups(testCase)
    % Exercise the large-outline rule without relying on geographic fixture data.
    topX_units   = (160:-1:0).';
    topY_units = 5 + 0.2 * mod(topX_units, 2);
    boundary_units     = [0 3; 160 3; topX_units, topY_units];
    obstacle         = obstacleAvoidance.obstacles.createObstacle("syntheticComb", [0; 50], boundary_units(:, 1), boundary_units(:, 2), 0);
    initialState     = createState(0, [-1 0]);
    goalState        = createState(50, [161 0]);
    limits           = struct();
    limits.xInterval_units    = [-5 165];
    limits.yInterval_units  = [-5 10];
    limits.maxVelocity_units_s      = [50 50];
    limits.maxAcceleration_units_s2 = [20 20];
    limits.maxJerk_units_s3         = [50 50];
    options = obstacleAvoidance.input.resolvePlannerOptions(struct("GoalTimeMode", "earliestArrival"));
    [obstacle, initialState, goalState, limits] = obstacleAvoidance.input.normalizePlannerRequest(obstacle, initialState, goalState, limits, options);
    seed = struct("Index", 1, "Source", "syntheticDirect", ...
        "position_units", [initialState.position_units; goalState.position_units], ...
        "tau", [0; 1], "ObstacleEnvelope_units", zeros(0, 2));

    geometry = obstacleAvoidance.planner.prepareStaticSolverGeometry(obstacleAvoidance.obstacles.prepareObstacles(obstacle), initialState.time_s, goalState.time_s);
    [candidate, diagnostics] = bmtpEngine.solve(seed, geometry.Regions_units, geometry.Coverage, initialState, goalState, limits, options);
    validation = obstacleAvoidance.validateTrajectory(candidate, obstacle, initialState, goalState, limits, options);

    grouping = diagnostics.Coverage.ConservativeGrouping;
    verifyTrue(testCase, grouping.Applied);
    verifyGreaterThan(testCase, grouping.ExactRegionCount, grouping.MaximumExactRegionCount);
    verifyEqual(testCase, grouping.SolverRegionCount, 8);
    verifyEqual(testCase, grouping.RelationToExactGeometry, "conservativeSuperset");
    verifyTrue(testCase, candidate.Success, candidate.Message);
    verifyTrue(testCase, validation.Passed, validation.Message);
    verifyTrue(testCase, validation.CollisionFree);
    verifyTrue(testCase, validation.PlaneCertificateCertified);
    verifyEqual(testCase, candidate.PlaneCertificate.ExactRegionCount, grouping.ExactRegionCount);
    verifyEqual(testCase, candidate.PlaneCertificate.SolverRegionCount, grouping.SolverRegionCount);

    % A missing exact cell must invalidate the plane proof regardless of its area;
    % adaptive validation may still prove the returned motion independently.
    corruptCandidate = candidate;
    corruptGrouping  = corruptCandidate.PlaneCertificate.Coverage.ConservativeGrouping;
    lastGroupIndex   = numel(corruptGrouping.GroupMemberIndices);
    corruptGrouping.GroupMemberIndices{lastGroupIndex}(end) = [];
    corruptCandidate.PlaneCertificate.Coverage.ConservativeGrouping = corruptGrouping;
    corruptValidation = obstacleAvoidance.validateTrajectory(corruptCandidate, obstacle, initialState, goalState, limits, options);
    verifyFalse(testCase, corruptValidation.PlaneCertificateCertified);
    verifyTrue(testCase, corruptValidation.Passed, corruptValidation.Message);
    verifyTrue(testCase, corruptValidation.CollisionFree);
end

function testExactCorridorPreservesSeparatedRegions(testCase)
    % Prevent conservative grouping from closing an exact free corridor.
    obstacleCells = cell(66, 1);
    obstacleIndex = 0;
    % Exercise each column covered by this regression.
    for columnIndex = 1:33
        % Exercise each y sign covered by this regression.
        for ySign = [-1 1]
            obstacleIndex = obstacleIndex + 1;
            center_units    = [columnIndex - 1, ySign];
            vertices_units  = center_units + [-0.2 -0.2; 0.2 -0.2; 0.2 0.2; -0.2 0.2];
            obstacleCells{obstacleIndex} = obstacleAvoidance.obstacles.createObstacle("separated region " + obstacleIndex, [0; 20], vertices_units(:, 1), vertices_units(:, 2), 0);
        end
    end
    obstacles    = obstacleAvoidance.obstacles.combineObstacles(obstacleCells);
    initialState = createState(0, [-1 0]);
    goalState    = createState(20, [33 0]);
    limits       = struct();
    limits.xInterval_units    = [-2 34];
    limits.yInterval_units  = [-0.5 0.5];
    limits.maxVelocity_units_s      = [10 10];
    limits.maxAcceleration_units_s2 = [10 10];
    limits.maxJerk_units_s3         = [20 20];
    options = obstacleAvoidance.input.resolvePlannerOptions(struct("GoalTimeMode", "earliestArrival"));
    [obstacles, initialState, goalState, limits] = obstacleAvoidance.input.normalizePlannerRequest(obstacles, initialState, goalState, limits, options);
    seed = struct("Index", 1, "Source", "separatedExactRegions", ...
        "position_units", [initialState.position_units; goalState.position_units], ...
        "tau", [0; 1], "ObstacleEnvelope_units", zeros(0, 2));

    [candidate, diagnostics] = obstacleAvoidance.planner.solveStaticBmtpTrajectory(seed, obstacleAvoidance.planner.prepareStaticSolverGeometry(obstacleAvoidance.obstacles.prepareObstacles(obstacles), initialState.time_s, goalState.time_s), initialState, goalState, limits, options);
    validation = obstacleAvoidance.validateTrajectory(candidate, obstacles, initialState, goalState, limits, options);

    verifyEqual(testCase, diagnostics.Identifier, "monotoneStaticCorridor");
    verifyTrue(testCase, candidate.Success, candidate.Message);
    verifyTrue(testCase, validation.Passed, validation.Message);
    verifyTrue(testCase, validation.PlaneCertificateCertified);
    verifyFalse(testCase, diagnostics.Coverage.ConservativeGrouping.Applied);
    verifyEqual(testCase, diagnostics.Coverage.SolverRegionCount, 66);
end

function state = createState(time_s, position_units)
    % Create one normalized rest endpoint.
    state = struct("time_s", time_s, "position_units", position_units, ...
        "velocity_units_s", [0 0], "acceleration_units_s2", [0 0]);
end
