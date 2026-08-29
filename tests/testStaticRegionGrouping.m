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
%   - Position is degrees and time is seconds.
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
topAzimuth_deg = (160:-1:0).';
topElevation_deg = 5 + 0.2 * mod(topAzimuth_deg, 2);
boundary_deg = [0 3; 160 3; topAzimuth_deg, topElevation_deg];
obstacle = obstacleAvoidance.obstacles.createObstacle( ...
    "syntheticComb", [0; 50], boundary_deg(:, 1), ...
    boundary_deg(:, 2), 0);
initialState = createState(0, [-1 0]);
goalState = createState(50, [161 0]);
limits = struct( ...
    "azimuthInterval_deg", [-5 165], ...
    "elevationInterval_deg", [-5 10], ...
    "maxVelocity_deg_s", [50 50], ...
    "maxAcceleration_deg_s2", [20 20], ...
    "maxJerk_deg_s3", [50 50]);
options = obstacleAvoidance.input.resolvePlannerOptions(struct( ...
    "GoalTimeMode", "earliestArrival", ...
    "Verbose", false));
[obstacle, initialState, goalState, limits] = ...
    obstacleAvoidance.input.normalizePlannerRequest( ...
    obstacle, initialState, goalState, limits, options);
seed = struct( ...
    "Index", 1, "Source", "syntheticDirect", ...
    "position_deg", [initialState.position_deg; goalState.position_deg], ...
    "tau", [0; 1], "CorridorBoundary_deg", zeros(0, 2));

[candidate, diagnostics] = ...
    obstacleAvoidance.planner.solveBmtpTrajectory( ...
    seed, obstacle, initialState, goalState, limits, options);
validation = obstacleAvoidance.validateTrajectory( ...
    candidate, obstacle, initialState, goalState, limits, options);

grouping = diagnostics.Coverage.ConservativeGrouping;
verifyTrue(testCase, grouping.Applied);
verifyGreaterThan(testCase, grouping.ExactRegionCount, ...
    grouping.MaximumExactRegionCount);
verifyEqual(testCase, grouping.SolverRegionCount, 8);
verifyEqual(testCase, grouping.RelationToExactGeometry, ...
    "conservativeSuperset");
verifyTrue(testCase, candidate.Success, candidate.Message);
verifyTrue(testCase, validation.Passed, validation.Message);
verifyTrue(testCase, validation.CollisionFree);
verifyTrue(testCase, validation.PlaneCertificateCertified);
verifyEqual(testCase, candidate.PlaneCertificate.ExactRegionCount, ...
    grouping.ExactRegionCount);
verifyEqual(testCase, candidate.PlaneCertificate.SolverRegionCount, ...
    grouping.SolverRegionCount);

% A missing exact cell must invalidate the plane proof regardless of its area;
% adaptive validation may still prove the returned motion independently.
corruptCandidate = candidate;
corruptGrouping = ...
    corruptCandidate.PlaneCertificate.Coverage.ConservativeGrouping;
lastGroupIndex = numel(corruptGrouping.GroupMemberIndices);
corruptGrouping.GroupMemberIndices{lastGroupIndex}(end) = [];
corruptCandidate.PlaneCertificate.Coverage.ConservativeGrouping = ...
    corruptGrouping;
corruptValidation = obstacleAvoidance.validateTrajectory( ...
    corruptCandidate, obstacle, initialState, goalState, limits, options);
verifyFalse(testCase, corruptValidation.PlaneCertificateCertified);
verifyTrue(testCase, corruptValidation.Passed, corruptValidation.Message);
verifyTrue(testCase, corruptValidation.CollisionFree);
end

function state = createState(time_s, position_deg)
% Create one normalized rest endpoint.
state = struct( ...
    "time_s", time_s, "position_deg", position_deg, ...
    "velocity_deg_s", [0 0], "acceleration_deg_s2", [0 0]);
end
