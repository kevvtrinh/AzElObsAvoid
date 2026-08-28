function tests = testHybridActivityMesh
%% Section 0: Header & Readme
% SYNTAX
%   results = runtests("tests/testHybridActivityMesh.m")
%**************************************************************************
% PURPOSE
%   - Verify activity-gated Ruckig-scale endpoint refinement and bounded
%     no-op behavior without running an obstacle-planning example.
%**************************************************************************
% INPUTS
%   - None.
%**************************************************************************
% OUTPUTS
%   - tests (matlab.unittest.FunctionTestCase array)
%       Function-based tests discovered by MATLAB runtests.
%**************************************************************************
% UNITS
%   - Test time and derivatives use arbitrary consistent units. Mesh
%     coordinates and activity ratios are dimensionless.
%**************************************************************************
tests = functiontests(localfunctions);
end

function setupOnce(~)
% Add the repository and trajectory package roots for isolated test runs.
repositoryRoot = fileparts(fileparts(mfilename("fullpath")));
addpath(repositoryRoot);
addpath(fullfile(repositoryRoot, "trajectory"));
end

function testRefinesOnlyEndpointIntervals(testCase)
% Preserve every coarse break while resolving two active endpoint intervals.
[candidate, profile, limits] = createInputs();

mesh = obstacleAvoidance.planner.createHybridActivityMesh( ...
    candidate, profile, limits, 6);

verifyTrue(testCase, mesh.Applied);
verifyEqual(testCase, mesh.Reason, "endpointSwitchingResolved");
verifyEqual(testCase, mesh.BaseSegmentCount, 4);
verifyEqual(testCase, mesh.SegmentCount, 6);
verifyEqual(testCase, mesh.SegmentBreakTau, ...
    [0; 0.125; 0.25; 0.5; 0.75; 0.875; 1], "AbsTol", 1e-14);
verifyGreaterThan(testCase, mesh.EndpointDominance, 2);
verifyEqual(testCase, mesh.VelocityCruiseFraction, 1);
end

function testSegmentCapReturnsStableNoOp(testCase)
% Respect a caller cap rather than silently dropping coarse mesh intervals.
[candidate, profile, limits] = createInputs();

mesh = obstacleAvoidance.planner.createHybridActivityMesh( ...
    candidate, profile, limits, 4);

verifyFalse(testCase, mesh.Applied);
verifyEqual(testCase, mesh.Reason, "segmentLimitReached");
verifyEqual(testCase, mesh.SegmentCount, 4);
verifyEqual(testCase, mesh.SegmentBreakTau, (0:4).' / 4);
end

function testInteriorActivityDoesNotTrigger(testCase)
% Reject a mesh whose endpoint dynamics do not dominate its interior.
[candidate, profile, limits] = createInputs();
candidate.Polynomial.accelerationPower_deg_s2(:, 1, 1) = 0.9;
candidate.Polynomial.jerkPower_deg_s3(:, 1, 1) = 0.8;

mesh = obstacleAvoidance.planner.createHybridActivityMesh( ...
    candidate, profile, limits, 10);

verifyFalse(testCase, mesh.Applied);
verifyEqual(testCase, mesh.Reason, "activityNotSeparated");
end

function [candidate, profile, limits] = createInputs()
% Create a dimensionally valid polynomial with endpoint-localized activity.
segmentCount = 4;
polynomial = struct( ...
    "SegmentCount", segmentCount, ...
    "SegmentStartTime_s", (0:segmentCount - 1).', ...
    "SegmentDuration_s", ones(segmentCount, 1), ...
    "FinalTime_s", segmentCount, ...
    "positionPower_deg", zeros(segmentCount, 2, 6), ...
    "velocityPower_deg_s", zeros(segmentCount, 2, 5), ...
    "accelerationPower_deg_s2", zeros(segmentCount, 2, 4), ...
    "jerkPower_deg_s3", zeros(segmentCount, 2, 3));
polynomial.velocityPower_deg_s(:, 1, 1) = 1;
polynomial.accelerationPower_deg_s2(:, 1, 1) = [0.9; 0.1; 0.1; 0.9];
polynomial.jerkPower_deg_s3(:, 1, 1) = [0.8; 0.1; 0.1; 0.8];
candidate = struct("Polynomial", polynomial);
profile = struct( ...
    "time_s", [0; 4], ...
    "FinalTime_s", 4, ...
    "Polynomial", struct( ...
    "SegmentDuration_s", [0.5; 1.5; 1.5; 0.5]));
limits = struct( ...
    "maxVelocity_deg_s", [1 1], ...
    "maxAcceleration_deg_s2", [1 1], ...
    "maxJerk_deg_s3", [1 1]);
end
