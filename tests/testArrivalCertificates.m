function tests = testArrivalCertificates
%% Section 0: Header & Readme
% SYNTAX
%   tests = testArrivalCertificates
%**************************************************************************
% PURPOSE
%   - Exercise stable arrival-certificate records, direct acceptance and
%     rejection paths, and request-wide portfolio ranking policy.
%**************************************************************************
% INPUTS
%   - None.
%**************************************************************************
% OUTPUTS
%   - tests (matlab.unittest function test array)
%**************************************************************************
% UNITS
%   - Position is degrees and lower-bound, arrival, and gap values are seconds.
%**************************************************************************
tests = functiontests(localfunctions);
end

function setupOnce(testCase)
% Add production roots once for direct package-certifier calls.
repositoryRoot = fileparts(fileparts(mfilename("fullpath")));
addpath(repositoryRoot);
addpath(fullfile(repositoryRoot, "trajectory"));
testCase.TestData.RepositoryRoot = repositoryRoot;
end

function testCertificateTemplatesHaveStableFields(testCase)
% Keep no-input certificate probes useful for failure diagnostics and plotting.
cavity = obstacleAvoidance.planner.certifyOrthogonalCavityLowerBound();
opening = obstacleAvoidance.planner.certifyTimedOpeningRequestLowerBound();
portfolio = obstacleAvoidance.planner.evaluateArrivalCertificatePortfolio();

verifyEqual(testCase, cavity.Passed, false);
verifyEqual(testCase, cavity.TerminationReason, "notRun");
verifyTrue(testCase, isfield(cavity, "LowerBound_s"));
verifyEqual(testCase, opening.Passed, false);
verifyEqual(testCase, opening.TerminationReason, "notRun");
verifyTrue(testCase, isfield(opening, "EventSideLowerBound_s"));
verifyEqual(testCase, portfolio.SelectedSeedIndex, 0);
verifyTrue(testCase, isfield(portfolio, "InfimumGapWithinPolicy"));
end

function testTimedOpeningLowerBoundPassesAndPolicyRejects(testCase)
% Prove a supported opening and reject the same records under fixed arrival.
[obstacles, initialState, goalState, limits, options, diagnostics] = ...
    timedOpeningFixture();
certificate = obstacleAvoidance.planner.certifyTimedOpeningRequestLowerBound( ...
    diagnostics, obstacles, initialState, goalState, limits, options);

verifyTrue(testCase, certificate.Passed, certificate.Message);
verifyEqual(testCase, certificate.TerminationReason, ...
    "allRoutesLowerBoundCertified");
verifyGreaterThan(testCase, certificate.LowerBound_s, 0);

options.GoalTimeMode = "fixedArrival";
rejected = obstacleAvoidance.planner.certifyTimedOpeningRequestLowerBound( ...
    diagnostics, obstacles, initialState, goalState, limits, options);
verifyFalse(testCase, rejected.Passed);
verifyEqual(testCase, rejected.TerminationReason, "unsupportedRequestPolicy");
end

function testCertificatePortfolioSelectsIndependentEvidence(testCase)
% Pair the earliest/shortest valid upper with the strongest lower certificate.
portfolio = obstacleAvoidance.planner.evaluateArrivalCertificatePortfolio( ...
    [8; 7; 7], [1; 3; 2], [6; 6.5; NaN], 0.5);

verifyEqual(testCase, portfolio.SelectedSeedIndex, 3);
verifyEqual(testCase, portfolio.BestLowerCertificateSeedIndex, 2);
verifyEqual(testCase, portfolio.BestValidatedUpper_s, 7, "AbsTol", 1e-12);
verifyEqual(testCase, portfolio.RequestLowerBound_s, 6.5, "AbsTol", 1e-12);
verifyEqual(testCase, portfolio.InfimumGap_s, 0.5, "AbsTol", 1e-12);
verifyTrue(testCase, portfolio.InfimumGapWithinPolicy);

outsidePolicy = obstacleAvoidance.planner.evaluateArrivalCertificatePortfolio( ...
    7, 2, 6.5, 0.49);
verifyFalse(testCase, outsidePolicy.InfimumGapWithinPolicy);
end

function [obstacles, initialState, goalState, limits, options, diagnostics] = ...
        timedOpeningFixture()
% Recreate one canonical stationary U history with an exact opening source time.
closedBoundary_deg = [ ...
    -8, 7; -5, 7; -5, -4; 5, -4; 5, 7; 8, 7; 8, -7; -8, -7];
leftOpenBoundary_deg = [ ...
    -8, 7; -5, 7; -5, -4; -1.5, -4; -1.5, -7; -8, -7];
rightOpenBoundary_deg = [ ...
    5, 7; 8, 7; 8, -7; 1.5, -7; 1.5, -4; 5, -4];
openBoundary_deg = vertcat( ...
    leftOpenBoundary_deg, NaN(1, 2), rightOpenBoundary_deg);
obstacles = obstacleAvoidance.obstacles.createObstacle( ...
    "timed opening", [0; 6.999; 7.001; 120], ...
    {closedBoundary_deg(:, 1); closedBoundary_deg(:, 1); ...
    openBoundary_deg(:, 1); openBoundary_deg(:, 1)}, ...
    {closedBoundary_deg(:, 2); closedBoundary_deg(:, 2); ...
    openBoundary_deg(:, 2); openBoundary_deg(:, 2)}, 0.2);
initialState = struct( ...
    "time_s", 0, "position_deg", [0 0], ...
    "velocity_deg_s", [0 0], "acceleration_deg_s2", [0 0]);
goalState = struct( ...
    "time_s", 120, "position_deg", [0 -10], ...
    "velocity_deg_s", [0 0], "acceleration_deg_s2", [0 0]);
limits = struct( ...
    "maxVelocity_deg_s", [2 2], ...
    "maxAcceleration_deg_s2", [0.75 0.75], ...
    "maxJerk_deg_s3", [2.5 2.5]);
options = obstacleAvoidance.planTrajectory();
diagnostics = struct( ...
    "EventTime_s", NaN, "BlockingFaceProgress_deg", NaN, ...
    "ClearanceTolerance_deg", NaN, "EventCompatibleLowerBound_s", NaN);
end
