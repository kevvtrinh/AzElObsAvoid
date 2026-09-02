function tests = testSpatialHomologySearch
%% Section 0: Header & Readme
% SYNTAX
%   tests = testSpatialHomologySearch
%**************************************************************************
% PURPOSE
%   - Verify numeric homology keys and shared cleanup shortest-path trees.
%**************************************************************************
% INPUTS
%   - None.
%**************************************************************************
% OUTPUTS
%   - tests (matlab.unittest function test array)
%       Exercises retained search evidence and numeric-key capacity errors.
%**************************************************************************
% UNITS
%   - Position and edge cost are degrees; signatures are dimensionless.
%**************************************************************************
tests = functiontests(localfunctions);
end

function setupOnce(testCase)
% Add the production package used by direct homology-search tests.
repositoryRoot = fileparts(fileparts(mfilename("fullpath")));
addpath(repositoryRoot);
testCase.TestData.RepositoryRoot = repositoryRoot;
end

function testNumericKeysAndTwoCleanupTrees(testCase)
% Shorten one chain and build start/goal cleanup trees exactly once each.
[cost_deg, positions_deg] = chainGraph(10);
visibilityFunction = @(first_deg, second_deg) ...
    true(size(first_deg, 1), 1);

[routes, signatures, record] = ...
    obstacleAvoidance.planner.searchSpatialHomologyRoutes( ...
    cost_deg, positions_deg, zeros(0, 2), 1, visibilityFunction);

verifyEqual(testCase, numel(routes), 1);
verifyEqual(testCase, routes{1}, positions_deg([1 2], :));
verifyEqual(testCase, size(signatures), [1 0]);
verifyEqual(testCase, record.StateKeyEncoding, "uint64Base3");
verifyEqual(testCase, record.FrontierSelection, "linearScan");
verifyEqual(testCase, record.CleanupTreeBuildCount, 2);
verifyGreaterThan(testCase, record.RouteCleanupAcceptedCount, 0);
end

function testNumericKeyOverflowIsExplicit(testCase)
% Reject a signature width whose complete node/key product exceeds uint64.
positions_deg = [0 0; 1 0];
cost_deg = [Inf 1; 1 Inf];
representatives_deg = zeros(40, 2);
visibilityFunction = @(first_deg, second_deg) ...
    true(size(first_deg, 1), 1);

verifyError(testCase, @() ...
    obstacleAvoidance.planner.searchSpatialHomologyRoutes( ...
    cost_deg, positions_deg, representatives_deg, 1, visibilityFunction), ...
    "searchSpatialHomologyRoutes:StateKeyCapacityExceeded");
end

function [cost_deg, positions_deg] = chainGraph(nodeCount)
% Create a single bent chain with start and goal stored in public order.
positions_deg = zeros(nodeCount, 2);
positions_deg(1, :) = [0 0];
positions_deg(2, :) = [nodeCount - 1, 0];
positions_deg(3:end, 1) = (1:nodeCount - 2).';
positions_deg(3:end, 2) = 0.1 * sin((1:nodeCount - 2).');
nodeOrder = [1, 3:nodeCount, 2];
cost_deg = Inf(nodeCount);
for edgeIndex = 1:numel(nodeOrder) - 1
    firstNode = nodeOrder(edgeIndex);
    secondNode = nodeOrder(edgeIndex + 1);
    edgeLength_deg = norm( ...
        positions_deg(firstNode, :) - positions_deg(secondNode, :));
    cost_deg(firstNode, secondNode) = edgeLength_deg;
    cost_deg(secondNode, firstNode) = edgeLength_deg;
end
end
