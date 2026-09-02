function [routes_deg, classPattern, searchRecord] = ...
        searchDistinctSpatialRoutes( ...
        edgeCost_deg, nodePosition_deg, obstacleReferencePoints_deg, ...
        maximumClassCount, edgeCheck, options)
%% Section 0: Header & Readme
% SYNTAX
%   [routes_deg, classPattern, searchRecord] = ...
%       obstacleAvoidance.search.searchDistinctSpatialRoutes( ...
%       edgeCost_deg, nodePosition_deg, obstacleReferencePoints_deg, ...
%       maximumClassCount, edgeCheck, options)
%**************************************************************************
% PURPOSE
%   - Find bounded visibility-graph routes that pass obstacle reference
%     points in distinct ways and retain complete search evidence.
%**************************************************************************
% INPUTS
%   - edgeCost_deg (N-by-N numeric array)
%       Finite visible-edge lengths and Inf for unavailable edges.
%   - nodePosition_deg (N-by-2 finite numeric array)
%       Visibility nodes with start and goal first.
%   - obstacleReferencePoints_deg (R-by-2 numeric array)
%       Points used to tell route classes apart.
%   - maximumClassCount (nonnegative integer scalar)
%       Maximum number of distinct routes to retain.
%   - edgeCheck (function handle)
%       Direct visibility check used by route shortening.
%   - options (resolved scalar struct)
%       Search work and cancellation controls.
%**************************************************************************
% OUTPUTS
%   - routes_deg (cell column of N-by-2 numeric arrays)
%       Distinct shortened route suggestions.
%   - classPattern (integer matrix)
%       Route-class pattern for each retained route.
%   - searchRecord (scalar struct)
%       Explored states, frontier, cleanup decisions, and work counts.
%**************************************************************************
% UNITS
%   - Positions and edge costs are degrees.
%**************************************************************************

%% Section 1: Search The Established Route Classes

% Preserve the proven search implementation while moving its production call
% into the search package and giving the stage a plain behavior-oriented name.
[routes_deg, classPattern, searchRecord] = ...
    obstacleAvoidance.planner.searchSpatialHomologyRoutes( ...
    edgeCost_deg, nodePosition_deg, obstacleReferencePoints_deg, ...
    maximumClassCount, edgeCheck, options);
end
