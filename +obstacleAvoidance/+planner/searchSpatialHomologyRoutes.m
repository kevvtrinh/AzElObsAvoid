function [routes, signatures, record] = searchSpatialHomologyRoutes( ...
        cost_deg, positions_deg, representatives_deg, maximumCount, ...
        visibilityFunction, options)
%% Section 0: Header & Readme
% SYNTAX
%   [routes, signatures, record] = ...
%       obstacleAvoidance.planner.searchSpatialHomologyRoutes( ...
%       cost_deg, positions_deg, representatives_deg, maximumCount, ...
%       visibilityFunction)
%   [routes, signatures, record] = ...
%       obstacleAvoidance.planner.searchSpatialHomologyRoutes( ...
%       cost_deg, positions_deg, representatives_deg, maximumCount, ...
%       visibilityFunction, options)
%**************************************************************************
% PURPOSE
%   - Provide a deprecated compatibility alias for the search-owned distinct
%     spatial-route implementation.
%**************************************************************************
% INPUTS
%   - cost_deg, positions_deg, representatives_deg (numeric arrays)
%       Historical visibility graph, nodes, and route reference points.
%   - maximumCount (nonnegative integer scalar)
%       Maximum distinct route count.
%   - visibilityFunction (scalar function handle)
%       Proposal-geometry chord predicate.
%   - options (scalar struct, optional)
%       Cancellation controls forwarded unchanged.
%**************************************************************************
% OUTPUTS
%   - routes, signatures, record
%       Outputs from searchDistinctSpatialRoutes without modification.
%**************************************************************************
% UNITS
%   - Position and edge cost are degrees; signatures are dimensionless.
%**************************************************************************

%% Section 1: Forward The Deprecated Name

if nargin < 6
    options = struct();
end
[routes, signatures, record] = ...
    obstacleAvoidance.search.searchDistinctSpatialRoutes( ...
    cost_deg, positions_deg, representatives_deg, maximumCount, ...
    visibilityFunction, options);
end
