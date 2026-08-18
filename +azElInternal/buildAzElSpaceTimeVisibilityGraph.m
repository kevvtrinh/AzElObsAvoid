function graph = buildAzElSpaceTimeVisibilityGraph( ...
        obstacleField, snapshotGraphs, initialState, goalState, options)
%% Section 0: Header & Readme
% SYNTAX
%   graph = azElInternal.buildAzElSpaceTimeVisibilityGraph()
%   graph = azElInternal.buildAzElSpaceTimeVisibilityGraph( ...
%       obstacleField, snapshotGraphs, initialState, goalState, options)
%**************************************************************************
% PURPOSE
%   - Preserve the deprecated discrete-graph function name as a forwarding
%     compatibility alias for the safe-interval roadmap implementation.
%**************************************************************************
% INPUTS
%   - obstacleField (scalar packed obstacle field)
%       Protected geometry from buildAzElTimeObstacleField.
%   - snapshotGraphs (structure array)
%       Spatial visibility graphs used to form sparse roadmap connections.
%   - initialState, goalState (scalar structs)
%       time_s and position_deg fields for the planning request.
%   - options (scalar struct)
%       Validated safe-interval roadmap and SIPP controls.
%**************************************************************************
% OUTPUTS
%   - graph (scalar struct)
%       Safe-interval roadmap, SIPP trace, candidates, and primary path.
%**************************************************************************
% UNITS
%   - Position is [azimuth elevation] in degrees. Time is seconds.
%**************************************************************************

%% Section 1: Forward To The Maintained Safe-Interval Search

if nargin == 0
    graph = azElInternal.buildAzElSafeIntervalRoadmap();
    return;
end
graph = azElInternal.buildAzElSafeIntervalRoadmap( ...
    obstacleField, snapshotGraphs, initialState, goalState, options);
end
