function [isOccupied, blockingObstacleIndex, queryDetails] = queryAzElTimeObstacle(obstacles, azimuth_deg, elevation_deg, queryTime, optionOverrides)
%% Section 0: Header & Readme
% SYNTAX
%   options = queryAzElTimeObstacle()
%   isOccupied = queryAzElTimeObstacle(obstacles, azimuth_deg, elevation_deg, queryTime)
%   [isOccupied, blockingObstacleIndex, queryDetails] = queryAzElTimeObstacle(obstacles, azimuth_deg, elevation_deg, queryTime, optionOverrides)
%**************************************************************************
% PURPOSE
%   - Query protected time-dependent geometry with the exact obstacle-query
%     implementation preserved beside the selected planner.
%**************************************************************************
% INPUTS
%   - obstacles (canonical obstacle array, nested cells, or [])
%   - azimuth_deg, elevation_deg (matching numeric arrays or scalars)
%   - queryTime (numeric seconds or datetime array)
%   - optionOverrides (scalar struct, optional)
%       PlannerMethod selects "corridorQuintic" or "hs3". The remaining
%       query options are forwarded only to that method's query function.
%**************************************************************************
% OUTPUTS
%   - isOccupied (logical array)
%   - blockingObstacleIndex (uint32 array, zero where clear)
%   - queryDetails (signed clearance, blocker, time, and option diagnostics)
%**************************************************************************

%% Section 1: Preserve Defaults And Query Call Forms

if nargin == 0
    isOccupied = azElPlannerMethods.corridor.queryTimeObstacle();
    isOccupied.PlannerMethod = "corridorQuintic";
    blockingObstacleIndex = [];
    queryDetails = struct();
    return;
end

if nargin ~= 4 && nargin ~= 5
    error("queryAzElTimeObstacle:InvalidCall", ...
        "Use zero inputs, four query inputs, or four query inputs plus options.");
end

if nargin < 5 || isempty(optionOverrides)
    optionOverrides = struct();
end

%% Section 2: Read And Remove The Public Selector

% Invalid option records are forwarded unchanged to the default corridor
% query so its established contract error remains authoritative.
plannerMethod = "corridorQuintic";
methodOverrides = optionOverrides;

if isstruct(optionOverrides) && isscalar(optionOverrides) && ...
        isfield(optionOverrides, "PlannerMethod") && ...
        ~isempty(optionOverrides.PlannerMethod)
    methodDefaults = planAzElMotion(optionOverrides.PlannerMethod);
    plannerMethod = methodDefaults.PlannerMethod;
end

if isstruct(methodOverrides) && isscalar(methodOverrides) && ...
        isfield(methodOverrides, "PlannerMethod")
    methodOverrides = rmfield(methodOverrides, "PlannerMethod");
end

switch plannerMethod
    case "corridorQuintic"
        queryFunction = @azElPlannerMethods.corridor.queryTimeObstacle;

    case "hs3"
        queryFunction = @azElPlannerMethods.hs3.queryTimeObstacle;
end

%% Section 3: Preserve The Caller's Requested Output Work

% The method query has a fast occupancy-only path. Request only the outputs
% the caller asked for so the dispatcher does not accidentally force the
% more expensive signed-clearance diagnostics on every query.
if nargout <= 1
    isOccupied = queryFunction(obstacles, azimuth_deg, elevation_deg, queryTime, methodOverrides);
    blockingObstacleIndex = [];
    queryDetails = struct();
elseif nargout == 2
    [isOccupied, blockingObstacleIndex] = queryFunction(obstacles, azimuth_deg, elevation_deg, queryTime, methodOverrides);
    queryDetails = struct();
else
    [isOccupied, blockingObstacleIndex, queryDetails] = queryFunction( ...
        obstacles, azimuth_deg, elevation_deg, queryTime, methodOverrides);
    queryDetails.Options.PlannerMethod = plannerMethod;
end
end
