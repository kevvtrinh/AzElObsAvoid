function output = ruckigWarmStart(operation, varargin)
%% Section 0: Header & Readme
% SYNTAX
%   output = obstacleAvoidance.planner.ruckigWarmStart( ...
%       operation, inputArguments...)
%**************************************************************************
% PURPOSE
%   - Provide the single optional boundary for Ruckig-derived HS3 warm-start
%     behavior.
%   - Permit this file to be deleted without disabling the maintained planner
%     or either standalone trajectory engine.
%**************************************************************************
% INPUTS
%   - operation (string scalar)
%       Internal operation: classifySearch, createActivityMesh, or
%       solveSeedCandidate.
%   - inputArguments (operation-dependent values)
%       Normalized planner-owned records forwarded to the selected internal
%       warm-start implementation.
%**************************************************************************
% OUTPUTS
%   - output (operation-dependent scalar struct)
%       Classification, mesh decision, or Ruckig-derived candidate record.
%       Invalid operations or argument lists throw identified errors.
%**************************************************************************
% UNITS
%   - Positions use degrees, time uses seconds, and derivatives use degrees
%     per corresponding power of seconds. Mesh coordinates are dimensionless.
%**************************************************************************

%% Section 1: Dispatch Optional Warm-Start Work

operation = string(operation);
if ~isscalar(operation)
    error("ruckigWarmStart:InvalidOperation", ...
        "operation must be a string scalar.");
end
switch operation
    case "classifySearch"
        requireArgumentCount(operation, varargin, 2);
        output = obstacleAvoidance.planner.classifyPassThroughSearch( ...
            varargin{1}, varargin{2});
    case "createActivityMesh"
        requireArgumentCount(operation, varargin, 4);
        output = obstacleAvoidance.planner.createHybridActivityMesh( ...
            varargin{1}, varargin{2}, varargin{3}, varargin{4});
    case "solveSeedCandidate"
        if numel(varargin) == 5
            output = ...
                obstacleAvoidance.planner.solvePassThroughSeedCandidate( ...
                varargin{1}, varargin{2}, varargin{3}, varargin{4}, ...
                varargin{5});
        elseif numel(varargin) == 6
            output = ...
                obstacleAvoidance.planner.solvePassThroughSeedCandidate( ...
                varargin{1}, varargin{2}, varargin{3}, varargin{4}, ...
                varargin{5}, varargin{6});
        else
            error("ruckigWarmStart:InvalidArgumentCount", ...
                "Operation %s requires five or six input arguments; " + ...
                "observed %d.", operation, numel(varargin));
        end
    otherwise
        error("ruckigWarmStart:InvalidOperation", ...
            "Unsupported warm-start operation: %s.", operation);
end
end

%% Section 2: Local Functions

function requireArgumentCount(operation, arguments, expectedCount)
% Reject incomplete internal calls at the optional component boundary.
if numel(arguments) ~= expectedCount
    error("ruckigWarmStart:InvalidArgumentCount", ...
        "Operation %s requires %d input arguments; observed %d.", ...
        operation, expectedCount, numel(arguments));
end
end
