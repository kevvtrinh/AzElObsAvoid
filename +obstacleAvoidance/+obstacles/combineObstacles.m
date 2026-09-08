function obstacleField = combineObstacles(varargin)
%% Section 0: Header & Readme
% SYNTAX
%   obstacles = obstacleAvoidance.obstacles.combineObstacles()
%   obstacles = obstacleAvoidance.obstacles.combineObstacles([])
%   obstacles = obstacleAvoidance.obstacles.combineObstacles(obstacle1, ...)
%   obstacles = obstacleAvoidance.obstacles.combineObstacles(obstacleArray)
%   obstacles = obstacleAvoidance.obstacles.combineObstacles(nestedCells)
%
% PURPOSE
%   - Flatten and validate canonical obstacle inputs in caller order.
%   - Return a field-preserving empty array for obstacle-free planning.
%
% INPUTS
%   - varargin (struct arrays, nested cell arrays, or empty numeric input)
%       Every nonempty leaf must be a canonical obstacle record.
%
% OUTPUTS
%   - obstacleField (column struct array)
%       Independently normalized obstacle records in caller order.
%
% UNITS
%   - Canonical x_units and y_units fields are coordinate units; time_s is seconds.
%

%% Section 1: Flatten Nested Inputs

% Flatten inputs while keeping their original index for error messages.
if nargin == 0
    obstacleField = createEmptyObstacleArray();
    return;
end
obstacleItems = cell(0, 1);
% Process each input needed to complete combine obstacles.
for inputIndex = 1:nargin
    obstacleItems = [obstacleItems; flattenValue(varargin{inputIndex}, inputIndex)]; %#ok<AGROW>
end

%% Section 2: Normalize The Public Format

if isempty(obstacleItems)
    obstacleField = createEmptyObstacleArray();
    return;
end
normalized = cell(size(obstacleItems));
% Evaluate each obstacle against the current geometry or motion.
for obstacleIndex = 1:numel(obstacleItems)
    normalized{obstacleIndex} = obstacleAvoidance.obstacles.createObstacle(obstacleItems{obstacleIndex});
end
obstacleField = vertcat(normalized{:});
end

function items = flattenValue(value, owner)
    % Flatten nested cells in input order.
    if isnumeric(value) && isempty(value)
        items = cell(0, 1);
    elseif isstruct(value)
        items = num2cell(value(:));
    elseif iscell(value)
        items = cell(0, 1);
        % Process each child needed to complete flatten value.
        for childIndex = 1:numel(value)
            items = [items; flattenValue(value{childIndex}, owner)]; %#ok<AGROW>
        end
    else
        error("combineObstacles:InvalidInput", "Input %d must contain only obstacle structs or empty values.", owner);
    end
end

function obstacleField = createEmptyObstacleArray()
    % Keep the same fields for an empty obstacle array.
    template = struct("targetName", "", "time_s", zeros(0, 1), ...
        "x_units", {cell(0, 1)}, "y_units", {cell(0, 1)}, "originalX_units", {cell(0, 1)}, ...
        "originalY_units", {cell(0, 1)}, "safetyMargin_units", 0, "status", strings(0, 1));
    obstacleField = repmat(template, 0, 1);
end
