function obstacleField = combineObstacles(varargin)
%% Section 0: Header & Readme
% SYNTAX
%   obstacles = obstacleAvoidance.obstacles.combineObstacles()
%   obstacles = obstacleAvoidance.obstacles.combineObstacles([])
%   obstacles = obstacleAvoidance.obstacles.combineObstacles(obstacle1, ...)
%   obstacles = obstacleAvoidance.obstacles.combineObstacles(obstacleArray)
%   obstacles = obstacleAvoidance.obstacles.combineObstacles(nestedCells)
%**************************************************************************
% PURPOSE
%   - Flatten and validate canonical obstacle inputs in caller order.
%   - Return a field-preserving empty array for obstacle-free planning.
%**************************************************************************
% INPUTS
%   - varargin (struct arrays, nested cell arrays, or empty numeric input)
%       Every nonempty leaf must be a canonical obstacle record.
%**************************************************************************
% OUTPUTS
%   - obstacleField (column struct array)
%       Independently normalized obstacle records in caller order.
%**************************************************************************
% UNITS
%   - Canonical az_deg and el_deg fields are degrees; time_s is seconds.
%**************************************************************************

%% Section 1: Flatten Nested Inputs

% Flatten each top-level input independently so errors retain its public index.
if nargin == 0
    obstacleField = createEmptyObstacleArray();
    return;
end
obstacleItems = cell(0, 1);
for inputIndex = 1:nargin
    obstacleItems = [obstacleItems; flattenValue(varargin{inputIndex}, inputIndex)]; %#ok<AGROW>
end

%% Section 2: Normalize The Public Format

if isempty(obstacleItems)
    obstacleField = createEmptyObstacleArray();
    return;
end
normalized = cell(size(obstacleItems));
hasReusablePreparation = true;
for obstacleIndex = 1:numel(obstacleItems)
    normalized{obstacleIndex} = ...
        obstacleAvoidance.obstacles.createObstacle(obstacleItems{obstacleIndex});
    hasReusablePreparation = hasReusablePreparation && ...
        isfield(obstacleItems{obstacleIndex}, "InternalPreparation");
end
% A complete collection-wide cache can survive normalization because
% prepareDynamic verifies its immutable source snapshot before every reuse.
% Drop partial caches so the resulting structure array remains uniform.
if hasReusablePreparation
    for obstacleIndex = 1:numel(obstacleItems)
        normalized{obstacleIndex}.InternalPreparation = ...
            obstacleItems{obstacleIndex}.InternalPreparation;
    end
end
obstacleField = vertcat(normalized{:});
end

function items = flattenValue(value, owner)
% Flatten one nested container in caller order and retain its top-level owner.
if isnumeric(value) && isempty(value)
    items = cell(0, 1);
elseif isstruct(value)
    items = num2cell(value(:));
elseif iscell(value)
    items = cell(0, 1);
    for childIndex = 1:numel(value)
        items = [items; flattenValue(value{childIndex}, owner)]; %#ok<AGROW>
    end
else
    error("combineObstacles:InvalidInput", ...
        "Input %d must contain only obstacle structs or empty values.", owner);
end
end

function obstacleField = createEmptyObstacleArray()
% Preserve canonical field order even when the collection has no records.
template = struct("targetName", "", "time_s", zeros(0, 1), ...
    "az_deg", {cell(0, 1)}, "el_deg", {cell(0, 1)}, "originalAz_deg", {cell(0, 1)}, ...
    "originalEl_deg", {cell(0, 1)}, "safetyMargin_deg", 0, "status", strings(0, 1));
obstacleField = repmat(template, 0, 1);
end
