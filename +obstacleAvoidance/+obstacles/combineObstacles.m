function obstacleField = combineObstacles(varargin)
%% Section 0: Header & Readme
% SYNTAX
%   obstacles = obstacleAvoidance.obstacles.combineObstacles()
%   obstacles = obstacleAvoidance.obstacles.combineObstacles([])
%   obstacles = obstacleAvoidance.obstacles.combineObstacles(obstacle1, obstacle2, ...)
%   obstacles = obstacleAvoidance.obstacles.combineObstacles(obstacleArray)
%   obstacles = obstacleAvoidance.obstacles.combineObstacles(nestedObstacleCells)
%**************************************************************************
% PURPOSE
%   - Flatten and validate canonical obstacle inputs in caller order.
%   - Return a field-preserving empty array for obstacle-free planning.
%**************************************************************************
% INPUTS
%   - varargin (struct arrays, nested cell arrays, or empty numeric input)
%       Every nonempty leaf must be a canonical scalar obstacleData struct.
%**************************************************************************
% OUTPUTS
%   - obstacleField (column struct array)
%       Independently retained, validated obstacle records.
%**************************************************************************
% UNITS
%   - Canonical az_deg and el_deg fields are degrees; time_s is seconds.
%**************************************************************************

%% Section 1: Validate Inputs

% Accept obstacle records, arrays, and nested cells. Reject unrelated values
% before flattening because an error later would not identify the bad input.

if nargin == 0
    obstacleField = createEmptyObstacleArray();
    return;
end

%% Section 2: Flatten Nested Inputs

% Walk nested inputs in their original order. Preserve obstacle order because
% diagnostics and corridor records refer to obstacles by numeric index.

% The stack carries each top-level argument number so a malformed nested
% value still identifies the public input that contained it. Reversing each
% push makes the next pop match the caller's original order.
% An explicit stack avoids recursion, which could overflow for deeply nested
% cells. Storage grows geometrically, so large collections do not repeatedly
% copy the entire working list for every newly discovered obstacle.
pendingCapacity = max(16, nargin);
pendingValues = cell(pendingCapacity, 1);
pendingInputIndices = zeros(pendingCapacity, 1);
pendingValues(1:nargin) = flipud(varargin(:));
pendingInputIndices(1:nargin) = (nargin:-1:1).';
pendingCount = nargin;
obstacleCapacity = 16;
obstacleItems = cell(obstacleCapacity, 1);
obstacleCount = 0;

% Pop nested inputs from the explicit work stack until every leaf obstacle
% has been collected or rejected with its top-level input index intact.
while pendingCount > 0
    pendingInputValue = pendingValues{pendingCount};
    topLevelInputIndex = pendingInputIndices(pendingCount);
    pendingValues{pendingCount} = [];
    pendingCount = pendingCount - 1;
    if isnumeric(pendingInputValue) && isempty(pendingInputValue)
        % Empty numeric values are the documented way to say "no obstacles."
        % They may appear at any nesting depth without creating a record.
        continue;
    elseif isstruct(pendingInputValue)
        % Convert a structure array into scalar items. Later normalization then
        % checks every record independently and preserves the caller's order.
        flattenedStructItems = num2cell(pendingInputValue(:));
        addedObstacleCount = numel(flattenedStructItems);
        requiredObstacleCapacity = obstacleCount + addedObstacleCount;
        if requiredObstacleCapacity > obstacleCapacity
            obstacleCapacity = max( 2 * obstacleCapacity, requiredObstacleCapacity);
            obstacleItems{obstacleCapacity, 1} = [];
        end
        obstacleWriteRows = obstacleCount + (1:addedObstacleCount);
        obstacleItems(obstacleWriteRows) = flattenedStructItems;
        obstacleCount = requiredObstacleCapacity;
    elseif iscell(pendingInputValue)
        % Push children in reverse because this is a last-in, first-out stack.
        % They will therefore be popped in their original left-to-right order.
        nestedValueCount = numel(pendingInputValue);
        reversedNestedValues = flipud(pendingInputValue(:));
        requiredPendingCapacity = pendingCount + nestedValueCount;
        if requiredPendingCapacity > pendingCapacity
            pendingCapacity = max( 2 * pendingCapacity, requiredPendingCapacity);
            pendingValues{pendingCapacity, 1} = [];
            pendingInputIndices(pendingCapacity, 1) = 0;
        end
        pendingRows = pendingCount + (1:nestedValueCount);
        pendingValues(pendingRows) = reversedNestedValues;
        pendingInputIndices(pendingRows) = topLevelInputIndex;
        pendingCount = requiredPendingCapacity;
    else
        error("combineObstacles:InvalidInput", ...
            "Input %d must be an obstacleData struct, struct array, or cell " + ...
            "array containing obstacleData structs.", topLevelInputIndex);
    end
end
obstacleItems = obstacleItems(1:obstacleCount);
if obstacleCount == 0
    obstacleField = createEmptyObstacleArray();
    return;
end

%% Section 3: Normalize The Public Format

% Send the combined array through the common obstacle normalizer. This applies
% the same field checks and safety-margin rules as direct construction.

% Validation occurs after flattening so all accepted container forms reach
% one format check. A bad obstacle therefore cannot survive merely because
% it arrived inside a cell or struct array.
normalizedObstacles = cell(size(obstacleItems));

% Send every collected obstacle through the same canonical format check before
% concatenating them into the single array returned to the planner.
for obstacleIndex = 1:numel(obstacleItems)
    normalizedObstacles{obstacleIndex} = ...
        obstacleAvoidance.obstacles.createObstacle( ...
            obstacleItems{obstacleIndex});
end
obstacleField = vertcat(normalizedObstacles{:});
end

function obstacleField = createEmptyObstacleArray()
% Define canonical obstacle field order for an empty collection. repmat with a
% zero row count preserves those fields, allowing downstream concatenation and
% field access without special handling for obstacle-free requests.
template = struct( ...
    "targetName", "", ...
    "time_s", zeros(0, 1), ...
    "az_deg", {cell(0, 1)}, ...
    "el_deg", {cell(0, 1)}, ...
    "originalAz_deg", {cell(0, 1)}, "originalEl_deg", {cell(0, 1)}, "safetyMargin_deg", 0, "status", strings(0, 1));
obstacleField = repmat(template, 0, 1);
end
