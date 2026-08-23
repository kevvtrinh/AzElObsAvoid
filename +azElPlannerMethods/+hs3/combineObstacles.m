function azElObstacles = combineObstacles(varargin)
%% Section 0: Header & Readme
% SYNTAX
%   azElObstacles = azElPlannerMethods.hs3.combineObstacles()
%   azElObstacles = azElPlannerMethods.hs3.combineObstacles([])
%   azElObstacles = azElPlannerMethods.hs3.combineObstacles(obstacle1, obstacle2, ...)
%   azElObstacles = azElPlannerMethods.hs3.combineObstacles(obstacleArray)
%   azElObstacles = azElPlannerMethods.hs3.combineObstacles(nestedObstacleCells)
%**************************************************************************
% PURPOSE
%   - Flatten and validate canonical obstacle inputs in caller order.
%   - Return a schema-preserving empty array for obstacle-free planning.
%**************************************************************************
% INPUTS
%   - varargin (struct arrays, nested cell arrays, or empty numeric input)
%       Every nonempty leaf must be a canonical scalar azElData struct.
%**************************************************************************
% OUTPUTS
%   - azElObstacles (column struct array)
%       Independently retained, validated obstacle records.
%**************************************************************************
% UNITS
%   - Canonical az_deg and el_deg fields are degrees; time_s is seconds.
%**************************************************************************

%% Section 1: Validate Inputs

if nargin == 0
    azElObstacles = emptyCanonicalAzElObstacles();
    return;
end

%% Section 2: Flatten Nested Inputs

% The stack carries each top-level argument number so a malformed nested
% value still identifies the public input that contained it. Reversing each
% push makes the next pop match the caller's original order.
pendingCapacity = max(16, nargin);
pendingValues = cell(pendingCapacity, 1);
pendingInputIndices = zeros(pendingCapacity, 1);
pendingValues(1:nargin) = flipud(varargin(:));
pendingInputIndices(1:nargin) = (nargin:-1:1).';
pendingCount = nargin;
obstacleCapacity = 16;
obstacleItems = cell(obstacleCapacity, 1);
obstacleCount = 0;

% Drain the explicit stack until every nested cell and struct array has been
% flattened without changing the caller's obstacle order.
while pendingCount > 0
    pendingInputValue = pendingValues{pendingCount};
    topLevelInputIndex = pendingInputIndices(pendingCount);
    pendingValues{pendingCount} = [];
    pendingCount = pendingCount - 1;
    if isnumeric(pendingInputValue) && isempty(pendingInputValue)
        continue;
    elseif isstruct(pendingInputValue)
        flattenedStructItems = num2cell(pendingInputValue(:));
        addedObstacleCount = numel(flattenedStructItems);
        requiredObstacleCapacity = obstacleCount + addedObstacleCount;
        if requiredObstacleCapacity > obstacleCapacity
            obstacleCapacity = max( ...
                2 * obstacleCapacity, requiredObstacleCapacity);
            obstacleItems{obstacleCapacity, 1} = [];
        end
        obstacleWriteRows = obstacleCount + (1:addedObstacleCount);
        obstacleItems(obstacleWriteRows) = flattenedStructItems;
        obstacleCount = requiredObstacleCapacity;
    elseif iscell(pendingInputValue)
        nestedValueCount = numel(pendingInputValue);
        reversedNestedValues = flipud(pendingInputValue(:));
        requiredPendingCapacity = pendingCount + nestedValueCount;
        if requiredPendingCapacity > pendingCapacity
            pendingCapacity = max( ...
                2 * pendingCapacity, requiredPendingCapacity);
            pendingValues{pendingCapacity, 1} = [];
            pendingInputIndices(pendingCapacity, 1) = 0;
        end
        pendingRows = pendingCount + (1:nestedValueCount);
        pendingValues(pendingRows) = reversedNestedValues;
        pendingInputIndices(pendingRows) = topLevelInputIndex;
        pendingCount = requiredPendingCapacity;
    else
        error("combineAzElObstacles:InvalidInput", ...
            "Input %d must be an azElData struct, struct array, or cell " + ...
            "array containing azElData structs.", topLevelInputIndex);
    end
end
obstacleItems = obstacleItems(1:obstacleCount);
if obstacleCount == 0
    azElObstacles = emptyCanonicalAzElObstacles();
    return;
end

%% Section 3: Normalize The Public Schema

% Validation occurs after flattening so all accepted container forms reach
% one schema gate. A bad obstacle therefore cannot survive merely because
% it arrived inside a cell or struct array.
normalizedObstacles = cell(size(obstacleItems));

% Send every flattened record through the same canonical schema gate before
% assembling the final obstacle array.
for obstacleIndex = 1:numel(obstacleItems)
    normalizedObstacles{obstacleIndex} = azElPlannerMethods.hs3.normalizeTimeObstacleData( ...
        obstacleItems{obstacleIndex});
end
azElObstacles = vertcat(normalizedObstacles{:});
end


function azElObstacles = emptyCanonicalAzElObstacles()
% Define canonical obstacle field order for an empty collection.
template = struct( ...
    "targetName", "", ...
    "time_s", zeros(0, 1), ...
    "az_deg", {cell(0, 1)}, ...
    "el_deg", {cell(0, 1)}, ...
    "originalAz_deg", {cell(0, 1)}, ...
    "originalEl_deg", {cell(0, 1)}, ...
    "safetyMargin_deg", 0, ...
    "status", strings(0, 1));
azElObstacles = repmat(template, 0, 1);
end
