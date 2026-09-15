function obstacleField = combineObstacles(varargin)
%% Section 0: Header & Readme
% SYNTAX
%   obstacles = obstacleAvoidance.obstacles.combineObstacles()
%   obstacles = obstacleAvoidance.obstacles.combineObstacles(obstacleInputs)
%   obstacles = obstacleAvoidance.obstacles.combineObstacles(obstacle1, obstacle2)
%**************************************************************************
% PURPOSE
%   - Flatten and validate canonical obstacle inputs in caller order.
%   - Return a field-preserving empty array for obstacle-free planning.
%**************************************************************************
% INPUTS
%   - obstacleInputs (struct arrays, nested cells, or empty numeric values)
%       Every nonempty leaf must be a canonical obstacle record.
%**************************************************************************
% OUTPUTS
%   - obstacleField (column struct array)
%       Independently normalized obstacle records in caller order. Invalid
%       input throws an error.
%**************************************************************************
% UNITS
%   - Boundary coordinates use coordinate units; time_s uses seconds.
%**************************************************************************

%% Section 1: Flatten Nested Inputs

% Flatten inputs while keeping their original index for error messages.
obstacleItems = cell(0, 1);
for inputIndex = 1:nargin
    obstacleItems = [obstacleItems; flattenValue(varargin{inputIndex}, inputIndex)]; %#ok<AGROW>
end

%% Section 2: Normalize The Public Format

if isempty(obstacleItems)
    obstacleField = createEmptyObstacleArray();
    return
end
normalizedObstacles = cell(size(obstacleItems));
for obstacleIndex = 1:numel(obstacleItems)
    normalizedObstacles{obstacleIndex} = obstacleAvoidance.obstacles.createObstacle( ...
        obstacleItems{obstacleIndex});
end
obstacleField = vertcat(normalizedObstacles{:});
end

%% Section 3: Local Functions

function items = flattenValue(value, ownerIndex)
    % Flatten nested cells in input order.
    if isnumeric(value) && isempty(value)
        items = cell(0, 1);
    elseif isstruct(value)
        items = num2cell(value(:));
    elseif iscell(value)
        items = cell(0, 1);
        for childIndex = 1:numel(value)
            items = [items; flattenValue(value{childIndex}, ownerIndex)]; %#ok<AGROW>
        end
    else
        error("combineObstacles:InvalidInput", ...
            "Input %d must contain only obstacle structs or empty values.", ownerIndex);
    end
end

function obstacleField = createEmptyObstacleArray()
    % Keep the same fields for an empty obstacle array.
    template = struct( ...
        "targetName",               "", ...
        "time_s",                   zeros(0, 1), ...
        "x_units",                  {cell(0, 1)}, ...
        "y_units",                  {cell(0, 1)}, ...
        "originalX_units",          {cell(0, 1)}, ...
        "originalY_units",          {cell(0, 1)}, ...
        "safetyMargin_units",       0, ...
        "status",                   strings(0, 1), ...
        "NormalizationDiagnostics", struct(), ...
        "vertexCorrespondence",     "circularCorrelation");
    obstacleField = repmat(template, 0, 1);
end
