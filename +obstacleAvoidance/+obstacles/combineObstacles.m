function obstacles = combineObstacles(varargin)
%% Section 0: Header & Readme
% SYNTAX
%   obstacles = obstacleAvoidance.obstacles.combineObstacles()
%   obstacles = obstacleAvoidance.obstacles.combineObstacles(obstacleInputs)
%   obstacles = obstacleAvoidance.obstacles.combineObstacles(obstacle1, obstacle2)
%**************************************************************************
% PURPOSE
%   - Combine individual records, arrays, and nested groups into one column
%     of checked obstacle records, keeping the caller's order.
%   - With no obstacles, return an empty array with the usual obstacle fields.
%**************************************************************************
% INPUTS
%   - obstacleInputs (struct arrays, nested cells, or empty numeric values)
%       Each nonempty entry must be a standard obstacle record.
%**************************************************************************
% OUTPUTS
%   - obstacles (column struct array)
%       Checked obstacle records in caller order. Invalid input throws an error.
%**************************************************************************
% UNITS
%   - Boundary coordinates use coordinate units; time_s uses seconds.
%**************************************************************************

%% Section 1: Flatten Nested Inputs

% For example, {A, {B, C}} becomes [A; B; C]. Keep each original input index
% so an error in a nested group can still identify the caller's argument.
obstacleItems = cell(0, 1);
for inputIndex = 1:nargin
    obstacleItems = [obstacleItems; flattenObstacleInputs(varargin{inputIndex}, inputIndex)]; %#ok<AGROW>
end

%% Section 2: Check Each Record And Build The Combined Array

if isempty(obstacleItems)
    obstacles = createEmptyObstacleArray();
    return
end

% Standardize every record before joining them into one struct array.
% createObstacle keeps original and protected geometry distinct.
normalizedObstacles = cell(size(obstacleItems));
for obstacleIndex = 1:numel(obstacleItems)
    normalizedObstacles{obstacleIndex} = obstacleAvoidance.obstacles.createObstacle( ...
        obstacleItems{obstacleIndex});
end
obstacles = vertcat(normalizedObstacles{:});
end

%% Section 3: Local Functions

function obstacleItems = flattenObstacleInputs(obstacleInput, inputIndex)
    % Visit nested cells in order and collect one struct per obstacle.
    % Keep the top-level inputIndex as we go deeper, for useful error messages.
    if isnumeric(obstacleInput) && isempty(obstacleInput)
        obstacleItems = cell(0, 1);
    elseif isstruct(obstacleInput)
        obstacleItems = num2cell(obstacleInput(:));
    elseif iscell(obstacleInput)
        obstacleItems = cell(0, 1);
        for itemIndex = 1:numel(obstacleInput)
            obstacleItems = [obstacleItems; flattenObstacleInputs(obstacleInput{itemIndex}, inputIndex)]; %#ok<AGROW>
        end
    else
        error("combineObstacles:InvalidInput", ...
            "Input %d must contain only obstacle structs or empty values.", inputIndex);
    end
end

function obstacles = createEmptyObstacleArray()
    % Even with no obstacles, callers can use the usual field names.
    % The template supplies those names; the returned array has no elements.
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
        "vertexCorrespondence",     "circularCorrelation", ...
        "UsesSourceIndex",          false);
    obstacles = repmat(template, 0, 1);
end
