function inputs = createRequestInputs(request)
%% Section 0: Header & Readme
% SYNTAX
%   inputs = webSandbox.fileProtocol.createRequestInputs(request)
%**************************************************************************
% PURPOSE
%   - Convert JSON-safe scene records into existing public planner inputs.
%**************************************************************************
% INPUTS
%   - request (scalar struct)
%       Polygon slices hold N-by-2 vertices_deg [azimuth elevation] arrays.
%**************************************************************************
% OUTPUTS
%   - inputs (scalar struct)
%       Canonical obstacles and unchanged planner state, limits, and options.
%**************************************************************************
% UNITS
%   - Positions are degrees; time is seconds; derivatives use degree units.
%**************************************************************************

%% Section 1: Validate The Request Shape

if ~isstruct(request) || ~isscalar(request)
    error("webSandbox:createRequestInputs:InvalidRequest", ...
        "request must be one JSON object.");
end
requiredFields = ["mode", "initialState", "limits"];
missingNames = requiredFields(~isfield(request, requiredFields));
if ~isempty(missingNames)
    error("webSandbox:createRequestInputs:MissingFields", ...
        "request is missing: %s.", strjoin(missingNames, ", "));
end
mode = lower(string(request.mode));
if ~isscalar(mode) || ~any(mode == ["trajectory", "intercept"])
    error("webSandbox:createRequestInputs:InvalidMode", ...
        "mode must be 'trajectory' or 'intercept'.");
end
if mode == "trajectory" && ~isfield(request, "goalState")
    error("webSandbox:createRequestInputs:MissingGoalState", ...
        "Trajectory requests require goalState.");
end
if mode == "intercept" && ~isfield(request, "targetMotion")
    error("webSandbox:createRequestInputs:MissingTargetMotion", ...
        "Intercept requests require targetMotion.");
end

%% Section 2: Construct Canonical Obstacles

descriptors = struct([]);
if isfield(request, "obstacles") && ~isempty(request.obstacles)
    descriptors = request.obstacles;
end
if ~isstruct(descriptors)
    error("webSandbox:createRequestInputs:InvalidObstacles", ...
        "obstacles must be an array of JSON obstacle objects.");
end
obstacleCells = cell(numel(descriptors), 1);
for obstacleIndex = 1:numel(descriptors)
    descriptor = descriptors(obstacleIndex);
    if ~isfield(descriptor, "slices") || ~isstruct(descriptor.slices) || ...
            isempty(descriptor.slices)
        error("webSandbox:createRequestInputs:InvalidObstacleSlices", ...
            "Obstacle %d requires a nonempty slices array.", obstacleIndex);
    end
    sliceCount = numel(descriptor.slices);
    if isfield(descriptor, "time_s") && ~isempty(descriptor.time_s)
        time_s = double(descriptor.time_s(:));
    else
        time_s = linspace(0, 1, sliceCount).';
    end
    if numel(time_s) ~= sliceCount
        error("webSandbox:createRequestInputs:SliceTimeMismatch", ...
            "Obstacle %d time_s must match its slice count.", obstacleIndex);
    end
    azimuthBySlice_deg = cell(sliceCount, 1);
    elevationBySlice_deg = cell(sliceCount, 1);
    for sliceIndex = 1:sliceCount
        slice = descriptor.slices(sliceIndex);
        if ~isfield(slice, "vertices_deg")
            error("webSandbox:createRequestInputs:MissingVertices", ...
                "Obstacle %d slice %d requires vertices_deg.", ...
                obstacleIndex, sliceIndex);
        end
        vertices_deg = double(slice.vertices_deg);
        validateattributes(vertices_deg, {'numeric'}, ...
            {'real', 'finite', '2d', 'ncols', 2});
        if size(vertices_deg, 1) < 3
            error("webSandbox:createRequestInputs:TooFewVertices", ...
                "Obstacle %d slice %d needs at least three vertices.", ...
                obstacleIndex, sliceIndex);
        end
        azimuthBySlice_deg{sliceIndex} = vertices_deg(:, 1);
        elevationBySlice_deg{sliceIndex} = vertices_deg(:, 2);
    end
    obstacleName = "web obstacle " + obstacleIndex;
    if isfield(descriptor, "name") && strlength(string(descriptor.name)) > 0
        obstacleName = string(descriptor.name);
    end
    safetyMargin_deg = 0;
    if isfield(descriptor, "safetyMargin_deg") && ...
            ~isempty(descriptor.safetyMargin_deg)
        safetyMargin_deg = double(descriptor.safetyMargin_deg);
    end
    obstacleCells{obstacleIndex} = obstacleAvoidance.obstacles.createObstacle( ...
        obstacleName, time_s, azimuthBySlice_deg, elevationBySlice_deg, ...
        safetyMargin_deg);
end
obstacles = obstacleAvoidance.obstacles.combineObstacles(obstacleCells);

%% Section 3: Assemble The Public Planner Call

inputs = struct( ...
    "Mode", mode, "obstacles", obstacles, ...
    "initialState", request.initialState, "limits", request.limits, ...
    "goalState", struct(), "targetMotion", struct(), ...
    "plannerOptions", struct(), "interceptOptions", struct());
if mode == "trajectory"
    inputs.goalState = request.goalState;
    if isfield(request, "plannerOptions") && ~isempty(request.plannerOptions)
        inputs.plannerOptions = request.plannerOptions;
    end
else
    inputs.targetMotion = request.targetMotion;
    if isfield(request, "interceptOptions") && ...
            ~isempty(request.interceptOptions)
        inputs.interceptOptions = request.interceptOptions;
    end
end
end
