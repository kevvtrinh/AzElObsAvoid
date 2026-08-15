function [obstacle, history] = createContiguousUSObstacle( ...
        time_s, safetyMargin_deg, options)
%% Section 0: Header & Readme
% SYNTAX
%   [obstacle, history] = createContiguousUSObstacle( ...
%       time_s, safetyMargin_deg)
%   [obstacle, history] = createContiguousUSObstacle( ...
%       time_s, safetyMargin_deg, options)
%**************************************************************************
% PURPOSE
%   - Load and union the Mapping Toolbox contiguous-U.S. outline.
%   - Construct either a static outline history or the maintained moving,
%     independently deforming dense-obstacle demonstration history.
%   - Keep source loading, deformation code, validation, and safety
%     protection out of example scripts.
%**************************************************************************
% INPUTS
%   - time_s (nonempty increasing numeric vector)
%   - safetyMargin_deg (nonnegative scalar)
%   - options (scalar struct, optional)
%       .MotionMode is static or movingDeforming (default static).
%       .Verbose is logical (default false).
%**************************************************************************
% OUTPUTS
%   - obstacle (canonical protected moving obstacle)
%   - history (generic slice history plus source outline metadata)
%**************************************************************************
% UNITS
%   - Longitude/latitude are treated as azimuth/elevation degrees; time_s
%     is seconds and safetyMargin_deg is degrees.
%**************************************************************************

%% Section 1: Validate Inputs & Apply Defaults

if nargin < 3 || isempty(options)
    options = struct();
end
if ~isstruct(options) || ~isscalar(options)
    error("createContiguousUSObstacle:InvalidOptions", ...
        "options must be a scalar struct.");
end
defaultOptions = struct( ...
    "MotionMode", "static", ...
    "Verbose", false);
unknownOptionFields = setdiff( ...
    fieldnames(options), fieldnames(defaultOptions), "stable");
if ~isempty(unknownOptionFields)
    warning("createContiguousUSObstacle:UnknownOptions", ...
        "Ignoring unknown option fields: %s. No behavior changed.", ...
        strjoin(string(unknownOptionFields), ", "));
    options = rmfield(options, unknownOptionFields);
end
resolvedOptions = defaultOptions;
optionNames = fieldnames(options);
for optionIndex = 1:numel(optionNames)
    optionName = optionNames{optionIndex};
    if ~isempty(options.(optionName))
        resolvedOptions.(optionName) = options.(optionName);
    end
end
motionMode = lower(string(resolvedOptions.MotionMode));
if ~isscalar(motionMode) || ...
        ~any(motionMode == ["static" "movingdeforming"])
    error("createContiguousUSObstacle:InvalidMotionMode", ...
        "MotionMode must be static or movingDeforming.");
end
validateattributes(resolvedOptions.Verbose, ...
    {'logical','numeric'}, {'real','finite','scalar'});
if isnumeric(resolvedOptions.Verbose) && ...
        ~any(resolvedOptions.Verbose == [0 1])
    error("createContiguousUSObstacle:InvalidVerbose", ...
        "Verbose must be scalar logical or binary numeric.");
end
verbose = logical(resolvedOptions.Verbose);
resolvedOptions.Verbose = verbose;

%% Section 2: Load One Dense Exterior Boundary

boundaryFile = which("usastatehi.shp");
if isempty(boundaryFile)
    error("createContiguousUSObstacle:MappingToolboxRequired", ...
        "Mapping Toolbox file usastatehi.shp was not found.");
end
if verbose
    fprintf("[U.S. obstacle] loading and unioning mainland boundaries...\n");
end
stateBoundary = shaperead(boundaryFile, "UseGeoCoords", true);
stateName = string({stateBoundary.Name});
stateBoundary = stateBoundary(~ismember(stateName, ["Alaska" "Hawaii"]));
if isempty(stateBoundary)
    error("createContiguousUSObstacle:NoMainlandStates", ...
        "No contiguous-U.S. state boundaries were found.");
end
mainlandUS = polyshape( ...
    stateBoundary(1).Lon, stateBoundary(1).Lat, ...
    "Simplify", false, "KeepCollinearPoints", true);
for stateIndex = 2:numel(stateBoundary)
    statePolygon = polyshape( ...
        stateBoundary(stateIndex).Lon, stateBoundary(stateIndex).Lat, ...
        "Simplify", false, "KeepCollinearPoints", true);
    mainlandUS = union(mainlandUS, statePolygon);
    if verbose && (mod(stateIndex, 10) == 0 || ...
            stateIndex == numel(stateBoundary))
        fprintf("[U.S. obstacle] state union %d/%d complete.\n", ...
            stateIndex, numel(stateBoundary));
    end
end
[allLongitude_deg, allLatitude_deg] = boundary(mainlandUS);
[baseLongitude_deg, baseLatitude_deg] = largestFiniteRing( ...
    allLongitude_deg, allLatitude_deg);

%% Section 3: Delegate All Slice Work To The Generic Constructor

time_s = double(time_s(:));
missionStartTime_s = time_s(1);
missionDuration_s = time_s(end) - missionStartTime_s;
baseCenter_deg = [mean(baseLongitude_deg), mean(baseLatitude_deg)];
basePosition_deg = [baseLongitude_deg, baseLatitude_deg];
localRange_deg = max(basePosition_deg - baseCenter_deg, [], 1) - ...
    min(basePosition_deg - baseCenter_deg, [], 1);
sliceTransform = @(sourcePosition_deg, sampleTime_s, sampleIndex) ...
    transformUSSlice(sourcePosition_deg, sampleTime_s, sampleIndex, ...
    motionMode, missionStartTime_s, missionDuration_s, ...
    baseCenter_deg, localRange_deg);
[obstacle, history] = makeMovingAzElObstacleData( ...
    "Contiguous United States", time_s, ...
    baseLongitude_deg, baseLatitude_deg, sliceTransform, ...
    safetyMargin_deg, struct("Verbose", verbose));
history.motionMode = motionMode;
history.sourceFile = string(boundaryFile);
history.sourceOutlineLatLon_deg = ...
    [baseLatitude_deg, baseLongitude_deg];
history.sourceOutlineVertexCount = numel(baseLongitude_deg);
history.ExampleOptions = resolvedOptions;
end

%% Section 4: Local Functions

function transformed_deg = transformUSSlice( ...
        sourcePosition_deg, sampleTime_s, ~, motionMode, ...
        missionStartTime_s, missionDuration_s, ...
        baseCenter_deg, localRange_deg)
%% Section 0: Header & Readme
% SYNTAX
%   transformed_deg = transformUSSlice( ...
%       sourcePosition_deg, sampleTime_s, sampleIndex, motionMode, ...
%       missionStartTime_s, missionDuration_s, ...
%       baseCenter_deg, localRange_deg)
%**************************************************************************
% PURPOSE
%   - Return one U.S. slice while generic code owns iteration and execution.
%**************************************************************************
% INPUTS
%   - Source boundary, sample time/index, motion mode, duration, and bounds.
%**************************************************************************
% OUTPUTS
%   - transformed_deg (N-by-2 azimuth/elevation boundary)
%**************************************************************************
% UNITS
%   - Position is degrees and time is seconds.
%**************************************************************************
if motionMode == "static" || missionDuration_s <= 0
    transformed_deg = sourcePosition_deg;
    return;
end
phase_rad = 2 * pi * ...
    (sampleTime_s - missionStartTime_s) / missionDuration_s;
baseLocal_deg = sourcePosition_deg - baseCenter_deg;
deformedLocal_deg = baseLocal_deg;
deformedLocal_deg(:, 1) = deformedLocal_deg(:, 1) + ...
    0.04 * sin(2 * pi * baseLocal_deg(:, 2) / ...
    localRange_deg(2) + phase_rad);
deformedLocal_deg(:, 2) = deformedLocal_deg(:, 2) + ...
    0.03 * sin(2 * pi * baseLocal_deg(:, 1) / ...
    localRange_deg(1) - 0.7 * phase_rad);
scaleAzimuth = 0.998 + 0.004 * sin(phase_rad + 0.4);
scaleElevation = 0.998 + 0.004 * cos(phase_rad - 0.2);
shear = 0.003 * sin(2 * phase_rad);
rotation_rad = deg2rad(0.35 * sin(phase_rad + 0.3));
deformationMatrix = [scaleAzimuth shear; 0 scaleElevation];
rotationMatrix = [cos(rotation_rad) -sin(rotation_rad); ...
    sin(rotation_rad) cos(rotation_rad)];
translation_deg = [ ...
    0.35 * sin(phase_rad), 0.25 * cos(phase_rad + 0.5)];
transformed_deg = deformedLocal_deg * ...
    deformationMatrix.' * rotationMatrix.' + ...
    baseCenter_deg + translation_deg;
end

function [largestX, largestY] = largestFiniteRing(x, y)
%% Section 0: Header & Readme
% SYNTAX
%   [largestX, largestY] = largestFiniteRing(x, y)
%**************************************************************************
% PURPOSE
%   - Extract the largest finite boundary ring and discard holes/islands.
%**************************************************************************
% INPUTS
%   - x, y (matching numeric boundary vectors with paired separators)
%**************************************************************************
% OUTPUTS
%   - largestX, largestY (finite column vectors)
%**************************************************************************
% UNITS
%   - Coordinates use the same degree units as x and y.
%**************************************************************************
x = double(x(:));
y = double(y(:));
finiteRows = isfinite(x) & isfinite(y);
changes = diff([false; finiteRows; false]);
ringStart = find(changes == 1);
ringStop = find(changes == -1) - 1;
if isempty(ringStart)
    error("createContiguousUSObstacle:EmptyOutline", ...
        "The state union did not produce a finite exterior boundary.");
end
ringArea = zeros(numel(ringStart), 1);
for ringIndex = 1:numel(ringStart)
    rows = ringStart(ringIndex):ringStop(ringIndex);
    ringArea(ringIndex) = abs(polyarea(x(rows), y(rows)));
end
[~, largestRingIndex] = max(ringArea);
rows = ringStart(largestRingIndex):ringStop(largestRingIndex);
largestX = x(rows);
largestY = y(rows);
if largestX(1) == largestX(end) && largestY(1) == largestY(end)
    largestX(end) = [];
    largestY(end) = [];
end
end
