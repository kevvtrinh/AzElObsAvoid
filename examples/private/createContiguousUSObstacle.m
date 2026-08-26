function [obstacle, history] = createContiguousUSObstacle( time_s, safetyMargin_deg, options)
%% Section 0: Header & Readme
% SYNTAX
%   [obstacle, history] = createContiguousUSObstacle( ...
%       time_s, safetyMargin_deg)
%   [obstacle, history] = createContiguousUSObstacle( ...
%       time_s, safetyMargin_deg, options)
%**************************************************************************
% PURPOSE
%   - Load and union the Mapping Toolbox contiguous-U.S. outline.
%   - Construct either a static outline history or the maintained extreme
%     growth, deformation, translation, and half-turn history.
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
    error("createContiguousUSObstacle:InvalidOptions", "options must be a scalar struct.");
end
defaultOptions = struct( "MotionMode", "static", "Verbose", false);
[resolvedOptions, unknownOptionFields] = azElInput.resolveOptions(defaultOptions, options);
if ~isempty(unknownOptionFields)
    warning("createContiguousUSObstacle:UnknownOptions", ...
        "Ignoring unknown option fields: %s. No behavior changed.", strjoin(unknownOptionFields, ", "));
end
motionMode = lower(string(resolvedOptions.MotionMode));
if ~isscalar(motionMode) || ~any(motionMode == ["static" "movingdeforming"])
    error("createContiguousUSObstacle:InvalidMotionMode", "MotionMode must be static or movingDeforming.");
end
verbose = azElInput.normalizeLogicalScalar( ...
    resolvedOptions.Verbose, "Verbose", "createContiguousUSObstacle:InvalidVerbose");
resolvedOptions.Verbose = verbose;

%% Section 2: Load One Dense Exterior Boundary

boundaryFile = which("usastatehi.shp");
if isempty(boundaryFile)
    error("createContiguousUSObstacle:MappingToolboxRequired", "Mapping Toolbox file usastatehi.shp was not found.");
end
if verbose
    fprintf("[U.S. obstacle] loading and unioning mainland boundaries...\n");
end
stateBoundary = shaperead(boundaryFile, "UseGeoCoords", true);
stateName = string({stateBoundary.Name});
stateBoundary = stateBoundary(~ismember(stateName, ["Alaska" "Hawaii"]));
if isempty(stateBoundary)
    error("createContiguousUSObstacle:NoMainlandStates", "No contiguous-U.S. state boundaries were found.");
end
mainlandUS = polyshape( stateBoundary(1).Lon, stateBoundary(1).Lat, "Simplify", false, "KeepCollinearPoints", true);

% Union every remaining contiguous-state polygon into one mainland boundary.
for stateIndex = 2:numel(stateBoundary)
    statePolygon = polyshape( ...
        stateBoundary(stateIndex).Lon, stateBoundary(stateIndex).Lat, "Simplify", false, "KeepCollinearPoints", true);
    mainlandUS = union(mainlandUS, statePolygon);
    if verbose && (mod(stateIndex, 10) == 0 || stateIndex == numel(stateBoundary))
        fprintf("[U.S. obstacle] state union %d/%d complete.\n", stateIndex, numel(stateBoundary));
    end
end
[allLongitude_deg, allLatitude_deg] = boundary(mainlandUS);
[baseLongitude_deg, baseLatitude_deg] = largestFiniteRing( allLongitude_deg, allLatitude_deg);

%% Section 3: Delegate All Slice Work To The Generic Constructor

time_s = double(time_s(:));
missionStartTime_s = time_s(1);
missionDuration_s = time_s(end) - missionStartTime_s;
baseCenter_deg = [mean(baseLongitude_deg), mean(baseLatitude_deg)];
basePosition_deg = [baseLongitude_deg, baseLatitude_deg];
localRange_deg = max(basePosition_deg - baseCenter_deg, [], 1) - min(basePosition_deg - baseCenter_deg, [], 1);
sliceTransform = @(sourcePosition_deg, sampleTime_s, sampleIndex) ...
    transformUSSlice(sourcePosition_deg, sampleTime_s, sampleIndex, ...
    motionMode, missionStartTime_s, missionDuration_s, baseCenter_deg, localRange_deg);
[obstacle, history] = makeMovingAzElObstacleData( ...
    "Growing and rotating contiguous United States", time_s, ...
    baseLongitude_deg, baseLatitude_deg, sliceTransform, safetyMargin_deg, struct("Verbose", verbose));
profile = extremeUSProfile( ...
    time_s, missionStartTime_s, missionDuration_s);
history.motionMode = motionMode;
history.sourceFile = string(boundaryFile);
history.sourceOutlineLatLon_deg = [baseLatitude_deg, baseLongitude_deg];
history.sourceOutlineVertexCount = numel(baseLongitude_deg);
history.scaleFactor = profile.ScaleFactor(:);
history.rotation_deg = profile.Rotation_deg(:);
history.translation_deg = profile.Translation_deg;
history.deformationWeight = profile.DeformationWeight(:);
history.ExampleOptions = resolvedOptions;
end


function transformed_deg = transformUSSlice( ...
        sourcePosition_deg, sampleTime_s, ~, motionMode, ...
        missionStartTime_s, missionDuration_s, baseCenter_deg, localRange_deg)
% Return one U.S. slice while generic code owns iteration and execution.
if motionMode == "static" || missionDuration_s <= 0
    transformed_deg = sourcePosition_deg;
    return;
end
missionProgress = ...
    (sampleTime_s - missionStartTime_s) / missionDuration_s;
profile = extremeUSProfile( ...
    sampleTime_s, missionStartTime_s, missionDuration_s);
phase_rad = 2 * pi * missionProgress;
baseLocal_deg = sourcePosition_deg - baseCenter_deg;
deformedLocal_deg = baseLocal_deg;
deformedLocal_deg(:, 1) = deformedLocal_deg(:, 1) + ...
    profile.DeformationWeight * 0.80 * sin( ...
    2 * pi * baseLocal_deg(:, 2) / localRange_deg(2) + phase_rad);
deformedLocal_deg(:, 2) = deformedLocal_deg(:, 2) + ...
    profile.DeformationWeight * 0.55 * sin( ...
    2 * pi * baseLocal_deg(:, 1) / localRange_deg(1) - 0.7 * phase_rad);
rotation_rad = deg2rad(profile.Rotation_deg);
rotationMatrix = [cos(rotation_rad) -sin(rotation_rad); sin(rotation_rad) cos(rotation_rad)];
transformed_deg = profile.ScaleFactor * deformedLocal_deg * ...
    rotationMatrix.' + baseCenter_deg + profile.Translation_deg;
end

function profile = extremeUSProfile( ...
        sampleTime_s, missionStartTime_s, missionDuration_s)
% Resolve one smooth input-time profile shared by geometry and diagnostics.
if missionDuration_s <= 0
    missionProgress = zeros(size(sampleTime_s));
else
    missionProgress = (sampleTime_s - missionStartTime_s) / ...
        missionDuration_s;
end
missionProgress = min(max(missionProgress, 0), 1);
smoothProgress = 10 * missionProgress.^3 - ...
    15 * missionProgress.^4 + 6 * missionProgress.^5;
profile = struct( ...
    "ScaleFactor", 0.08 + 1.27 * smoothProgress, ...
    "Rotation_deg", 180 * smoothProgress, ...
    "Translation_deg", [ ...
        2.5 * sin(2 * pi * missionProgress(:)), ...
        1.5 * sin(pi * missionProgress(:))], ...
    "DeformationWeight", sin(pi * missionProgress));
end

function [largestX, largestY] = largestFiniteRing(x, y)
% Extract the largest finite boundary ring and discard holes/islands.
x = double(x(:));
y = double(y(:));
finiteRows = isfinite(x) & isfinite(y);
changes = diff([false; finiteRows; false]);
ringStart = find(changes == 1);
ringStop = find(changes == -1) - 1;
if isempty(ringStart)
    error("createContiguousUSObstacle:EmptyOutline", "The state union did not produce a finite exterior boundary.");
end
ringArea = zeros(numel(ringStart), 1);

% Measure every finite boundary ring so the largest mainland outline can be retained.
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
