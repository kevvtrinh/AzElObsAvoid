function animation = animateAzElTimedSlopePath( ...
        timedSlopePath, obstacleField, optionOverrides)
%% Section 0: Header & Readme
% SYNTAX
%   options = animateAzElTimedSlopePath()
%   animation = animateAzElTimedSlopePath(timedSlopePath)
%   animation = animateAzElTimedSlopePath(timedSlopePath, obstacleField)
%   animation = animateAzElTimedSlopePath( ...
%       timedSlopePath, obstacleField, optionOverrides)
%**************************************************************************
% PURPOSE
%   - Animate one retimed visibility path in synchronized 3-D az/el/time
%     and 2-D azimuth/elevation views.
%   - Show the instantaneous velocity as a spatial arrow and as a 3-D
%     slope arrow whose time component is one configured look-ahead span.
%**************************************************************************
% INPUTS
%   - timedSlopePath (scalar struct)
%       Successful retimeAzElVisibilityPath output.
%   - obstacleField (scalar struct, optional)
%       Packed AzElTimeObstacleField. Pass [] to animate only the path.
%   - optionOverrides (scalar struct, optional)
%       Playback and display overrides described by a zero-argument call.
%       TargetTime_s and TargetPosition_deg optionally add a synchronized
%       moving-target world line, trail, and current-position marker.
%**************************************************************************
% OUTPUTS
%   - animation (scalar struct)
%       Figure, axes, graphics handles, frame indices, and resolved options.
%**************************************************************************
% UNITS
%   - Azimuth/elevation are degrees, time is seconds, and velocity is deg/s.

%% Section 1: Resolve Options & Validate The Timed Path

defaultOptions = struct( ...
    "FrameStride", 10, ...
    "Pause_s", 0.001, ...
    "FigureVisible", "on", ...
    "ShowObstacles", true, ...
    "ShowSweptSurfaces", true, ...
    "MaximumDisplayedSlicesPerObstacle", 30, ...
    "ObstacleFaceAlpha", 0.10, ...
    "SweptSurfaceAlpha", 0.08, ...
    "SlopeArrowDuration_s", 1.0, ...
    "TargetTime_s", zeros(0, 1), ...
    "TargetPosition_deg", zeros(0, 2), ...
    "TargetLabel", "Moving target", ...
    "Title", "Timed slope path");
if nargin == 0
    animation = defaultOptions;
    return;
end
if nargin < 2
    obstacleField = [];
end
if nargin < 3 || isempty(optionOverrides)
    optionOverrides = struct();
end
if ~isstruct(optionOverrides) || ~isscalar(optionOverrides)
    error("animateAzElTimedSlopePath:InvalidOptions", ...
        "optionOverrides must be a scalar struct.");
end
legacyGeometryFields = intersect(fieldnames(optionOverrides), ...
    {'SafetyMarginDeg', 'OriginalObstacleField'}, "stable");
if ~isempty(legacyGeometryFields)
    error("animateAzElTimedSlopePath:ObstacleGeometryMoved", ...
        "Original and protected geometry are recovered from the packed " + ...
        "field. Remove legacy options: %s.", ...
        strjoin(string(legacyGeometryFields), ", "));
end
unknownFields = setdiff( ...
    fieldnames(optionOverrides), fieldnames(defaultOptions), "stable");
if ~isempty(unknownFields)
    warning("animateAzElTimedSlopePath:UnknownOptions", ...
        "Ignoring unknown option fields: %s.", ...
        strjoin(string(unknownFields), ", "));
    optionOverrides = rmfield(optionOverrides, unknownFields);
end
options = defaultOptions;
overrideFields = fieldnames(optionOverrides);
for fieldIndex = 1:numel(overrideFields)
    fieldName = overrideFields{fieldIndex};
    if ~isempty(optionOverrides.(fieldName))
        options.(fieldName) = optionOverrides.(fieldName);
    end
end
validateattributes(options.FrameStride, {'numeric'}, ...
    {'scalar', 'integer', 'positive'});
validateattributes(options.Pause_s, {'numeric'}, ...
    {'scalar', 'real', 'finite', 'nonnegative'});
validateattributes(options.MaximumDisplayedSlicesPerObstacle, {'numeric'}, ...
    {'scalar', 'integer', 'positive'});
validateattributes(options.ObstacleFaceAlpha, {'numeric'}, ...
    {'scalar', 'real', 'finite', '>=', 0, '<=', 1});
validateattributes(options.SweptSurfaceAlpha, {'numeric'}, ...
    {'scalar', 'real', 'finite', '>=', 0, '<=', 1});
validateattributes(options.SlopeArrowDuration_s, {'numeric'}, ...
    {'scalar', 'real', 'finite', 'positive'});
options.ShowObstacles = logicalScalar( ...
    options.ShowObstacles, "ShowObstacles");
options.ShowSweptSurfaces = logicalScalar( ...
    options.ShowSweptSurfaces, "ShowSweptSurfaces");
options.FigureVisible = lower(string(options.FigureVisible));
if ~isscalar(options.FigureVisible) || ...
        ~any(options.FigureVisible == ["on", "off"])
    error("animateAzElTimedSlopePath:InvalidFigureVisible", ...
        "FigureVisible must be on or off.");
end
options.Title = string(options.Title);
if ~isscalar(options.Title)
    error("animateAzElTimedSlopePath:InvalidTitle", ...
        "Title must be scalar text.");
end
options.TargetLabel = string(options.TargetLabel);
if ~isscalar(options.TargetLabel)
    error("animateAzElTimedSlopePath:InvalidTargetLabel", ...
        "TargetLabel must be scalar text.");
end
targetTime_s = double(options.TargetTime_s(:));
targetPosition_deg = double(options.TargetPosition_deg);
hasTarget = ~isempty(targetTime_s) || ~isempty(targetPosition_deg);
if hasTarget
    if numel(targetTime_s) < 2
        error("animateAzElTimedSlopePath:InvalidTargetTime", ...
            "TargetTime_s must contain at least two samples.");
    end
    validateattributes(targetTime_s, {'numeric'}, ...
        {'vector', 'real', 'finite', 'increasing'});
    validateattributes(targetPosition_deg, {'numeric'}, ...
        {'2d', 'ncols', 2, 'real', 'finite'});
    if size(targetPosition_deg, 1) ~= numel(targetTime_s)
        error("animateAzElTimedSlopePath:TargetSizeMismatch", ...
            "TargetTime_s and TargetPosition_deg must have the same row count.");
    end
else
    targetTime_s = zeros(0, 1);
    targetPosition_deg = zeros(0, 2);
end
options.TargetTime_s = targetTime_s;
options.TargetPosition_deg = targetPosition_deg;
requiredPathFields = ["time_s", "position_deg", "velocity_deg_s"];
if ~isstruct(timedSlopePath) || ~isscalar(timedSlopePath) || ...
        ~all(isfield(timedSlopePath, requiredPathFields)) || ...
        (isfield(timedSlopePath, "Success") && ~timedSlopePath.Success)
    error("animateAzElTimedSlopePath:InvalidTimedSlopePath", ...
        "timedSlopePath must be a successful retimed slope path.");
end
time_s = double(timedSlopePath.time_s(:));
position_deg = double(timedSlopePath.position_deg);
velocity_deg_s = double(timedSlopePath.velocity_deg_s);
validateattributes(time_s, {'numeric'}, ...
    {'vector', 'real', 'finite'});
if any(diff(time_s) <= 0)
    error("animateAzElTimedSlopePath:NonIncreasingTime", ...
        "timedSlopePath.time_s must be strictly increasing.");
end
validateattributes(position_deg, {'numeric'}, ...
    {'2d', 'ncols', 2, 'real', 'finite'});
validateattributes(velocity_deg_s, {'numeric'}, ...
    {'2d', 'ncols', 2, 'real', 'finite'});
if size(position_deg, 1) ~= numel(time_s) || ...
        size(velocity_deg_s, 1) ~= numel(time_s)
    error("animateAzElTimedSlopePath:PathSizeMismatch", ...
        "Time, position, and velocity must have the same row count.");
end
hasObstacleField = ~isempty(obstacleField);
if hasObstacleField && (~isstruct(obstacleField) || ...
        ~isscalar(obstacleField) || ...
        ~isfield(obstacleField, "Obstacles"))
    error("animateAzElTimedSlopePath:InvalidObstacleField", ...
        "obstacleField must be a packed AzElTimeObstacleField or [].");
end
originalObstacleField = obstacleField;
obstacleSafetyMargins_deg = zeros(0, 1);
if hasObstacleField
    [storedOriginalField, obstacleSafetyMargins_deg, hasStoredOriginal] = ...
        recoverOriginalAzElObstacleField(obstacleField);
    if hasStoredOriginal
        originalObstacleField = storedOriginalField;
    end
end
if hasObstacleField && numel(originalObstacleField.Obstacles) ~= ...
        numel(obstacleField.Obstacles)
    error("animateAzElTimedSlopePath:OriginalFieldSizeMismatch", ...
        "OriginalObstacleField must contain the same obstacle count.");
end
options.OriginalObstacleField = originalObstacleField;
options.ObstacleSafetyMargins_deg = obstacleSafetyMargins_deg;
displayTitle = options.Title;
if any(obstacleSafetyMargins_deg > 0)
    displayTitle = displayTitle + sprintf( ...
        " | protected margins = %s deg", ...
        char(mat2str(obstacleSafetyMargins_deg.', 3)));
end

%% Section 2: Create Equal-Scale 3-D & 2-D Views

[allAzimuth_deg, allElevation_deg, allTime_s] = ...
    collectDisplayCoordinates( ...
    position_deg, time_s, obstacleField, hasObstacleField, options);
cubeLimits = equalCubeLimits( ...
    allAzimuth_deg, allElevation_deg, allTime_s);
planeLimits = equalPlaneLimits(allAzimuth_deg, allElevation_deg);

figureHandle = figure( ...
    "Name", char(displayTitle), ...
    "Color", "w", ...
    "Visible", options.FigureVisible);
axes3D = subplot(1, 2, 1, "Parent", figureHandle);
hold(axes3D, "on");
grid(axes3D, "on");
box(axes3D, "on");
xlabel(axes3D, "Azimuth (deg)");
ylabel(axes3D, "Elevation (deg)");
zlabel(axes3D, "Time (s)");
title(axes3D, displayTitle + " - az/el/time");
view(axes3D, 42, 25);
xlim(axes3D, cubeLimits(1, :));
ylim(axes3D, cubeLimits(2, :));
zlim(axes3D, cubeLimits(3, :));
daspect(axes3D, [1 1 1]);
pbaspect(axes3D, [1 1 1]);
axis(axes3D, "vis3d");

axes2D = subplot(1, 2, 2, "Parent", figureHandle);
hold(axes2D, "on");
grid(axes2D, "on");
box(axes2D, "on");
xlabel(axes2D, "Azimuth (deg)");
ylabel(axes2D, "Elevation (deg)");
title(axes2D, "Current azimuth/elevation");
xlim(axes2D, planeLimits(1, :));
ylim(axes2D, planeLimits(2, :));
axis(axes2D, "equal");
pbaspect(axes2D, [1 1 1]);

obstacleColors = lines(max(1, obstacleCount(obstacleField, hasObstacleField)));
obstacle3DHandles = gobjects(0, 1);
if hasObstacleField && options.ShowObstacles
    obstacle3DHandles = drawObstacleTimeSpace( ...
        axes3D, obstacleField, obstacleColors, options);
end

% The full route is a faint reference. Bright trails show the portion that
% has actually elapsed in both synchronized views.
plot3(axes3D, position_deg(:, 1), position_deg(:, 2), time_s, ...
    "-", "Color", [0.72 0.78 0.84], "LineWidth", 1.2, ...
    "DisplayName", "Complete timed path");
plot(axes2D, position_deg(:, 1), position_deg(:, 2), ...
    "-", "Color", [0.72 0.78 0.84], "LineWidth", 1.2, ...
    "DisplayName", "Complete az/el path");
targetWorldLine3D = gobjects(0, 1);
targetWorldLine2D = gobjects(0, 1);
targetTrail3D = gobjects(0, 1);
targetTrail2D = gobjects(0, 1);
targetCurrent3D = gobjects(0, 1);
targetCurrent2D = gobjects(0, 1);
targetAtPathTime_deg = zeros(0, 2);
if hasTarget
    targetAtPathTime_deg = interp1( ...
        targetTime_s, targetPosition_deg, time_s, "linear", "extrap");
    targetWorldLine3D = plot3(axes3D, ...
        targetPosition_deg(:, 1), targetPosition_deg(:, 2), targetTime_s, ...
        "-.", "Color", [0.78 0.55 0.78], "LineWidth", 1.3, ...
        "DisplayName", options.TargetLabel + " world line");
    targetWorldLine2D = plot(axes2D, ...
        targetPosition_deg(:, 1), targetPosition_deg(:, 2), "-.", ...
        "Color", [0.78 0.55 0.78], "LineWidth", 1.3, ...
        "DisplayName", options.TargetLabel + " track");
end
goalPosition_deg = position_deg(end, :);
goalLineHandle = plot3(axes3D, ...
    [goalPosition_deg(1) goalPosition_deg(1)], ...
    [goalPosition_deg(2) goalPosition_deg(2)], ...
    cubeLimits(3, :), "--", "Color", [0.20 0.65 0.20], ...
    "LineWidth", 1.7, "DisplayName", "Vertical goal line");
plot(axes2D, goalPosition_deg(1), goalPosition_deg(2), "p", ...
    "MarkerSize", 12, "MarkerFaceColor", [0.20 0.65 0.20], ...
    "MarkerEdgeColor", [0.10 0.35 0.10], "DisplayName", "Goal");
plot3(axes3D, position_deg(1, 1), position_deg(1, 2), time_s(1), ...
    "o", "MarkerSize", 8, "MarkerFaceColor", [0.15 0.35 0.85], ...
    "MarkerEdgeColor", "none", "DisplayName", "Start");
plot(axes2D, position_deg(1, 1), position_deg(1, 2), "o", ...
    "MarkerSize", 8, "MarkerFaceColor", [0.15 0.35 0.85], ...
    "MarkerEdgeColor", "none", "DisplayName", "Start");

trail3D = plot3(axes3D, NaN, NaN, NaN, "-", ...
    "Color", [0.05 0.70 0.82], "LineWidth", 3.5, ...
    "DisplayName", "Elapsed path");
trail2D = plot(axes2D, NaN, NaN, "-", ...
    "Color", [0.05 0.70 0.82], "LineWidth", 3.5, ...
    "DisplayName", "Elapsed path");
current3D = plot3(axes3D, NaN, NaN, NaN, "o", ...
    "MarkerSize", 9, "MarkerFaceColor", [0.95 0.25 0.15], ...
    "MarkerEdgeColor", "w", "LineWidth", 1.0, ...
    "DisplayName", "Current state");
current2D = plot(axes2D, NaN, NaN, "o", ...
    "MarkerSize", 9, "MarkerFaceColor", [0.95 0.25 0.15], ...
    "MarkerEdgeColor", "w", "LineWidth", 1.0, ...
    "DisplayName", "Current state");
slopeArrow3D = quiver3(axes3D, NaN, NaN, NaN, NaN, NaN, NaN, 0, ...
    "Color", [0.95 0.25 0.15], "LineWidth", 2.0, ...
    "MaxHeadSize", 0.8, "DisplayName", "Timed slope");
velocityArrow2D = quiver(axes2D, NaN, NaN, NaN, NaN, 0, ...
    "Color", [0.95 0.25 0.15], "LineWidth", 2.0, ...
    "MaxHeadSize", 0.8, "DisplayName", "Velocity");
if hasTarget
    targetTrail3D = plot3(axes3D, NaN, NaN, NaN, "-", ...
        "Color", [0.72 0.10 0.72], "LineWidth", 2.8, ...
        "DisplayName", options.TargetLabel + " elapsed");
    targetTrail2D = plot(axes2D, NaN, NaN, "-", ...
        "Color", [0.72 0.10 0.72], "LineWidth", 2.8, ...
        "DisplayName", options.TargetLabel + " elapsed");
    targetCurrent3D = plot3(axes3D, NaN, NaN, NaN, "p", ...
        "MarkerSize", 12, "MarkerFaceColor", [0.95 0.30 0.82], ...
        "MarkerEdgeColor", "k", "LineWidth", 1.0, ...
        "DisplayName", options.TargetLabel);
    targetCurrent2D = plot(axes2D, NaN, NaN, "p", ...
        "MarkerSize", 12, "MarkerFaceColor", [0.95 0.30 0.82], ...
        "MarkerEdgeColor", "k", "LineWidth", 1.0, ...
        "DisplayName", options.TargetLabel);
end
legendHandles3D = [trail3D current3D slopeArrow3D goalLineHandle];
legendHandles2D = [trail2D current2D velocityArrow2D];
if hasTarget
    legend(axes3D, [legendHandles3D targetCurrent3D], ...
        "Location", "best");
    legend(axes2D, [legendHandles2D targetCurrent2D], ...
        "Location", "best");
else
    legend(axes3D, legendHandles3D, "Location", "best");
    legend(axes2D, legendHandles2D, "Location", "best");
end

%% Section 3: Animate The Shared Timeline

frameIndices = unique([ ...
    (1:options.FrameStride:numel(time_s)).'; numel(time_s)]);
obstacle2DHandles = gobjects(0, 1);
for frameCursor = 1:numel(frameIndices)
    frameIndex = frameIndices(frameCursor);
    currentTime_s = time_s(frameIndex);
    currentPosition_deg = position_deg(frameIndex, :);
    currentVelocity_deg_s = velocity_deg_s(frameIndex, :);
    set(trail3D, ...
        "XData", position_deg(1:frameIndex, 1), ...
        "YData", position_deg(1:frameIndex, 2), ...
        "ZData", time_s(1:frameIndex));
    set(trail2D, ...
        "XData", position_deg(1:frameIndex, 1), ...
        "YData", position_deg(1:frameIndex, 2));
    set(current3D, ...
        "XData", currentPosition_deg(1), ...
        "YData", currentPosition_deg(2), ...
        "ZData", currentTime_s);
    set(current2D, ...
        "XData", currentPosition_deg(1), ...
        "YData", currentPosition_deg(2));
    arrowDuration_s = options.SlopeArrowDuration_s;
    set(slopeArrow3D, ...
        "XData", currentPosition_deg(1), ...
        "YData", currentPosition_deg(2), ...
        "ZData", currentTime_s, ...
        "UData", currentVelocity_deg_s(1) * arrowDuration_s, ...
        "VData", currentVelocity_deg_s(2) * arrowDuration_s, ...
        "WData", arrowDuration_s);
    set(velocityArrow2D, ...
        "XData", currentPosition_deg(1), ...
        "YData", currentPosition_deg(2), ...
        "UData", currentVelocity_deg_s(1) * arrowDuration_s, ...
        "VData", currentVelocity_deg_s(2) * arrowDuration_s);
    if hasTarget
        currentTargetPosition_deg = targetAtPathTime_deg(frameIndex, :);
        set(targetTrail3D, ...
            "XData", targetAtPathTime_deg(1:frameIndex, 1), ...
            "YData", targetAtPathTime_deg(1:frameIndex, 2), ...
            "ZData", time_s(1:frameIndex));
        set(targetTrail2D, ...
            "XData", targetAtPathTime_deg(1:frameIndex, 1), ...
            "YData", targetAtPathTime_deg(1:frameIndex, 2));
        set(targetCurrent3D, ...
            "XData", currentTargetPosition_deg(1), ...
            "YData", currentTargetPosition_deg(2), ...
            "ZData", currentTime_s);
        set(targetCurrent2D, ...
            "XData", currentTargetPosition_deg(1), ...
            "YData", currentTargetPosition_deg(2));
    end

    if hasObstacleField && options.ShowObstacles
        deleteValidGraphics(obstacle2DHandles);
        obstacle2DHandles = drawObstacleSnapshot( ...
            axes2D, obstacleField, currentTime_s, obstacleColors, options);
        foregroundHandles = [trail2D current2D velocityArrow2D];
        if hasTarget
            foregroundHandles = [foregroundHandles ...
                targetTrail2D targetCurrent2D]; %#ok<AGROW>
        end
        uistack(foregroundHandles, "top");
    end
    title(axes2D, sprintf( ...
        "t = %.2f s, velocity = [%.2f  %.2f] deg/s", ...
        currentTime_s, currentVelocity_deg_s(1), ...
        currentVelocity_deg_s(2)));
    drawnow;
    if options.Pause_s > 0
        pause(options.Pause_s);
    end
end

animation = struct( ...
    "Figure", figureHandle, ...
    "Axes3D", axes3D, ...
    "Axes2D", axes2D, ...
    "Trail3D", trail3D, ...
    "Trail2D", trail2D, ...
    "Current3D", current3D, ...
    "Current2D", current2D, ...
    "SlopeArrow3D", slopeArrow3D, ...
    "VelocityArrow2D", velocityArrow2D, ...
    "GoalLine", goalLineHandle, ...
    "TargetWorldLine3D", targetWorldLine3D, ...
    "TargetWorldLine2D", targetWorldLine2D, ...
    "TargetTrail3D", targetTrail3D, ...
    "TargetTrail2D", targetTrail2D, ...
    "TargetCurrent3D", targetCurrent3D, ...
    "TargetCurrent2D", targetCurrent2D, ...
    "Obstacle3D", obstacle3DHandles, ...
    "Obstacle2D", obstacle2DHandles, ...
    "OriginalBoundary3D", findall(axes3D, ...
        "Tag", "AzElOriginalBoundary"), ...
    "ProtectedBoundary3D", findall(axes3D, ...
        "Tag", "AzElProtectedBoundary"), ...
    "OriginalBoundary2D", findall(axes2D, ...
        "Tag", "AzElOriginalBoundary"), ...
    "ProtectedBoundary2D", findall(axes2D, ...
        "Tag", "AzElProtectedBoundary"), ...
    "FrameIndices", frameIndices, ...
    "Options", options);
end

%% Section 4: Local Functions

function handles = drawObstacleTimeSpace( ...
        axesHandle, obstacleField, colors, options)
%% Section 0: Header & Readme
% SYNTAX
%   handles = drawObstacleTimeSpace( ...
%       axesHandle, obstacleField, colors, options)
%**************************************************************************
% PURPOSE
%   - Draw selected obstacle slices and compatible swept surfaces.
%**************************************************************************
% INPUTS
%   - axesHandle (scalar axes handle)
%   - obstacleField (scalar packed obstacle field)
%   - colors (N-by-3 numeric matrix)
%   - options (resolved scalar animation-options struct)
%**************************************************************************
% OUTPUTS
%   - handles (M-by-1 graphics-handle array)
%**************************************************************************
% UNITS
%   - Azimuth/elevation are degrees and time is seconds.
%**************************************************************************
handles = gobjects(0, 1);
for obstacleIndex = 1:numel(obstacleField.Obstacles)
    obstacle = obstacleField.Obstacles(obstacleIndex);
    obstacleMargin_deg = ...
        options.ObstacleSafetyMargins_deg(obstacleIndex);
    if obstacle.SampleCount == 0
        continue;
    end
    displayedSlices = unique(round(linspace(1, obstacle.SampleCount, ...
        min(obstacle.SampleCount, ...
        options.MaximumDisplayedSlicesPerObstacle))));
    previousRegions = cell(0, 1);
    previousTime_s = NaN;
    for displayedIndex = 1:numel(displayedSlices)
        sampleIndex = displayedSlices(displayedIndex);
        sampleTime_s = double(obstacle.TimeSeconds(sampleIndex));
        safetyRegions = unpackSliceRegions(obstacle, sampleIndex);
        originalObstacle = ...
            options.OriginalObstacleField.Obstacles(obstacleIndex);
        [~, originalSampleIndex] = min(abs( ...
            double(originalObstacle.TimeSeconds(:)) - sampleTime_s));
        rawRegions = unpackSliceRegions( ...
            originalObstacle, originalSampleIndex);
        if obstacleMargin_deg > 0
            for safetyRegionIndex = 1:numel(safetyRegions)
                safetyRegion_deg = safetyRegions{safetyRegionIndex};
                closedSafetyRegion_deg = [ ...
                    safetyRegion_deg; safetyRegion_deg(1, :)];
                handles(end + 1, 1) = plot3( ...
                    axesHandle, closedSafetyRegion_deg(:, 1), ...
                    closedSafetyRegion_deg(:, 2), repmat(sampleTime_s, ...
                    size(closedSafetyRegion_deg, 1), 1), "--", ...
                    "Color", [0.92 0.18 0.08], ...
                    "LineWidth", 2.0, "HandleVisibility", "off", ...
                    "Tag", "AzElProtectedBoundary", ...
                    "UserData", obstacleIndex); %#ok<AGROW>
            end
        end
        regions = rawRegions;
        for regionIndex = 1:numel(regions)
            region_deg = regions{regionIndex};
            inflatedEdgeColor = colors(obstacleIndex, :);
            inflatedLineWidth = 0.8;
            handles(end + 1, 1) = patch(axesHandle, ...
                region_deg(:, 1), region_deg(:, 2), ...
                repmat(sampleTime_s, size(region_deg, 1), 1), ...
                colors(obstacleIndex, :), ...
                "FaceAlpha", options.ObstacleFaceAlpha, ...
                "EdgeColor", inflatedEdgeColor, ...
                "LineStyle", "-", ...
                "LineWidth", inflatedLineWidth, ...
                "HandleVisibility", "off", ...
                "Tag", "AzElOriginalBoundary", ...
                "UserData", obstacleIndex); %#ok<AGROW>
        end
        if options.ShowSweptSurfaces && ...
                numel(regions) == numel(previousRegions)
            for regionIndex = 1:numel(regions)
                currentRegion_deg = regions{regionIndex};
                previousRegion_deg = previousRegions{regionIndex};
                if size(currentRegion_deg, 1) ~= ...
                        size(previousRegion_deg, 1)
                    continue;
                end
                closedCurrent_deg = [currentRegion_deg; currentRegion_deg(1, :)];
                closedPrevious_deg = [previousRegion_deg; previousRegion_deg(1, :)];
                handles(end + 1, 1) = surf(axesHandle, ...
                    [closedPrevious_deg(:, 1), closedCurrent_deg(:, 1)], ...
                    [closedPrevious_deg(:, 2), closedCurrent_deg(:, 2)], ...
                    [repmat(previousTime_s, size(closedPrevious_deg, 1), 1), ...
                    repmat(sampleTime_s, size(closedCurrent_deg, 1), 1)], ...
                    "FaceColor", colors(obstacleIndex, :), ...
                    "FaceAlpha", options.SweptSurfaceAlpha, ...
                    "EdgeColor", "none", ...
                    "HandleVisibility", "off"); %#ok<AGROW>
            end
        end
        previousRegions = regions;
        previousTime_s = sampleTime_s;
    end
end
end

function handles = drawObstacleSnapshot( ...
        axesHandle, obstacleField, currentTime_s, colors, options)
%% Section 0: Header & Readme
% SYNTAX
%   handles = drawObstacleSnapshot(axesHandle, obstacleField, ...
%       currentTime_s, colors, options)
%**************************************************************************
% PURPOSE
%   - Draw the obstacle slice nearest to the current animation time.
%**************************************************************************
% INPUTS
%   - axesHandle (scalar axes handle)
%   - obstacleField (scalar packed obstacle field)
%   - currentTime_s (finite numeric scalar)
%   - colors (N-by-3 numeric matrix)
%   - options (resolved scalar animation-options struct)
%**************************************************************************
% OUTPUTS
%   - handles (M-by-1 graphics-handle array)
%**************************************************************************
% UNITS
%   - Azimuth/elevation are degrees and time is seconds.
%**************************************************************************
handles = gobjects(0, 1);
for obstacleIndex = 1:numel(obstacleField.Obstacles)
    obstacle = obstacleField.Obstacles(obstacleIndex);
    obstacleMargin_deg = ...
        options.ObstacleSafetyMargins_deg(obstacleIndex);
    if obstacle.SampleCount == 0
        continue;
    end
    [~, sampleIndex] = min(abs( ...
        double(obstacle.TimeSeconds(:)) - currentTime_s));
    safetyRegions = unpackSliceRegions(obstacle, sampleIndex);
    originalObstacle = ...
        options.OriginalObstacleField.Obstacles(obstacleIndex);
    [~, originalSampleIndex] = min(abs( ...
        double(originalObstacle.TimeSeconds(:)) - currentTime_s));
    rawRegions = unpackSliceRegions(originalObstacle, originalSampleIndex);
    if obstacleMargin_deg > 0
        for safetyRegionIndex = 1:numel(safetyRegions)
            safetyRegion_deg = safetyRegions{safetyRegionIndex};
            closedSafetyRegion_deg = [ ...
                safetyRegion_deg; safetyRegion_deg(1, :)];
            handles(end + 1, 1) = plot( ...
                axesHandle, closedSafetyRegion_deg(:, 1), ...
                closedSafetyRegion_deg(:, 2), "--", ...
                "Color", [0.92 0.18 0.08], ...
                "LineWidth", 2.2, "HandleVisibility", "off", ...
                "Tag", "AzElProtectedBoundary", ...
                "UserData", obstacleIndex); %#ok<AGROW>
        end
    end
    regions = rawRegions;
    for regionIndex = 1:numel(regions)
        region_deg = regions{regionIndex};
        inflatedEdgeColor = colors(obstacleIndex, :);
        inflatedLineWidth = 1.2;
        handles(end + 1, 1) = patch(axesHandle, ...
            region_deg(:, 1), region_deg(:, 2), ...
            colors(obstacleIndex, :), ...
            "FaceAlpha", max(0.18, options.ObstacleFaceAlpha), ...
            "EdgeColor", inflatedEdgeColor, ...
            "LineStyle", "-", ...
            "LineWidth", inflatedLineWidth, ...
            "HandleVisibility", "off", ...
            "Tag", "AzElOriginalBoundary", ...
            "UserData", obstacleIndex); %#ok<AGROW>
    end
end
end

function regions = unpackSliceRegions(obstacle, sampleIndex)
%% Section 0: Header & Readme
% SYNTAX
%   regions = unpackSliceRegions(obstacle, sampleIndex)
%**************************************************************************
% PURPOSE
%   - Recover finite polygon rings from one packed obstacle slice.
%**************************************************************************
% INPUTS
%   - obstacle (scalar packed-obstacle struct)
%   - sampleIndex (positive integer scalar)
%**************************************************************************
% OUTPUTS
%   - regions (N-by-1 cell array)
%       Each cell contains one M-by-2 [azimuth elevation] ring.
%**************************************************************************
% UNITS
%   - Polygon coordinates are degrees.
%**************************************************************************
firstVertex = double(obstacle.SliceOffsets(sampleIndex));
finalVertex = double(obstacle.SliceOffsets(sampleIndex + 1) - 1);
if finalVertex < firstVertex
    regions = cell(0, 1);
    return;
end
azimuth_deg = double(obstacle.AzimuthDeg(firstVertex:finalVertex));
elevation_deg = double(obstacle.ElevationDeg(firstVertex:finalVertex));
finiteVertex = isfinite(azimuth_deg) & isfinite(elevation_deg);
regionChanges = diff([false; finiteVertex; false]);
regionStarts = find(regionChanges == 1);
regionStops = find(regionChanges == -1) - 1;
regions = cell(numel(regionStarts), 1);
for regionIndex = 1:numel(regionStarts)
    rows = regionStarts(regionIndex):regionStops(regionIndex);
    regions{regionIndex} = [azimuth_deg(rows), elevation_deg(rows)];
end
end

function [azimuth_deg, elevation_deg, time_s] = ...
        collectDisplayCoordinates( ...
        pathPosition_deg, pathTime_s, obstacleField, hasObstacleField, options)
%% Section 0: Header & Readme
% SYNTAX
%   [azimuth_deg, elevation_deg, time_s] = ...
%       collectDisplayCoordinates(pathPosition_deg, pathTime_s, ...
%       obstacleField, hasObstacleField, options)
%**************************************************************************
% PURPOSE
%   - Collect all coordinates needed for stable shared display limits.
%**************************************************************************
% INPUTS
%   - pathPosition_deg (N-by-2 numeric matrix)
%   - pathTime_s (N-by-1 numeric vector)
%   - obstacleField (scalar packed obstacle field or empty)
%   - hasObstacleField (logical scalar)
%   - options (resolved scalar animation-options struct)
%**************************************************************************
% OUTPUTS
%   - azimuth_deg, elevation_deg, time_s (numeric column vectors)
%**************************************************************************
% UNITS
%   - Azimuth/elevation are degrees and time is seconds.
%**************************************************************************
azimuth_deg = pathPosition_deg(:, 1);
elevation_deg = pathPosition_deg(:, 2);
time_s = pathTime_s;
if ~isempty(options.TargetTime_s)
    azimuth_deg = [azimuth_deg; options.TargetPosition_deg(:, 1)];
    elevation_deg = [elevation_deg; options.TargetPosition_deg(:, 2)];
    time_s = [time_s; options.TargetTime_s];
end
if ~hasObstacleField
    return;
end
for obstacle = reshape(obstacleField.Obstacles, 1, [])
    finiteVertex = isfinite(obstacle.AzimuthDeg) & ...
        isfinite(obstacle.ElevationDeg);
    rawAzimuth_deg = double(obstacle.AzimuthDeg(finiteVertex));
    rawElevation_deg = double(obstacle.ElevationDeg(finiteVertex));
    azimuth_deg = [azimuth_deg; rawAzimuth_deg]; %#ok<AGROW>
    elevation_deg = [elevation_deg; rawElevation_deg]; %#ok<AGROW>
    time_s = [time_s; double(obstacle.TimeSeconds(:))]; %#ok<AGROW>
end
if isfield(options.OriginalObstacleField, "Obstacles")
    for obstacle = reshape( ...
            options.OriginalObstacleField.Obstacles, 1, [])
        finiteVertex = isfinite(obstacle.AzimuthDeg) & ...
            isfinite(obstacle.ElevationDeg);
        azimuth_deg = [azimuth_deg; ...
            double(obstacle.AzimuthDeg(finiteVertex))]; %#ok<AGROW>
        elevation_deg = [elevation_deg; ...
            double(obstacle.ElevationDeg(finiteVertex))]; %#ok<AGROW>
        time_s = [time_s; ...
            double(obstacle.TimeSeconds(:))]; %#ok<AGROW>
    end
end
end

function limits = equalCubeLimits(azimuth_deg, elevation_deg, time_s)
%% Section 0: Header & Readme
% SYNTAX
%   limits = equalCubeLimits(azimuth_deg, elevation_deg, time_s)
%**************************************************************************
% PURPOSE
%   - Give three display axes one shared numeric span.
%**************************************************************************
% INPUTS
%   - azimuth_deg, elevation_deg (numeric vectors)
%   - time_s (numeric vector)
%**************************************************************************
% OUTPUTS
%   - limits (3-by-2 numeric matrix)
%       Minimum and maximum limits for azimuth, elevation, and time.
%**************************************************************************
% UNITS
%   - Rows use degrees, degrees, and seconds respectively.
%**************************************************************************
minimumValues = [min(azimuth_deg), min(elevation_deg), min(time_s)];
maximumValues = [max(azimuth_deg), max(elevation_deg), max(time_s)];
centerValues = (minimumValues + maximumValues) / 2;
sharedSpan = 1.10 * max(maximumValues - minimumValues);
if ~isfinite(sharedSpan) || sharedSpan <= 1e-9
    sharedSpan = 1;
end
limits = centerValues(:) + 0.5 * sharedSpan * [-1 1];
end

function limits = equalPlaneLimits(azimuth_deg, elevation_deg)
%% Section 0: Header & Readme
% SYNTAX
%   limits = equalPlaneLimits(azimuth_deg, elevation_deg)
%**************************************************************************
% PURPOSE
%   - Use one az/el span so circular geometry remains circular.
%**************************************************************************
% INPUTS
%   - azimuth_deg, elevation_deg (numeric vectors)
%**************************************************************************
% OUTPUTS
%   - limits (2-by-2 numeric matrix)
%       Minimum and maximum limits for azimuth and elevation.
%**************************************************************************
% UNITS
%   - Limits are degrees.
%**************************************************************************
minimumValues = [min(azimuth_deg), min(elevation_deg)];
maximumValues = [max(azimuth_deg), max(elevation_deg)];
centerValues = (minimumValues + maximumValues) / 2;
sharedSpan = 1.10 * max(maximumValues - minimumValues);
if ~isfinite(sharedSpan) || sharedSpan <= 1e-9
    sharedSpan = 1;
end
limits = centerValues(:) + 0.5 * sharedSpan * [-1 1];
end

function count = obstacleCount(obstacleField, hasObstacleField)
%% Section 0: Header & Readme
% SYNTAX
%   count = obstacleCount(obstacleField, hasObstacleField)
%**************************************************************************
% PURPOSE
%   - Count obstacles without dereferencing an omitted field.
%**************************************************************************
% INPUTS
%   - obstacleField (scalar packed obstacle field or empty)
%   - hasObstacleField (logical scalar)
%**************************************************************************
% OUTPUTS
%   - count (nonnegative integer scalar)
%**************************************************************************
% UNITS
%   - Count is dimensionless.
%**************************************************************************
if hasObstacleField
    count = numel(obstacleField.Obstacles);
else
    count = 0;
end
end

function value = logicalScalar(value, fieldName)
%% Section 0: Header & Readme
% SYNTAX
%   value = logicalScalar(value, fieldName)
%**************************************************************************
% PURPOSE
%   - Normalize one scalar logical animation option.
%**************************************************************************
% INPUTS
%   - value (scalar logical or numeric value)
%   - fieldName (scalar text)
%       Option name used in diagnostics.
%**************************************************************************
% OUTPUTS
%   - value (logical scalar)
%**************************************************************************
% UNITS
%   - Values are dimensionless.
%**************************************************************************
validateattributes(value, {'logical', 'numeric'}, {'scalar'});
value = logical(value);
if ~isscalar(value)
    error("animateAzElTimedSlopePath:InvalidLogicalOption", ...
        "%s must be scalar.", fieldName);
end
end

function deleteValidGraphics(handles)
%% Section 0: Header & Readme
% SYNTAX
%   deleteValidGraphics(handles)
%**************************************************************************
% PURPOSE
%   - Delete live graphics handles from the previous 2-D snapshot.
%**************************************************************************
% INPUTS
%   - handles (graphics-handle array)
%**************************************************************************
% OUTPUTS
%   - None.
%**************************************************************************
% UNITS
%   - Not applicable.
%**************************************************************************
if isempty(handles)
    return;
end
delete(handles(isgraphics(handles)));
end
