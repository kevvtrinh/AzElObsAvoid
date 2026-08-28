function handles = plotTrajectory(result, optionOverrides)
%% Section 0: Header & Readme
% SYNTAX
%   options = obstacleAvoidance.plotting.plotTrajectory()
%   handles = obstacleAvoidance.plotting.plotTrajectory(result)
%   handles = obstacleAvoidance.plotting.plotTrajectory(result, optionOverrides)
%**************************************************************************
% PURPOSE
%   - Plot returned HS3 motion, search diagnostics, and physical limits.
%   - Animate returned motion without rerunning planning or collision logic.
%**************************************************************************
% INPUTS
%   - result (scalar planTrajectory result)
%   - optionOverrides (scalar struct, optional; default struct())
%       Controls workspace, all-seed-path, search-edge, visibility, kinematic,
%       swept-surface, animation displays, and synchronized GIF export.
%       Hidden figures never pause.
%**************************************************************************
% OUTPUTS
%   - handles (scalar struct)
%       The result contains workspace, visibility, kinematic, and animation
%       graphics handles.
%**************************************************************************
% UNITS
%   - Axes show degrees, seconds, deg/s, deg/s^2, and deg/s^3.
%**************************************************************************

%% Section 1: Resolve Display Controls

% Plot only data in the planner result. Do not run route search or motion
% optimization again. Each plot then shows the same data that validation uses.
% If a plot looks incorrect, inspect the related result field before changing
% plotting code.
defaults = struct( ...
    "FigureVisible", "on", ...
    "Title", "Az/El motion plan", ...
    "ShowWorkspace", true, ...
    "ShowKinematics", true, ...
    "ShowAnimation", true, ...
    "ShowSeedPaths", false, ...
    "ShowSearchEdges", true, ...
    "ShowVisibilityGraphs", true, ...
    "FrameStride", 5, ...
    "Pause_s", 0.001, ...
    "SaveAnimationGif", false, ...
    "AnimationGifFile", "obstacleAvoidanceTrajectory.gif", ...
    "AnimationGifDelay_s", 0.01, ...
    "ShowSweptSurfaces", true, "MaximumDisplayedSlicesPerObstacle", 30, "MaximumDisplayedVisibilitySnapshots", 30);
if nargin == 0
    handles = defaults;
    return;
end
if nargin < 2 || isempty(optionOverrides)
    optionOverrides = struct();
end
if ~isstruct(result) || ~isscalar(result) || ~all(isfield(result, {'Inputs', 'Success', 'SearchDiagnostics'}))
    error("plotTrajectory:InvalidResult", "result must be a scalar planTrajectory result.");
end
optionOverrides = normalizePlotAliases(optionOverrides);
[options, unknownNames] = obstacleAvoidance.input.resolveOptions( defaults, optionOverrides);
if ~isempty(unknownNames)
    warning("plotTrajectory:UnknownOptions", ...
        "Ignoring unknown option fields: %s. No behavior changed.", strjoin(unknownNames, ", "));
end
if isfield(result.Options, "CollectAllSeedCandidates") && ...
        result.Options.CollectAllSeedCandidates && ...
        ~isfield(optionOverrides, "ShowSeedPaths")
    options.ShowSeedPaths = true;
end
options.FigureVisible = lower(string(options.FigureVisible));
options.Title = string(options.Title);
options.AnimationGifFile = string(options.AnimationGifFile);
if ~isscalar(options.FigureVisible) || ~any(options.FigureVisible == ["on", "off"])
    error("plotTrajectory:InvalidFigureVisible", "FigureVisible must be 'on' or 'off'.");
end
if ~isscalar(options.Title)
    error("plotTrajectory:InvalidTitle", "Title must be scalar text.");
end
if ~isscalar(options.AnimationGifFile) || ...
        strlength(options.AnimationGifFile) == 0
    error("plotTrajectory:InvalidAnimationGifFile", ...
        "AnimationGifFile must be nonempty scalar text.");
end
logicalNames = ["ShowWorkspace", "ShowKinematics", "ShowAnimation", ...
    "ShowSeedPaths", "ShowSearchEdges", "ShowVisibilityGraphs", ...
    "ShowSweptSurfaces", ...
    "SaveAnimationGif"];

% Convert each display switch to one logical value. Reject arrays and other
% values so each display branch has one clear state.
for name = logicalNames
    options.(name) = obstacleAvoidance.input.normalizeLogicalScalar( options.(name), name, "plotTrajectory:InvalidLogicalOption");
end
validateattributes(options.FrameStride, {'numeric'}, {'real', 'finite', 'scalar', 'integer', 'positive'});
validateattributes(options.Pause_s, {'numeric'}, {'real', 'finite', 'scalar', 'nonnegative'});
validateattributes(options.AnimationGifDelay_s, {'numeric'}, ...
    {'real', 'finite', 'scalar', 'nonnegative'});
countNames = ["MaximumDisplayedSlicesPerObstacle", "MaximumDisplayedVisibilitySnapshots"];

% Validate display limits before graphics use them. These limits reduce only
% drawing work. They do not remove planner diagnostics.
for name = countNames
    validateattributes(options.(name), {'numeric'}, {'real', 'finite', 'scalar', 'integer', 'positive'});
end
handles = createEmptyHandles(options);
% Prepare a local copy of obstacle histories for time queries. Workspace plots
% and animation frames use this copy. The planner result does not change.
obstacles = obstacleAvoidance.obstacles.prepareDynamic(result.Inputs.obstacles);

%% Section 2: Plot Workspace And Failure Diagnostics

% The workspace view shows protected geometry, explored search data, and the
% selected route. A failed result can show a best partial route instead.
if options.ShowWorkspace
    workspaceFigure = figure( "Name", options.Title, "Visible", options.FigureVisible);
    workspaceAxes = axes(workspaceFigure);
    configureSpatialAxes(workspaceAxes);
    displayTime_s = result.Inputs.initialState.time_s;
    drawObstacles(workspaceAxes, obstacles, displayTime_s, true);
    gridRecord = result.SearchDiagnostics.Grid;
    % Draw search edges before nodes and final motion. Search detail then stays
    % behind the important route data.
    if options.ShowSearchEdges
        drawSearchEdges(workspaceAxes, gridRecord);
    end
    if isfield(gridRecord, "ExploredNodes_deg") && ~isempty(gridRecord.ExploredNodes_deg)
        explored_deg = gridRecord.ExploredNodes_deg;
        scatter(workspaceAxes, explored_deg(:, 1), explored_deg(:, 2), ...
            9, [0.45 0.45 0.45], "filled", "DisplayName", "Expanded search node");
    end
    if isfield(gridRecord, "FrontierNodes_deg") && ~isempty(gridRecord.FrontierNodes_deg)
        frontier_deg = gridRecord.FrontierNodes_deg;
        scatter(workspaceAxes, frontier_deg(:, 1), frontier_deg(:, 2), ...
            18, [0.95 0.65 0.15], "filled", "DisplayName", "Final search frontier");
    end

    % Draw candidate seed routes first. Then emphasize the selected route or
    % the best partial route from a failed search.
    seedColors = lines(max(1, numel(result.Seeds)));
    for seedIndex = 1:numel(result.Seeds)
        route_deg = result.Seeds(seedIndex).position_deg;
        if options.ShowSeedPaths
            [route_deg, seedLabel] = displayedSeedPath(result, seedIndex);
            plot(workspaceAxes, route_deg(:, 1), route_deg(:, 2), ...
                "-o", "Color", seedColors(seedIndex, :), ...
                "LineWidth", 1.6, "MarkerSize", 4, ...
                "DisplayName", seedLabel);
            labelPoint_deg = route_deg(ceil(size(route_deg, 1) / 2), :);
            text(workspaceAxes, labelPoint_deg(1), labelPoint_deg(2), ...
                "  " + seedIndex, "Color", seedColors(seedIndex, :), ...
                "FontWeight", "bold", "HandleVisibility", "off");
        else
            plot(workspaceAxes, route_deg(:, 1), route_deg(:, 2), ...
                "-", "Color", [0.75 0.75 0.75], ...
                "HandleVisibility", "off");
        end
    end
    if result.Success
        % Show both successful planning stages. Search selects a piecewise
        % linear seed. HS3 returns a smooth polynomial with time data.
        plot(workspaceAxes, result.SelectedSeed_deg(:, 1), ...
            result.SelectedSeed_deg(:, 2), "--", "Color", [0.15 0.35 0.8], ...
            "LineWidth", 1.2, "DisplayName", "Selected geometric route");
        plot(workspaceAxes, result.position_deg(:, 1), ...
            result.position_deg(:, 2), "k-", "LineWidth", 2, "DisplayName", "Timed motion");
    elseif result.SearchDiagnostics.BestPartialSeedIndex > 0
        % A failed search can keep the route closest to the goal. Plot this
        % route to show search progress. Do not display it as a successful path.
        partialSeed = result.Seeds( result.SearchDiagnostics.BestPartialSeedIndex).position_deg;
        plot(workspaceAxes, partialSeed(:, 1), partialSeed(:, 2), ...
            "--", "Color", [0.15 0.35 0.8], "LineWidth", 1.2, "DisplayName", "Best partial seed");
    end
    drawTargetTrack(workspaceAxes, result.Inputs.goalState, displayTime_s);
    start_deg = result.Inputs.initialState.position_deg;
goal_deg = obstacleAvoidance.input.goalPositionAtTime( result.Inputs.goalState, result.Inputs.goalState.time_s);
    plot(workspaceAxes, start_deg(1), start_deg(2), "go", "MarkerFaceColor", "g", "DisplayName", "Start");
    plot(workspaceAxes, goal_deg(1), goal_deg(2), "ro", "MarkerFaceColor", "r", "DisplayName", "Goal");
    xlabel(workspaceAxes, "Azimuth (deg)");
    ylabel(workspaceAxes, "Elevation (deg)");
    title(workspaceAxes, diagnosticTitle(result, options.Title));
    legend(workspaceAxes, "Location", "best");
    handles.WorkspaceFigure = workspaceFigure;
    handles.WorkspaceAxes = workspaceAxes;
end

%% Section 3: Plot Time-Expanded Visibility Diagnostics

% Use time as the vertical axis for moving-obstacle diagnostics. The same
% position at two times represents two different search states. An obstacle can
% occupy the position at one time and leave it clear at another time.
if options.ShowVisibilityGraphs
    visibilityFigure = figure( "Name", options.Title + " visibility diagnostics", "Visible", options.FigureVisible);
    visibilityAxes = axes(visibilityFigure);
    hold(visibilityAxes, "on");
    grid(visibilityAxes, "on");
    box(visibilityAxes, "on");
    gridRecord = result.SearchDiagnostics.Grid;
    layerTimes_s = visibilityLayerTimes(gridRecord, result.Inputs);
    layerIndices = sampledIndices(numel(layerTimes_s), options.MaximumDisplayedVisibilitySnapshots);
    % Display sampling reduces graphics work only. Search statistics and the
    % result still describe all retained search states.
    if options.ShowSweptSurfaces
        drawObstacleLayers(visibilityAxes, obstacles, ...
            layerTimes_s(layerIndices), options.MaximumDisplayedSlicesPerObstacle);
    end
    if isfield(gridRecord, "NodePosition_deg") && ~isempty(gridRecord.NodePosition_deg)
        nodePosition_deg = gridRecord.NodePosition_deg;

        % Repeat spatial graph nodes at each displayed time layer. This shows
        % how available search states change with time.
        for layerIndex = reshape(layerIndices, 1, [])
            scatter3(visibilityAxes, nodePosition_deg(:, 1), ...
                nodePosition_deg(:, 2), ...
                repmat(layerTimes_s(layerIndex), ...
                size(nodePosition_deg, 1), 1), 5, [0.55 0.55 0.55], "filled", "HandleVisibility", "off");
        end
    end

    % Plot each seed with its estimated times. This shows candidate routes in
    % both space and time.
    for seedIndex = 1:numel(result.Seeds)
        seed = result.Seeds(seedIndex);
        seedTime_s = result.Inputs.initialState.time_s + seed.tau * seed.EstimatedDuration_s;
        seedPosition_deg = seed.position_deg;
        seedColor = [0.45 0.45 0.45];
        lineStyle = ":";
        if options.ShowSeedPaths
            seedColor = seedColors(seedIndex, :);
            lineStyle = "-o";
            [seedPosition_deg, seedLabel, seedTime_s] = ...
                displayedSeedPath(result, seedIndex);
        else
            seedLabel = "Seed " + seedIndex + ": " + seed.Source;
        end
        plot3(visibilityAxes, seedPosition_deg(:, 1), ...
            seedPosition_deg(:, 2), seedTime_s, lineStyle, ...
            "Color", seedColor, "DisplayName", ...
            seedLabel);
    end
    if result.Success
        plot3(visibilityAxes, result.position_deg(:, 1), ...
            result.position_deg(:, 2), result.time_s, "k-", "LineWidth", 2.2, "DisplayName", "Timed motion");
    end
    drawTargetTimeTrack(visibilityAxes, result.Inputs.goalState);
    xlabel(visibilityAxes, "Azimuth (deg)");
    ylabel(visibilityAxes, "Elevation (deg)");
    zlabel(visibilityAxes, "Time (s)");
    title(visibilityAxes, diagnosticTitle(result, options.Title));
    view(visibilityAxes, 3);
    legend(visibilityAxes, "Location", "best");
    handles.VisibilityFigure = visibilityFigure;
    handles.VisibilityAxes = visibilityAxes;
    handles.VisibilityGraphs = struct("Figure", visibilityFigure, "Axes", visibilityAxes);
end

%% Section 4: Plot Returned Kinematics

% Use one time axis for position, velocity, acceleration, and jerk. A reader can
% compare continuity and physical limits at the same times.
if options.ShowKinematics && result.Success
    kinematicFigure = figure( "Name", options.Title + " kinematics", "Visible", options.FigureVisible);
    tiledLayout = tiledlayout(kinematicFigure, 4, 1, "TileSpacing", "compact", "Padding", "compact");
    quantityNames = ["position_deg", "velocity_deg_s", "acceleration_deg_s2", "jerk_deg_s3"];
    yLabels = ["Position (deg)", "Velocity (deg/s)", "Acceleration (deg/s^2)", "Jerk (deg/s^3)"];
    limitValues = {[], result.Inputs.limits.maxVelocity_deg_s, ...
        result.Inputs.limits.maxAcceleration_deg_s2, result.Inputs.limits.maxJerk_deg_s3};
    axesHandles = gobjects(4, 1);

    % Create one aligned panel for position and each derivative. Keep all time
    % axes aligned.
    for quantityIndex = 1:4
        axesHandles(quantityIndex) = nexttile(tiledLayout);
        axesHandle = axesHandles(quantityIndex);
        hold(axesHandle, "on");
        grid(axesHandle, "on");
        box(axesHandle, "on");
        values = result.(quantityNames(quantityIndex));
        plot(axesHandle, result.time_s, values(:, 1), "DisplayName", "Azimuth");
        plot(axesHandle, result.time_s, values(:, 2), "DisplayName", "Elevation");
        if ~isempty(limitValues{quantityIndex})

            % Extend each derivative axis 20 percent beyond its largest limit.
            % This keeps limit lines and motion data visible.
            plotLimit = 1.20 * max(abs(limitValues{quantityIndex}));

            % Draw positive and negative limits for both coordinates. These
            % lines show the permitted range in each derivative panel.
            for axisIndex = 1:2
                yline(axesHandle, limitValues{quantityIndex}(axisIndex), ...
                    "--", "Color", "r", "LineWidth", 3, ...
                    "HandleVisibility", "off");
                yline(axesHandle, -limitValues{quantityIndex}(axisIndex), ...
                    "--", "Color", "r", "LineWidth", 3, ...
                    "HandleVisibility", "off");
            end
            ylim(axesHandle, [-plotLimit, plotLimit]);
        end
        ylabel(axesHandle, yLabels(quantityIndex));
    end
    timeLimits_s = [result.time_s(1), result.time_s(end)];
    set(axesHandles, "XLim", timeLimits_s);
    xlabel(axesHandles(end), "Time (s)");
    legend(axesHandles(1), "Location", "best");
    title(tiledLayout, options.Title);
    handles.KinematicFigure = kinematicFigure;
    handles.KinematicAxes = axesHandles;
    handles.KinematicsFigure = kinematicFigure;
    handles.KinematicsAxes = axesHandles;
end

%% Section 5: Animate Returned Motion

% Animation uses the returned sample history. Query obstacles and moving targets
% at the same time as the trajectory marker. All displayed motion then uses the
% planner time instead of computer clock time.
if (options.ShowAnimation || options.SaveAnimationGif) && result.Success
    animationFigureVisibility = options.FigureVisible;
    if options.SaveAnimationGif
        animationFigureVisibility = "on";
    end
    animationFigure = figure( ...
        "Name", options.Title + " animation", ...
        "Visible", animationFigureVisibility);
    animationFigure.Position(3:4) = [900 500];
    animationLayout = tiledlayout(animationFigure, 4, 2, ...
        "TileSpacing", "compact", "Padding", "compact");
    animationAxes = nexttile(animationLayout, 1, [4 1]);
    animationKinematicAxes = gobjects(4, 1);
    currentKinematicMarkers = gobjects(4, 2);
    elapsedKinematicLines = gobjects(4, 2);
    timeCursors = gobjects(4, 1);
    animationQuantityNames = ["position_deg", "velocity_deg_s", ...
        "acceleration_deg_s2", "jerk_deg_s3"];
    animationYLabels = ["Position (deg)", "Velocity (deg/s)", ...
        "Acceleration (deg/s^2)", "Jerk (deg/s^3)"];
    animationLimitValues = {[], result.Inputs.limits.maxVelocity_deg_s, ...
        result.Inputs.limits.maxAcceleration_deg_s2, ...
        result.Inputs.limits.maxJerk_deg_s3};
    animationAxisNames = ["Azimuth", "Elevation"];
    motionColors = [0.00 0.45 0.74; 0.85 0.33 0.10];
    timeLimits_s = [result.time_s(1), result.time_s(end)];

    % Prepare synchronized histories one time. Each frame advances traces,
    % current-state markers, and the time cursor to the same sample.
    for quantityIndex = 1:4
        animationKinematicAxes(quantityIndex) = ...
            nexttile(animationLayout, 2 * quantityIndex);
        axesHandle = animationKinematicAxes(quantityIndex);
        hold(axesHandle, "on");
        grid(axesHandle, "on");
        box(axesHandle, "on");
        values = result.(animationQuantityNames(quantityIndex));
        for axisIndex = 1:2
            plot(axesHandle, result.time_s, values(:, axisIndex), ...
                "-", "Color", 0.65 + 0.25 * motionColors(axisIndex, :), ...
                "LineWidth", 1, "HandleVisibility", "off");
            elapsedKinematicLines(quantityIndex, axisIndex) = ...
                plot(axesHandle, result.time_s(1), values(1, axisIndex), ...
                "-", "Color", motionColors(axisIndex, :), "LineWidth", 2, ...
                "DisplayName", animationAxisNames(axisIndex));
            currentKinematicMarkers(quantityIndex, axisIndex) = ...
                plot(axesHandle, result.time_s(1), values(1, axisIndex), ...
                "o", "Color", motionColors(axisIndex, :), ...
                "MarkerFaceColor", motionColors(axisIndex, :), ...
                "MarkerEdgeColor", "w", "MarkerSize", 6, ...
                "HandleVisibility", "off");
        end
        if ~isempty(animationLimitValues{quantityIndex})
            plotLimit = 1.20 * max(abs( ...
                animationLimitValues{quantityIndex}));
            for axisIndex = 1:2
                yline(axesHandle, ...
                    animationLimitValues{quantityIndex}(axisIndex), ...
                    "--", "Color", "r", "LineWidth", 2, ...
                    "HandleVisibility", "off");
                yline(axesHandle, ...
                    -animationLimitValues{quantityIndex}(axisIndex), ...
                    "--", "Color", "r", "LineWidth", 2, ...
                    "HandleVisibility", "off");
            end
            ylim(axesHandle, [-plotLimit, plotLimit]);
        end
        xlim(axesHandle, timeLimits_s);
        ylabel(axesHandle, animationYLabels(quantityIndex));
        timeCursors(quantityIndex) = xline( ...
            axesHandle, result.time_s(1), "k-", "LineWidth", 1.4, ...
            "HandleVisibility", "off");
    end
    xlabel(animationKinematicAxes(end), "Time (s)");

    % Use a figure annotation for time text. It stays above the right panels
    % when drawnow updates the tiled layout.
    legendText = "\color[rgb]{0 0.45 0.74}Azimuth" + ...
        "     \color[rgb]{0.85 0.33 0.10}Elevation";
    animationLegend = annotation(animationFigure, "textbox", ...
        [0.60 0.955 0.30 0.045], ...
        "String", legendText, ...
        "Interpreter", "tex", ...
        "HorizontalAlignment", "center", ...
        "VerticalAlignment", "middle", ...
        "FontSize", 10, ...
        "FontWeight", "bold", ...
        "BackgroundColor", "none", ...
        "EdgeColor", "none", ...
        "FitBoxToText", "off");
    frameIndices = unique([1:options.FrameStride:numel(result.time_s), numel(result.time_s)]);
    animationGifFile = char(options.AnimationGifFile);
    gifFrameCount = 0;

    % Build each frame from returned motion and obstacle histories. Animation
    % does not call the planner.
    for frameIndex = frameIndices
        cla(animationAxes);
        configureSpatialAxes(animationAxes);
        drawObstacles(animationAxes, obstacles, result.time_s(frameIndex), true);
        drawTargetTrack(animationAxes, result.Inputs.goalState, result.time_s(frameIndex));
        plot(animationAxes, result.position_deg(:, 1), result.position_deg(:, 2), ...
            "-", "Color", [0.72 0.78 0.84], "LineWidth", 1.2, "DisplayName", "Complete timed path");
        plot(animationAxes, result.position_deg(1:frameIndex, 1), ...
            result.position_deg(1:frameIndex, 2), "-", ...
            "Color", [0.05 0.70 0.82], "LineWidth", 3.5, "DisplayName", "Elapsed path");
        scatter(animationAxes, result.position_deg(frameIndex, 1), ...
            result.position_deg(frameIndex, 2), 60, [0.95 0.25 0.15], ...
            "filled", "MarkerEdgeColor", "w", "LineWidth", 1, "DisplayName", "Current state");
        xlabel(animationAxes, "Azimuth (deg)");
        ylabel(animationAxes, "Elevation (deg)");
        title(animationAxes, sprintf("%s | t = %.3f s", options.Title, result.time_s(frameIndex)));

        % Advance each kinematic plot to the sample in the spatial animation.
        for quantityIndex = 1:4
            values = result.(animationQuantityNames(quantityIndex));
            for axisIndex = 1:2
                set(elapsedKinematicLines(quantityIndex, axisIndex), ...
                    "XData", result.time_s(1:frameIndex), ...
                    "YData", values(1:frameIndex, axisIndex));
                set(currentKinematicMarkers(quantityIndex, axisIndex), ...
                    "XData", result.time_s(frameIndex), ...
                    "YData", values(frameIndex, axisIndex));
            end
            timeCursors(quantityIndex).Value = result.time_s(frameIndex);
        end
        drawnow;

        % Capture the complete dashboard. Exported spatial and kinematic views
        % then show the same time.
        if options.SaveAnimationGif
            capturedFrame = getframe(animationFigure);
            rgbImage = frame2im(capturedFrame);
            [indexedImage, colorMap] = rgb2ind(rgbImage, 256);
            if gifFrameCount == 0
                imwrite(indexedImage, colorMap, animationGifFile, "gif", ...
                    "LoopCount", inf, ...
                    "DelayTime", options.AnimationGifDelay_s);
            else
                imwrite(indexedImage, colorMap, animationGifFile, "gif", ...
                    "WriteMode", "append", ...
                    "DelayTime", options.AnimationGifDelay_s);
            end
            gifFrameCount = gifFrameCount + 1;
        end
        if options.FigureVisible == "on" && options.Pause_s > 0
            pause(options.Pause_s);
        end
    end
    if options.FigureVisible == "off"
        animationFigure.Visible = "off";
    end
    handles.AnimationFigure = animationFigure;
    handles.AnimationAxes = animationAxes;
    handles.AnimationKinematicAxes = animationKinematicAxes;
    handles.AnimationLegend = animationLegend;
    if options.SaveAnimationGif
        handles.AnimationGifFile = string(animationGifFile);
    end
    handles.Animation = struct( ...
        "Figure", animationFigure, ...
        "Axes", animationAxes, ...
        "KinematicAxes", animationKinematicAxes, ...
        "CurrentKinematicMarkers", currentKinematicMarkers, ...
        "ElapsedKinematicLines", elapsedKinematicLines, ...
        "TimeCursors", timeCursors, ...
        "Legend", animationLegend, ...
        "GifFile", handles.AnimationGifFile);
end
end

function options = normalizePlotAliases(options)
% Convert old display option names in one location. Plotting sections can then
% use only current option names.
if ~isstruct(options) || ~isscalar(options)
    error("plotTrajectory:InvalidOptions", "optionOverrides must be a scalar struct.");
end
aliases = [ ...
    "AnimationFrameStride", "FrameStride"; "ShowKinematicPlot", "ShowKinematics"; "AnimationPause_s", "Pause_s"];

% Translate each old display name one time. If both names are present, use the
% current name.
for aliasIndex = 1:size(aliases, 1)
    oldName = aliases(aliasIndex, 1);
    newName = aliases(aliasIndex, 2);
    if isfield(options, oldName)
        if ~isfield(options, newName)
            options.(newName) = options.(oldName);
        end
        options = rmfield(options, oldName);
    end
end
compatibilityNames = ["JerkConstraintEnabled", "MaxJerk_deg_s3", "ConfiguredFiniteMaxJerk_deg_s3", "PlotOptions"];

% Remove example-reporting fields before plot option validation. These fields
% do not control plotting.
for name = compatibilityNames
    if isfield(options, name)
        options = rmfield(options, name);
    end
end
end

function configureSpatialAxes(axesHandle)
% Apply one explicit spatial-axis style to workspace and animation axes.
hold(axesHandle, "on");
grid(axesHandle, "on");
box(axesHandle, "on");
axis(axesHandle, "equal");
end

function [position_deg, label, time_s] = displayedSeedPath(result, seedIndex)
% Prefer a retained completed candidate; otherwise show its geometric seed.
seed = result.Seeds(seedIndex);
position_deg = seed.position_deg;
time_s = result.Inputs.initialState.time_s + ...
    seed.tau * seed.EstimatedDuration_s;
summary = result.SeedSummaries(seedIndex);
label = sprintf( ...
    "Seed %d: %s | seed %.3f deg | arrival %.3f s | " + ...
    "motion %.3f deg | valid %d", ...
    seedIndex, seed.Source, seed.Length_deg, summary.ArrivalTime_s, ...
    summary.MotionLength_deg, summary.ValidationPassed);
if ~isfield(result, "CandidatePaths") || isempty(result.CandidatePaths)
    return;
end
candidateIndex = find( ...
    [result.CandidatePaths.SeedIndex] == seedIndex, 1, "first");
if isempty(candidateIndex)
    return;
end
candidatePath = result.CandidatePaths(candidateIndex);
if ~isempty(candidatePath.position_deg)
    position_deg = candidatePath.position_deg;
    time_s = candidatePath.time_s;
end
end

function drawSearchEdges(axesHandle, gridRecord)
% Draw retained accepted and collision-rejected visibility tests.
% Accepted edges show moves that search can use. Rejected edges show moves that
% protected geometry blocks.
if isfield(gridRecord, "DenseSeedEnvelopeUsed") && ...
        gridRecord.DenseSeedEnvelopeUsed && ~isempty(gridRecord.DenseSeedEnvelope_deg)
    boundary_deg = gridRecord.DenseSeedEnvelope_deg;
    plot(axesHandle, ...
        [boundary_deg(:, 1); boundary_deg(1, 1)], ...
        [boundary_deg(:, 2); boundary_deg(1, 2)], ...
        "-.", "Color", [0.55 0.25 0.75], "LineWidth", 1.5, "DisplayName", "Dense seed envelope");
end
if isfield(gridRecord, "SeedCluster") && ~isempty(gridRecord.SeedCluster.ClusterBoundary_deg)
    boundary_deg = gridRecord.SeedCluster.ClusterBoundary_deg;
    plot(axesHandle, ...
        [boundary_deg(:, 1); boundary_deg(1, 1)], ...
        [boundary_deg(:, 2); boundary_deg(1, 2)], ...
        "--", "Color", [0.85 0.45 0.10], "LineWidth", 1.5, "DisplayName", "Seed-only obstacle cluster");
end
if isfield(gridRecord, "AcceptedEdges_deg") && ~isempty(gridRecord.AcceptedEdges_deg)
    [azimuth_deg, elevation_deg] = edgeLineData( gridRecord.AcceptedEdges_deg);
    plot(axesHandle, azimuth_deg, elevation_deg, ...
        "-", "Color", [0.30 0.75 0.78], "LineWidth", 0.4, "DisplayName", "Accepted visibility edge");
end
if isfield(gridRecord, "RejectedEdges_deg") && ~isempty(gridRecord.RejectedEdges_deg)
    [azimuth_deg, elevation_deg] = edgeLineData( gridRecord.RejectedEdges_deg);
    plot(axesHandle, azimuth_deg, elevation_deg, ...
        ":", "Color", [0.90 0.55 0.55], "LineWidth", 0.4, "DisplayName", "Collision-rejected edge");
end
end

function [azimuth_deg, elevation_deg] = edgeLineData(edges_deg)
% Convert N-by-4 edge endpoints to NaN-separated plot vectors.
% Insert NaN between independent edges. MATLAB can then draw many edges with
% one graphics call without connecting adjacent edge records.
edgeCount = size(edges_deg, 1);
azimuth_deg = reshape([ edges_deg(:, 1), edges_deg(:, 3), nan(edgeCount, 1)].', [], 1);
elevation_deg = reshape([ edges_deg(:, 2), edges_deg(:, 4), nan(edgeCount, 1)].', [], 1);
end

function drawObstacles(axesHandle, obstacles, time_s, showOriginal)
% Draw original and protected geometry from the same obstacle data.
obstacleColors = lines(max(1, numel(obstacles)));

% Use one color for both boundaries of an obstacle. A reader can then associate
% the original boundary with its safety-protected boundary.
for obstacleIndex = 1:numel(obstacles)
    obstacle = obstacles(obstacleIndex);
    obstacleColor = obstacleColors(obstacleIndex, :);
    if showOriginal
        originalObstacle = obstacle;
        originalObstacle.az_deg = obstacle.originalAz_deg;
        originalObstacle.el_deg = obstacle.originalEl_deg;
        % Prepared data describes the protected boundary. Remove it before
        % preparing the original boundary. Safety inflation can change vertex
        % count and time-interval correspondence.
        if isfield(originalObstacle, "InternalPreparation")
            originalObstacle = rmfield( originalObstacle, "InternalPreparation");
        end
        originalShape = obstacleAvoidance.obstacles.shapeAtTime( originalObstacle, time_s);
        drawShape(axesHandle, originalShape, obstacleColor, [0.12 0.12 0.12], "-", 0.22, "Original obstacle");
    end
    protectedShape = obstacleAvoidance.obstacles.shapeAtTime(obstacle, time_s);
    drawShape(axesHandle, protectedShape, "none", 0.65 * obstacleColor, "--", 0, "Protected obstacle");
end
end

function drawShape(axesHandle, shape, faceColor, edgeColor, lineStyle, faceAlpha, displayName)
% Draw one filled polygon and its boundary on explicit axes.
if isempty(shape.Vertices)
    return;
end
plot(axesHandle, shape, ...
    "FaceColor", faceColor, "FaceAlpha", faceAlpha, ...
    "EdgeColor", edgeColor, "LineStyle", lineStyle, "LineWidth", 1.2, "DisplayName", displayName);
end

function drawObstacleLayers(axesHandle, obstacles, times_s, maximumCount)
% Draw colored obstacle slices and compatible swept surfaces.
% Each slice is a protected boundary at one time. Connect matching vertices at
% adjacent times to show a swept surface. This surface is for explanation only.
% Collision validation does not use it.
timeIndices = sampledIndices(numel(times_s), maximumCount);
obstacleColors = lines(max(1, numel(obstacles)));
isFirstShape = true;

% Build a separate swept surface for each moving obstacle. Do not connect the
% histories of different obstacles.
for obstacleIndex = 1:numel(obstacles)
    obstacleColor = obstacleColors(obstacleIndex, :);
    previousAzimuth_deg = zeros(0, 1);
    previousElevation_deg = zeros(0, 1);
    previousTime_s = NaN;

    % Process retained times in order. Connect only adjacent nonempty shapes.
    for timeIndex = reshape(timeIndices, 1, [])
        shape = obstacleAvoidance.obstacles.shapeAtTime( obstacles(obstacleIndex), times_s(timeIndex));
        if isempty(shape.Vertices)
            previousAzimuth_deg = zeros(0, 1);
            previousElevation_deg = zeros(0, 1);
            previousTime_s = NaN;
            continue;
        end
        [azimuth_deg, elevation_deg] = boundary(shape);
        topologyMatches = isequal(size(previousAzimuth_deg), size(azimuth_deg)) && ...
            isequal(isfinite(previousAzimuth_deg), isfinite(azimuth_deg)) && ...
            isequal(isfinite(previousElevation_deg), isfinite(elevation_deg));
        if topologyMatches
            surface(axesHandle, ...
                [previousAzimuth_deg.'; azimuth_deg.'], ...
                [previousElevation_deg.'; elevation_deg.'], ...
                [repmat(previousTime_s, 1, numel(azimuth_deg)); ...
                repmat(times_s(timeIndex), 1, numel(azimuth_deg))], ...
                "FaceColor", obstacleColor, "FaceAlpha", 0.10, ...
                "EdgeColor", 0.55 * obstacleColor, ...
                "EdgeAlpha", 0.65, "LineWidth", 0.55, "MeshStyle", "both", "HandleVisibility", "off");
        end
        displayName = "";
        visibility = "off";
        if isFirstShape
            displayName = "Protected obstacle slices";
            visibility = "on";
            isFirstShape = false;
        end
        patch(axesHandle, azimuth_deg, elevation_deg, ...
            repmat(times_s(timeIndex), size(azimuth_deg)), ...
            obstacleColor, "FaceAlpha", 0.18, ...
            "EdgeColor", 0.65 * obstacleColor, "LineWidth", 0.7, ...
            "DisplayName", displayName, "HandleVisibility", visibility);
        previousAzimuth_deg = azimuth_deg;
        previousElevation_deg = elevation_deg;
        previousTime_s = times_s(timeIndex);
    end
end
end

function drawTargetTrack(axesHandle, goalState, displayTime_s)
% Draw a sampled moving-target track and its position at display time.
if isfield(goalState, "targetPosition_deg") && ~isempty(goalState.targetPosition_deg)
    plot(axesHandle, goalState.targetPosition_deg(:, 1), ...
        goalState.targetPosition_deg(:, 2), "-.", ...
        "Color", [0.65 0.10 0.65], "LineWidth", 1.4, "DisplayName", "Moving target track");
    targetPosition_deg = obstacleAvoidance.input.goalPositionAtTime( goalState, displayTime_s);
    plot(axesHandle, targetPosition_deg(1), targetPosition_deg(2), ...
        "d", "Color", [0.65 0.10 0.65], "MarkerFaceColor", ...
        [0.95 0.35 0.95], "MarkerSize", 8, "LineWidth", 1.2, "DisplayName", "Moving target");
end
end

function drawTargetTimeTrack(axesHandle, goalState)
% Draw a moving target in the time-expanded diagnostic view.
if isfield(goalState, "targetPosition_deg") && ~isempty(goalState.targetPosition_deg)
    plot3(axesHandle, goalState.targetPosition_deg(:, 1), ...
        goalState.targetPosition_deg(:, 2), goalState.targetTime_s, ...
        "-.", "Color", [0.65 0.10 0.65], "LineWidth", 1.4, "DisplayName", "Moving target track");
targetPosition_deg = obstacleAvoidance.input.goalPositionAtTime( goalState, goalState.time_s);
    plot3(axesHandle, targetPosition_deg(1), targetPosition_deg(2), ...
        goalState.time_s, "d", "Color", [0.65 0.10 0.65], ...
        "MarkerFaceColor", [0.95 0.35 0.95], "MarkerSize", 8, "LineWidth", 1.2, "DisplayName", "Moving target");
end
end

function times_s = visibilityLayerTimes(gridRecord, inputs)
% Select retained temporal layers or the planning endpoints.
times_s = zeros(0, 1);
if isfield(gridRecord, "TemporalLayerTimes_s")
    times_s = gridRecord.TemporalLayerTimes_s;
end
if isempty(times_s) && isfield(gridRecord, "SampleTimes_s")
    times_s = gridRecord.SampleTimes_s;
end
if isempty(times_s)
    times_s = [inputs.initialState.time_s; inputs.goalState.time_s];
end
times_s = unique(times_s(:));
end

function indices = sampledIndices(count, maximumCount)
% Retain evenly distributed display indices without changing counts.
% Keep the first and last indices. Select the other indices at even intervals.
% This preserves the displayed span and limits interactive graphics work.
if count <= 0
    indices = zeros(0, 1);
elseif count <= maximumCount
    indices = (1:count).';
else
    indices = unique(round(linspace(1, count, maximumCount))).';
end
end

function titleText = diagnosticTitle(result, prefix)
% Include the termination reason and the main search counts. For a confusing
% failure plot, compare this title with result.SearchDiagnostics.
gridRecord = result.SearchDiagnostics.Grid;
expanded = fieldOrZero(gridRecord, "ExpandedCount");
rejected = fieldOrZero(gridRecord, "RejectedTransitionCount");
titleText = sprintf("%s | %s | seeds %d | expanded %d | rejected %d", ...
    prefix, result.TerminationReason, numel(result.Seeds), expanded, rejected);
end

function value = fieldOrZero(record, fieldName)
% Read an optional diagnostic count. Return zero when it is not available.
value = 0;
if isfield(record, fieldName)
    value = record.(fieldName);
end
end

function handles = createEmptyHandles(options)
% Define all graphics output fields before any display is created.
handles = struct( ...
    "WorkspaceFigure", gobjects(0), ...
    "WorkspaceAxes", gobjects(0), ...
    "VisibilityFigure", gobjects(0), ...
    "VisibilityAxes", gobjects(0), ...
    "VisibilityGraphs", struct(), ...
    "KinematicFigure", gobjects(0), ...
    "KinematicAxes", gobjects(0), ...
    "KinematicsFigure", gobjects(0), ...
    "KinematicsAxes", gobjects(0), ...
    "AnimationFigure", gobjects(0), ...
    "AnimationAxes", gobjects(0), ...
    "AnimationKinematicAxes", gobjects(0), ...
    "AnimationLegend", gobjects(0), ...
    "AnimationGifFile", "", ...
    "Animation", struct(), ...
    "Options", options);
end
