function handles = plotTrajectory(result, optionOverrides)
%% Section 0: Header & Readme
% SYNTAX
%   options = obstacleAvoidance.plotting.plotTrajectory()
%   handles = obstacleAvoidance.plotting.plotTrajectory(result)
%   handles = obstacleAvoidance.plotting.plotTrajectory(result, optionOverrides)
%   handles = obstacleAvoidance.plotting.plotTrajectory(result, axesHandle)
%**************************************************************************
% PURPOSE
%   - Plot the obstacle geometry, route search, motion, and physical limits
%     stored in a planner result. Plotting does not rerun motion planning.
%   - Animate returned samples against time-varying obstacles and targets.
%**************************************************************************
% INPUTS
%   - result (scalar planner result)
%       Successful or failed planner result with the available geometry,
%       route, and motion data.
%   - optionOverrides (scalar struct or Cartesian axes handle, optional)
%       Display, animation, and GIF controls. An axes handle requests only
%       the workspace plot. Hidden figures never pause.
%**************************************************************************
% OUTPUTS
%   - handles (scalar struct)
%       Figure and axes handles for the requested plots. Unused fields
%       contain empty handles. Axes selects the workspace or visibility view.
%       Invalid input throws an error.
%   - options (scalar struct, zero-input call)
%       Default plotting controls.
%**************************************************************************
% UNITS
%   - Axes use coordinate units, seconds, units/s, units/s^2, and units/s^3.
%**************************************************************************

%% Section 1: Resolve Display Controls

defaults = struct();
defaults.FigureVisible = "on";
defaults.Title         = "X/Y motion plan";

defaults.ShowWorkspace        = true;
defaults.ShowKinematics       = true;
defaults.ShowAnimation        = true;
defaults.ShowSearchEdges      = true;
defaults.ShowVisibilityGraphs = true;

defaults.FrameStride = 5;
defaults.Pause_s     = 0.001;

defaults.SaveAnimationGif    = false;
defaults.AnimationGifFile    = "obstacleAvoidanceTrajectory.gif";
defaults.AnimationGifDelay_s = 0.01;
if nargin == 0
    handles = defaults;
    return
end
if nargin < 2 || isempty(optionOverrides)
    optionOverrides = struct();
end

% A failure result can still contain useful obstacles and route diagnostics.
% Require the usual result fields, but do not require successful motion.
requiredResultFieldNames = {'Inputs', 'Options', 'Limits', 'RequestedLimits', 'Success', ...
    'TerminationReason', 'PreparedObstacles', 'VisibilityGraph', 'Route_units', ...
    'time_s', 'position_units', 'velocity_units_s', 'acceleration_units_s2', 'jerk_units_s3'};
if ~isstruct(result) || ~isscalar(result) || ~all(isfield(result, requiredResultFieldNames))
    error("plotTrajectory:InvalidResult", "result must be a scalar planner result.");
end

% A supplied axes handle lets the caller place the spatial view in a
% larger figure. It disables the separate kinematic and animation figures.
workspaceAxesHandle = gobjects(0);
if ~isstruct(optionOverrides)
    if ~isscalar(optionOverrides) || ~isgraphics(optionOverrides, "axes")
        error("plotTrajectory:InvalidAxes", "The second input must be plot options or Cartesian axes.");
    end
    workspaceAxesHandle = optionOverrides;
    optionOverrides     = struct('ShowKinematics', false, 'ShowAnimation', false);
end
useSuppliedAxes = ~isempty(workspaceAxesHandle);
if ~isscalar(optionOverrides)
    error("plotTrajectory:InvalidOptions", "optionOverrides must be a scalar struct.");
end
[options, unknownOptionNames] = obstacleAvoidance.input.resolveOptions(defaults, optionOverrides);
if ~isempty(unknownOptionNames)
    warning("plotTrajectory:UnknownOptions", ...
        "Ignoring unknown fields: %s. No behavior changed.", strjoin(unknownOptionNames, ", "));
end

options.FigureVisible    = lower(string(options.FigureVisible));
options.Title            = string(options.Title);
options.AnimationGifFile = string(options.AnimationGifFile);
if ~isscalar(options.FigureVisible) || ~any(options.FigureVisible == ["on", "off"])
    error("plotTrajectory:InvalidFigureVisible", "FigureVisible must be 'on' or 'off'.");
end
if ~isscalar(options.Title) || ~isscalar(options.AnimationGifFile) || strlength(options.AnimationGifFile) == 0
    error("plotTrajectory:InvalidTextOption", "Title/file must be nonempty scalar text.");
end

logicalOptionNames = ["ShowWorkspace", "ShowKinematics", "ShowAnimation", ...
    "ShowSearchEdges", "ShowVisibilityGraphs", "SaveAnimationGif"];
for optionName = logicalOptionNames
    options.(optionName) = obstacleAvoidance.input.normalizeLogicalScalar( ...
        options.(optionName), optionName, "plotTrajectory:InvalidLogicalOption");
end
nonnegativeOptionNames = ["Pause_s", "AnimationGifDelay_s"];
for optionName = nonnegativeOptionNames
    validateattributes(options.(optionName), {'numeric'}, {'real', 'finite', 'scalar', 'nonnegative'});
end
validateattributes(options.FrameStride, {'numeric'}, {'real', 'finite', 'scalar', 'integer', 'positive'});
handles = createEmptyHandles(options);

%% Section 2: Prepare Obstacles For The Displayed Time Range

% A failed arrival trial can end before the requested goal time. For example,
% a trial at 12 s in a request ending at 20 s should still display through 20 s.
plotEndTime_s = result.Inputs.goalState.time_s;
% Use the parent request's goal time when it extends that display range.
if ~result.Success && isfield(result, 'ParentRequest')
    parentGoalTime_s = result.ParentRequest.GoalTime_s;
    if isnumeric(parentGoalTime_s) && isscalar(parentGoalTime_s) && ...
            isreal(parentGoalTime_s) && isfinite(parentGoalTime_s)
        plotEndTime_s = max(plotEndTime_s, double(parentGoalTime_s));
    end
end
plotTimeRange_s    = [result.Inputs.initialState.time_s, plotEndTime_s];
protectedObstacles = obstacleAvoidance.obstacles.prepareObstacles(result.PreparedObstacles, plotTimeRange_s);
originalObstacles  = protectedObstacles;
if options.ShowWorkspace || options.ShowVisibilityGraphs || options.ShowAnimation || options.SaveAnimationGif
    % Show the original boundary beside the protected boundary used for
    % planning. Prepare the originals with zero margin; the protected
    % geometry above already includes its margin and must keep it unchanged.
    for obstacleIndex = 1:numel(originalObstacles)
        originalObstacles(obstacleIndex).x_units             = protectedObstacles(obstacleIndex).originalX_units;
        originalObstacles(obstacleIndex).y_units             = protectedObstacles(obstacleIndex).originalY_units;
        originalObstacles(obstacleIndex).safetyMargin_units  = 0;
        originalObstacles(obstacleIndex).InternalPreparation = struct();
    end
    originalObstacles  = obstacleAvoidance.obstacles.prepareObstacles(originalObstacles, plotTimeRange_s);
end

%% Section 3: Plot Workspace And Failure Diagnostics

if options.ShowWorkspace
    if isempty(workspaceAxesHandle)
        workspaceFigureHandle = figure("Name", options.Title, "Visible", options.FigureVisible);
        workspaceAxesHandle   = axes(workspaceFigureHandle);
    else
        workspaceFigureHandle = ancestor(workspaceAxesHandle, 'figure');
        cla(workspaceAxesHandle);
    end
    configureSpatialAxes(workspaceAxesHandle, result);
    if useSuppliedAxes
        xlim(workspaceAxesHandle, result.RequestedLimits.xInterval_units);
        ylim(workspaceAxesHandle, result.RequestedLimits.yInterval_units);
    end
    drawObstacles(workspaceAxesHandle, protectedObstacles, originalObstacles, result.Inputs.initialState.time_s);
    if options.ShowVisibilityGraphs
        drawSearchDiagnostics(workspaceAxesHandle, result, options.ShowSearchEdges);
    end
    drawPlannerRoute(workspaceAxesHandle, result);
    drawTarget(workspaceAxesHandle, result, result.Inputs.initialState.time_s);
    drawEndpoints(workspaceAxesHandle, result);
    finishAxes(workspaceAxesHandle, result, options.Title);
    handles.WorkspaceFigure = workspaceFigureHandle;
    handles.WorkspaceAxes   = workspaceAxesHandle;
end

% A wrapped path can look disconnected at the plot boundary. Also show its
% continuous coordinates so the full movement remains easy to understand.
pathCrossesSeam = false;
if result.Success && (result.Options.WrapX || result.Options.WrapY)
    wrappedPath_units = createDisplayPath(result, result.position_units);
    pathCrossesSeam    = any(isnan(wrappedPath_units(:, 1)));
end
createsSpatialDisplay = options.ShowWorkspace || options.ShowVisibilityGraphs || ...
    options.ShowAnimation || options.SaveAnimationGif;
if createsSpatialDisplay && pathCrossesSeam && ~useSuppliedAxes
    [handles.ContinuousWorkspaceFigure, handles.ContinuousWorkspaceAxes] = ...
        createContinuousWorkspace(result, options);
end

%% Section 4: Plot The Stored Visibility Graph

% Reuse the workspace view when it already shows the graph. Otherwise,
% create a separate view using the same obstacles, route, and endpoints.
if options.ShowVisibilityGraphs
    if options.ShowWorkspace
        handles.VisibilityFigure = handles.WorkspaceFigure;
        handles.VisibilityAxes   = handles.WorkspaceAxes;
    else
        handles.VisibilityFigure = figure("Name", options.Title + " search", "Visible", options.FigureVisible);
        handles.VisibilityAxes   = axes(handles.VisibilityFigure);
        configureSpatialAxes(handles.VisibilityAxes, result);
        drawObstacles(handles.VisibilityAxes, protectedObstacles, originalObstacles, result.Inputs.initialState.time_s);
        drawSearchDiagnostics(handles.VisibilityAxes, result, options.ShowSearchEdges);
        drawPlannerRoute(handles.VisibilityAxes, result);
        drawTarget(handles.VisibilityAxes, result, result.Inputs.initialState.time_s);
        drawEndpoints(handles.VisibilityAxes, result);
        finishAxes(handles.VisibilityAxes, result, options.Title);
    end
end

%% Section 5: Plot Returned Position And Motion Rates

if options.ShowKinematics && result.Success
    kinematicFigureHandle = figure("Name", options.Title + " kinematics", "Visible", options.FigureVisible);
    kinematicLayoutHandle = tiledlayout( ...
        kinematicFigureHandle, 4, 1, "TileSpacing", "compact", "Padding", "compact");
    kinematicAxesHandles = createKinematicPanels(kinematicLayoutHandle, result, false);
    title(kinematicLayoutHandle, options.Title);
    handles.KinematicFigure = kinematicFigureHandle;
    handles.KinematicAxes   = kinematicAxesHandles;
end

%% Section 6: Animate Returned Motion

if (options.ShowAnimation || options.SaveAnimationGif) && result.Success
    animationVisibility = options.FigureVisible;
    % Make the figure visible while getframe captures the GIF frames, then
    % hide it afterward if FigureVisible was set to "off".
    if options.SaveAnimationGif
        animationVisibility = "on";
    end
    animationFigureHandle = figure("Name", options.Title + " animation", "Visible", animationVisibility);
    animationLayoutHandle = tiledlayout( ...
        animationFigureHandle, 4, 2, "TileSpacing", "compact", "Padding", "compact");
    animationAxesHandle   = nexttile(animationLayoutHandle, 1, [4 1]);
    kinematicAxesHandles  = createKinematicPanels(animationLayoutHandle, result, true);
    animationLegendHandle = legend(kinematicAxesHandles(1), "Location", "best");

    % Always include the final sample, even when FrameStride skips over it.
    frameIndices = unique([1:options.FrameStride:numel(result.time_s), numel(result.time_s)]);
    [completeDisplayPath_units, sourceSampleIndices] = createDisplayPath(result, result.position_units);
    gifFrameCount = 0;
    for frameIndex = frameIndices
        cla(animationAxesHandle);
        configureSpatialAxes(animationAxesHandle, result);
        drawObstacles(animationAxesHandle, protectedObstacles, originalObstacles, result.time_s(frameIndex));
        drawTarget(animationAxesHandle, result, result.time_s(frameIndex));
        drawLine(animationAxesHandle, completeDisplayPath_units, "-", "Complete timed path", 1);

        % Wrapped display paths contain extra seam points. Use their source
        % sample indices to show only the path reached by this frame.
        elapsedPathEndIndex   = find(sourceSampleIndices <= frameIndex, 1, "last");
        elapsedPath_units     = completeDisplayPath_units(1:elapsedPathEndIndex, :);
        currentPosition_units = createDisplayPath(result, result.position_units(frameIndex, :));
        drawLine(animationAxesHandle, elapsedPath_units, "c-", "Elapsed path", 3);
        scatter(animationAxesHandle, currentPosition_units(1), currentPosition_units(2), ...
            60, [0.95 0.25 0.15], "filled", "DisplayName", "Current state");
        xlabel(animationAxesHandle, "X (units)");
        ylabel(animationAxesHandle, "Y (units)");
        title(animationAxesHandle, sprintf("%s | t = %.3f s", options.Title, result.time_s(frameIndex)));
        drawnow
        if options.SaveAnimationGif
            [indexedImage, colorMap] = rgb2ind(frame2im(getframe(animationFigureHandle)), 256);
            % The first frame creates the GIF; later frames append to it.
            if gifFrameCount == 0
                imwrite(indexedImage, colorMap, char(options.AnimationGifFile), ...
                    "gif", "LoopCount", inf, "DelayTime", options.AnimationGifDelay_s);
            else
                imwrite(indexedImage, colorMap, char(options.AnimationGifFile), ...
                    "gif", "WriteMode", "append", "DelayTime", options.AnimationGifDelay_s);
            end
            gifFrameCount = gifFrameCount + 1;
        end
        if options.FigureVisible == "on" && options.Pause_s > 0
            pause(options.Pause_s);
        end
    end
    if options.FigureVisible == "off"
        animationFigureHandle.Visible = "off";
    end
    handles.AnimationFigure        = animationFigureHandle;
    handles.AnimationAxes          = animationAxesHandle;
    handles.AnimationKinematicAxes = kinematicAxesHandles;
    handles.AnimationLegend        = animationLegendHandle;
    if options.SaveAnimationGif
        handles.AnimationGifFile = options.AnimationGifFile;
    end
end

% Axes is the shared output for callers that need one spatial plot handle.
handles.Axes = handles.WorkspaceAxes;
if isempty(handles.Axes)
    handles.Axes = handles.VisibilityAxes;
end
end

%% Section 7: Local Functions

function configureSpatialAxes(axesHandle, result)
    % Use equal x/y scales. Wrapped views stay within the requested bounds.
    hold(axesHandle, "on");
    grid(axesHandle, "on");
    box(axesHandle, "on");
    axis(axesHandle, "equal");
    if result.Options.WrapX || result.Options.WrapY
        xlim(axesHandle, result.RequestedLimits.xInterval_units);
        ylim(axesHandle, result.RequestedLimits.yInterval_units);
    end
end

function [position_units, sourceSampleIndices] = createDisplayPath(result, position_units)
    % Apply the request's display bounds and split lines at wrapped edges.
    % Source sample indices keep the animation aligned with the original times.
    intervals_units = [result.RequestedLimits.xInterval_units; result.RequestedLimits.yInterval_units];
    wrapAxes        = [result.Options.WrapX result.Options.WrapY];
    [position_units, sourceSampleIndices] = obstacleAvoidance.plotting.createWrappedSpatialPath( ...
        position_units, intervals_units, wrapAxes);
end

function [figureHandle, axesHandle] = createContinuousWorkspace(result, options)
    % Show the continuous path, such as 350 -> 370, with a line at 360.
    % This complements the wrapped view, which splits at 360 -> 0.
    figureHandle = figure("Name", options.Title + " continuous coordinates", "Visible", options.FigureVisible);
    axesHandle   = axes(figureHandle);
    hold(axesHandle, "on");
    grid(axesHandle, "on");
    box(axesHandle, "on");
    axis(axesHandle, "equal");
    drawLine(axesHandle, result.position_units, "k-", "Timed motion", 2);
    intervals_units = [result.RequestedLimits.xInterval_units; result.RequestedLimits.yInterval_units];
    wrapAxes        = [result.Options.WrapX result.Options.WrapY];
    for axisIndex = find(wrapAxes)
        interval_units   = intervals_units(axisIndex, :);
        wrapLength_units = diff(interval_units);
        firstWrapCount = ceil((min(result.position_units(:, axisIndex)) - interval_units(1)) / wrapLength_units);
        lastWrapCount  = floor((max(result.position_units(:, axisIndex)) - interval_units(1)) / wrapLength_units);
        seamWrapCounts = firstWrapCount:lastWrapCount;
        for seam_units = interval_units(1) + wrapLength_units * seamWrapCounts
            if axisIndex == 1
                xline(axesHandle, seam_units, ":", "HandleVisibility", "off");
            else
                yline(axesHandle, seam_units, ":", "HandleVisibility", "off");
            end
        end
    end
    xlabel(axesHandle, "Continuous x (units)");
    ylabel(axesHandle, "Continuous y (units)");
end

function drawPlannerRoute(axesHandle, result)
    % Show a saved route or candidate motion even if planning later failed.
    % The plot title retains the failure reason; these lines do not imply success.
    if ~isempty(result.Route_units)
        selectedRoute_units = createDisplayPath(result, result.Route_units);
        drawLine(axesHandle, selectedRoute_units, "--", "Selected geometric route", 1);
    end
    if ~isempty(result.position_units)
        motionPath_units = createDisplayPath(result, result.position_units);
        drawLine(axesHandle, motionPath_units, "k-", "Timed motion", 2);
    end
end

function drawEndpoints(axesHandle, result)
    % Mark the requested start and goal. A successful intercept uses the
    % target position at the actual arrival time, which can be earlier.
    startPosition_units = createDisplayPath(result, result.Inputs.initialState.position_units);
    goalPosition_units  = result.Inputs.goalState.position_units;
    if result.Success && hasData(result, 'Intercept') && isfinite(result.Intercept.Time_s)
        goalPosition_units = result.Intercept.TargetPosition_units;
    end
    goalPosition_units = createDisplayPath(result, goalPosition_units);
    plot(axesHandle, startPosition_units(1), startPosition_units(2), "go", "DisplayName", "Start");
    plot(axesHandle, goalPosition_units(1), goalPosition_units(2), "ro", "DisplayName", "Goal");
end

function lineHandle = drawLine(axesHandle, position_units, lineStyle, displayName, lineWidth)
    % Draw connected x/y points; NaN rows leave gaps between separate pieces.
    lineHandle = plot(axesHandle, position_units(:, 1), position_units(:, 2), ...
        lineStyle, "LineWidth", lineWidth, "DisplayName", displayName);
end

function drawSearchDiagnostics(axesHandle, result, showEdges)
    % Draw the saved search connections and nodes; do not rebuild the graph.
    visibilityGraph     = result.VisibilityGraph;
    nodePositions_units = visibilityGraph.NodePosition_units;
    edgeFieldNames      = ["AcceptedNodeIndex", "RejectedNodeIndex"];
    edgeStyles          = ["-", ":"];
    edgeLegendLabels    = ["Accepted visibility edge", "Collision-rejected edge"];
    if showEdges
        for categoryIndex = 1:2
            if hasData(visibilityGraph, edgeFieldNames(categoryIndex))
                edgeNodeIndices = visibilityGraph.(edgeFieldNames(categoryIndex));
                edgeCount       = size(edgeNodeIndices, 1);

                % Add a NaN after each two-node edge so MATLAB draws
                % separate connections instead of one continuous line.
                x_units = reshape([nodePositions_units(edgeNodeIndices(:, 1), 1), ...
                    nodePositions_units(edgeNodeIndices(:, 2), 1), nan(edgeCount, 1)].', [], 1);
                y_units = reshape([nodePositions_units(edgeNodeIndices(:, 1), 2), ...
                    nodePositions_units(edgeNodeIndices(:, 2), 2), nan(edgeCount, 1)].', [], 1);

                % Split each wrapped edge at the display boundary before
                % joining the edge arrays for plotting.
                if result.Options.WrapX || result.Options.WrapY
                    edgePaths_units = cell(edgeCount, 1);
                    for edgeIndex = 1:edgeCount
                        edgeEndpoints_units = nodePositions_units(edgeNodeIndices(edgeIndex, :), :);
                        edgePaths_units{edgeIndex} = [createDisplayPath(result, edgeEndpoints_units); NaN NaN];
                    end
                    edgePaths_units = vertcat(edgePaths_units{:});
                    x_units         = edgePaths_units(:, 1);
                    y_units         = edgePaths_units(:, 2);
                end
                plot(axesHandle, x_units, y_units, edgeStyles(categoryIndex), ...
                    "DisplayName", edgeLegendLabels(categoryIndex));
            end
        end
    end
    intervals_units = [result.RequestedLimits.xInterval_units; result.RequestedLimits.yInterval_units];
    for axisIndex = find([result.Options.WrapX result.Options.WrapY])
        nodePositions_units(:, axisIndex) = intervals_units(axisIndex, 1) + ...
            mod(nodePositions_units(:, axisIndex) - intervals_units(axisIndex, 1), diff(intervals_units(axisIndex, :)));
    end
    if ~isempty(nodePositions_units)
        scatter(axesHandle, nodePositions_units(:, 1), nodePositions_units(:, 2), ...
            17, "filled", "DisplayName", "Visibility node");
    end
end

function drawObstacles(axesHandle, protectedObstacles, originalObstacles, time_s)
    % Fill the original obstacle and outline its protected shape. Both
    % histories are already prepared for this time; no margin is added here.
    obstacleColors = lines(max(1, numel(protectedObstacles)));
    for obstacleIndex = 1:numel(protectedObstacles)
        obstacle = protectedObstacles(obstacleIndex);
        originalShape = obstacleAvoidance.obstacles.preparedShapeAtTime( ...
            originalObstacles(obstacleIndex), time_s);
        drawShape(axesHandle, originalShape, obstacleColors(obstacleIndex, :), "-", "Original obstacle");

        protectedShape = obstacleAvoidance.obstacles.preparedShapeAtTime(obstacle, time_s);
        drawShape(axesHandle, protectedShape, "none", "--", "Protected obstacle");
    end
end

function drawShape(axesHandle, obstacleShape, faceColor, lineStyle, displayName)
    % Draw one nonempty polygon with the requested face and line styling.
    if ~isempty(obstacleShape.Vertices)
        plot(axesHandle, obstacleShape, "FaceColor", faceColor, "FaceAlpha", 0.18, ...
            "LineStyle", lineStyle, "LineWidth", 1.2, "DisplayName", displayName);
    end
end

function drawTarget(axesHandle, result, displayTime_s)
    % Draw the moving target's track and current position.
    goalState = result.Inputs.goalState;
    if ~hasData(goalState, "targetMotion")
        return
    end
    targetMotion = goalState.targetMotion;

    % Include every supplied target time plus display samples between them.
    targetSampleTimes_s = unique([targetMotion.time_s(:); ...
        linspace(targetMotion.time_s(1), targetMotion.time_s(end), 200).']);
    targetPath_units = createDisplayPath( ...
        result, obstacleAvoidance.input.targetPositionAtTime(targetMotion, targetSampleTimes_s));
    drawLine(axesHandle, targetPath_units, "-.", "Moving target track", 1);

    % Keep the full track when the display time is outside the target
    % history, but omit the current-position marker for that time.
    if displayTime_s < targetMotion.time_s(1) || displayTime_s > targetMotion.time_s(end)
        return
    end
    targetPosition_units = obstacleAvoidance.input.targetPositionAtTime(targetMotion, displayTime_s);
    targetPosition_units = createDisplayPath(result, targetPosition_units);
    plot(axesHandle, targetPosition_units(1), targetPosition_units(2), ...
        "md", "MarkerFaceColor", "m", "DisplayName", "Moving target");
end

function axesHandles = createKinematicPanels(layoutHandle, result, useAnimationLayout)
    % Plot the complete x/y histories. Velocity, acceleration, and jerk
    % each show positive and negative limits. These panels do not draw
    % position bounds.
    quantityFieldNames = ["position_units", "velocity_units_s", "acceleration_units_s2", "jerk_units_s3"];
    quantityAxisLabels = ["Position (units)", "Velocity (units/s)", "Acceleration (units/s^2)", "Jerk (units/s^3)"];
    quantityLimits     = [nan(1, 2); result.Limits.maxVelocity_units_s; ...
        result.Limits.maxAcceleration_units_s2; result.Limits.maxJerk_units_s3];
    axesHandles = gobjects(4, 1);
    for quantityIndex = 1:4
        % In the animation layout, the spatial plot fills the left column;
        % the four motion panels occupy tiles 2, 4, 6, and 8 on the right.
        tileIndex = quantityIndex * (1 + useAnimationLayout);
        axesHandles(quantityIndex) = nexttile(layoutHandle, tileIndex);
        axesHandle = axesHandles(quantityIndex);
        hold(axesHandle, "on");
        grid(axesHandle, "on");
        box(axesHandle, "on");
        quantityValues = result.(quantityFieldNames(quantityIndex));
        lineHandles    = plot(axesHandle, result.time_s, quantityValues);
        set(lineHandles, {'DisplayName'}, {'X'; 'Y'});
        if quantityIndex > 1
            yline(axesHandle, [-quantityLimits(quantityIndex, :), quantityLimits(quantityIndex, :)], ...
                "r--", "HandleVisibility", "off");
        end
        ylabel(axesHandle, quantityAxisLabels(quantityIndex));
        xlim(axesHandle, result.time_s([1 end]));
    end
    xlabel(axesHandles(end), "Time (s)");
    legend(axesHandles(1), "Location", "best");
end

function finishAxes(axesHandle, result, titlePrefix)
    % Label the plot with the planner's actual termination reason.
    xlabel(axesHandle, "X (units)");
    ylabel(axesHandle, "Y (units)");
    title(axesHandle, sprintf("%s | %s", titlePrefix, result.TerminationReason));
    legend(axesHandle, "Location", "best");
end

function hasValue = hasData(record, fieldName)
    % Report whether the named field exists and contains data.
    hasValue = isfield(record, fieldName) && ~isempty(record.(fieldName));
end

function handles = createEmptyHandles(options)
    % Keep the same output fields even when some plots are disabled.
    % Empty handles mean that the corresponding figure or axes was not created.
    emptyHandles = gobjects(0);
    handles = struct( ...
        "WorkspaceFigure",           emptyHandles, ...
        "WorkspaceAxes",             emptyHandles, ...
        "ContinuousWorkspaceFigure", emptyHandles, ...
        "ContinuousWorkspaceAxes",   emptyHandles, ...
        "VisibilityFigure",          emptyHandles, ...
        "VisibilityAxes",            emptyHandles, ...
        "KinematicFigure",           emptyHandles, ...
        "KinematicAxes",             emptyHandles, ...
        "AnimationFigure",           emptyHandles, ...
        "AnimationAxes",             emptyHandles, ...
        "AnimationKinematicAxes",    emptyHandles, ...
        "AnimationLegend",           emptyHandles, ...
        "AnimationGifFile",          "", ...
        "Options",                   options, ...
        "Axes",                      emptyHandles);
end
