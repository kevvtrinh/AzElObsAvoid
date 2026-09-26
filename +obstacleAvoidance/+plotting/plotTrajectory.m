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
%   - Show protected obstacles and returned motion in x/y/time space.
%   - Animate returned samples against time-varying obstacles and targets.
%   - When wrapping is on, show the expanded workspace: every obstacle and
%     goal copy inside the planning range, in the unwrapped coordinates the
%     planner used.
%   - Optionally show an azimuth/elevation path and a protected obstacle
%     snapshot on a unit sphere, including paths that cross a pole.
%**************************************************************************
% INPUTS
%   - result (scalar planner result)
%       Successful or failed planner result with the available geometry,
%       route, and motion data.
%   - optionOverrides (scalar struct or Cartesian axes handle, optional)
%       Display, animation, and GIF controls. ShowSpaceTime opens the x/y/time
%       view; ShowSweptSurfaces draws translucent motion walls where input
%       vertices correspond and protected interval envelopes otherwise.
%       MaximumDisplayedTimeSlices caps outline snapshots per obstacle. The graph
%       display caps sample nodes in 3D and edges in both 2D and 3D views.
%       FastRotationPreview hides filled sweeps only while the 3D view rotates;
%       the complete surfaces return when the drag ends.
%       ShowExpandedWorkspace opens the expanded workspace when wrapping is
%       on. ShowSphere maps degree-valued azimuth/elevation to a unit sphere;
%       elevation must lie within [-90 90]. With WrapY on, the intervals must
%       be one 360-degree azimuth turn and elevation [-90 90]. SphereTime_s
%       chooses the obstacle snapshot; NaN uses the start time. An axes handle
%       requests only the workspace plot. Hidden figures never pause.
%**************************************************************************
% OUTPUTS
%   - handles (scalar struct)
%       Figure and axes handles for the requested plots. Unused fields
%       contain empty handles. SpaceTimeAxes selects the x/y/time view and
%       ExpandedWorkspaceAxes the expanded workspace; SphereAxes selects the
%       azimuth/elevation sphere. Invalid input throws an error.
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

defaults.ShowWorkspace              = true;
defaults.ShowKinematics             = true;
defaults.ShowAnimation              = true;
defaults.ShowSearchEdges            = true;
defaults.ShowVisibilityGraphs       = true;
defaults.ShowSpaceTime              = true;
defaults.ShowSweptSurfaces          = true;
defaults.ShowExpandedWorkspace      = true;
defaults.ShowSphere                 = false;
defaults.SphereTime_s               = NaN;
defaults.MaximumDisplayedTimeSlices = 5;
defaults.MaximumDisplayedGraphNodes = 300;
defaults.MaximumDisplayedGraphEdges = 40;
defaults.FastRotationPreview         = true;

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
requiredResultFieldNames = {'Inputs', 'Options', 'Success', 'TerminationReason', ...
    'time_s', 'position_units', 'velocity_units_s', 'acceleration_units_s2', ...
    'jerk_units_s3', 'Diagnostics'};
requiredDiagnosticFieldNames = {'Limits', 'RequestedLimits', ...
    'PreparedObstacles', 'VisibilityGraph', 'Route_units'};
if ~isstruct(result) || ~isscalar(result) || ~all(isfield(result, requiredResultFieldNames)) || ...
        ~isstruct(result.Diagnostics) || ~isscalar(result.Diagnostics) || ...
        ~all(isfield(result.Diagnostics, requiredDiagnosticFieldNames))
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
    optionOverrides     = struct('ShowKinematics', false, 'ShowAnimation', false, ...
        'ShowSpaceTime', false, 'ShowExpandedWorkspace', false);
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
    "ShowSearchEdges", "ShowVisibilityGraphs", "ShowSpaceTime", ...
    "ShowSweptSurfaces", "ShowExpandedWorkspace", "ShowSphere", ...
    "FastRotationPreview", "SaveAnimationGif"];
for optionName = logicalOptionNames
    options.(optionName) = obstacleAvoidance.input.normalizeLogicalScalar( ...
        options.(optionName), optionName, "plotTrajectory:InvalidLogicalOption");
end
nonnegativeOptionNames = ["Pause_s", "AnimationGifDelay_s"];
for optionName = nonnegativeOptionNames
    validateattributes(options.(optionName), {'numeric'}, {'real', 'finite', 'scalar', 'nonnegative'});
end
validateattributes(options.FrameStride, {'numeric'}, {'real', 'finite', 'scalar', 'integer', 'positive'});
validateattributes(options.SphereTime_s, {'numeric'}, {'real', 'scalar'});
if ~isnan(options.SphereTime_s) && ~isfinite(options.SphereTime_s)
    error("plotTrajectory:InvalidSphereTime", "SphereTime_s must be finite or NaN.");
end
graphDisplayCountNames = ["MaximumDisplayedTimeSlices", ...
    "MaximumDisplayedGraphNodes", "MaximumDisplayedGraphEdges"];
for optionName = graphDisplayCountNames
    validateattributes(options.(optionName), {'numeric'}, ...
        {'real', 'finite', 'scalar', 'integer', 'positive'});
end
handles = createEmptyHandles(options);

% Wrap options are "false", "both", "forward", or "backward". Results saved
% before the modes existed store true or false.
wrapModes    = readWrapModes(result.Options);
wrapsAnyAxis = any(wrapModes ~= "false");
if options.ShowSphere && ~useSuppliedAxes
    xInterval_units = result.Diagnostics.RequestedLimits.xInterval_units;
    yInterval_units = result.Diagnostics.RequestedLimits.yInterval_units;
    azimuthIsFullTurn = abs(diff(xInterval_units) - 360) <= 1e-8;
    elevationIsPhysical = yInterval_units(1) >= -90 - 1e-8 && ...
        yInterval_units(2) <= 90 + 1e-8;
    poleWrapIsPhysical = azimuthIsFullTurn && all(abs(yInterval_units - [-90 90]) <= 1e-8);
    if ~elevationIsPhysical || (wrapModes(1) ~= "false" && ~azimuthIsFullTurn) || ...
            (wrapModes(2) ~= "false" && ~poleWrapIsPhysical)
        error("plotTrajectory:SphereRequiresAzEl", ...
            "ShowSphere needs elevation within [-90 90]; wrapped x needs a 360-degree interval, " + ...
            "and wrapped y needs x spanning 360 degrees and y interval [-90 90].");
    end
end

%% Section 2: Prepare Obstacles For The Displayed Time Range

% A failed arrival trial can end before the requested goal time. For example,
% a trial at 12 s in a request ending at 20 s should still display through 20 s.
plotEndTime_s = result.Inputs.goalState.time_s;
% Use the parent request's goal time when it extends that display range.
if ~result.Success && isfield(result.Diagnostics, 'ParentRequest')
    parentGoalTime_s = result.Diagnostics.ParentRequest.GoalTime_s;
    if isnumeric(parentGoalTime_s) && isscalar(parentGoalTime_s) && ...
            isreal(parentGoalTime_s) && isfinite(parentGoalTime_s)
        plotEndTime_s = max(plotEndTime_s, double(parentGoalTime_s));
    end
end
plotTimeRange_s    = [result.Inputs.initialState.time_s, plotEndTime_s];
if options.ShowSphere && ~useSuppliedAxes && isfinite(options.SphereTime_s) && ...
        (options.SphereTime_s < plotTimeRange_s(1) || options.SphereTime_s > plotTimeRange_s(2))
    error("plotTrajectory:InvalidSphereTime", "SphereTime_s must be within the plotted time range.");
end
planningObstacles  = obstacleAvoidance.obstacles.prepareObstacles(result.Diagnostics.PreparedObstacles, plotTimeRange_s);
protectedObstacles = planningObstacles;
requestedIntervals_units = [result.Diagnostics.RequestedLimits.xInterval_units; result.Diagnostics.RequestedLimits.yInterval_units];
if wrapsAnyAxis
    % The 2D views fold motion into the requested interval. Prepare each
    % source obstacle once; draw its copies from the shape at the displayed
    % time below. A display copy does not need another moving-motion model.
    % Space-time and expanded views keep the planner's unwrapped copies.
    protectedObstacles = obstacleAvoidance.obstacles.prepareObstacles( ...
        result.Inputs.obstacles, plotTimeRange_s);
end
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
        xlim(workspaceAxesHandle, result.Diagnostics.RequestedLimits.xInterval_units);
        ylim(workspaceAxesHandle, result.Diagnostics.RequestedLimits.yInterval_units);
    end
    drawObstacles(workspaceAxesHandle, protectedObstacles, originalObstacles, ...
        result.Inputs.initialState.time_s, requestedIntervals_units, wrapModes);
    if options.ShowVisibilityGraphs
        drawSearchDiagnostics(workspaceAxesHandle, result, options);
    end
    drawPlannerRoute(workspaceAxesHandle, result);
    drawTarget(workspaceAxesHandle, result, result.Inputs.initialState.time_s);
    drawEndpoints(workspaceAxesHandle, result);
    legendLocation = "bestoutside";
    if useSuppliedAxes
        legendLocation = "best";
    end
    finishAxes(workspaceAxesHandle, result, options.Title, legendLocation);
    handles.WorkspaceFigure = workspaceFigureHandle;
    handles.WorkspaceAxes   = workspaceAxesHandle;
end

% A wrapped path can look disconnected at the plot boundary. Also show its
% continuous coordinates so the full movement remains easy to understand.
pathCrossesSeam = false;
if result.Success && wrapsAnyAxis
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
        drawObstacles(handles.VisibilityAxes, protectedObstacles, originalObstacles, ...
            result.Inputs.initialState.time_s, requestedIntervals_units, wrapModes);
        drawSearchDiagnostics(handles.VisibilityAxes, result, options);
        drawPlannerRoute(handles.VisibilityAxes, result);
        drawTarget(handles.VisibilityAxes, result, result.Inputs.initialState.time_s);
        drawEndpoints(handles.VisibilityAxes, result);
        finishAxes(handles.VisibilityAxes, result, options.Title, "bestoutside");
    end
end

%% Section 5: Plot Protected Obstacles And Motion In Space-Time

if options.ShowSpaceTime
    spaceTimeFigureHandle = figure("Name", options.Title + " space-time", ...
        "Visible", options.FigureVisible);
    spaceTimeAxesHandle = axes(spaceTimeFigureHandle);
    hold(spaceTimeAxesHandle, "on");
    grid(spaceTimeAxesHandle, "on");
    box(spaceTimeAxesHandle, "on");

    % The obstacle and motion histories supply actual time coordinates.
    % Spatial graph edges belong to one snapshot, not every time layer.
    % Everything here uses the planner's unwrapped coordinates, so wrapped
    % obstacle copies and the motion they constrain stay lined up.
    drawSpaceTimeObstacles(spaceTimeAxesHandle, planningObstacles, ...
        plotTimeRange_s, options);
    if options.ShowVisibilityGraphs
        drawSpaceTimeSearch(spaceTimeAxesHandle, result, options);
    end
    if ~isempty(result.time_s) && size(result.position_units, 1) == numel(result.time_s)
        drawSpaceTimePath(spaceTimeAxesHandle, result.position_units, ...
            result.time_s, "k-", "Returned motion", 2.2);
    end
    drawSpaceTimeTarget(spaceTimeAxesHandle, result, plotTimeRange_s);

    startPosition_units = result.Inputs.initialState.position_units;
    plot3(spaceTimeAxesHandle, startPosition_units(1), startPosition_units(2), ...
        plotTimeRange_s(1), "go", "MarkerFaceColor", "g", "DisplayName", "Start");
    goalPosition_units = result.Inputs.goalState.position_units;
    plot3(spaceTimeAxesHandle, repmat(goalPosition_units(1), 1, 2), ...
        repmat(goalPosition_units(2), 1, 2), plotTimeRange_s, ...
        "r--", "LineWidth", 1.3, "DisplayName", "Requested goal position");

    % The requested horizon is only a deadline for earliest arrival. Place
    % the red dot at the last returned motion sample instead of that horizon.
    if ~isempty(result.time_s) && size(result.position_units, 1) == numel(result.time_s)
        terminalPosition_units = result.position_units(end, :);
        terminalLabel = "Returned arrival";
        if ~result.Success
            terminalLabel = "Last returned sample";
        end
        plot3(spaceTimeAxesHandle, terminalPosition_units(1), terminalPosition_units(2), ...
            result.time_s(end), "ro", "MarkerFaceColor", "r", ...
            "MarkerSize", 8, "DisplayName", terminalLabel);
    end
    spatialLabelPrefix = "";
    if wrapsAnyAxis
        spatialLabelPrefix = "Unwrapped ";
    end
    xlabel(spaceTimeAxesHandle, spatialLabelPrefix + "X (units)");
    ylabel(spaceTimeAxesHandle, spatialLabelPrefix + "Y (units)");
    zlabel(spaceTimeAxesHandle, "Time (s)");
    title(spaceTimeAxesHandle, options.Title + " | " + string(result.TerminationReason));
    view(spaceTimeAxesHandle, 3);
    if diff(plotTimeRange_s) > 0
        zlim(spaceTimeAxesHandle, plotTimeRange_s);
    end
    legend(spaceTimeAxesHandle, "Location", "best");
    if options.FastRotationPreview
        sweepSurfaceHandles = findall(spaceTimeAxesHandle, ...
            "Type", "surface", "Tag", "SpaceTimeObstacleSweep");
        if ~isempty(sweepSurfaceHandles)
            % Transparency is costly to redraw on every mouse movement.
            % Keep the outlines, rails, and path visible during rotation,
            % then restore the complete sweep when the drag ends.
            rotationMode = rotate3d(spaceTimeFigureHandle);
            rotationMode.ActionPreCallback = @(~, ~) ...
                setSweepVisibility(sweepSurfaceHandles, "off");
            rotationMode.ActionPostCallback = @(~, ~) ...
                setSweepVisibility(sweepSurfaceHandles, "on");
            rotationMode.Enable = "on";
        end
    end
    handles.SpaceTimeFigure = spaceTimeFigureHandle;
    handles.SpaceTimeAxes   = spaceTimeAxesHandle;
end

%% Section 6: Plot The Expanded Workspace With Every Wrapped Copy

% The planner works in unwrapped coordinates over the planning range and
% copies obstacles and the goal across the wrapped ends. Show that frame
% directly, so each wrap option's copies and range limits are visible.
if options.ShowExpandedWorkspace && wrapsAnyAxis && ~useSuppliedAxes
    [handles.ExpandedWorkspaceFigure, handles.ExpandedWorkspaceAxes] = ...
        createExpandedWorkspace(result, wrapModes, plotTimeRange_s, options);
end

%% Section 7: Plot Path And An Obstacle Snapshot On A Sphere

if options.ShowSphere && ~useSuppliedAxes
    sphereTime_s = options.SphereTime_s;
    if isnan(sphereTime_s)
        sphereTime_s = result.Inputs.initialState.time_s;
    end
    obstacleShapes = cell(numel(protectedObstacles), 1);
    for obstacleIndex = 1:numel(protectedObstacles)
        obstacle = protectedObstacles(obstacleIndex);
        if shapeIsAvailable(obstacle, sphereTime_s)
            obstacleShapes{obstacleIndex} = ...
                obstacleAvoidance.obstacles.preparedShapeAtTime(obstacle, sphereTime_s);
        end
    end
    [handles.SphereFigure, handles.SphereAxes] = ...
        obstacleAvoidance.plotting.createSphereView(result, obstacleShapes, sphereTime_s, options);
end

%% Section 8: Plot Returned Position And Motion Rates

if options.ShowKinematics && result.Success
    kinematicFigureHandle = figure("Name", options.Title + " kinematics", "Visible", options.FigureVisible);
    kinematicLayoutHandle = tiledlayout( ...
        kinematicFigureHandle, 4, 1, "TileSpacing", "compact", "Padding", "compact");
    kinematicAxesHandles = createKinematicPanels(kinematicLayoutHandle, result, false);
    title(kinematicLayoutHandle, options.Title);
    handles.KinematicFigure = kinematicFigureHandle;
    handles.KinematicAxes   = kinematicAxesHandles;
end

%% Section 9: Animate Returned Motion

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
        drawObstacles(animationAxesHandle, protectedObstacles, originalObstacles, ...
            result.time_s(frameIndex), requestedIntervals_units, wrapModes);
        drawTarget(animationAxesHandle, result, result.time_s(frameIndex));
        drawLine(animationAxesHandle, completeDisplayPath_units, "-", "Complete timed path", 1);

        % Wrapped display paths contain extra seam points. Use their source
        % sample indices to show only the path reached by this frame.
        elapsedPathEndIndex   = find(sourceSampleIndices <= frameIndex, 1, "last");
        elapsedPath_units     = completeDisplayPath_units(1:elapsedPathEndIndex, :);
        currentPosition_units = completeDisplayPath_units(elapsedPathEndIndex, :);
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
if isempty(handles.Axes)
    handles.Axes = handles.SpaceTimeAxes;
end
if isempty(handles.Axes)
    handles.Axes = handles.SphereAxes;
end
end

%% Section 10: Local Functions

function wrapModes = readWrapModes(options)
    % Read [WrapX WrapY] as wrap modes, accepting only what the planner
    % accepts: a scalar true/false (or 1/0) meaning "both"/"false", or one of
    % the four text modes. Anything else is refused rather than drawn under
    % a different wrap rule.
    wrapValues = {options.WrapX, options.WrapY};
    wrapModes  = ["false", "false"];
    for axisIndex = 1:2
        wrapValue = wrapValues{axisIndex};
        if (islogical(wrapValue) || isnumeric(wrapValue)) && isscalar(wrapValue) && any(wrapValue == [0, 1])
            if wrapValue
                wrapModes(axisIndex) = "both";
            end
        elseif ((isstring(wrapValue) && isscalar(wrapValue)) || (ischar(wrapValue) && isrow(wrapValue))) && ...
                any(lower(string(wrapValue)) == ["false", "both", "forward", "backward"])
            wrapModes(axisIndex) = lower(string(wrapValue));
        else
            error("plotTrajectory:InvalidWrapOption", ...
                "WrapX and WrapY must be false, both, forward, or backward.");
        end
    end
end

function configureSpatialAxes(axesHandle, result)
    % Use equal x/y scales. Wrapped views stay within the requested bounds.
    hold(axesHandle, "on");
    grid(axesHandle, "on");
    box(axesHandle, "on");
    axis(axesHandle, "equal");
    if any(readWrapModes(result.Options) ~= "false")
        xlim(axesHandle, result.Diagnostics.RequestedLimits.xInterval_units);
        ylim(axesHandle, result.Diagnostics.RequestedLimits.yInterval_units);
    end
end

function [position_units, sourceSampleIndices] = createDisplayPath(result, position_units)
    % Apply the request's display bounds and split lines at wrapped edges.
    % Source sample indices keep the animation aligned with the original times.
    % A point over a y end (a pole) is folded back by the planner's pole
    % copy rule: y mirrored about that end and x turned by half a turn.
    intervals_units = [result.Diagnostics.RequestedLimits.xInterval_units; result.Diagnostics.RequestedLimits.yInterval_units];
    wrapAxes        = readWrapModes(result.Options) ~= "false";
    [position_units, sourceSampleIndices] = obstacleAvoidance.plotting.createWrappedSpatialPath( ...
        position_units, intervals_units, wrapAxes);
end

function nodePositions_units = foldNodes(result, nodePositions_units, edgeNodeIndices)
    % Show each node inside the display intervals with the same fold the
    % path display uses. A node exactly on a boundary, such as x = 360 with
    % y wrapping, cannot choose a side alone; it takes the side its first
    % incident edge is drawn on, so node and edge stay joined. A node
    % without edges is folded on its own.
    if all(readWrapModes(result.Options) == "false")
        return
    end
    % One pass over the edge rows, in drawing order, records each node's
    % first neighbor; 0 means the node has no edge.
    nodeCount     = size(nodePositions_units, 1);
    firstNeighbor = zeros(nodeCount, 1);
    firstEdgeRow  = inf(nodeCount, 1);
    for edgeColumn = 1:2
        [nodeIndices, edgeRows] = unique(edgeNodeIndices(:, edgeColumn), 'first');
        isEarlier = edgeRows < firstEdgeRow(nodeIndices);
        firstEdgeRow(nodeIndices(isEarlier))  = edgeRows(isEarlier);
        firstNeighbor(nodeIndices(isEarlier)) = edgeNodeIndices(edgeRows(isEarlier), 3 - edgeColumn);
    end
    unfoldedPositions_units = nodePositions_units;
    for nodeIndex = 1:nodeCount
        if firstNeighbor(nodeIndex) == 0
            nodePositions_units(nodeIndex, :) = createDisplayPath(result, unfoldedPositions_units(nodeIndex, :));
        else
            edgePath_units = createDisplayPath(result, ...
                unfoldedPositions_units([firstNeighbor(nodeIndex), nodeIndex], :));
            nodePositions_units(nodeIndex, :) = edgePath_units(end, :);
        end
    end
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
    intervals_units = [result.Diagnostics.RequestedLimits.xInterval_units; result.Diagnostics.RequestedLimits.yInterval_units];
    wrapAxes        = readWrapModes(result.Options) ~= "false";
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

function [figureHandle, axesHandle] = createExpandedWorkspace(result, wrapModes, plotTimeRange_s, options)
    % Draw the unwrapped planning frame: the requested interval, the planning
    % range for this wrap option, every interval end inside it, and every
    % obstacle and goal copy the planner and validator use. Copies come from
    % the supplied obstacles through the same copy rule, not from a cache.
    % Obstacles are drawn at the start time.
    figureHandle = figure("Name", options.Title + " expanded workspace", "Visible", options.FigureVisible);
    axesHandle   = axes(figureHandle);
    hold(axesHandle, "on");
    grid(axesHandle, "on");
    box(axesHandle, "on");
    axis(axesHandle, "equal");

    requestedIntervals_units = [result.Diagnostics.RequestedLimits.xInterval_units; result.Diagnostics.RequestedLimits.yInterval_units];
    planningRange_units      = [result.Diagnostics.Limits.xInterval_units; result.Diagnostics.Limits.yInterval_units];
    turnLength_units         = diff(requestedIntervals_units(1, :));
    height_units             = diff(requestedIntervals_units(2, :));
    startTime_s              = result.Inputs.initialState.time_s;

    % The planning range is where the vehicle may go. "forward" and
    % "backward" stop it at one interval end.
    drawRectangle(axesHandle, planningRange_units, [0.55 0.55 0.55], "-", 1.4, ...
        sprintf("Planning range (WrapX = %s, WrapY = %s)", wrapModes(1), wrapModes(2)));
    drawRectangle(axesHandle, requestedIntervals_units, [0.10 0.45 0.10], "-", 2, "Requested interval");

    % Interval ends inside the planning range. On a wrapped y axis each end
    % is a pole: a copy across it is mirrored and turned half a turn in x.
    if wrapModes(1) ~= "false"
        xEnds_units = requestedIntervals_units(1, 1) + turnLength_units * ...
            (ceil((planningRange_units(1, 1) - requestedIntervals_units(1, 1)) / turnLength_units): ...
            floor((planningRange_units(1, 2) - requestedIntervals_units(1, 1)) / turnLength_units));
        for xEnd_units = xEnds_units
            xline(axesHandle, xEnd_units, ":", "Color", [0.3 0.3 0.3], "HandleVisibility", "off");
        end
    end
    if wrapModes(2) ~= "false"
        poleLines_units = requestedIntervals_units(2, 1) + height_units * ...
            (ceil((planningRange_units(2, 1) - requestedIntervals_units(2, 1)) / height_units): ...
            floor((planningRange_units(2, 2) - requestedIntervals_units(2, 1)) / height_units));
        for poleIndex = 1:numel(poleLines_units)
            legendVisibility = "off";
            if poleIndex == 1
                legendVisibility = "on";
            end
            yline(axesHandle, poleLines_units(poleIndex), "-.", "Color", [0.55 0.20 0.65], ...
                "DisplayName", "Pole (y interval end)", "HandleVisibility", legendVisibility);
        end
    end

    % Obstacle copies: the original is filled, a shifted copy is outlined,
    % and a pole copy is outlined with dots. The copier records each
    % transform after normalizing the source, so the style follows the copy
    % even when normalization removes a ring or a repeated closing vertex.
    obstacles      = obstacleAvoidance.obstacles.canonicalizeObstacles(result.Inputs.obstacles);
    obstacleColors = lines(max(1, numel(obstacles)));
    legendShown    = false(1, 3);
    copyLabels     = ["Obstacle", "Shifted obstacle copy", "Pole obstacle copy"];
    for obstacleIndex = 1:numel(obstacles)
        copies = obstacleAvoidance.input.copyObstaclesAcrossWraps( ...
            obstacles(obstacleIndex), requestedIntervals_units, wrapModes, planningRange_units, plotTimeRange_s);
        copies = obstacleAvoidance.obstacles.prepareObstacles(copies, plotTimeRange_s);
        for copyIndex = 1:numel(copies)
            copyTransform = copies(copyIndex).WrapTransform;
            copyKind = 1;
            if copyTransform(2) == -1
                copyKind = 3;
            elseif copyTransform(1) ~= 0 || copyTransform(3) ~= 0
                copyKind = 2;
            end
            if ~shapeIsAvailable(copies(copyIndex), startTime_s)
                continue
            end
            copyShape = obstacleAvoidance.obstacles.preparedShapeAtTime(copies(copyIndex), startTime_s);
            if isempty(copyShape.Vertices)
                continue
            end
            faceAlpha = 0.30 * (copyKind == 1) + 0.08 * (copyKind ~= 1);
            lineStyles = ["-", "--", ":"];
            legendVisibility = "off";
            if ~legendShown(copyKind)
                legendVisibility = "on";
                legendShown(copyKind) = true;
            end
            plot(axesHandle, copyShape, "FaceColor", obstacleColors(obstacleIndex, :), ...
                "FaceAlpha", faceAlpha, "EdgeColor", obstacleColors(obstacleIndex, :), ...
                "LineStyle", lineStyles(copyKind), "LineWidth", 1.2, ...
                "DisplayName", copyLabels(copyKind), "HandleVisibility", legendVisibility);
        end
    end

    % Goal copies inside the planning range. The planner tries these
    % nearest first; the filled red marker is the copy it planned to.
    requestedGoalState = result.Inputs.goalState;
    if isfield(result.Diagnostics, 'RequestedGoalState')
        requestedGoalState = result.Diagnostics.RequestedGoalState;
    end
    goalHasTarget = isfield(requestedGoalState, 'targetMotion') && ~isempty(requestedGoalState.targetMotion);
    if ~goalHasTarget
        goal_units = double(requestedGoalState.position_units);
        goalImages = obstacleAvoidance.input.listWrapImages([goal_units(:), goal_units(:)], ...
            requestedIntervals_units, wrapModes, planningRange_units);
        goalCopies_units = [goal_units(1) + goalImages.XOffset_units, ...
            goalImages.YScale * goal_units(2) + goalImages.YOffset_units];
        isPoleCopy = goalImages.YScale == -1;
        if any(~isPoleCopy)
            plot(axesHandle, goalCopies_units(~isPoleCopy, 1), goalCopies_units(~isPoleCopy, 2), "rx", ...
                "MarkerSize", 9, "LineWidth", 1.5, "DisplayName", "Goal copy");
        end
        if any(isPoleCopy)
            plot(axesHandle, goalCopies_units(isPoleCopy, 1), goalCopies_units(isPoleCopy, 2), "r+", ...
                "MarkerSize", 10, "LineWidth", 1.5, "DisplayName", "Goal pole copy");
        end
    end
    if hasData(result.Inputs.goalState, 'targetMotion')
        % Draw the path the planner follows, including PCHIP curves between
        % samples, and list its copies as the planner does: for an earliest
        % arrival, every copy that enters the planning range between the
        % start time and the deadline; for a fixed arrival, every copy whose
        % deadline position is in it. The solid path is the one it planned to.
        targetMotion      = result.Inputs.goalState.targetMotion;
        targetTimes_s     = double(targetMotion.time_s(:));
        % Draw the target over the same window its copies are listed for:
        % the start time to the deadline, within its recorded samples.
        drawWindow_s      = [max(startTime_s, targetTimes_s(1)), min(result.Inputs.goalState.time_s, targetTimes_s(end))];
        targetDrawTimes_s = unique([targetTimes_s(targetTimes_s >= drawWindow_s(1) & targetTimes_s <= drawWindow_s(2)); ...
            linspace(drawWindow_s(1), max(drawWindow_s), 200).']);
        targetPath_units  = obstacleAvoidance.input.targetPositionAtTime(targetMotion, targetDrawTimes_s);
        windowTimes_s     = unique([startTime_s; result.Inputs.goalState.time_s; targetTimes_s( ...
            targetTimes_s > startTime_s & targetTimes_s < result.Inputs.goalState.time_s)]);
        windowTimes_s     = windowTimes_s(windowTimes_s >= targetTimes_s(1) & windowTimes_s <= targetTimes_s(end));
        if string(result.Options.GoalTimeMode) == "fixedArrival"
            windowTimes_s = result.Inputs.goalState.time_s;
        end
        windowTarget_units = obstacleAvoidance.input.targetPositionAtTime(targetMotion, windowTimes_s);
        targetImages = obstacleAvoidance.input.listWrapImages( ...
            [min(windowTarget_units, [], 1).', max(windowTarget_units, [], 1).'], ...
            requestedIntervals_units, wrapModes, planningRange_units);
        isOtherCopy = targetImages.XOffset_units ~= 0 | targetImages.YScale ~= 1 | targetImages.YOffset_units ~= 0;
        for copyIndex = reshape(find(isOtherCopy), 1, [])
            legendVisibility = "off";
            if copyIndex == find(isOtherCopy, 1)
                legendVisibility = "on";
            end
            plot(axesHandle, targetPath_units(:, 1) + targetImages.XOffset_units(copyIndex), ...
                targetImages.YScale(copyIndex) * targetPath_units(:, 2) + targetImages.YOffset_units(copyIndex), ...
                "m:", "LineWidth", 1, "DisplayName", "Moving target copy", "HandleVisibility", legendVisibility);
        end
        drawLine(axesHandle, targetPath_units, "m-.", "Moving target (unwrapped)", 1.2);
    end

    % The returned motion and route are already in these coordinates.
    if ~isempty(result.Diagnostics.Route_units)
        routeHandle = drawLine(axesHandle, result.Diagnostics.Route_units, ...
            "--", "Selected geometric route", 1);
        routeHandle.Color = [0.43 0.64 0.13];
    end
    if ~isempty(result.position_units)
        drawLine(axesHandle, result.position_units, "k-", "Timed motion", 2);
    end
    plot(axesHandle, result.Inputs.initialState.position_units(1), ...
        result.Inputs.initialState.position_units(2), "go", "MarkerFaceColor", "g", "DisplayName", "Start");
    hasPlannedGoalCopy = ~isfield(result.Diagnostics, 'WrappedGoalCopies') || ...
        any(result.Diagnostics.WrappedGoalCopies.CandidatePlanned);
    if hasPlannedGoalCopy
        plot(axesHandle, result.Inputs.goalState.position_units(1), result.Inputs.goalState.position_units(2), ...
            "ro", "MarkerFaceColor", "r", "MarkerSize", 8, "DisplayName", "Planned goal copy");
    end

    padding_units = 0.03 * max(diff(planningRange_units, 1, 2));
    xlim(axesHandle, planningRange_units(1, :) + [-padding_units, padding_units]);
    ylim(axesHandle, planningRange_units(2, :) + [-padding_units, padding_units]);
    xlabel(axesHandle, "Unwrapped x (units)");
    ylabel(axesHandle, "Unwrapped y (units)");
    title(axesHandle, sprintf("%s | expanded workspace at t = %.3f s | %s", ...
        options.Title, startTime_s, result.TerminationReason));
    legend(axesHandle, "Location", "bestoutside");
end

function drawRectangle(axesHandle, intervals_units, color, lineStyle, lineWidth, displayName)
    % Outline [xmin xmax; ymin ymax].
    corners_units = [intervals_units(1, [1 2 2 1 1]).', intervals_units(2, [1 1 2 2 1]).'];
    plot(axesHandle, corners_units(:, 1), corners_units(:, 2), lineStyle, "Color", color, ...
        "LineWidth", lineWidth, "DisplayName", displayName);
end

function drawPlannerRoute(axesHandle, result)
    % Show a saved route or candidate motion even if planning later failed.
    % The plot title retains the failure reason; these lines do not imply success.
    if ~isempty(result.Diagnostics.Route_units)
        selectedRoute_units = createDisplayPath(result, result.Diagnostics.Route_units);
        routeHandle = drawLine(axesHandle, selectedRoute_units, ...
            "--", "Selected geometric route", 1);
        routeHandle.Color = [0.43 0.64 0.13];
    end
    if ~isempty(result.position_units)
        motionPath_units = createDisplayPath(result, result.position_units);
        drawLine(axesHandle, motionPath_units, "k-", "Timed motion", 2);
    end
end

function drawEndpoints(axesHandle, result)
    % Mark the start and goal on the same folded path as the motion. A
    % successful intercept ends at the target's actual arrival position.
    startPosition_units = result.Inputs.initialState.position_units;
    goalPosition_units  = result.Inputs.goalState.position_units;
    if result.Success && hasData(result.Diagnostics, 'Intercept') && isfinite(result.Diagnostics.Intercept.Time_s)
        goalPosition_units = result.Diagnostics.Intercept.TargetPosition_units;
    end
    if result.Success && ~isempty(result.position_units)
        endpointPath_units = createDisplayPath(result, result.position_units);
    else
        endpointPath_units = createDisplayPath(result, [startPosition_units; goalPosition_units]);
    end
    startPosition_units = endpointPath_units(1, :);
    goalPosition_units  = endpointPath_units(end, :);
    plot(axesHandle, startPosition_units(1), startPosition_units(2), "go", "DisplayName", "Start");
    plot(axesHandle, goalPosition_units(1), goalPosition_units(2), "ro", "DisplayName", "Goal");
end

function lineHandle = drawLine(axesHandle, position_units, lineStyle, displayName, lineWidth)
    % Draw connected x/y points; NaN rows leave gaps between separate pieces.
    lineHandle = plot(axesHandle, position_units(:, 1), position_units(:, 2), ...
        lineStyle, "LineWidth", lineWidth, "DisplayName", displayName);
end

function drawSearchDiagnostics(axesHandle, result, options)
    % Show a small sample of accepted connections. Rejected pairs remain in
    % the result for diagnostics but do not cover the workspace plot.
    visibilityGraph     = result.Diagnostics.VisibilityGraph;
    nodePositions_units = visibilityGraph.NodePosition_units;
    edgeFieldNames      = ["AcceptedNodeIndex", "RejectedNodeIndex"];
    allEdgeNodeIndices  = zeros(0, 2);
    for categoryIndex = 1:2
        if hasData(visibilityGraph, edgeFieldNames(categoryIndex))
            allEdgeNodeIndices = [allEdgeNodeIndices; ...
                visibilityGraph.(edgeFieldNames(categoryIndex))]; %#ok<AGROW>
        end
    end
    if options.ShowSearchEdges && hasData(visibilityGraph, 'AcceptedNodeIndex')
        edgeNodeIndices = visibilityGraph.AcceptedNodeIndex;
        selectedIndices = selectDisplayedVisibilityEdges( ...
            edgeNodeIndices, options.MaximumDisplayedGraphEdges);
        edgeNodeIndices = edgeNodeIndices(selectedIndices, :);
        edgeCount = size(edgeNodeIndices, 1);

        % Add a NaN after each two-node edge so MATLAB draws separate
        % connections instead of one continuous line.
        x_units = reshape([nodePositions_units(edgeNodeIndices(:, 1), 1), ...
            nodePositions_units(edgeNodeIndices(:, 2), 1), nan(edgeCount, 1)].', [], 1);
        y_units = reshape([nodePositions_units(edgeNodeIndices(:, 1), 2), ...
            nodePositions_units(edgeNodeIndices(:, 2), 2), nan(edgeCount, 1)].', [], 1);

        % Split each wrapped edge at the display boundary before joining
        % the edge arrays for plotting.
        if any(readWrapModes(result.Options) ~= "false")
            edgePaths_units = cell(edgeCount, 1);
            for edgeIndex = 1:edgeCount
                edgeEndpoints_units = nodePositions_units(edgeNodeIndices(edgeIndex, :), :);
                edgePaths_units{edgeIndex} = [createDisplayPath(result, edgeEndpoints_units); NaN NaN];
            end
            edgePaths_units = vertcat(edgePaths_units{:});
            x_units         = edgePaths_units(:, 1);
            y_units         = edgePaths_units(:, 2);
        end
        plot(axesHandle, x_units, y_units, "-", ...
            "Color", [0.02 0.30 0.85], "LineWidth", 2.2, ...
            "DisplayName", "Accepted visibility edges (sampled)");
    end
    nodePositions_units = foldNodes(result, nodePositions_units, allEdgeNodeIndices);
    if ~isempty(nodePositions_units)
        scatter(axesHandle, nodePositions_units(:, 1), nodePositions_units(:, 2), ...
            10, [0.45 0.16 0.60], "filled", "DisplayName", "Visibility node");
    end
end

function drawSpaceTimeSearch(axesHandle, result, options)
    % A spatial visibility graph was checked at one obstacle snapshot.
    % Timed search stores candidate positions and a timed route proposal,
    % but does not retain the tested connections for every time layer.
    visibilityGraph = result.Diagnostics.VisibilityGraph;
    searchKind = "";
    if isfield(visibilityGraph, 'SearchKind')
        searchKind = string(visibilityGraph.SearchKind);
    end
    isSpatialSnapshot = any(searchKind == ["initialSpatialSnapshot", "arrivalSpatialSnapshot"]);
    isTimedSearch = searchKind == "timeExpandedVisibilityGraph";

    if isSpatialSnapshot || isTimedSearch
        nodePositions_units = visibilityGraph.NodePosition_units;
        if ~isempty(nodePositions_units)
            snapshotTime_s = result.Inputs.initialState.time_s;
            nodeLabel = "Visibility nodes at obstacle snapshot";
            if searchKind == "arrivalSpatialSnapshot"
                snapshotTime_s = result.Inputs.goalState.time_s;
            elseif isTimedSearch
                nodeLabel = "Timed search positions (start-plane projection)";
            end

            nodeCount = size(nodePositions_units, 1);
            displayedNodeIndices = 1:min([2, nodeCount, options.MaximumDisplayedGraphNodes]);
            remainingNodeCount = min(nodeCount - numel(displayedNodeIndices), ...
                options.MaximumDisplayedGraphNodes - numel(displayedNodeIndices));
            if remainingNodeCount > 0
                displayedNodeIndices = [displayedNodeIndices, ...
                    unique(round(linspace(3, nodeCount, remainingNodeCount)))];
            end
            displayedNodes_units = nodePositions_units(displayedNodeIndices, :);
            scatter3(axesHandle, displayedNodes_units(:, 1), displayedNodes_units(:, 2), ...
                repmat(snapshotTime_s, size(displayedNodes_units, 1), 1), ...
                12, [0.20 0.52 0.77], "filled", "DisplayName", nodeLabel);

            if isSpatialSnapshot && options.ShowSearchEdges && ...
                    hasData(visibilityGraph, 'AcceptedNodeIndex')
                acceptedNodeIndices = visibilityGraph.AcceptedNodeIndex;
                displayedEdgeIndices = selectDisplayedVisibilityEdges( ...
                    acceptedNodeIndices, options.MaximumDisplayedGraphEdges);
                acceptedNodeIndices = acceptedNodeIndices(displayedEdgeIndices, :);
                edgeCount = size(acceptedNodeIndices, 1);
                edgePositions_units = reshape(permute(cat(3, ...
                    nodePositions_units(acceptedNodeIndices(:, 1), :), ...
                    nodePositions_units(acceptedNodeIndices(:, 2), :), NaN(edgeCount, 2)), [3 1 2]), [], 2);
                plot3(axesHandle, edgePositions_units(:, 1), edgePositions_units(:, 2), ...
                    repmat(snapshotTime_s, size(edgePositions_units, 1), 1), ...
                    "-", "Color", [0.02 0.30 0.85], "LineWidth", 1.0, ...
                    "DisplayName", "Accepted snapshot edges (sampled)");
            end
        end
    end

    if isTimedSearch && hasData(visibilityGraph, 'RouteTime_s') && ...
            size(visibilityGraph.Route_units, 1) == numel(visibilityGraph.RouteTime_s)
        drawSpaceTimePath(axesHandle, visibilityGraph.Route_units, ...
            visibilityGraph.RouteTime_s, "b--", "Timed route proposal", 1.3);
    end
end

function selectedRows = selectDisplayedVisibilityEdges(edgeNodeIndices, maximumEdges)
    % Saved visibility graphs place the start at node 1 and the goal at node 2.
    % Reserve roughly half the display for each endpoint, then fill unused
    % slots with other accepted edges. A direct start-to-goal edge comes first.
    touchesStart = any(edgeNodeIndices == 1, 2);
    touchesGoal  = any(edgeNodeIndices == 2, 2);
    directRows   = find(touchesStart & touchesGoal);
    selectedRows = directRows(1:min(numel(directRows), maximumEdges));

    remainingCount = maximumEdges - numel(selectedRows);
    startRows = find(touchesStart & ~touchesGoal);
    goalRows  = find(touchesGoal & ~touchesStart);
    selectedRows = [selectedRows; ...
        evenlySpacedRows(startRows, ceil(remainingCount / 2)); ...
        evenlySpacedRows(goalRows, floor(remainingCount / 2))];

    remainingCount = maximumEdges - numel(selectedRows);
    endpointRows = find(touchesStart | touchesGoal);
    unusedEndpointRows = setdiff(endpointRows, selectedRows, 'stable');
    selectedRows = [selectedRows; evenlySpacedRows(unusedEndpointRows, remainingCount)];

    remainingCount = maximumEdges - numel(selectedRows);
    otherRows = find(~touchesStart & ~touchesGoal);
    selectedRows = [selectedRows; evenlySpacedRows(otherRows, remainingCount)];
end

function selectedRows = evenlySpacedRows(candidateRows, maximumCount)
    % Choose from the whole saved group, including its ends when possible.
    selectedCount = min(numel(candidateRows), maximumCount);
    if selectedCount == 0
        selectedRows = zeros(0, 1);
    elseif selectedCount == 1
        selectedRows = candidateRows(round((numel(candidateRows) + 1) / 2));
    else
        rowPositions = unique(round(linspace(1, numel(candidateRows), selectedCount)));
        selectedRows = candidateRows(rowPositions);
    end
end

function drawObstacles(axesHandle, protectedObstacles, originalObstacles, ...
        time_s, requestedIntervals_units, wrapModes)
    % Fill the original obstacle and outline its protected shape. A wrapped
    % 2D view copies each snapshot, not the obstacle's motion history. This
    % keeps a shape at [358 362] visible at [-2 2] on x [0 360] without
    % asking preparation to prove motion for an extra display-only copy.
    obstacleColors = lines(max(1, numel(protectedObstacles)));
    for obstacleIndex = 1:numel(protectedObstacles)
        obstacle = protectedObstacles(obstacleIndex);
        if ~shapeIsAvailable(obstacle, time_s)
            continue
        end
        originalShape = obstacleAvoidance.obstacles.preparedShapeAtTime( ...
            originalObstacles(obstacleIndex), time_s);
        protectedShape = obstacleAvoidance.obstacles.preparedShapeAtTime(obstacle, time_s);
        if all(wrapModes == "false")
            drawShape(axesHandle, originalShape, obstacleColors(obstacleIndex, :), "-", "Original obstacle");
            drawShape(axesHandle, protectedShape, "none", "--", "Protected obstacle");
            continue
        end
        if isempty(protectedShape.Vertices)
            continue
        end

        % The protected snapshot contains the original snapshot. Its bounds
        % therefore select every transform needed to draw both boundaries.
        protectedVertices_units = protectedShape.Vertices;
        protectedVertices_units = protectedVertices_units(all(isfinite(protectedVertices_units), 2), :);
        snapshotBounds_units = [min(protectedVertices_units, [], 1).', ...
            max(protectedVertices_units, [], 1).'];
        images = obstacleAvoidance.input.listWrapImages( ...
            snapshotBounds_units, requestedIntervals_units, wrapModes, requestedIntervals_units);
        [protectedX_units, protectedY_units] = boundary(protectedShape);
        originalIsAvailable = ~isempty(originalShape.Vertices);
        if originalIsAvailable
            [originalX_units, originalY_units] = boundary(originalShape);
        end
        for imageIndex = 1:numel(images.XOffset_units)
            xOffset_units = images.XOffset_units(imageIndex);
            yScale        = images.YScale(imageIndex);
            yOffset_units = images.YOffset_units(imageIndex);
            if originalIsAvailable
                originalCopy = polyshape(originalX_units + xOffset_units, ...
                    yScale * originalY_units + yOffset_units, 'Simplify', false);
                drawShape(axesHandle, originalCopy, obstacleColors(obstacleIndex, :), "-", "Original obstacle");
            end
            protectedCopy = polyshape(protectedX_units + xOffset_units, ...
                yScale * protectedY_units + yOffset_units, 'Simplify', false);
            drawShape(axesHandle, protectedCopy, "none", "--", "Protected obstacle");
        end
    end
end

function drawSpaceTimeObstacles(axesHandle, protectedObstacles, plotTimeRange_s, options)
    % Draw a few outlines so rotating obstacles remain readable. An interval
    % enclosure covers the whole motion and is not an instantaneous shape,
    % so show recorded protected shapes for those obstacles. Draw solid
    % source motion when input vertices correspond; otherwise show the
    % protected interval envelope as a solid.
    obstacleColors = lines(max(1, numel(protectedObstacles)));
    firstSlice = true;
    firstOriginalSweep = true;
    firstProtectedSweep = true;
    firstEnvelopeSweep = true;
    for obstacleIndex = 1:numel(protectedObstacles)
        obstacle = protectedObstacles(obstacleIndex);
        preparation = obstacle.InternalPreparation;
        originalSweepData  = [];
        protectedSweepData = [];
        envelopeSweepData  = [];
        if isscalar(obstacle.time_s)
            activeRange_s = plotTimeRange_s;
        else
            activeRange_s = [max(plotTimeRange_s(1), obstacle.time_s(1)), ...
                min(plotTimeRange_s(2), obstacle.time_s(end))];
        end
        sliceCount = options.MaximumDisplayedTimeSlices;
        if preparation.IsTimeInvariant
            sliceCount = min(2, sliceCount);
        end
        if activeRange_s(1) > activeRange_s(2)
            continue
        end
        usesIntervalEnclosures = any(preparation.IntervalUsesMovingCells | ...
            preparation.IntervalUsesEndpointHull);
        if usesIntervalEnclosures
            % A broad interval enclosure can make a spinning bar look like
            % a different polygon at each arbitrary display time. Recorded
            % samples show the actual protected boundaries supplied there.
            recordedTimes_s = obstacle.time_s(:);
            recordedTimes_s = recordedTimes_s( ...
                recordedTimes_s >= activeRange_s(1) & recordedTimes_s <= activeRange_s(2));
            sliceTimes_s = zeros(1, 0);
            if ~isempty(recordedTimes_s)
                selectedIndices = unique(round(linspace( ...
                    1, numel(recordedTimes_s), min(sliceCount, numel(recordedTimes_s)))));
                sliceTimes_s = recordedTimes_s(selectedIndices).';
            end
        else
            sliceTimes_s = unique(linspace(activeRange_s(1), activeRange_s(2), sliceCount));
        end
        snapshotX_units = cell(1, numel(sliceTimes_s));
        snapshotY_units = cell(1, numel(sliceTimes_s));
        snapshotTime_s = cell(1, numel(sliceTimes_s));
        snapshotCount = 0;
        for time_s = sliceTimes_s
            % An interval without a verified continuous model has no shape
            % between its samples; leave a gap there.
            if ~shapeIsAvailable(obstacle, time_s)
                continue
            end
            shape = obstacleAvoidance.obstacles.preparedShapeAtTime(obstacle, time_s);
            if isempty(shape.Vertices)
                continue
            end
            [x_units, y_units] = boundary(shape);
            snapshotCount = snapshotCount + 1;
            snapshotX_units{snapshotCount} = [x_units(:).', NaN];
            snapshotY_units{snapshotCount} = [y_units(:).', NaN];
            snapshotTime_s{snapshotCount} = [repmat(time_s, 1, numel(x_units)), NaN];
        end
        if snapshotCount > 0
            legendVisibility = "off";
            if firstSlice
                legendVisibility = "on";
                firstSlice = false;
            end
            plot3(axesHandle, [snapshotX_units{1:snapshotCount}], ...
                [snapshotY_units{1:snapshotCount}], [snapshotTime_s{1:snapshotCount}], ...
                "-", "Color", obstacleColors(obstacleIndex, :), ...
                "LineWidth", 0.8, "DisplayName", "Protected obstacle snapshots", ...
                "HandleVisibility", legendVisibility);
        end

        if ~options.ShowSweptSurfaces || activeRange_s(1) == activeRange_s(2)
            continue
        end
        if preparation.IsTimeInvariant
            % A fixed shape has one straight extrusion through the time range.
            [~, fixedBoundary] = obstacleAvoidance.obstacles.preparedShapeAtTime( ...
                obstacle, activeRange_s(1));
            protectedSweepData = appendSweptBoundary(protectedSweepData, ...
                fixedBoundary, fixedBoundary, activeRange_s(1), activeRange_s(2));
            if ~isempty(protectedSweepData)
                renderSweptBoundary(axesHandle, protectedSweepData, ...
                    obstacleColors(obstacleIndex, :), "Protected obstacle sweep", firstProtectedSweep);
                firstProtectedSweep = false;
            end
            continue
        end

        % Split at each source sample. Across a sample boundary, the matched
        % vertex order can change even when the visible polygons look alike.
        recordedTimes_s = obstacle.time_s(:);
        timesInsideRange_s = recordedTimes_s( ...
            recordedTimes_s > activeRange_s(1) & recordedTimes_s < activeRange_s(2));
        sweepTimes_s = unique([activeRange_s(1); timesInsideRange_s; activeRange_s(2)]);
        for intervalIndex = 1:numel(sweepTimes_s) - 1
            startTime_s = sweepTimes_s(intervalIndex);
            endTime_s   = sweepTimes_s(intervalIndex + 1);
            middleTime_s = startTime_s + 0.5 * (endTime_s - startTime_s);
            if ~shapeIsAvailable(obstacle, middleTime_s)
                continue
            end

            sourceIntervalIndex = find(obstacle.time_s <= middleTime_s, 1, "last");
            usesFixedEnvelope = preparation.IntervalIsStationary(sourceIntervalIndex) || ...
                preparation.IntervalUsesMovingCells(sourceIntervalIndex) || ...
                preparation.IntervalUsesEndpointHull(sourceIntervalIndex);
            if usesFixedEnvelope
                sourceStartX_units = obstacle.originalX_units{sourceIntervalIndex};
                sourceStartY_units = obstacle.originalY_units{sourceIntervalIndex};
                sourceEndX_units   = obstacle.originalX_units{sourceIntervalIndex + 1};
                sourceEndY_units   = obstacle.originalY_units{sourceIntervalIndex + 1};
                sourceVerticesCorrespond = obstacle.UsesSourceIndex && ...
                    isequal(size(sourceStartX_units), size(sourceEndX_units)) && ...
                    isequal(size(sourceStartY_units), size(sourceEndY_units)) && ...
                    isequal(isfinite(sourceStartX_units), isfinite(sourceEndX_units)) && ...
                    isequal(isfinite(sourceStartY_units), isfinite(sourceEndY_units));
                if preparation.IntervalUsesMovingCells(sourceIntervalIndex) && ...
                        sourceVerticesCorrespond
                    % Draw straight motion between the supplied source
                    % vertices. This is a visualization of original motion;
                    % planning still uses its protected interval envelope.
                    sourceStartTime_s = obstacle.time_s(sourceIntervalIndex);
                    sourceDuration_s = obstacle.time_s(sourceIntervalIndex + 1) - sourceStartTime_s;
                    startFraction = (startTime_s - sourceStartTime_s) / sourceDuration_s;
                    endFraction   = (endTime_s - sourceStartTime_s) / sourceDuration_s;
                    startBoundary = struct( ...
                        'x_units', sourceStartX_units + startFraction * (sourceEndX_units - sourceStartX_units), ...
                        'y_units', sourceStartY_units + startFraction * (sourceEndY_units - sourceStartY_units));
                    endBoundary = struct( ...
                        'x_units', sourceStartX_units + endFraction * (sourceEndX_units - sourceStartX_units), ...
                        'y_units', sourceStartY_units + endFraction * (sourceEndY_units - sourceStartY_units));
                    originalSweepData = appendSweptBoundary(originalSweepData, ...
                        startBoundary, endBoundary, startTime_s, endTime_s);
                    continue
                end

                % This solid is the planner's protected occupancy envelope
                % for the whole interval. It can include space the original
                % obstacle did not visit at a particular instant.
                envelopeShape = preparation.IntervalUnionShapes{sourceIntervalIndex};
                if ~isempty(envelopeShape.Vertices)
                    [x_units, y_units] = boundary(envelopeShape);
                    envelopeBoundary = struct('x_units', x_units, 'y_units', y_units);
                    envelopeSweepData = appendSweptBoundary(envelopeSweepData, ...
                        envelopeBoundary, envelopeBoundary, startTime_s, endTime_s);
                end
                continue
            end

            % Query just inside one verified interval. Its prepared vertices
            % correspond there even if recorded sample boundaries use a
            % different starting corner or loop order.
            inset_s = max(1e-9 * (endTime_s - startTime_s), ...
                64 * eps(max(abs([startTime_s, endTime_s]))));
            if 4 * inset_s >= endTime_s - startTime_s
                continue
            end
            [~, faceStart] = obstacleAvoidance.obstacles.preparedShapeAtTime( ...
                obstacle, startTime_s + inset_s);
            [~, faceEnd] = obstacleAvoidance.obstacles.preparedShapeAtTime( ...
                obstacle, endTime_s - inset_s);
            if boundaryCorresponds(obstacle, faceStart, faceEnd, startTime_s, endTime_s)
                protectedSweepData = appendSweptBoundary(protectedSweepData, ...
                    faceStart, faceEnd, startTime_s, endTime_s);
            end
        end
        if ~isempty(originalSweepData)
            renderSweptBoundary(axesHandle, originalSweepData, ...
                obstacleColors(obstacleIndex, :), "Original sampled-motion sweep", firstOriginalSweep);
            firstOriginalSweep = false;
        end
        if ~isempty(protectedSweepData)
            renderSweptBoundary(axesHandle, protectedSweepData, ...
                obstacleColors(obstacleIndex, :), "Protected obstacle sweep", firstProtectedSweep);
            firstProtectedSweep = false;
        end
        if ~isempty(envelopeSweepData)
            renderSweptBoundary(axesHandle, envelopeSweepData, ...
                obstacleColors(obstacleIndex, :), "Protected occupancy envelope", firstEnvelopeSweep);
            firstEnvelopeSweep = false;
        end
    end
end

function matches = boundaryCorresponds(obstacle, previousBoundary, currentBoundary, ...
        previousTime_s, currentTime_s)
    % Prepared histories identify intervals where vertices correspond.
    previousX_units = previousBoundary.x_units;
    currentX_units = currentBoundary.x_units;
    matches = isequal(size(previousX_units), size(currentX_units)) && ...
        isequal(isfinite(previousX_units), isfinite(currentX_units)) && ...
        isequal(size(previousBoundary.y_units), size(currentBoundary.y_units)) && ...
        isequal(isfinite(previousBoundary.y_units), isfinite(currentBoundary.y_units));
    if ~matches || currentTime_s <= previousTime_s
        return
    end
    if isscalar(obstacle.time_s) || obstacle.InternalPreparation.IsTimeInvariant
        return
    end
    startSampleIndex = find(obstacle.time_s <= previousTime_s, 1, "last");
    endSampleIndex   = find(obstacle.time_s >= currentTime_s, 1, "first");
    matches = endSampleIndex == startSampleIndex + 1 && ...
        obstacle.InternalPreparation.MatchingTopology(startSampleIndex);
end

function sweepData = appendSweptBoundary(sweepData, previousBoundary, currentBoundary, ...
        previousTime_s, currentTime_s)
    % NaN separates polygon loops. Close each loop separately so a surface
    % cannot bridge a hole or a disconnected piece. Store the panels until
    % one surface can draw all intervals of this obstacle at once.
    finiteVertex = isfinite(previousBoundary.x_units) & isfinite(previousBoundary.y_units);
    runStarts = find(diff([false; finiteVertex; false]) == 1);
    runEnds   = find(diff([false; finiteVertex; false]) == -1) - 1;
    for runIndex = 1:numel(runStarts)
        vertexIndices = runStarts(runIndex):runEnds(runIndex);
        if numel(vertexIndices) < 2
            continue
        end
        if isempty(sweepData)
            sweepData = struct('SurfaceX', {{}}, 'SurfaceY', {{}}, ...
                'SurfaceTime', {{}}, 'RailX', {{}}, 'RailY', {{}}, 'RailTime', {{}});
        end
        closedVertexIndices = [vertexIndices, vertexIndices(1)];
        sweepData.SurfaceX{end + 1} = [[ ...
            previousBoundary.x_units(closedVertexIndices).'; ...
            currentBoundary.x_units(closedVertexIndices).'], nan(2, 1)];
        sweepData.SurfaceY{end + 1} = [[ ...
            previousBoundary.y_units(closedVertexIndices).'; ...
            currentBoundary.y_units(closedVertexIndices).'], nan(2, 1)];
        sweepData.SurfaceTime{end + 1} = [[ ...
            repmat(previousTime_s, 1, numel(closedVertexIndices)); ...
            repmat(currentTime_s, 1, numel(closedVertexIndices))], nan(2, 1)];

        % Follow two opposite corners per loop through time. These rails
        % reveal the twist without redrawing every polygon at every sample.
        railPositions = unique([1, 1 + floor(numel(vertexIndices) / 2)]);
        for railPosition = railPositions
            vertexIndex = vertexIndices(railPosition);
            sweepData.RailX{end + 1} = [ ...
                previousBoundary.x_units(vertexIndex), currentBoundary.x_units(vertexIndex), NaN];
            sweepData.RailY{end + 1} = [ ...
                previousBoundary.y_units(vertexIndex), currentBoundary.y_units(vertexIndex), NaN];
            sweepData.RailTime{end + 1} = [previousTime_s, currentTime_s, NaN];
        end
    end
end

function renderSweptBoundary(axesHandle, sweepData, obstacleColor, legendLabel, showInLegend)
    % A NaN column separates adjacent panels without creating another graphics
    % object. The same separation keeps the two wireframe rails independent.
    legendVisibility = "off";
    if showInLegend
        legendVisibility = "on";
    end
    surface(axesHandle, [sweepData.SurfaceX{:}], [sweepData.SurfaceY{:}], ...
        [sweepData.SurfaceTime{:}], "FaceColor", obstacleColor, ...
        "FaceAlpha", 0.12, "EdgeColor", "none", ...
        "Tag", "SpaceTimeObstacleSweep", ...
        "DisplayName", legendLabel, "HandleVisibility", legendVisibility);
    railColor = 0.45 * obstacleColor + 0.55 * [1 1 1];
    plot3(axesHandle, [sweepData.RailX{:}], [sweepData.RailY{:}], ...
        [sweepData.RailTime{:}], "-", "Color", railColor, ...
        "LineWidth", 0.45, "HandleVisibility", "off");
end

function setSweepVisibility(sweepSurfaceHandles, visibility)
    % A closed figure can delete these handles before a pending mouse callback.
    validHandles = sweepSurfaceHandles(isgraphics(sweepSurfaceHandles));
    if ~isempty(validHandles)
        set(validHandles, "Visible", visibility);
    end
end

function drawSpaceTimeTarget(axesHandle, result, plotTimeRange_s)
    goalState = result.Inputs.goalState;
    if ~hasData(goalState, 'targetMotion')
        return
    end
    targetMotion = goalState.targetMotion;
    activeRange_s = [max(plotTimeRange_s(1), targetMotion.time_s(1)), ...
        min(plotTimeRange_s(2), targetMotion.time_s(end))];
    if activeRange_s(1) > activeRange_s(2)
        return
    end
    % Include the recorded sample times so sharp turns are drawn exactly.
    recordedTimes_s = double(targetMotion.time_s(:));
    targetTimes_s   = unique([linspace(activeRange_s(1), activeRange_s(2), 200).'; ...
        recordedTimes_s(recordedTimes_s >= activeRange_s(1) & recordedTimes_s <= activeRange_s(2))]);
    targetPosition_units = obstacleAvoidance.input.targetPositionAtTime(targetMotion, targetTimes_s);
    drawSpaceTimePath(axesHandle, targetPosition_units, ...
        targetTimes_s, "m-.", "Moving target", 1.4);
end

function drawSpaceTimePath(axesHandle, position_units, time_s, lineStyle, displayName, lineWidth)
    % Plot each sample at its own time in the planner's unwrapped coordinates.
    plot3(axesHandle, position_units(:, 1), position_units(:, 2), time_s, lineStyle, ...
        "LineWidth", lineWidth, "DisplayName", displayName);
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
    displayTimeIsInHistory = displayTime_s >= targetMotion.time_s(1) && ...
        displayTime_s <= targetMotion.time_s(end);
    if displayTimeIsInHistory
        targetSampleTimes_s = unique([targetSampleTimes_s; displayTime_s]);
    end
    [targetPath_units, sourceSampleIndices] = createDisplayPath( ...
        result, obstacleAvoidance.input.targetPositionAtTime(targetMotion, targetSampleTimes_s));
    targetTrackHandle = drawLine(axesHandle, targetPath_units, ...
        "-.", "Moving target track", 1);
    targetTrackHandle.Color = [0.75 0.00 0.65];

    % Keep the full track when the display time is outside the target
    % history, but omit the current-position marker for that time.
    if ~displayTimeIsInHistory
        return
    end
    targetSampleIndex   = find(targetSampleTimes_s == displayTime_s, 1);
    targetDisplayIndex  = find(sourceSampleIndices == targetSampleIndex, 1, 'last');
    targetPosition_units = targetPath_units(targetDisplayIndex, :);
    plot(axesHandle, targetPosition_units(1), targetPosition_units(2), ...
        "md", "MarkerFaceColor", "m", "DisplayName", "Moving target");
end

function axesHandles = createKinematicPanels(layoutHandle, result, useAnimationLayout)
    % Plot the complete x/y histories. Velocity, acceleration, and jerk
    % each show positive and negative limits. These panels do not draw
    % position bounds.
    quantityFieldNames = ["position_units", "velocity_units_s", "acceleration_units_s2", "jerk_units_s3"];
    quantityAxisLabels = ["Position (units)", "Velocity (units/s)", "Acceleration (units/s^2)", "Jerk (units/s^3)"];
    quantityLimits     = [nan(1, 2); result.Diagnostics.Limits.maxVelocity_units_s; ...
        result.Diagnostics.Limits.maxAcceleration_units_s2; result.Diagnostics.Limits.maxJerk_units_s3];
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

function finishAxes(axesHandle, result, titlePrefix, legendLocation)
    % Label the plot with the planner's actual termination reason.
    xlabel(axesHandle, "X (units)");
    ylabel(axesHandle, "Y (units)");
    title(axesHandle, sprintf("%s | %s", titlePrefix, result.TerminationReason));
    legend(axesHandle, "Location", legendLocation);
end

function isAvailable = shapeIsAvailable(obstacle, time_s)
    % An interval without a verified continuous model has no shape between
    % its samples, so drawing there would throw. Recorded samples and times
    % outside the history are still available.
    sampleTimes_s = obstacle.time_s(:);
    intervalIndex = find(sampleTimes_s < time_s, 1, 'last');
    isAvailable   = isempty(intervalIndex) || intervalIndex >= numel(sampleTimes_s) || ...
        time_s == sampleTimes_s(intervalIndex + 1) || ...
        ~obstacle.InternalPreparation.IntervalIsUnsupported(intervalIndex);
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
        "ExpandedWorkspaceFigure",   emptyHandles, ...
        "ExpandedWorkspaceAxes",     emptyHandles, ...
        "SphereFigure",              emptyHandles, ...
        "SphereAxes",                emptyHandles, ...
        "VisibilityFigure",          emptyHandles, ...
        "VisibilityAxes",            emptyHandles, ...
        "SpaceTimeFigure",           emptyHandles, ...
        "SpaceTimeAxes",             emptyHandles, ...
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
