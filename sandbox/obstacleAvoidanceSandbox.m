function sandboxState = obstacleAvoidanceSandbox(sandboxOverrides)
%% Section 0: Header & Readme
% SYNTAX
%   sandboxState = obstacleAvoidanceSandbox()
%   sandboxState = obstacleAvoidanceSandbox([])
%   sandboxState = obstacleAvoidanceSandbox(sandboxOverrides)
%**************************************************************************
% PURPOSE
%   - Open one azimuth/elevation planning UI.
%   - Keep interactive drawing outside the production planner.
%**************************************************************************
% INPUTS
%   - sandboxOverrides (scalar struct, optional; default struct())
%       Empty fields retain defaults. Supported fields are FigureVisible,
%       FigurePosition, MissionTime_s,
%       MaxVelocity_deg_s, MaxAcceleration_deg_s2, MaxJerk_deg_s3,
%       PathObstacleRadius_deg, PathSafetyMargin_deg,
%       ObstacleSafetyMargin_deg, WorkspaceAzimuthInterval_deg,
%       WorkspaceElevationInterval_deg, Verbose, AnimateOnRun,
%       AnimationFrameStride, AnimationPause_s, and PlannerOptions.
%       PlannerOptions is a partial planTrajectory options struct. Its
%       sandbox defaults bound interactive HS3 work and favor responsiveness
%       over the production planner's finer earliest-arrival search.
%       Goal-time mode and verbosity are owned by the active sandbox tab.
%**************************************************************************
% OUTPUTS
%   - sandboxState (scalar struct)
%       Initial data structure, figure handle, and Goal Mode record.
%       ReadState and ExportBundle function handles. Call ReadState() after
%       interaction to inspect the current guidata-backed state. Call
%       ExportBundle(filePath, modeName) to save without a file dialog.
%**************************************************************************
% UNITS
%   - Positions and boundaries are N-by-2 [azimuth elevation] in degrees.
%   - Time is in seconds. Derivatives use deg/s, deg/s^2, and deg/s^3.
%**************************************************************************

%% Section 1: Resolve Sandbox Defaults

% Resolve all options before creating graphics. Invalid limits or workspace
% ranges then fail before callbacks and partial UI state exist.

if nargin < 1 || isempty(sandboxOverrides)
    sandboxOverrides = struct();
end
options = resolveSandboxOptions(sandboxOverrides);

%% Section 2: Create The Goal-Mode Figure

% Create one Goal Mode tab. The figure stores the
% current application data in guidata. Callbacks always read the latest guidata
% value instead of keeping stale copies in nested functions.

figureHandle = figure( ...
    "Name", "Az/El Interactive Sandbox", ...
    "NumberTitle", "off", ...
    "MenuBar", "none", ...
    "ToolBar", "figure", ...
    "Units", "pixels", ...
    "Position", options.FigurePosition, ...
    "Visible", options.FigureVisible, ...
    "Color", [0.94 0.94 0.94]);
tabGroupHandle = uitabgroup(figureHandle, ...
    "Units", "normalized", ...
    "Position", [0 0 1 1], ...
    "SelectionChangedFcn", @handleTabSelection);
goalTabHandle = uitab(tabGroupHandle, "Title", "Goal Mode", "Tag", "goal");
goalHandles = createModeTab(goalTabHandle, "goal", options);

%% Section 3: Initialize Goal Mode

% Goal Mode plans one motion from a start point to one goal point. It keeps its
% own controls, obstacles, result, validation, status, and log.

applicationState = initializeApplicationState( ...
    figureHandle, options, goalHandles);
guidata(figureHandle, applicationState);

applyDefaultControls(goalHandles.Controls, options);
refreshApplication(figureHandle);
beginGuidedScene(figureHandle, "goal");

%% Section 4: Return Initial Sandbox State

% Return a plain snapshot plus functions that read current state and export a
% diagnosis file. The initial snapshot does not update after UI interaction.
% Use ReadState to get current values.

sandboxState = publicStateSnapshot(figureHandle);
sandboxState.ReadState = @() publicStateSnapshot(figureHandle);

end

% --- Defaults And UI Construction ---------------------------------------

function defaults = sandboxDefaults()
% Define all sandbox default values in one location.
defaults = struct( ...
    "FigureVisible", "on", ...
    "FigurePosition", [60 60 1460 860], ...
    "MissionTime_s", 180, ...
    "MaxVelocity_deg_s", [2 2], ...
    "MaxAcceleration_deg_s2", [0.75 0.75], ...
    "MaxJerk_deg_s3", [2.5 2.5], ...
    "PathObstacleRadius_deg", 0.5, ...
    "PathSafetyMargin_deg", 0, ...
    "ObstacleSafetyMargin_deg", 0.2, ...
    "WorkspaceAzimuthInterval_deg", [-180 180], ...
    "WorkspaceElevationInterval_deg", [-90 90], ...
    "Verbose", true, ...
    "AnimateOnRun", true, ...
    "AnimationFrameStride", 20, ...
    "AnimationPause_s", 0.001, ...
    "PlannerOptions", interactivePlannerDefaults());
end

function options = interactivePlannerDefaults()
% Bound nonlinear work for an interactive preview. Returned motions still pass
% the public planner's collision, kinematic, and independent validation gates.
% Callers can replace these values through sandboxOverrides.PlannerOptions when
% finer arrival-time quality is more important than UI response time.
options = struct( ...
    "MaximumSeedCount", 3, ...
    "CollocationSegmentCount", 8, ...
    "MaximumNlpIterations", 80, ...
    "ArrivalTimeTolerance_s", 0.05);
end

function options = resolveSandboxOptions(overrides)
% Merge partial option values. Report unknown fields one time. Validate all
% values before UI creation.
if ~isstruct(overrides) || ~isscalar(overrides)
    error("obstacleAvoidanceSandbox:InvalidOverrides", "sandboxOverrides must be a scalar struct or empty.");
end
defaults = sandboxDefaults();
options = defaults;
knownNames = string(fieldnames(defaults));
overrideNames = string(fieldnames(overrides));
unknownNames = setdiff(overrideNames, knownNames, "stable");
if ~isempty(unknownNames)
    warning("obstacleAvoidanceSandbox:UnknownOptions", ...
        "Ignoring unknown sandbox fields: %s. No behavior changed.", ...
        strjoin(unknownNames, ", "));
end

% Apply only known, nonempty values. An empty field keeps its default value.
for name = reshape(intersect(overrideNames, knownNames, "stable"), 1, [])
    if ~isempty(overrides.(name))
        options.(name) = overrides.(name);
    end
end
options.FigureVisible = lower(string(options.FigureVisible));
if ~isscalar(options.FigureVisible) || ~any(options.FigureVisible == ["on", "off"])
    error("obstacleAvoidanceSandbox:InvalidFigureVisible", "FigureVisible must be 'on' or 'off'.");
end
options.Verbose = obstacleAvoidance.input.normalizeLogicalScalar( ...
    options.Verbose, "Verbose", ...
    "obstacleAvoidanceSandbox:InvalidLogicalOption");
options.AnimateOnRun = obstacleAvoidance.input.normalizeLogicalScalar( ...
    options.AnimateOnRun, "AnimateOnRun", ...
    "obstacleAvoidanceSandbox:InvalidLogicalOption");
validateattributes(options.FigurePosition, {'numeric'}, {'real', 'finite', 'vector', 'numel', 4});
options.FigurePosition = reshape(double(options.FigurePosition), 1, 4);
positiveScalarNames = ["MissionTime_s", "PathObstacleRadius_deg"];

% Validate each duration or radius that must be strictly positive.
for name = positiveScalarNames
    validateattributes(options.(name), {'numeric'}, {'real', 'finite', 'scalar', 'positive'}, "obstacleAvoidanceSandbox", name);
end
nonnegativeScalarNames = ["PathSafetyMargin_deg", "ObstacleSafetyMargin_deg"];

% A safety margin can be zero. It cannot be negative or nonfinite.
for name = nonnegativeScalarNames
    validateattributes(options.(name), {'numeric'}, ...
        {'real', 'finite', 'scalar', 'nonnegative'}, ...
        "obstacleAvoidanceSandbox", name);
end
pairNames = ["MaxVelocity_deg_s", "MaxAcceleration_deg_s2", "MaxJerk_deg_s3"];

% Normalize each azimuth/elevation derivative limit into one row pair.
for name = pairNames
    validateattributes(options.(name), {'numeric'}, ...
        {'real', 'finite', 'vector', 'numel', 2, 'positive'}, ...
        "obstacleAvoidanceSandbox", name);
    options.(name) = reshape(double(options.(name)), 1, 2);
end
intervalNames = ["WorkspaceAzimuthInterval_deg", "WorkspaceElevationInterval_deg"];

% Require an increasing lower/upper interval for each workspace axis.
for name = intervalNames
    validateattributes(options.(name), {'numeric'}, ...
        {'real', 'finite', 'vector', 'numel', 2, 'increasing'}, ...
        "obstacleAvoidanceSandbox", name);
    options.(name) = reshape(double(options.(name)), 1, 2);
end
countNames = "AnimationFrameStride";

% Candidate counts limit the number of latest-arrival planner calls.
for name = countNames
    validateattributes(options.(name), {'numeric'}, ...
        {'real', 'finite', 'scalar', 'integer', 'nonnegative'}, ...
        "obstacleAvoidanceSandbox", name);
end
if ~isstruct(options.PlannerOptions) || ~isscalar(options.PlannerOptions)
    error("obstacleAvoidanceSandbox:InvalidPlannerOptions", "PlannerOptions must be a scalar partial options struct.");
end
options.PlannerOptions = ...
    obstacleAvoidance.input.resolvePlannerOptions(options.PlannerOptions);
validateattributes(options.AnimationPause_s, {'numeric'}, ...
    {'real', 'finite', 'scalar', 'nonnegative'}, ...
    "obstacleAvoidanceSandbox", "AnimationPause_s");
end

function handles = createModeTab(tabHandle, modeName, options)
% Create one complete tab. Each tab has a canvas, controls, action buttons,
% status text, and a planner log.

% Reserve the complete outer rectangle for axes ticks and labels. A smaller
% Position can move the azimuth label under the action-button row.
axesHandle = axes( ...
    tabHandle, ...
    "Units", "normalized", ...
    "OuterPosition", [0.02 0.34 0.68 0.63], ...
    "PositionConstraint", "outerposition");
hold(axesHandle, "on");
grid(axesHandle, "on");
box(axesHandle, "on");
axis(axesHandle, [options.WorkspaceAzimuthInterval_deg, options.WorkspaceElevationInterval_deg]);
axis(axesHandle, "equal");
xlabel(axesHandle, "Azimuth (deg)");
ylabel(axesHandle, "Elevation (deg)");
controlPanelHandle = uipanel(tabHandle, ...
    "Title", "Planning controls", ...
    "Units", "normalized", ...
    "Position", [0.71 0.30 0.275 0.66]);
controls = struct();

% Use shared column headings for paired values. This saves vertical space and
% remains readable when Windows display scaling increases text height.
addControlSectionLabel(controlPanelHandle, "Workspace", 0.89);
addPairColumnLabels(controlPanelHandle, "Lower", "Upper", 0.835);
controls.WorkspaceAzimuthHandles = addCompactPairControl( ...
    controlPanelHandle, "Azimuth (deg)", 0.775, ...
    options.WorkspaceAzimuthInterval_deg);
controls.WorkspaceElevationHandles = addCompactPairControl( ...
    controlPanelHandle, "Elevation (deg)", 0.71, ...
    options.WorkspaceElevationInterval_deg);

addControlSectionLabel(controlPanelHandle, "Kinematic limits", 0.625);
addPairColumnLabels(controlPanelHandle, "Azimuth", "Elevation", 0.57);
controls.VelocityHandles = addCompactPairControl( ...
    controlPanelHandle, "Velocity (deg/s)", 0.51, ...
    options.MaxVelocity_deg_s);
controls.AccelerationHandles = addCompactPairControl( ...
    controlPanelHandle, "Acceleration (deg/s^2)", 0.445, ...
    options.MaxAcceleration_deg_s2);
controls.JerkHandles = addCompactPairControl(controlPanelHandle, "Jerk (deg/s^3)", 0.38, options.MaxJerk_deg_s3);

horizonLabel = "Mission horizon (s)";
horizonValue = options.MissionTime_s;

addControlSectionLabel(controlPanelHandle, "Timing and obstacle geometry", 0.295);
controls.HorizonHandle = addScalarControl(controlPanelHandle, horizonLabel, 0.235, horizonValue);
controls.PathRadiusHandle = addScalarControl( ...
    controlPanelHandle, "Line/capsule radius (deg)", 0.17, ...
    options.PathObstacleRadius_deg);
controls.PathMarginHandle = addScalarControl( ...
    controlPanelHandle, "Line safety margin (deg)", 0.105, ...
    options.PathSafetyMargin_deg);
controls.ObstacleMarginHandle = addScalarControl( ...
    controlPanelHandle, "Polygon safety margin (deg)", 0.04, ...
    options.ObstacleSafetyMargin_deg);

% The narrow strip below the panel keeps the verbose flag visible without
% adding another row to the already compact planning panel.
controls.VerboseHandle = uicontrol(tabHandle, ...
    "Style", "checkbox", ...
    "String", "Verbose output", ...
    "Units", "normalized", ...
    "Position", [0.882 0.263 0.103 0.03], ...
    "Value", options.Verbose, ...
    "HorizontalAlignment", "left");
controls.MotionProfileHandle = uicontrol(tabHandle, ...
    "Style", "popupmenu", ...
    "String", { ...
        "Non-zero velocity from start", ...
        "Zero velocity from start", ...
        "Trapezoidal", ...
        "Oscillating"}, ...
    "Units", "normalized", ...
    "Position", [0.71 0.263 0.165 0.03], ...
    "Value", 1, ...
    "TooltipString", ...
        "Motion profile used by Set Motion for the selected polygon.");
addPanelHandle = uipanel(tabHandle, ...
    "Title", "Add", ...
    "Units", "normalized", ...
    "Position", [0.01 0.268 0.24 0.095]);
addNames = ["AddPolygon", "AddCircle", "AddHandDrawn", "AddSquare"];
addLabels = ["Polygon", "Circle", "Hand Drawn", "Square"];
actions = createAddButtons(addPanelHandle, modeName, addNames, addLabels);
actionPanelHandle = uipanel(tabHandle, "BorderType", "none", ...
    "Units", "normalized", "Position", [0.265 0.275 0.42 0.052]);
actionNames = [ ...
    "SetMotion", "Run", "Stop", "Reset", ...
    "Diagnostics", "Export"];
actionLabels = [ ...
    "Set Motion", "Run", "Stop", "Reset", ...
    "Diagnostics", "Export Bundle"];
actionButtons = createActionButtons( ...
    actionPanelHandle, modeName, actionNames, actionLabels);
actionFields = fieldnames(actionButtons);
for actionIndex = 1:numel(actionFields)
    actionName = actionFields{actionIndex};
    actions.(actionName) = actionButtons.(actionName);
end
statusPanelHandle = uipanel(tabHandle, ...
    "Title", "Mode status and planner log", ...
    "Units", "normalized", ...
    "Position", [0.045 0.025 0.64 0.23]);
statusHandle = uicontrol(statusPanelHandle, ...
    "Style", "text", ...
    "String", "Ready", ...
    "Units", "normalized", ...
    "Position", [0.015 0.08 0.33 0.86], ...
    "HorizontalAlignment", "left");
logHandle = uicontrol(statusPanelHandle, ...
    "Style", "listbox", ...
    "String", {"Planner output will appear here."}, ...
    "Units", "normalized", ...
    "Position", [0.36 0.08 0.625 0.86], ...
    "HorizontalAlignment", "left", ...
    "Min", 0, ...
    "Max", 2);
plannerOptionsPanelHandle = uipanel(tabHandle, ...
    "Title", "Planner options", ...
    "Units", "normalized", ...
    "Position", [0.71 0.025 0.275 0.23]);
controls.UnsupportedTimedTopologyHandle = addPopupControl( ...
    plannerOptionsPanelHandle, "Unsupported timed route", 0.76, ...
    ["Fail and diagnose", "Ruckig stop at waypoints"], ...
    1 + double(options.PlannerOptions.UnsupportedTimedTopologyPolicy == ...
        "ruckigStopAtWaypoints"), ...
    "Choose whether an unsupported smooth timed route may stop at every waypoint.");
controls.GoalTimeModeHandle = addPopupControl( ...
    plannerOptionsPanelHandle, "Goal timing", 0.49, ...
    ["Balanced travel/time", "Earliest arrival", ...
    "Minimum travel at horizon"], ...
    find(options.PlannerOptions.GoalTimeMode == ...
    ["balancedArrival", "earliestArrival", "fixedArrival"], 1), ...
    "Choose an explicit travel/time tradeoff or either endpoint policy.");
controls.MinimumTravelSavingsRateHandle = addScalarControl( ...
    plannerOptionsPanelHandle, "Travel threshold (deg/s)", 0.28, ...
    options.PlannerOptions.MinimumTravelSavingsRate_deg_s);
controls.AllowAzimuthWrappingHandle = uicontrol( ...
    plannerOptionsPanelHandle, ...
    "Style", "checkbox", ...
    "String", "Allow azimuth wrapping", ...
    "Units", "normalized", ...
    "Position", [0.05 0.08 0.90 0.18], ...
    "Value", options.PlannerOptions.AllowAzimuthWrapping, ...
    "HorizontalAlignment", "left", ...
    "TooltipString", ...
        "Allow equivalent azimuth positions separated by 360 degrees.");
handles = struct( ...
    "Tab", tabHandle, ...
    "Axes", axesHandle, ...
    "ControlPanel", controlPanelHandle, ...
    "Controls", controls, ...
    "AddPanel", addPanelHandle, ...
    "ActionPanel", actionPanelHandle, ...
    "Actions", actions, ...
    "StatusPanel", statusPanelHandle, ...
    "PlannerOptionsPanel", plannerOptionsPanelHandle, ...
    "StatusHandle", statusHandle, ...
    "LogHandle", logHandle, ...
    "DiagnosticPlotHandles", struct(), ...
    "AnimationPlotHandles", struct());
end

function addControlSectionLabel(panelHandle, labelText, rowPosition)
% Separate related settings with one bold label instead of another panel.
uicontrol(panelHandle, ...
    "Style", "text", ...
    "String", labelText, ...
    "Units", "normalized", ...
    "Position", [0.05 rowPosition 0.91 0.045], ...
    "HorizontalAlignment", "left", ...
    "FontWeight", "bold");
end

function addPairColumnLabels(panelHandle, firstLabel, secondLabel, rowPosition)
% Name the two shared value columns once for the rows below them.
columnLabels = [firstLabel secondLabel];

% Both headings use the same coordinates as their corresponding edit boxes.
for axisIndex = 1:2
    leftPosition = 0.52 + (axisIndex - 1) * 0.24;
    uicontrol(panelHandle, ...
        "Style", "text", ...
        "String", columnLabels(axisIndex), ...
        "Units", "normalized", ...
        "Position", [leftPosition rowPosition 0.20 0.035], ...
        "HorizontalAlignment", "center");
end
end

function handles = addCompactPairControl(panelHandle, labelText, rowPosition, values)
% Put one pair on a single row beneath its shared column headings.
uicontrol(panelHandle, ...
    "Style", "text", ...
    "String", labelText, ...
    "Units", "normalized", ...
    "Position", [0.05 rowPosition 0.44 0.045], ...
    "HorizontalAlignment", "left");
handles = struct( ...
    "FirstHandle", uicontrol(panelHandle, ...
        "Style", "edit", ...
        "String", sprintf("%.8g", values(1)), ...
        "Units", "normalized", ...
        "Position", [0.52 rowPosition 0.20 0.045]), ...
    "SecondHandle", uicontrol(panelHandle, ...
        "Style", "edit", ...
        "String", sprintf("%.8g", values(2)), ...
        "Units", "normalized", ...
        "Position", [0.76 rowPosition 0.20 0.045]));
end

function editHandle = addScalarControl(panelHandle, labelText, rowPosition, value)
% Add one labeled scalar edit control to a planning panel.
uicontrol(panelHandle, ...
    "Style", "text", ...
    "String", labelText, ...
    "Units", "normalized", ...
    "Position", [0.05 rowPosition 0.68 0.045], ...
    "HorizontalAlignment", "left");
editHandle = uicontrol(panelHandle, ...
    "Style", "edit", ...
    "String", sprintf("%.8g", value), ...
    "Units", "normalized", ...
    "Position", [0.76 rowPosition 0.20 0.045]);
end

function actions = createActionButtons( panelHandle, modeName, actionNames, actionLabels)
% Create one evenly spaced action row with one shared callback.
actions = struct();
buttonCount = numel(actionNames);
gap = 0.008;
buttonWidth = (1 - gap * (buttonCount - 1)) / buttonCount;

% Give each retained action the same width and route it through one dispatcher.
for actionIndex = 1:buttonCount
    actionName = actionNames(actionIndex);
    leftPosition = (actionIndex - 1) * (buttonWidth + gap);
    actions.(actionName) = uicontrol(panelHandle, ...
        "Style", "pushbutton", ...
        "String", actionLabels(actionIndex), ...
        "Units", "normalized", ...
        "Position", [leftPosition 0 buttonWidth 1], ...
        "UserData", struct("Mode", modeName, "Action", actionName), ...
        "Callback", @handleAction);
end
end

function actions = createAddButtons(panelHandle, modeName, actionNames, actionLabels)
% Place four obstacle constructors in a compact two-by-two panel at the left.
actions = struct();
for actionIndex = 1:numel(actionNames)
    columnIndex = mod(actionIndex - 1, 2);
    rowIndex = floor((actionIndex - 1) / 2);
    actions.(actionNames(actionIndex)) = uicontrol(panelHandle, ...
        "Style", "pushbutton", ...
        "String", actionLabels(actionIndex), ...
        "Units", "normalized", ...
        "Position", [0.02 + 0.50 * columnIndex, ...
            0.52 - 0.48 * rowIndex, 0.46, 0.42], ...
        "UserData", struct( ...
            "Mode", modeName, "Action", actionNames(actionIndex)), ...
        "Callback", @handleAction);
end
end

function applicationState = initializeApplicationState( ...
        figureHandle, options, goalHandles)
% Create the stable Goal Mode record and application interaction state.
applicationState = struct( ...
    "FigureHandle", figureHandle, ...
    "Options", options, ...
    "ActiveMode", "goal", ...
    "InteractionState", "idle", ...
    "ActiveStroke_deg", zeros(0, 2), ...
    "ActiveTraceHandle", gobjects(0), ...
    "GoalMode", emptyModeState(goalHandles));
end

function modeState = emptyModeState(graphicsHandles)
% Define all fields in one mode record. Initialization and Reset use the same
% field set so callbacks can read state without optional-field branches.
instruction = ...
    "Click the start and goal. Choose an obstacle from the Add panel, " + ...
    "then Run.";

modeState = struct( ...
    "StartPosition_deg", zeros(0, 2), ...
    "GoalPosition_deg", zeros(0, 2), ...
    "RawObstacleStrokes_deg", {cell(0, 1)}, ...
    "LineObstaclePositions_deg", {cell(0, 1)}, ...
    "PolygonObstaclePositions_deg", {cell(0, 1)}, ...
    "PolygonMotionVectors_deg", zeros(0, 2), ...
    "PolygonMotionProfiles", strings(0, 1), ...
    "SelectedPolygonIndex", 0, ...
    "CanonicalObstacles", obstacleAvoidance.obstacles.combineObstacles(), ...
    "LastPlannerResult", struct(), ...
    "LastValidation", obstacleAvoidance.validateTrajectory(), ...
    "GraphicsHandles", graphicsHandles, ...
    "InteractionState", "idle", ...
    "Status", instruction, ...
    "PlannerLog", strings(0, 1), ...
    "ResolvedControls", struct());
end

function popupHandle = addPopupControl( ...
        panelHandle, labelText, rowPosition, choices, selectedIndex, tooltip)
% Add one labeled planner-choice popup with a stable string-to-index mapping.
uicontrol(panelHandle, ...
    "Style", "text", ...
    "String", labelText, ...
    "Units", "normalized", ...
    "Position", [0.05 rowPosition 0.44 0.18], ...
    "HorizontalAlignment", "left");
popupHandle = uicontrol(panelHandle, ...
    "Style", "popupmenu", ...
    "String", cellstr(choices), ...
    "Units", "normalized", ...
    "Position", [0.51 rowPosition 0.44 0.20], ...
    "Value", selectedIndex, ...
    "TooltipString", tooltip);
end

% --- Interaction State And Callbacks ------------------------------------

function handleAction(sourceHandle, ~)
% Send each button action to one callback branch. Convert UI input errors into
% status text. Unexpected programmer errors keep their identifier and location
% in the retained log.
figureHandle = ancestor(sourceHandle, "figure");
request = get(sourceHandle, "UserData");
modeName = string(request.Mode);
actionName = string(request.Action);
try
    switch actionName
        case "AddPolygon"
            activateInteraction(figureHandle, modeName, "addingPolygon");
        case "AddCircle"
            activateInteraction(figureHandle, modeName, "addingCircle");
        case "AddHandDrawn"
            activateInteraction(figureHandle, modeName, "addingHandDrawn");
        case "AddSquare"
            activateInteraction(figureHandle, modeName, "addingSquare");
        case "SetMotion"
            activateInteraction(figureHandle, modeName, "selectingObstacleMotion");
        case "Run"
            executeGoalPlan(figureHandle);
        case "Stop"
            requestPlanningStop(figureHandle, modeName);
        case "Reset"
            resetMode(figureHandle, modeName);
        case "Diagnostics"
            openDiagnostics(figureHandle, modeName);
        case "Export"
            exportModeDiagnosis(figureHandle, modeName);
    end
catch exception
    cancelInteraction(figureHandle);
    applicationState = guidata(figureHandle);
    modeState = getModeState(applicationState, modeName);
    exceptionText = formatSandboxException(exception);
    if string(exception.identifier) == "planTrajectory:UserCancelled"
        modeState.Status = ...
            "Planning stopped. Export Bundle is available for this request.";
        modeState = appendLogLines(modeState, ...
            "[Planner stopped] " + string(exception.message));
    else
        modeState.Status = "Input or planning error: " + string(exceptionText);
        modeState = appendLogLines( ...
            modeState, "[Sandbox error] " + string(exceptionText));
    end
    applicationState = setModeState( applicationState, modeName, modeState);
    guidata(figureHandle, applicationState);
    refreshApplication(figureHandle);
    if actionName == "Export" && get(figureHandle, "Visible") == "on"
        errordlg( ...
            ['Bundle export failed: ' exceptionText], ...
            'Sandbox export failed', 'modal');
    end
end
end

function requestPlanningStop(figureHandle, modeName)
% Record a cooperative stop request while retaining the scene for export.
applicationState = guidata(figureHandle);
if applicationState.InteractionState ~= "planning"
    return;
end
setappdata(figureHandle, "SandboxStopRequested", true);
modeState = getModeState(applicationState, modeName);
modeState.Status = "Stopping planning at the next safe checkpoint...";
modeState = appendLogLines(modeState, "[Stop requested]");
applicationState = setModeState(applicationState, modeName, modeState);
guidata(figureHandle, applicationState);
set(modeState.GraphicsHandles.Actions.Stop, "Enable", "off");
updateModeStatusDisplay(modeState);
drawnow limitrate;
end

function exceptionText = formatSandboxException(exception)
% Preserve the error identifier and first source location. Start debugging at
% this location instead of the shared callback dispatcher.
exceptionText = char(exception.message);
if strlength(string(exception.identifier)) > 0
    exceptionText = sprintf( ...
        '%s [%s]', exceptionText, char(exception.identifier));
end
if ~isempty(exception.stack)
    exceptionText = sprintf( ...
        '%s at %s:%d', exceptionText, ...
        exception.stack(1).name, exception.stack(1).line);
end
end

function activateInteraction(figureHandle, modeName, requestedState)
% Allow only one mouse interaction at a time. Store old callbacks before a draw
% starts. Restore them when the draw finishes or is canceled.
cancelInteraction(figureHandle);
applicationState = guidata(figureHandle);
modeState = getModeState(applicationState, modeName);
applicationState.ActiveMode = modeName;
switch requestedState
    case "placingStart"
        interactionState = "placing" + upperFirst(modeName) + "Start";
        modeState.Status = "Place the start point with one left click.";
    case "placingGoal"
        interactionState = "placingGoalStop";
        modeState.Status = "Place or replace the goal with one left click.";
    case "addingPolygon"
        interactionState = "adding" + upperFirst(modeName) + "Polygon";
        modeState.Status = ...
            "Left-click each polygon vertex. " + ...
            "Right-click to close and add the polygon.";
    case "addingCircle"
        interactionState = "placing" + upperFirst(modeName) + "CircleCenter";
        modeState.Status = ...
            "Click the circle center, then click a point on its edge.";
    case "addingHandDrawn"
        interactionState = "drawing" + upperFirst(modeName) + "Obstacle";
        modeState.Status = ...
            "Press and drag to draw an obstacle, then release to add it.";
    case "addingSquare"
        interactionState = "placing" + upperFirst(modeName) + "SquareCorner";
        modeState.Status = ...
            "Click one square corner, then click toward the opposite corner.";
    case "selectingObstacleMotion"
        interactionState = "selecting" + upperFirst(modeName) + ...
            "ObstacleMotion";
        modeState.SelectedPolygonIndex = 0;
        modeState.Status = ...
            "Click inside a polygon. Then click the arrow endpoint " + ...
            "to set its motion vector.";
end
applicationState.InteractionState = interactionState;
modeState.InteractionState = interactionState;
applicationState = setModeState(applicationState, modeName, modeState);
guidata(figureHandle, applicationState);
set(figureHandle, "WindowButtonDownFcn", @handleFigureMouseDown, "WindowButtonMotionFcn", "", "WindowButtonUpFcn", "");
refreshApplication(figureHandle);
end

function beginGuidedScene(figureHandle, modeName)
% Continue the guided setup at its next unfinished step. The order is start
% point, first obstacle, and goal or waypoint input.
applicationState = guidata(figureHandle);
modeState = getModeState(applicationState, modeName);

if isempty(modeState.StartPosition_deg)
    requestedState = "placingStart";
elseif isempty(modeState.GoalPosition_deg)
    requestedState = "placingGoal";
else
    requestedState = "";
end

% Automatic guidance owns only the first obstacle. Once one is retained, the
% explicit Add Obstacle button prevents an accidental stroke on tab changes.
if strlength(requestedState) > 0
    activateInteraction(figureHandle, modeName, requestedState);
else
    cancelInteraction(figureHandle);
    refreshApplication(figureHandle);
end
end

function handleFigureMouseDown(figureHandle, ~)
% Process one click for point placement, polygon creation, or motion editing.
% Ignore clicks outside the active mode axes.
applicationState = guidata(figureHandle);
if applicationState.InteractionState == "idle" || applicationState.InteractionState == "planning"
    return;
end
modeName = applicationState.ActiveMode;
modeState = getModeState(applicationState, modeName);
clickedHandle = hittest(figureHandle);
clickedAxes = ancestor(clickedHandle, "axes");
if isempty(clickedAxes) || ~isequal(clickedAxes, modeState.GraphicsHandles.Axes)
    return;
end
selectionType = string(get(figureHandle, "SelectionType"));
if contains(applicationState.InteractionState, "adding") && ...
        endsWith(applicationState.InteractionState, "Polygon")
    if selectionType == "alt"
        finishPolygonInteraction(figureHandle);
    elseif selectionType == "normal"
        addPolygonVertex(figureHandle);
    end
    return;
end
if selectionType ~= "normal"
    return;
end
point_deg = cursorPoint(modeState.GraphicsHandles.Axes);
controls = readModeControls(applicationState, modeName);
if ~pointInWorkspace(point_deg, controls)
    modeState.Status = "The selected point is outside the workspace limits.";
    applicationState = setModeState(applicationState, modeName, modeState);
    guidata(figureHandle, applicationState);
    refreshApplication(figureHandle);
    return;
end
interactionState = applicationState.InteractionState;
if endsWith(interactionState, "CircleCenter") || ...
        endsWith(interactionState, "SquareCorner")
    applicationState.ActiveStroke_deg = point_deg;
    if endsWith(interactionState, "CircleCenter")
        nextState = "placing" + upperFirst(modeName) + "CircleEdge";
        modeState.Status = "Circle center set. Click a point on its edge.";
    else
        nextState = "placing" + upperFirst(modeName) + "SquareOpposite";
        modeState.Status = ...
            "Square corner set. Click toward the opposite corner.";
    end
    applicationState.InteractionState = nextState;
    modeState.InteractionState = nextState;
    applicationState = setModeState(applicationState, modeName, modeState);
    guidata(figureHandle, applicationState);
    updateModeStatusDisplay(modeState);
    return;
end
if endsWith(interactionState, "CircleEdge")
    center_deg = applicationState.ActiveStroke_deg(1, :);
    radius_deg = norm(point_deg - center_deg);
    if radius_deg <= 1e-6
        modeState.Status = "The circle radius must have nonzero length.";
        applicationState = setModeState(applicationState, modeName, modeState);
        guidata(figureHandle, applicationState);
        updateModeStatusDisplay(modeState);
        return;
    end
    angle_rad = linspace(0, 2 * pi, 25).';
    circle_deg = center_deg + radius_deg * [ ...
        cos(angle_rad(1:end - 1)), sin(angle_rad(1:end - 1))];
    isInsideWorkspace = true;
    for vertexIndex = 1:size(circle_deg, 1)
        isInsideWorkspace = isInsideWorkspace && ...
            pointInWorkspace(circle_deg(vertexIndex, :), controls);
    end
    if ~isInsideWorkspace
        modeState.Status = "The circle extends outside the workspace limits.";
        applicationState = setModeState(applicationState, modeName, modeState);
        guidata(figureHandle, applicationState);
        updateModeStatusDisplay(modeState);
        return;
    end
    completeCreatedPolygon(figureHandle, circle_deg, "Circle");
    return;
end
if endsWith(interactionState, "SquareOpposite")
    firstCorner_deg = applicationState.ActiveStroke_deg(1, :);
    cornerOffset_deg = point_deg - firstCorner_deg;
    sideLength_deg = max(abs(cornerOffset_deg));
    if sideLength_deg <= 1e-6
        modeState.Status = "The square side must have nonzero length.";
        applicationState = setModeState(applicationState, modeName, modeState);
        guidata(figureHandle, applicationState);
        updateModeStatusDisplay(modeState);
        return;
    end
    direction = sign(cornerOffset_deg);
    direction(direction == 0) = 1;
    oppositeCorner_deg = firstCorner_deg + direction * sideLength_deg;
    square_deg = [ ...
        firstCorner_deg; ...
        oppositeCorner_deg(1), firstCorner_deg(2); ...
        oppositeCorner_deg; ...
        firstCorner_deg(1), oppositeCorner_deg(2)];
    isInsideWorkspace = true;
    for cornerIndex = 1:4
        isInsideWorkspace = isInsideWorkspace && ...
            pointInWorkspace(square_deg(cornerIndex, :), controls);
    end
    if ~isInsideWorkspace
        modeState.Status = "The square extends outside the workspace limits.";
        applicationState = setModeState(applicationState, modeName, modeState);
        guidata(figureHandle, applicationState);
        updateModeStatusDisplay(modeState);
        return;
    end
    completeCreatedPolygon(figureHandle, square_deg, "Square");
    return;
end
if contains(applicationState.InteractionState, ...
        "selecting") && endsWith( ...
        applicationState.InteractionState, "ObstacleMotion")
    polygonIndex = polygonIndexAtPoint( ...
        modeState.PolygonObstaclePositions_deg, point_deg);
    if polygonIndex == 0
        modeState.Status = "No polygon contains the selected point.";
    else
        polygon_deg = modeState.PolygonObstaclePositions_deg{polygonIndex};
        [centroidAzimuth_deg, centroidElevation_deg] = ...
            centroid(polyshape(polygon_deg));
        modeState.SelectedPolygonIndex = polygonIndex;
        modeState.InteractionState = ...
            "placing" + upperFirst(modeName) + "ObstacleMotionEnd";
        modeState.Status = ...
            "Polygon " + polygonIndex + " selected. " + ...
            "Click the arrow endpoint.";
        applicationState.InteractionState = modeState.InteractionState;
        applicationState.ActiveStroke_deg = [ ...
            centroidAzimuth_deg, centroidElevation_deg];
        applicationState.ActiveTraceHandle = quiver( ...
            modeState.GraphicsHandles.Axes, centroidAzimuth_deg, ...
            centroidElevation_deg, 0, 0, 0, ...
            "Color", [0.15 0.55 0.15], "LineWidth", 2, ...
            "MaxHeadSize", 0.35, "HandleVisibility", "off");
        set(figureHandle, "WindowButtonMotionFcn", ...
            @handleObstacleMotionPreview);
    end
    applicationState = setModeState( ...
        applicationState, modeName, modeState);
    guidata(figureHandle, applicationState);
    updateModeStatusDisplay(modeState);
    return;
end
if contains(applicationState.InteractionState, ...
        "placing") && endsWith( ...
        applicationState.InteractionState, "ObstacleMotionEnd")
    finishObstacleMotionInteraction(figureHandle, point_deg);
    return;
end
if contains(applicationState.InteractionState, "drawing")
    applicationState.ActiveStroke_deg = point_deg;
    applicationState.ActiveTraceHandle = plot( ...
        modeState.GraphicsHandles.Axes, point_deg(1), point_deg(2), ...
        "b-", "LineWidth", 1.4, "HandleVisibility", "off");
    guidata(figureHandle, applicationState);
    set(figureHandle, "WindowButtonMotionFcn", @handleFigureMouseMotion, "WindowButtonUpFcn", @handleFigureMouseUp);
    return;
end
nextInteraction = "";
switch applicationState.InteractionState
    case "placingGoalStart"
        modeState.StartPosition_deg = point_deg;
        modeState = clearModeSolution(modeState);
        modeState.Status = "Start point set. Click the goal next.";
        nextInteraction = "placingGoal";
    case "placingGoalStop"
        modeState.GoalPosition_deg = point_deg;
        modeState = clearModeSolution(modeState);
        modeState.Status = ...
            "Goal point set. Choose a shape from the Add panel, or Run.";
end
applicationState = setModeState(applicationState, modeName, modeState);
guidata(figureHandle, applicationState);
cancelInteraction(figureHandle);
if strlength(nextInteraction) > 0
    activateInteraction(figureHandle, modeName, nextInteraction);
else
    refreshApplication(figureHandle);
end
end

function handleObstacleMotionPreview(figureHandle, ~)
% Update the temporary arrow while the user selects its endpoint.
applicationState = guidata(figureHandle);
if ~endsWith(applicationState.InteractionState, "ObstacleMotionEnd") || ...
        isempty(applicationState.ActiveTraceHandle) || ...
        ~isgraphics(applicationState.ActiveTraceHandle)
    return;
end
modeState = getModeState(applicationState, applicationState.ActiveMode);
endpoint_deg = cursorPoint(modeState.GraphicsHandles.Axes);
origin_deg = applicationState.ActiveStroke_deg(1, :);
motionVector_deg = endpoint_deg - origin_deg;
set(applicationState.ActiveTraceHandle, ...
    "UData", motionVector_deg(1), "VData", motionVector_deg(2));
drawnow("limitrate");
end

function addPolygonVertex(figureHandle)
% Add one distinct vertex to the active polygon preview.
applicationState = guidata(figureHandle);
modeName = applicationState.ActiveMode;
modeState = getModeState(applicationState, modeName);
point_deg = cursorPoint(modeState.GraphicsHandles.Axes);
controls = readModeControls(applicationState, modeName);
if ~pointInWorkspace(point_deg, controls)
    modeState.Status = "The polygon vertex is outside the workspace limits.";
    applicationState = setModeState( ...
        applicationState, modeName, modeState);
    guidata(figureHandle, applicationState);
    updateModeStatusDisplay(modeState);
    return;
end
minimumVertexSpacing_deg = 1e-6;
if ~isempty(applicationState.ActiveStroke_deg) && ...
        norm(point_deg - applicationState.ActiveStroke_deg(end, :)) <= ...
        minimumVertexSpacing_deg
    modeState.Status = "That polygon vertex duplicates the previous vertex.";
    applicationState = setModeState( ...
        applicationState, modeName, modeState);
    guidata(figureHandle, applicationState);
    updateModeStatusDisplay(modeState);
    return;
end
applicationState.ActiveStroke_deg(end + 1, :) = point_deg;
if isempty(applicationState.ActiveTraceHandle) || ...
        ~isgraphics(applicationState.ActiveTraceHandle)
    applicationState.ActiveTraceHandle = plot( ...
        modeState.GraphicsHandles.Axes, ...
        applicationState.ActiveStroke_deg(:, 1), ...
        applicationState.ActiveStroke_deg(:, 2), ...
        "bo-", "LineWidth", 1.4, "MarkerFaceColor", "b", ...
        "HandleVisibility", "off");
else
    set(applicationState.ActiveTraceHandle, ...
        "XData", applicationState.ActiveStroke_deg(:, 1), ...
        "YData", applicationState.ActiveStroke_deg(:, 2));
end
modeState.Status = ...
    "Polygon has " + size(applicationState.ActiveStroke_deg, 1) + ...
    " vertices. Right-click to finish.";
applicationState = setModeState(applicationState, modeName, modeState);
guidata(figureHandle, applicationState);
updateModeStatusDisplay(modeState);
drawnow("limitrate");
end

function finishPolygonInteraction(figureHandle)
% Validate and store the active polygon after a right-click.
applicationState = guidata(figureHandle);
modeName = applicationState.ActiveMode;
modeState = getModeState(applicationState, modeName);
polygon_deg = applicationState.ActiveStroke_deg;
if size(polygon_deg, 1) < 3
    modeState.Status = ...
        "A polygon needs at least three vertices. Continue with left-clicks.";
    applicationState = setModeState( ...
        applicationState, modeName, modeState);
    guidata(figureHandle, applicationState);
    updateModeStatusDisplay(modeState);
    return;
end
polygonShape = polyshape(polygon_deg);
if area(polygonShape) <= eps
    modeState.Status = ...
        "The polygon has no enclosed area. Add non-collinear vertices.";
    applicationState = setModeState( ...
        applicationState, modeName, modeState);
    guidata(figureHandle, applicationState);
    updateModeStatusDisplay(modeState);
    return;
end
modeState.RawObstacleStrokes_deg{end + 1, 1} = polygon_deg;
modeState.PolygonObstaclePositions_deg{end + 1, 1} = polygon_deg;
modeState.PolygonMotionVectors_deg(end + 1, :) = [0 0];
modeState.PolygonMotionProfiles(end + 1, 1) = "stationary";
modeState = clearModeSolution(modeState);
modeState.Status = ...
    "Polygon added. Use Set Motion to move it, or add another polygon.";
applicationState = setModeState(applicationState, modeName, modeState);
guidata(figureHandle, applicationState);
cancelInteraction(figureHandle);
refreshApplication(figureHandle);
end

function completeCreatedPolygon(figureHandle, polygon_deg, shapeName)
% Store a circle or square through the same canonical polygon representation.
applicationState = guidata(figureHandle);
modeName = applicationState.ActiveMode;
modeState = getModeState(applicationState, modeName);
modeState.RawObstacleStrokes_deg{end + 1, 1} = polygon_deg;
modeState.PolygonObstaclePositions_deg{end + 1, 1} = polygon_deg;
modeState.PolygonMotionVectors_deg(end + 1, :) = [0 0];
modeState.PolygonMotionProfiles(end + 1, 1) = "stationary";
modeState = clearModeSolution(modeState);
modeState.Status = shapeName + ...
    " added. Use Set Motion to move it, or add another obstacle.";
applicationState = setModeState(applicationState, modeName, modeState);
guidata(figureHandle, applicationState);
cancelInteraction(figureHandle);
refreshApplication(figureHandle);
end

function polygonIndex = polygonIndexAtPoint(polygonCollection_deg, point_deg)
% Return the last drawn polygon that contains the selected point.
polygonIndex = 0;
for candidateIndex = numel(polygonCollection_deg):-1:1
    polygonShape = polyshape(polygonCollection_deg{candidateIndex});
    if isinterior(polygonShape, point_deg(1), point_deg(2))
        polygonIndex = candidateIndex;
        return;
    end
end
end

function finishObstacleMotionInteraction(figureHandle, endpoint_deg)
% Store one polygon motion vector and the selected motion profile.
applicationState = guidata(figureHandle);
modeName = applicationState.ActiveMode;
modeState = getModeState(applicationState, modeName);
polygonIndex = modeState.SelectedPolygonIndex;
if polygonIndex < 1 || ...
        polygonIndex > numel(modeState.PolygonObstaclePositions_deg)
    error("obstacleAvoidanceSandbox:InvalidMotionSelection", ...
        "Select a polygon before setting its motion vector.");
end
origin_deg = applicationState.ActiveStroke_deg(1, :);
motionVector_deg = endpoint_deg - origin_deg;
if norm(motionVector_deg) <= 1e-9
    modeState.Status = "The motion vector must have nonzero length.";
    applicationState = setModeState( ...
        applicationState, modeName, modeState);
    guidata(figureHandle, applicationState);
    updateModeStatusDisplay(modeState);
    return;
end
profile = selectedMotionProfile( ...
    modeState.GraphicsHandles.Controls.MotionProfileHandle);
modeState.PolygonMotionVectors_deg(polygonIndex, :) = motionVector_deg;
modeState.PolygonMotionProfiles(polygonIndex, 1) = profile;
modeState.SelectedPolygonIndex = 0;
modeState = clearModeSolution(modeState);
modeState.Status = ...
    "Motion set for polygon " + polygonIndex + ": " + profile + ".";
applicationState = setModeState(applicationState, modeName, modeState);
guidata(figureHandle, applicationState);
cancelInteraction(figureHandle);
refreshApplication(figureHandle);
end

function profile = selectedMotionProfile(profileHandle)
% Convert the popup selection to the stored profile name.
profileNames = [ ...
    "nonzeroVelocity", "zeroStart", "trapezoidal", "oscillating"];
selectedIndex = get(profileHandle, "Value");
profile = profileNames(selectedIndex);
end

function handleFigureMouseMotion(figureHandle, ~)
% Extend the active freehand trace without invoking planning.
applicationState = guidata(figureHandle);
if ~contains(applicationState.InteractionState, "drawing") || ...
        isempty(applicationState.ActiveTraceHandle) || ...
        ~isgraphics(applicationState.ActiveTraceHandle)
    return;
end
modeState = getModeState(applicationState, applicationState.ActiveMode);
point_deg = cursorPoint(modeState.GraphicsHandles.Axes);
minimumTraceSpacing_deg = 0.25;
if norm(point_deg - applicationState.ActiveStroke_deg(end, :)) < minimumTraceSpacing_deg
    return;
end
applicationState.ActiveStroke_deg(end + 1, :) = point_deg;
set(applicationState.ActiveTraceHandle, ...
    "XData", applicationState.ActiveStroke_deg(:, 1), ...
    "YData", applicationState.ActiveStroke_deg(:, 2));
guidata(figureHandle, applicationState);
drawnow("limitrate");
end

function handleFigureMouseUp(figureHandle, ~)
% Finish one trace and classify it as a line or polygon. Simplify retained
% geometry before obstacle construction. Replan only when the active mode has
% enough request data.
applicationState = guidata(figureHandle);
if ~contains(applicationState.InteractionState, "drawing")
    return;
end
modeName = applicationState.ActiveMode;
modeState = getModeState(applicationState, modeName);
point_deg = cursorPoint(modeState.GraphicsHandles.Axes);
if isempty(applicationState.ActiveStroke_deg) || norm(point_deg - applicationState.ActiveStroke_deg(end, :)) >= 0.25
    applicationState.ActiveStroke_deg(end + 1, :) = point_deg;
end
rawStroke_deg = applicationState.ActiveStroke_deg;
simplifiedStroke_deg = simplifyFreehandBoundary(rawStroke_deg, 1);
if size(simplifiedStroke_deg, 1) >= 2
    modeState.RawObstacleStrokes_deg{end + 1, 1} = rawStroke_deg;
    if size(simplifiedStroke_deg, 1) == 2
        modeState.LineObstaclePositions_deg{end + 1, 1} = simplifiedStroke_deg;
        geometryDescription = "line/capsule";
    else
        modeState.PolygonObstaclePositions_deg{end + 1, 1} = simplifiedStroke_deg;
        modeState.PolygonMotionVectors_deg(end + 1, :) = [0 0];
        modeState.PolygonMotionProfiles(end + 1, 1) = "stationary";
        geometryDescription = "polygon";
    end
    modeState = clearModeSolution(modeState);
    modeState.Status = "Added one " + geometryDescription + " obstacle.";
else
    modeState.Status = "The stroke was too short and was ignored.";
end
applicationState = setModeState(applicationState, modeName, modeState);
guidata(figureHandle, applicationState);
cancelInteraction(figureHandle);
if size(simplifiedStroke_deg, 1) >= 2
    refreshApplication(figureHandle);
else
    % A rejected trace leaves no obstacle, so keep the initial draw step active.
    beginGuidedScene(figureHandle, modeName);
end
end

function cancelInteraction(figureHandle)
% Restore idle callbacks and discard an unfinished trace. This prevents a tab
% change or Reset from leaving mouse callbacks active.
if isempty(figureHandle) || ~isgraphics(figureHandle)
    return;
end
applicationState = guidata(figureHandle);
if isempty(applicationState)
    return;
end
if ~isempty(applicationState.ActiveTraceHandle) && isgraphics(applicationState.ActiveTraceHandle)
    delete(applicationState.ActiveTraceHandle);
end
activeMode = applicationState.ActiveMode;
modeState = getModeState(applicationState, activeMode);
modeState.InteractionState = "idle";
applicationState = setModeState(applicationState, activeMode, modeState);
applicationState.InteractionState = "idle";
applicationState.ActiveStroke_deg = zeros(0, 2);
applicationState.ActiveTraceHandle = gobjects(0);
guidata(figureHandle, applicationState);
set(figureHandle, "WindowButtonDownFcn", "", "WindowButtonMotionFcn", "", "WindowButtonUpFcn", "");
end

function handleTabSelection(tabGroupHandle, eventData)
% Cancel active drawing on tab changes without clearing either scene.
figureHandle = ancestor(tabGroupHandle, "figure");
cancelInteraction(figureHandle);
applicationState = guidata(figureHandle);
applicationState.ActiveMode = string(get(eventData.NewValue, "Tag"));
guidata(figureHandle, applicationState);
beginGuidedScene(figureHandle, applicationState.ActiveMode);
end

function point_deg = cursorPoint(axesHandle)
% Read one N-by-2-compatible point from explicit axes coordinates.
currentPoint = get(axesHandle, "CurrentPoint");
point_deg = double(currentPoint(1, 1:2));
end

function isInside = pointInWorkspace(point_deg, controls)
% Reject click geometry outside the explicitly configured workspace.
isInside = point_deg(1) >= controls.WorkspaceAzimuthInterval_deg(1) && ...
    point_deg(1) <= controls.WorkspaceAzimuthInterval_deg(2) && ...
    point_deg(2) >= controls.WorkspaceElevationInterval_deg(1) && ...
    point_deg(2) <= controls.WorkspaceElevationInterval_deg(2);
end

function text = upperFirst(value)
% Capitalize one internal mode name for readable state identifiers.
value = char(value);
text = string([upper(value(1)) value(2:end)]);
end

% --- Planner Calls And Segment Composition ------------------------------

function executeGoalPlan(figureHandle)
% Read Goal Mode controls and create protected obstacles. Build planner inputs.
% Call the public planner once. Then run independent validation on its result.
% On failure, inspect the planner termination reason before validation details.
cancelInteraction(figureHandle);
applicationState = guidata(figureHandle);
modeState = applicationState.GoalMode;
if isempty(modeState.StartPosition_deg) || isempty(modeState.GoalPosition_deg)
    error("obstacleAvoidanceSandbox:IncompleteGoalScene", "Goal Mode requires both a start point and a goal point.");
end
controls = readModeControls(applicationState, "goal");
obstacleTime_s = [0; controls.MissionTime_s];
canonicalObstacles = buildCanonicalObstacles( modeState, obstacleTime_s, controls);
[initialState, goalState, limits] = buildPlannerInputs( ...
    modeState.StartPosition_deg, modeState.GoalPosition_deg, ...
    0, controls.MissionTime_s, controls);
plannerOptions = applicationState.Options.PlannerOptions;
plannerOptions.GoalTimeMode = controls.GoalTimeMode;
plannerOptions.MinimumTravelSavingsRate_deg_s = ...
    controls.MinimumTravelSavingsRate_deg_s;
plannerOptions.UnsupportedTimedTopologyPolicy = ...
    controls.UnsupportedTimedTopologyPolicy;
plannerOptions.AllowAzimuthWrapping = controls.AllowAzimuthWrapping;
callerCancellationCheckFcn = plannerOptions.CancellationCheckFcn;
setappdata(figureHandle, "SandboxStopRequested", false);
plannerOptions.CancellationCheckFcn = ...
    @() sandboxStopRequested(figureHandle, callerCancellationCheckFcn);
stopCleanup = onCleanup(@() clearPlanningStopRequest(figureHandle));
modeState = clearModeSolution(modeState);
modeState.CanonicalObstacles = canonicalObstacles;
modeState.ResolvedControls = controls;
modeState.PlannerLog = "[Goal Mode]";
modeState.Status = "Planning...";
modeState.InteractionState = "planning";
applicationState.GoalMode = modeState;
applicationState.InteractionState = "planning";
guidata(figureHandle, applicationState);
refreshApplication(figureHandle);
drawnow;
[result, validation, logLines] = callPlanner( ...
    canonicalObstacles, initialState, goalState, limits, ...
    plannerOptions, controls.Verbose, "Goal Mode");
result.Options.CancellationCheckFcn = [];
applicationState = guidata(figureHandle);
modeState = applicationState.GoalMode;
modeState.LastPlannerResult = result;
modeState.LastValidation = validation;
modeState.PlannerLog = [modeState.PlannerLog; logLines];
modeState.Status = formatGoalStatus(result, validation);
modeState.InteractionState = "idle";
applicationState.GoalMode = modeState;
applicationState.InteractionState = "idle";
guidata(figureHandle, applicationState);
refreshApplication(figureHandle);
playGoalAnimationAfterRun(figureHandle, result, validation);
end

function playGoalAnimationAfterRun(figureHandle, result, validation)
% Play a validated returned trajectory without rerunning planning. The public
% plotter owns obstacle-time queries and synchronized motion visualization.
applicationState = guidata(figureHandle);
options = applicationState.Options;
if ~result.Success || ~validation.Passed || ~options.AnimateOnRun || ...
        options.FigureVisible ~= "on" || ~isgraphics(figureHandle)
    return;
end
modeState = applicationState.GoalMode;
finalStatus = modeState.Status;
modeState.Status = "Plan validated. Playing solved-motion animation...";
applicationState.GoalMode = modeState;
guidata(figureHandle, applicationState);
updateModeStatusDisplay(modeState);
drawnow;
plotOptions = struct( ...
    "FigureVisible", "on", ...
    "Title", "Goal Mode solved motion", ...
    "ShowWorkspace", false, ...
    "ShowKinematics", false, ...
    "ShowAnimation", true, ...
    "ShowSearchEdges", false, ...
    "ShowVisibilityGraphs", false, ...
    "ShowSweptSurfaces", false, ...
    "FrameStride", options.AnimationFrameStride, ...
    "Pause_s", options.AnimationPause_s);
try
    animationHandles = obstacleAvoidance.plotting.plotTrajectory( ...
        result, plotOptions);
    applicationState = guidata(figureHandle);
    modeState = applicationState.GoalMode;
    modeState.GraphicsHandles.AnimationPlotHandles = animationHandles;
    modeState.Status = finalStatus;
catch exception
    applicationState = guidata(figureHandle);
    modeState = applicationState.GoalMode;
    modeState.Status = finalStatus + newline + ...
        "Animation could not be played: " + string(exception.message);
    modeState = appendLogLines(modeState, ...
        "[Animation error] " + string(formatSandboxException(exception)));
end
applicationState.GoalMode = modeState;
guidata(figureHandle, applicationState);
refreshApplication(figureHandle);
end

function [result, validation, logLines] = callPlanner( ...
        obstacles, initialState, goalState, limits, options, ...
        captureVerbose, labelText)
% Capture optional verbose text for the sandbox log. Keep structured planner
% diagnostics in the returned result.
result = struct();
plannerText = "";
if captureVerbose
    plannerText = string(evalc( 'result = obstacleAvoidance.planTrajectory(obstacles, initialState, goalState, limits, options);'));
else
    result = obstacleAvoidance.planTrajectory( obstacles, initialState, goalState, limits, options);
end
if result.Success
    validation = obstacleAvoidance.validateTrajectory(result);
else
    validation = result.Validation;
end
logLines = string(labelText);
capturedLines = splitlines(plannerText);
capturedLines = capturedLines(strlength(strtrim(capturedLines)) > 0);
if ~isempty(capturedLines)
    logLines = [logLines; capturedLines];
end
logLines(end + 1, 1) = sprintf( ...
    "Result: success=%s, validation=%s, reason=%s, arrival=%.6g s", ...
    logicalText(result.Success), logicalText(validation.Passed), ...
    result.TerminationReason, ...
    result.ArrivalTime_s);
end

function [initialState, goalState, limits] = buildPlannerInputs( ...
        startPosition_deg, stopPosition_deg, startTime_s, goalTime_s, controls)
% Build initial and goal states with zero endpoint velocity and acceleration.
% Build explicit physical limits and workspace bounds from current controls.
initialState = struct( ...
    "time_s", startTime_s, ...
    "position_deg", reshape(startPosition_deg, 1, 2), ...
    "velocity_deg_s", [0 0], ...
    "acceleration_deg_s2", [0 0]);
goalState = struct( ...
    "time_s", goalTime_s, ...
    "position_deg", reshape(stopPosition_deg, 1, 2), ...
    "velocity_deg_s", [0 0], ...
    "acceleration_deg_s2", [0 0]);
limits = struct( ...
    "maxVelocity_deg_s", controls.MaxVelocity_deg_s, ...
    "maxAcceleration_deg_s2", controls.MaxAcceleration_deg_s2, ...
    "maxJerk_deg_s3", controls.MaxJerk_deg_s3, ...
    "azimuthInterval_deg", controls.WorkspaceAzimuthInterval_deg, ...
    "elevationInterval_deg", controls.WorkspaceElevationInterval_deg);
end

function controls = readModeControls(applicationState, modeName)
% Read physical and geometry controls from one tab. Validate them before any
% obstacle construction or planner call.
modeState = getModeState(applicationState, modeName);
handles = modeState.GraphicsHandles.Controls;
workspaceAzimuthInterval_deg = readAxisPairControl( ...
    handles.WorkspaceAzimuthHandles, ...
    "Workspace azimuth interval (deg)", false);
workspaceElevationInterval_deg = readAxisPairControl( ...
    handles.WorkspaceElevationHandles, ...
    "Workspace elevation interval (deg)", false);
if workspaceAzimuthInterval_deg(2) <= workspaceAzimuthInterval_deg(1)
    error("obstacleAvoidanceSandbox:InvalidAzimuthInterval", "Workspace azimuth lower bound must be below the upper bound.");
end
if workspaceElevationInterval_deg(2) <= workspaceElevationInterval_deg(1)
    error("obstacleAvoidanceSandbox:InvalidElevationInterval", ...
        "Workspace elevation lower bound must be below the upper bound.");
end
maxVelocity_deg_s = readAxisPairControl( handles.VelocityHandles, "Maximum velocity (deg/s)", true);
maxAcceleration_deg_s2 = readAxisPairControl( handles.AccelerationHandles, "Maximum acceleration (deg/s^2)", true);
maxJerk_deg_s3 = readAxisPairControl( handles.JerkHandles, "Maximum jerk (deg/s^3)", true);
horizon_s = readScalarControl( handles.HorizonHandle, "Planning horizon (s)", true);
pathObstacleRadius_deg = readScalarControl( handles.PathRadiusHandle, "Line/capsule radius (deg)", true);
pathSafetyMargin_deg = readScalarControl( handles.PathMarginHandle, "Line safety margin (deg)", false);
obstacleSafetyMargin_deg = readScalarControl( handles.ObstacleMarginHandle, "Polygon safety margin (deg)", false);
goalTimeModes = ["balancedArrival", "earliestArrival", "fixedArrival"];
minimumTravelSavingsRate_deg_s = readScalarControl( ...
    handles.MinimumTravelSavingsRateHandle, ...
    "Minimum travel savings rate (deg/s)", true);
unsupportedTimedTopologyPolicies = ["fail", "ruckigStopAtWaypoints"];
controls = struct( ...
    "WorkspaceAzimuthInterval_deg", workspaceAzimuthInterval_deg, ...
    "WorkspaceElevationInterval_deg", workspaceElevationInterval_deg, ...
    "MaxVelocity_deg_s", maxVelocity_deg_s, ...
    "MaxAcceleration_deg_s2", maxAcceleration_deg_s2, ...
    "MaxJerk_deg_s3", maxJerk_deg_s3, ...
    "MissionTime_s", horizon_s, ...
    "PathObstacleRadius_deg", pathObstacleRadius_deg, ...
    "PathSafetyMargin_deg", pathSafetyMargin_deg, ...
    "ObstacleSafetyMargin_deg", obstacleSafetyMargin_deg, ...
    "GoalTimeMode", goalTimeModes( ...
        get(handles.GoalTimeModeHandle, "Value")), ...
    "MinimumTravelSavingsRate_deg_s", ...
    minimumTravelSavingsRate_deg_s, ...
    "UnsupportedTimedTopologyPolicy", ...
    unsupportedTimedTopologyPolicies( ...
        get(handles.UnsupportedTimedTopologyHandle, "Value")), ...
    "AllowAzimuthWrapping", logical(get( ...
        handles.AllowAzimuthWrappingHandle, "Value")), ...
    "Verbose", logical(get(handles.VerboseHandle, "Value")));
end

function stopRequested = sandboxStopRequested( ...
        figureHandle, callerCancellationCheckFcn)
% Combine the Stop button with any programmatic caller cancellation policy.
if isempty(figureHandle) || ~isgraphics(figureHandle)
    stopRequested = true;
    return;
end
stopRequested = isappdata(figureHandle, "SandboxStopRequested") && ...
    logical(getappdata(figureHandle, "SandboxStopRequested"));
if ~stopRequested && ~isempty(callerCancellationCheckFcn)
    stopRequested = callerCancellationCheckFcn();
end
end

function clearPlanningStopRequest(figureHandle)
% Remove the transient callback flag from a surviving sandbox figure.
if ~isempty(figureHandle) && isgraphics(figureHandle) && ...
        isappdata(figureHandle, "SandboxStopRequested")
    rmappdata(figureHandle, "SandboxStopRequested");
end
end

function values = readAxisPairControl(handles, labelText, mustBePositive)
% Parse one finite two-value edit pair with a named error context.
values = [ str2double(get(handles.FirstHandle, "String")), str2double(get(handles.SecondHandle, "String"))];
validateattributes(values, {'numeric'}, {'real', 'finite', 'vector', 'numel', 2}, "obstacleAvoidanceSandbox", labelText);
if mustBePositive && any(values <= 0)
    error("obstacleAvoidanceSandbox:NonpositiveControl", "%s values must both be positive.", labelText);
end
values = reshape(double(values), 1, 2);
end

function value = readScalarControl(handle, labelText, mustBePositive)
% Parse one finite scalar edit value with positive/nonnegative policy.
value = str2double(get(handle, "String"));
validateattributes(value, {'numeric'}, {'real', 'finite', 'scalar'}, "obstacleAvoidanceSandbox", labelText);
if mustBePositive && value <= 0
    error("obstacleAvoidanceSandbox:NonpositiveControl", "%s must be positive.", labelText);
elseif ~mustBePositive && value < 0
    error("obstacleAvoidanceSandbox:NegativeControl", "%s must be nonnegative.", labelText);
end
value = double(value);
end

function applyDefaultControls(handles, options)
% Restore one tab's editable values without touching the other tab.
writeAxisPair(handles.WorkspaceAzimuthHandles, options.WorkspaceAzimuthInterval_deg);
writeAxisPair(handles.WorkspaceElevationHandles, options.WorkspaceElevationInterval_deg);
writeAxisPair(handles.VelocityHandles, options.MaxVelocity_deg_s);
writeAxisPair(handles.AccelerationHandles, options.MaxAcceleration_deg_s2);
writeAxisPair(handles.JerkHandles, options.MaxJerk_deg_s3);
horizon_s = options.MissionTime_s;
set(handles.HorizonHandle, "String", sprintf("%.8g", horizon_s));
set(handles.PathRadiusHandle, "String", sprintf("%.8g", options.PathObstacleRadius_deg));
set(handles.PathMarginHandle, "String", sprintf("%.8g", options.PathSafetyMargin_deg));
set(handles.ObstacleMarginHandle, "String", sprintf("%.8g", options.ObstacleSafetyMargin_deg));
set(handles.VerboseHandle, "Value", options.Verbose);
set(handles.MotionProfileHandle, "Value", 1);
set(handles.UnsupportedTimedTopologyHandle, "Value", 1 + double( ...
    options.PlannerOptions.UnsupportedTimedTopologyPolicy == ...
    "ruckigStopAtWaypoints"));
set(handles.GoalTimeModeHandle, "Value", find( ...
    options.PlannerOptions.GoalTimeMode == ...
    ["balancedArrival", "earliestArrival", "fixedArrival"], 1));
set(handles.MinimumTravelSavingsRateHandle, "String", sprintf("%.8g", ...
    options.PlannerOptions.MinimumTravelSavingsRate_deg_s));
set(handles.AllowAzimuthWrappingHandle, "Value", ...
    options.PlannerOptions.AllowAzimuthWrapping);
end

function writeAxisPair(handles, values)
% Write one two-value default into explicit edit handles.
set(handles.FirstHandle, "String", sprintf("%.8g", values(1)));
set(handles.SecondHandle, "String", sprintf("%.8g", values(2)));
end

function obstacles = buildCanonicalObstacles( modeState, obstacleTime_s, controls)
% Rebuild protected obstacles from retained geometry. Apply each safety margin
% one time during canonical obstacle construction.
pathObstacleData = cell(numel(modeState.LineObstaclePositions_deg), 1);

% Convert each drawn line to a capsule obstacle. A capsule is the area within a
% fixed radius of the line, with rounded ends.
for lineIndex = 1:numel(modeState.LineObstaclePositions_deg)
    pathObstacleData{lineIndex} = pathToObstacleData( ...
        modeState.LineObstaclePositions_deg{lineIndex}, obstacleTime_s, ...
        controls.PathObstacleRadius_deg, ...
        controls.PathSafetyMargin_deg, lineIndex);
end
polygonObstacleData = obstaclePolygonsToData( ...
    modeState.PolygonObstaclePositions_deg, obstacleTime_s, ...
    controls.ObstacleSafetyMargin_deg, ...
    modeState.PolygonMotionVectors_deg, ...
    modeState.PolygonMotionProfiles);
obstacles = obstacleAvoidance.obstacles.combineObstacles(pathObstacleData, polygonObstacleData);
end

function obstacleData = pathToObstacleData( path_deg, time_s, radius_deg, safetyMargin_deg, lineIndex)
% Convert one simplified freehand line into a continuous capsule with few
% vertices. Fewer vertices reduce planning cost.
path_deg = double(path_deg);
path_deg = path_deg(all(isfinite(path_deg), 2), :);
if size(path_deg, 1) > 1
    isDistinctPoint = [true; vecnorm(diff(path_deg, 1, 1), 2, 2) > eps];
    path_deg = path_deg(isDistinctPoint, :);
end
if size(path_deg, 1) < 2
    obstacleData = cell(0, 1);
    return;
end
pathShape = buildPathCapsuleShape(path_deg, radius_deg);
vertices_deg = pathShape.Vertices;
if size(vertices_deg, 1) < 3
    obstacleData = cell(0, 1);
    return;
end
obstacleData = obstacleAvoidance.obstacles.createObstacle( ...
    "drawn line obstacle " + lineIndex, time_s, ...
    vertices_deg(:, 1), vertices_deg(:, 2), safetyMargin_deg);
end

function pathShape = buildPathCapsuleShape(path_deg, radius_deg)
% Combine segment capsules into one shape without gaps.
endCapSegmentCount = 3;
arcIncrement_rad = pi / endCapSegmentCount;
% Increase the construction radius to account for straight polygon edges. The
% polygonal end cap then contains the requested circular radius.
constructionRadius_deg = radius_deg / cos(arcIncrement_rad / 2);
pathShape = polyshape();
hasSegment = false;

% Union one low-vertex capsule per nondegenerate path segment.
for segmentIndex = 1:size(path_deg, 1) - 1
    startPosition_deg = path_deg(segmentIndex, :);
    endPosition_deg = path_deg(segmentIndex + 1, :);
    direction_deg = endPosition_deg - startPosition_deg;
    segmentLength_deg = norm(direction_deg);
    if segmentLength_deg <= eps
        continue;
    end
    direction_1 = direction_deg / segmentLength_deg;
    directionAngle_rad = atan2(direction_1(2), direction_1(1));
    startAngles_rad = linspace( directionAngle_rad + pi / 2, directionAngle_rad + 3 * pi / 2, endCapSegmentCount + 1).';
    endAngles_rad = linspace( directionAngle_rad - pi / 2, directionAngle_rad + pi / 2, endCapSegmentCount + 1).';
    startCap_deg = startPosition_deg + constructionRadius_deg * [cos(startAngles_rad), sin(startAngles_rad)];
    endCap_deg = endPosition_deg + constructionRadius_deg * [cos(endAngles_rad), sin(endAngles_rad)];
    segmentShape = polyshape([startCap_deg; endCap_deg]);
    if hasSegment
        pathShape = union(pathShape, segmentShape);
    else
        pathShape = segmentShape;
        hasSegment = true;
    end
end
end

function obstacleData = obstaclePolygonsToData( ...
        polygonCollection_deg, time_s, safetyMargin_deg, ...
        motionVectors_deg, motionProfiles)
% Construct protected static or moving polygons. Apply the margin once.
obstacleData = cell(numel(polygonCollection_deg), 1);

% Close and canonicalize every user-drawn polygon independently.
for polygonIndex = 1:numel(polygonCollection_deg)
    polygon_deg = polygonCollection_deg{polygonIndex};
    if ~isequal(polygon_deg(1, :), polygon_deg(end, :))
        polygon_deg = [polygon_deg; polygon_deg(1, :)]; %#ok<AGROW>
    end
    motionVector_deg = [0 0];
    motionProfile = "stationary";
    if polygonIndex <= size(motionVectors_deg, 1)
        motionVector_deg = motionVectors_deg(polygonIndex, :);
    end
    if polygonIndex <= numel(motionProfiles) && ...
            strlength(motionProfiles(polygonIndex)) > 0
        motionProfile = motionProfiles(polygonIndex);
    end
    [profileTime_s, azimuthBySlice_deg, elevationBySlice_deg] = ...
        createSandboxPolygonMotionHistory( ...
            polygon_deg, time_s, motionVector_deg, motionProfile);
    obstacleData{polygonIndex} = ...
        obstacleAvoidance.obstacles.createObstacle( ...
            "drawn polygon obstacle " + polygonIndex, profileTime_s, ...
            azimuthBySlice_deg, elevationBySlice_deg, safetyMargin_deg);
end
end

function simplified_deg = simplifyFreehandBoundary(points_deg, tolerance_deg)
% Keep points that define turns. Remove dense mouse-event samples along nearly
% straight parts. If geometry loses an important corner, inspect this tolerance.
if size(points_deg, 1) <= 2
    simplified_deg = points_deg;
    return;
end
lineVector_deg = points_deg(end, :) - points_deg(1, :);
lineLength_deg = norm(lineVector_deg);
interiorPoints_deg = points_deg(2:end - 1, :);
if lineLength_deg <= eps
    distance_deg = vecnorm( interiorPoints_deg - points_deg(1, :), 2, 2);
else
    relativePoints_deg = interiorPoints_deg - points_deg(1, :);
    distance_deg = abs( ...
        relativePoints_deg(:, 1) * lineVector_deg(2) - ...
        relativePoints_deg(:, 2) * lineVector_deg(1)) / lineLength_deg;
end
[maximumDistance_deg, maximumIndex] = max(distance_deg);
if maximumDistance_deg <= tolerance_deg
    simplified_deg = points_deg([1 end], :);
    return;
end
splitIndex = maximumIndex + 1;
firstHalf_deg = simplifyFreehandBoundary( points_deg(1:splitIndex, :), tolerance_deg);
secondHalf_deg = simplifyFreehandBoundary( points_deg(splitIndex:end, :), tolerance_deg);
simplified_deg = [firstHalf_deg(1:end - 1, :); secondHalf_deg];
end

% --- Rendering, Status, Reset, And State Access -------------------------

function refreshApplication(figureHandle)
% Redraw Goal Mode and update status text and enabled controls.
if isempty(figureHandle) || ~isgraphics(figureHandle)
    return;
end
applicationState = guidata(figureHandle);
redrawMode(applicationState, "goal");
updateModeStatusDisplay(applicationState.GoalMode);
updateControlEnablement(applicationState);
end

function redrawMode(applicationState, modeName)
% Redraw one tab from retained data only. Do not call the planner during redraw.
modeState = getModeState(applicationState, modeName);
axesHandle = modeState.GraphicsHandles.Axes;
if ~isgraphics(axesHandle)
    return;
end

% Hide raw strokes and temporary traces from the legend. Delete every axes child
% explicitly. cla can preserve hidden objects and leave old obstacle outlines.
delete(allchild(axesHandle));
cla(axesHandle, "reset");
hold(axesHandle, "on");
grid(axesHandle, "on");
box(axesHandle, "on");
try
    controls = readModeControls(applicationState, modeName);
    azimuthInterval_deg = controls.WorkspaceAzimuthInterval_deg;
    elevationInterval_deg = controls.WorkspaceElevationInterval_deg;
catch
    azimuthInterval_deg = applicationState.Options.WorkspaceAzimuthInterval_deg;
    elevationInterval_deg = applicationState.Options.WorkspaceElevationInterval_deg;
    controls = struct( ...
        "WorkspaceAzimuthInterval_deg", azimuthInterval_deg, ...
        "AllowAzimuthWrapping", ...
        applicationState.Options.PlannerOptions.AllowAzimuthWrapping);
end
axis(axesHandle, [azimuthInterval_deg, elevationInterval_deg]);
axis(axesHandle, "equal");
xlabel(axesHandle, "Azimuth (deg)");
ylabel(axesHandle, "Elevation (deg)");

% Redraw raw mouse traces first as low-emphasis provenance.
for strokeIndex = 1:numel(modeState.RawObstacleStrokes_deg)
    stroke_deg = modeState.RawObstacleStrokes_deg{strokeIndex};
    plot(axesHandle, stroke_deg(:, 1), stroke_deg(:, 2), ...
        ":", "Color", [0.45 0.45 0.65], "LineWidth", 0.8, ...
        "HandleVisibility", "off");
end

% Overlay simplified line centerlines used to construct capsule obstacles.
% Obstacle graphics stay out of the legend because their count can be large.
for lineIndex = 1:numel(modeState.LineObstaclePositions_deg)
    line_deg = modeState.LineObstaclePositions_deg{lineIndex};
    plot(axesHandle, line_deg(:, 1), line_deg(:, 2), ...
        "--", "Color", [0.75 0.15 0.15], "LineWidth", 1.4, ...
        "HandleVisibility", "off");
end

% Draw user polygon boundaries before protected obstacle geometry. This shows
% how canonicalization and safety margin change the drawn shape.
for polygonIndex = 1:numel(modeState.PolygonObstaclePositions_deg)
    polygon_deg = modeState.PolygonObstaclePositions_deg{polygonIndex};
    fill(axesHandle, polygon_deg(:, 1), polygon_deg(:, 2), ...
        [0.25 0.35 0.85], "FaceAlpha", 0.10, ...
        "EdgeColor", [0.25 0.35 0.85], "LineStyle", ":", ...
        "HandleVisibility", "off");
    if polygonIndex <= size(modeState.PolygonMotionVectors_deg, 1)
        motionVector_deg = ...
            modeState.PolygonMotionVectors_deg(polygonIndex, :);
        if norm(motionVector_deg) > 1e-12
            [centroidAzimuth_deg, centroidElevation_deg] = ...
                centroid(polyshape(polygon_deg));
            quiver(axesHandle, centroidAzimuth_deg, ...
                centroidElevation_deg, motionVector_deg(1), ...
                motionVector_deg(2), 0, "Color", [0.15 0.55 0.15], ...
                "LineWidth", 2, "MaxHeadSize", 0.35, ...
                "HandleVisibility", "off");
            if polygonIndex <= numel(modeState.PolygonMotionProfiles)
                profileLabel = motionProfileLabel( ...
                    modeState.PolygonMotionProfiles(polygonIndex));
                text(axesHandle, ...
                    centroidAzimuth_deg + motionVector_deg(1), ...
                    centroidElevation_deg + motionVector_deg(2), ...
                    "  " + profileLabel, "Color", [0.10 0.40 0.10], ...
                    "FontSize", 8, "HandleVisibility", "off");
            end
        end
    end
end
renderCanonicalObstacles(axesHandle, modeState.CanonicalObstacles);

if ~isempty(modeState.StartPosition_deg)
    plot(axesHandle, modeState.StartPosition_deg(1), ...
        modeState.StartPosition_deg(2), "go", ...
        "MarkerFaceColor", "g", "MarkerSize", 8, ...
        "LineWidth", 1.2, "DisplayName", "Start");
end
redrawGoalRequest(axesHandle, modeState, controls);
if ~isempty(findobj(axesHandle, "-property", "DisplayName"))
    legend(axesHandle, "Location", "best");
end
title(axesHandle, "Goal Mode: requested geometry and solved motion");
end

function label = motionProfileLabel(profile)
% Convert an internal profile name to concise display text.
switch string(profile)
    case "nonzeroVelocity"
        label = "constant velocity";
    case "zeroStart"
        label = "zero-start acceleration";
    case "trapezoidal"
        label = "trapezoidal";
    case "oscillating"
        label = "oscillating";
    otherwise
        label = "stationary";
end
end

function redrawGoalRequest(axesHandle, modeState, controls)
% Show Goal Mode request geometry. Show the result or retained partial route
% when available.
if ~isempty(modeState.GoalPosition_deg)
    plot(axesHandle, modeState.GoalPosition_deg(1), ...
        modeState.GoalPosition_deg(2), "ro", ...
        "MarkerFaceColor", "r", "MarkerSize", 8, ...
        "LineWidth", 1.2, "DisplayName", "Goal");
end
if ~isempty(modeState.StartPosition_deg) && ~isempty(modeState.GoalPosition_deg)
    request_deg = [modeState.StartPosition_deg; modeState.GoalPosition_deg];
    if controls.AllowAzimuthWrapping
        period_deg = diff(controls.WorkspaceAzimuthInterval_deg);
        request_deg(2, 1) = request_deg(2, 1) + period_deg * round( ...
            (request_deg(1, 1) - request_deg(2, 1)) / period_deg);
    end
    request_deg = obstacleAvoidance.plotting.createWrappedSpatialPath( ...
        request_deg, controls.WorkspaceAzimuthInterval_deg, ...
        controls.AllowAzimuthWrapping);
    plot(axesHandle, request_deg(:, 1), request_deg(:, 2), ...
        "--", "Color", [0.35 0.55 0.85], "LineWidth", 1.2, ...
        "DisplayName", "Requested direct geometry");
end
result = modeState.LastPlannerResult;
if isempty(fieldnames(result))
    return;
end
if result.Success
    displayPosition_deg = ...
        obstacleAvoidance.plotting.createWrappedSpatialPath( ...
        result.position_deg, result.Inputs.limits.azimuthInterval_deg, ...
        result.Options.AllowAzimuthWrapping);
    plot(axesHandle, displayPosition_deg(:, 1), ...
        displayPosition_deg(:, 2), ...
        "k-", "LineWidth", 2.4, "DisplayName", "Solved motion");
elseif result.SearchDiagnostics.BestPartialSeedIndex > 0
    partialIndex = result.SearchDiagnostics.BestPartialSeedIndex;
    partialRoute_deg = result.Seeds(partialIndex).position_deg;
    partialRoute_deg = ...
        obstacleAvoidance.plotting.createWrappedSpatialPath( ...
        partialRoute_deg, result.Inputs.limits.azimuthInterval_deg, ...
        result.Options.AllowAzimuthWrapping);
    plot(axesHandle, partialRoute_deg(:, 1), partialRoute_deg(:, 2), ...
        "-.", "Color", [0.90 0.55 0.10], "LineWidth", 1.8, ...
        "DisplayName", "Best partial route");
end
end

function renderCanonicalObstacles(axesHandle, obstacles)
% Draw obstacle geometry without allowing obstacle count to expand the legend.
for obstacleIndex = 1:numel(obstacles)
    originalShape = polyshape( obstacles(obstacleIndex).originalAz_deg{1}, obstacles(obstacleIndex).originalEl_deg{1});
    protectedShape = polyshape( obstacles(obstacleIndex).az_deg{1}, obstacles(obstacleIndex).el_deg{1});
    plot(axesHandle, originalShape, ...
        "FaceColor", [0.80 0.82 0.86], "FaceAlpha", 0.25, ...
        "EdgeColor", [0.20 0.20 0.20], "LineStyle", "-", ...
        "LineWidth", 1.0, "HandleVisibility", "off");
    plot(axesHandle, protectedShape, ...
        "FaceColor", "none", "EdgeColor", [0.80 0.15 0.15], ...
        "LineStyle", "--", "LineWidth", 1.5, ...
        "HandleVisibility", "off");
end
end

function updateModeStatusDisplay(modeState)
% Show a short status above the complete retained log for this mode.
statusLines = splitlines(string(modeState.Status));
statusLines = statusLines(strlength(statusLines) > 0);
if isempty(statusLines)
    statusLines = "Ready";
end
set(modeState.GraphicsHandles.StatusHandle, "String", cellstr(statusLines));
logLines = modeState.PlannerLog;
if isempty(logLines)
    logLines = "Planner output will appear here.";
end
set(modeState.GraphicsHandles.LogHandle, "String", cellstr(logLines), "Value", 1);
end

function updateControlEnablement(applicationState)
% Disable actions that are not valid for current state. Disable all geometry
% edits during a synchronous planner call.
isPlanning = applicationState.InteractionState == "planning";
modeState = applicationState.GoalMode;
    actionNames = string(fieldnames(modeState.GraphicsHandles.Actions));

    % Start from the common planning lock before applying mode-specific availability.
    for actionName = reshape(actionNames, 1, [])
        set(modeState.GraphicsHandles.Actions.(actionName), "Enable", onOff(~isPlanning));
    end
    controlHandles = findall( modeState.GraphicsHandles.ControlPanel, "Type", "uicontrol");
    set(controlHandles, "Enable", onOff(~isPlanning));
    set(modeState.GraphicsHandles.Controls.VerboseHandle, "Enable", onOff(~isPlanning));
    set(modeState.GraphicsHandles.Controls.MotionProfileHandle, ...
        "Enable", onOff(~isPlanning));
    if isPlanning
        set(modeState.GraphicsHandles.Actions.Stop, "Enable", "on");
        return;
    end
    canRun = ~isempty(modeState.StartPosition_deg) && ...
        ~isempty(modeState.GoalPosition_deg);
    setObstacleConstructorAvailability(modeState, canRun);
    set(modeState.GraphicsHandles.Actions.SetMotion, "Enable", ...
        onOff(~isempty(modeState.PolygonObstaclePositions_deg)));
    set(modeState.GraphicsHandles.Actions.Run, "Enable", onOff(canRun));
    set(modeState.GraphicsHandles.Actions.Stop, "Enable", "off");
    hasResult = ~isempty(fieldnames(modeState.LastPlannerResult));
    hasSceneData = ~isempty(modeState.StartPosition_deg) || ...
        ~isempty(modeState.GoalPosition_deg) || ...
        ~isempty(modeState.RawObstacleStrokes_deg);
    set(modeState.GraphicsHandles.Actions.Diagnostics, "Enable", onOff(hasResult));
    set(modeState.GraphicsHandles.Actions.Export, ...
        "Enable", onOff(hasResult || hasSceneData));
end

function value = onOff(condition)
% Convert one logical UI decision into MATLAB's on/off text form.
if condition
    value = "on";
else
    value = "off";
end
end

function status = formatGoalStatus(result, validation)
% Report Goal Mode success or failure. Show unavailable values as unavailable.
status = [ ...
    "Success: " + logicalText(result.Success); ...
    "TerminationReason: " + result.TerminationReason; ...
    "ArrivalTime_s: " + sprintf("%.6g", result.ArrivalTime_s); ...
    "TrajectoryDuration_s: " + ...
        sprintf("%.6g", result.TrajectoryDuration_s); ...
    "Independent validation: " + logicalText(validation.Passed)];
end

function text = formatNumericVector(values)
% Format one diagnostic vector without hiding NaN or unavailable values.
if isempty(values)
    text = "";
else
    text = strtrim(string(sprintf("%.6g ", values)));
end
end

function modeState = appendLogLines(modeState, lines)
% Append nonempty mode-specific log lines while preserving segment order.
lines = splitlines(string(lines));
lines = lines(strlength(strtrim(lines)) > 0);
modeState.PlannerLog = [modeState.PlannerLog; lines];
end

function modeState = clearModeSolution(modeState)
% Clear solved data after geometry or controls change. An old plan is not valid
% for a changed request.
modeState.CanonicalObstacles = obstacleAvoidance.obstacles.combineObstacles();
modeState.LastPlannerResult = struct();
modeState.LastValidation = obstacleAvoidance.validateTrajectory();
modeState.ResolvedControls = struct();
end

function resetMode(figureHandle, modeName)
% Reset one tab. Do not change the other tab.
cancelInteraction(figureHandle);
applicationState = guidata(figureHandle);
modeState = getModeState(applicationState, modeName);
graphicsHandles = modeState.GraphicsHandles;
applyDefaultControls(graphicsHandles.Controls, applicationState.Options);
modeState = emptyModeState(graphicsHandles);
applicationState = setModeState(applicationState, modeName, modeState);
guidata(figureHandle, applicationState);
beginGuidedScene(figureHandle, modeName);
end

function openDiagnostics(figureHandle, modeName)
% Plot retained planner diagnostics. Do not run the planner again. If no result
% exists, explain that status instead of creating replacement data.
applicationState = guidata(figureHandle);
modeState = getModeState(applicationState, modeName);
result = modeState.LastPlannerResult;
if isempty(fieldnames(result))
    error("obstacleAvoidanceSandbox:NoDiagnosticResult", "No planner result is available for %s mode.", modeName);
end
plotOptions = struct( ...
    "FigureVisible", applicationState.Options.FigureVisible, ...
    "Title", upperFirst(modeName) + " Mode diagnostics", ...
    "ShowSeedPaths", true, ...
    "ShowAnimation", false);
modeState.GraphicsHandles.DiagnosticPlotHandles = obstacleAvoidance.plotting.plotTrajectory(result, plotOptions);
applicationState = setModeState(applicationState, modeName, modeState);
guidata(figureHandle, applicationState);
end

function exportModeDiagnosis(figureHandle, modeName)
% Save retained scene, input, result, and validation data for diagnosis.
timestamp = string(datetime('now', 'Format', 'yyyyMMdd_HHmmss'));
defaultName = "az_el_sandbox_" + modeName + "_" + timestamp + ".mat";
[fileName, folderName] = uiputfile( ...
    {'*.mat', 'MATLAB diagnosis bundle (*.mat)'}, ...
    'Export sandbox input and result', char(defaultName));
if isequal(fileName, 0) || isequal(folderName, 0)
    return;
end
exportInfo = exportCurrentSandboxDiagnosis( ...
    figureHandle, fullfile(folderName, fileName), modeName);
applicationState = guidata(figureHandle);
modeState = getModeState(applicationState, modeName);
modeState.Status = "Diagnosis bundle exported: " + exportInfo.FilePath;
modeState = appendLogLines(modeState, ...
    "[Sandbox export] " + exportInfo.FilePath + ...
    " (" + exportInfo.Bytes + " bytes)");
applicationState = setModeState( ...
    applicationState, modeName, modeState);
guidata(figureHandle, applicationState);
refreshApplication(figureHandle);
if get(figureHandle, "Visible") == "on"
    confirmationMessage = sprintf( ...
        'Saved %.0f bytes to:\n%s', ...
        exportInfo.Bytes, char(exportInfo.FilePath));
    msgbox( ...
        confirmationMessage, ...
        'Sandbox bundle exported', 'help', 'non-modal');
end
end

function exportInfo = exportCurrentSandboxDiagnosis( ...
        figureHandle, filePath, modeName)
% Export the current guidata-backed sandbox state to an explicit path.
if isempty(figureHandle) || ~isgraphics(figureHandle)
    error("obstacleAvoidanceSandbox:ClosedFigure", ...
        "The sandbox figure must remain open while exporting a bundle.");
end
applicationState = guidata(figureHandle);
applicationState = prepareSandboxStateForExport( ...
    applicationState, modeName);
exportInfo = exportSandboxDiagnosis( ...
    filePath, applicationState, modeName);
end

function applicationState = prepareSandboxStateForExport( ...
        applicationState, modeName)
% Capture current controls and geometry. Do not call the planner. This permits
% export before a run when input preparation itself is under investigation.
modeState = getModeState(applicationState, modeName);
controls = readModeControls(applicationState, modeName);
plannerOptions = applicationState.Options.PlannerOptions;
plannerOptions.UnsupportedTimedTopologyPolicy = ...
    controls.UnsupportedTimedTopologyPolicy;
plannerOptions.AllowAzimuthWrapping = controls.AllowAzimuthWrapping;
plannerOptions.CancellationCheckFcn = [];
plannerInputs = struct();
obstacleEndTime_s = controls.MissionTime_s;
hasCompleteScene = ~isempty(modeState.StartPosition_deg) && ...
    ~isempty(modeState.GoalPosition_deg);
plannerOptions.GoalTimeMode = controls.GoalTimeMode;
plannerOptions.MinimumTravelSavingsRate_deg_s = ...
    controls.MinimumTravelSavingsRate_deg_s;
if hasCompleteScene
    [initialState, goalState, limits] = buildPlannerInputs( ...
        modeState.StartPosition_deg, modeState.GoalPosition_deg, ...
        0, controls.MissionTime_s, controls);
    plannerInputs = struct( ...
        "obstacles", obstacleAvoidance.obstacles.combineObstacles(), ...
        "initialState", initialState, ...
        "goalState", goalState, ...
        "limits", limits);
end
canonicalObstacles = buildCanonicalObstacles( ...
    modeState, [0; obstacleEndTime_s], controls);
if isfield(plannerInputs, "obstacles")
    plannerInputs.obstacles = canonicalObstacles;
end
modeState.CanonicalObstacles = canonicalObstacles;
modeState.ResolvedControls = controls;
modeState.ExportRequest = struct( ...
    "HasCompleteScene", hasCompleteScene, ...
    "PlannerInputs", plannerInputs, ...
    "PlannerOptions", plannerOptions, ...
    "RequestedStart_deg", modeState.StartPosition_deg, ...
    "RequestedGoal_deg", modeState.GoalPosition_deg);
applicationState = setModeState(applicationState, modeName, modeState);
end

function modeState = getModeState(applicationState, modeName)
% Read Goal Mode and reject removed mode identifiers.
if modeName ~= "goal"
    error("obstacleAvoidanceSandbox:UnsupportedMode", ...
        "Only Goal Mode is supported.");
end
modeState = applicationState.GoalMode;
end

function applicationState = setModeState( ...
        applicationState, modeName, modeState)
% Write Goal Mode and reject removed mode identifiers.
if modeName ~= "goal"
    error("obstacleAvoidanceSandbox:UnsupportedMode", ...
        "Only Goal Mode is supported.");
end
applicationState.GoalMode = modeState;
end

function snapshot = publicStateSnapshot(figureHandle)
% Return current state without callback functions. This snapshot is safe for
% inspection and diagnosis export.
if isempty(figureHandle) || ~isgraphics(figureHandle)
    snapshot = struct( "FigureHandle", gobjects(0), "Status", "The sandbox figure is closed.");
    return;
end
snapshot = guidata(figureHandle);
snapshot.ExportBundle = @(filePath, modeName) ...
    exportCurrentSandboxDiagnosis(figureHandle, filePath, modeName);
end

function setObstacleConstructorAvailability(modeState, isEnabled)
% Keep every constructor in the Add panel synchronized with scene readiness.
constructorNames = [ ...
    "AddPolygon", "AddCircle", "AddHandDrawn", "AddSquare"];
for constructorName = constructorNames
    set(modeState.GraphicsHandles.Actions.(constructorName), ...
        "Enable", onOff(isEnabled));
end
end

function text = logicalText(value)
% Render scalar logical status as true or false for user-facing output.
if logical(value)
    text = "true";
else
    text = "false";
end
end
