function sandboxState = azElInteractiveSandbox(sandboxOverrides)
%% Section 0: Header & Readme
% SYNTAX
%   sandboxState = azElInteractiveSandbox()
%   sandboxState = azElInteractiveSandbox([])
%   sandboxState = azElInteractiveSandbox(sandboxOverrides)
%**************************************************************************
% PURPOSE
%   - Open one persistent azimuth/elevation planning UI with independent
%     Goal Mode and Free Mode tabs.
%   - Keep interactive scene construction and bounded latest-tested-arrival
%     search outside the maintained production planner.
%**************************************************************************
% INPUTS
%   - sandboxOverrides (scalar struct, optional; default struct())
%       Empty fields retain defaults. Supported fields are FigureVisible,
%       FigurePosition, MissionTime_s, MaximumSegmentDuration_s,
%       MaxVelocity_deg_s, MaxAcceleration_deg_s2, MaxJerk_deg_s3,
%       PathObstacleRadius_deg, PathSafetyMargin_deg,
%       ObstacleSafetyMargin_deg, WorkspaceAzimuthInterval_deg,
%       WorkspaceElevationInterval_deg, Verbose,
%       LatestArrivalCoarseCandidateCount,
%       LatestArrivalRefinementCandidateCount, and PlannerOptions.
%       PlannerOptions is a partial HS3 planAzElMotion options struct.
%       PlannerMethod may be omitted or equal "hs3". Goal-time mode and
%       verbosity are then owned by the active sandbox tab.
%**************************************************************************
% OUTPUTS
%   - sandboxState (scalar struct)
%       Initial plain-struct state, figure handle, independent mode records,
%       ReadState and ExportBundle function handles. Call ReadState() after
%       interaction to inspect the current guidata-backed state. Call
%       ExportBundle(filePath, modeName) to save without a file dialog.
%**************************************************************************
% UNITS
%   - Positions and obstacle boundaries are N-by-2 [azimuth elevation] in
%     degrees. Time is seconds and derivatives use deg/s, deg/s^2, and
%     deg/s^3.
%**************************************************************************

%% Section 1: Resolve Sandbox Defaults

if nargin < 1 || isempty(sandboxOverrides)
    sandboxOverrides = struct();
end
options = resolveSandboxOptions(sandboxOverrides);

%% Section 2: Create Figure And Tabs

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
freeTabHandle = uitab(tabGroupHandle, "Title", "Free Mode", "Tag", "free");
goalHandles = createModeTab(goalTabHandle, "goal", options);
freeHandles = createModeTab(freeTabHandle, "free", options);

%% Section 3: Initialize Goal Mode

applicationState = initializeApplicationState( figureHandle, options, goalHandles, freeHandles);
guidata(figureHandle, applicationState);

%% Section 4: Initialize Free Mode

applyDefaultControls(goalHandles.Controls, "goal", options);
applyDefaultControls(freeHandles.Controls, "free", options);
refreshApplication(figureHandle);
beginGuidedScene(figureHandle, "goal");

%% Section 5: Return Initial Sandbox State

sandboxState = publicStateSnapshot(figureHandle);
sandboxState.ReadState = @() publicStateSnapshot(figureHandle);

end

% --- Defaults And UI Construction ---------------------------------------

function defaults = sandboxDefaults()
% Define the single source of argument-independent sandbox defaults.
defaults = struct( ...
    "FigureVisible", "on", ...
    "FigurePosition", [60 60 1460 860], ...
    "MissionTime_s", 180, ...
    "MaximumSegmentDuration_s", 30, ...
    "MaxVelocity_deg_s", [2 2], ...
    "MaxAcceleration_deg_s2", [0.75 0.75], ...
    "MaxJerk_deg_s3", [2.5 2.5], ...
    "PathObstacleRadius_deg", 0.5, ...
    "PathSafetyMargin_deg", 0, ...
    "ObstacleSafetyMargin_deg", 0.2, ...
    "WorkspaceAzimuthInterval_deg", [-180 180], ...
    "WorkspaceElevationInterval_deg", [-90 90], ...
    "Verbose", true, ...
    "LatestArrivalCoarseCandidateCount", 5, ...
    "LatestArrivalRefinementCandidateCount", 3, ...
    "PlannerOptions", struct());
end

function options = resolveSandboxOptions(overrides)
% Merge partial overrides, warn once about unknown fields, and validate.
if ~isstruct(overrides) || ~isscalar(overrides)
    error("azElInteractiveSandbox:InvalidOverrides", "sandboxOverrides must be a scalar struct or empty.");
end
defaults = sandboxDefaults();
options = defaults;
knownNames = string(fieldnames(defaults));
overrideNames = string(fieldnames(overrides));
unknownNames = setdiff(overrideNames, knownNames, "stable");
if ~isempty(unknownNames)
    warning("azElInteractiveSandbox:UnknownOptions", ...
        "Ignoring unknown sandbox fields: %s. No behavior changed.", ...
        strjoin(unknownNames, ", "));
end

% Apply only recognized, nonempty overrides so empty fields preserve defaults.
for name = reshape(intersect(overrideNames, knownNames, "stable"), 1, [])
    if ~isempty(overrides.(name))
        options.(name) = overrides.(name);
    end
end
options.FigureVisible = lower(string(options.FigureVisible));
if ~isscalar(options.FigureVisible) || ~any(options.FigureVisible == ["on", "off"])
    error("azElInteractiveSandbox:InvalidFigureVisible", "FigureVisible must be 'on' or 'off'.");
end
options.Verbose = normalizeLogicalScalar(options.Verbose, "Verbose");
validateattributes(options.FigurePosition, {'numeric'}, {'real', 'finite', 'vector', 'numel', 4});
options.FigurePosition = reshape(double(options.FigurePosition), 1, 4);
positiveScalarNames = ["MissionTime_s", "MaximumSegmentDuration_s", "PathObstacleRadius_deg"];

% Validate each duration or radius that must be strictly positive.
for name = positiveScalarNames
    validateattributes(options.(name), {'numeric'}, {'real', 'finite', 'scalar', 'positive'}, "azElInteractiveSandbox", name);
end
nonnegativeScalarNames = ["PathSafetyMargin_deg", "ObstacleSafetyMargin_deg"];

% Safety margins may be zero, but never negative or nonfinite.
for name = nonnegativeScalarNames
    validateattributes(options.(name), {'numeric'}, ...
        {'real', 'finite', 'scalar', 'nonnegative'}, ...
        "azElInteractiveSandbox", name);
end
pairNames = ["MaxVelocity_deg_s", "MaxAcceleration_deg_s2", "MaxJerk_deg_s3"];

% Normalize each azimuth/elevation derivative limit into one row pair.
for name = pairNames
    validateattributes(options.(name), {'numeric'}, ...
        {'real', 'finite', 'vector', 'numel', 2, 'positive'}, ...
        "azElInteractiveSandbox", name);
    options.(name) = reshape(double(options.(name)), 1, 2);
end
intervalNames = ["WorkspaceAzimuthInterval_deg", "WorkspaceElevationInterval_deg"];

% Require an increasing lower/upper interval for each workspace axis.
for name = intervalNames
    validateattributes(options.(name), {'numeric'}, ...
        {'real', 'finite', 'vector', 'numel', 2, 'increasing'}, ...
        "azElInteractiveSandbox", name);
    options.(name) = reshape(double(options.(name)), 1, 2);
end
countNames = ["LatestArrivalCoarseCandidateCount", "LatestArrivalRefinementCandidateCount"];

% Candidate counts bound the finite latest-tested-arrival search work.
for name = countNames
    validateattributes(options.(name), {'numeric'}, ...
        {'real', 'finite', 'scalar', 'integer', 'nonnegative'}, ...
        "azElInteractiveSandbox", name);
end
if options.LatestArrivalCoarseCandidateCount < 1
    error("azElInteractiveSandbox:InvalidCandidateCount", "LatestArrivalCoarseCandidateCount must be at least one.");
end
if ~isstruct(options.PlannerOptions) || ~isscalar(options.PlannerOptions)
    error("azElInteractiveSandbox:InvalidPlannerOptions", "PlannerOptions must be a scalar partial options struct.");
end

% Keep exported options explicit while leaving all other partial HS3 options
% for the public planner to resolve at plan time.
if isfield(options.PlannerOptions, "PlannerMethod") && ...
        ~isempty(options.PlannerOptions.PlannerMethod)
    plannerMethod = lower(string(options.PlannerOptions.PlannerMethod));
    if ~isscalar(plannerMethod) || plannerMethod ~= "hs3"
        error("azElInteractiveSandbox:InvalidPlannerMethod", ...
            "PlannerOptions.PlannerMethod must be 'hs3'.");
    end
end
options.PlannerOptions.PlannerMethod = "hs3";
end

function value = normalizeLogicalScalar(value, fieldName)
% Normalize a scalar logical or binary numeric sandbox control.
isValid = (islogical(value) && isscalar(value)) || ...
    (isnumeric(value) && isreal(value) && isfinite(value) && ...
    isscalar(value) && any(value == [0 1]));
if ~isValid
    error("azElInteractiveSandbox:InvalidLogicalOption", "%s must be scalar logical or numeric zero/one.", fieldName);
end
value = logical(value);
end

function handles = createModeTab(tabHandle, modeName, options)
% Build one complete tab with canvas, controls, actions, status, and log.

% Reserve the complete outer rectangle for ticks and labels. Using Position
% here lets the azimuth label extend into the action-button row and disappear.
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

% Shared column headings avoid repeating two extra label rows for every pair.
% This grid stays readable when Windows display scaling increases text height.
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

if modeName == "goal"
    horizonLabel = "Mission horizon (s)";
    horizonValue = options.MissionTime_s;
else
    horizonLabel = "Maximum segment duration (s)";
    horizonValue = options.MaximumSegmentDuration_s;
end

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
actionPanelHandle = uipanel(tabHandle, "BorderType", "none", "Units", "normalized", "Position", [0.045 0.275 0.64 0.052]);
if modeName == "goal"
    actionNames = [ ...
        "AddObstacle", "Run", "Reset", "Diagnostics", "Export"];
    actionLabels = [ ...
        "Add Obstacle", "Run", "Reset", "Diagnostics", "Export Bundle"];
else
    actionNames = [ ...
        "AddObstacle", "AddSegment", "Recalculate", "Undo", ...
        "Reset", "Diagnostics", "Export"];
    actionLabels = [ ...
        "Add Obstacle", "Add Segment", "Recalculate", "Undo Point", ...
        "Reset", "Diagnostics", "Export Bundle"];
end
actions = createActionButtons( actionPanelHandle, modeName, actionNames, actionLabels);
statusPanelHandle = uipanel(tabHandle, ...
    "Title", "Mode status and planner log", ...
    "Units", "normalized", ...
    "Position", [0.045 0.025 0.94 0.23]);
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
handles = struct( ...
    "Tab", tabHandle, ...
    "Axes", axesHandle, ...
    "ControlPanel", controlPanelHandle, ...
    "Controls", controls, ...
    "ActionPanel", actionPanelHandle, ...
    "Actions", actions, ...
    "StatusPanel", statusPanelHandle, ...
    "StatusHandle", statusHandle, ...
    "LogHandle", logHandle, ...
    "DiagnosticPlotHandles", struct());
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

function applicationState = initializeApplicationState( figureHandle, options, goalHandles, freeHandles)
% Create stable independent tab records and application interaction state.
applicationState = struct( ...
    "FigureHandle", figureHandle, ...
    "Options", options, ...
    "ActiveMode", "goal", ...
    "InteractionState", "idle", ...
    "ActiveStroke_deg", zeros(0, 2), ...
    "ActiveTraceHandle", gobjects(0), ...
    "GoalMode", emptyModeState("goal", goalHandles), ...
    "FreeMode", emptyModeState("free", freeHandles));
end

function modeState = emptyModeState(modeName, graphicsHandles)
% Define the stable mode schema used by initialization and Reset.
if modeName == "goal"
    instruction = "Click the start, click the goal, draw obstacles, then Run.";
else
    instruction = "Click the start, click the first endpoint, draw obstacles, then recalculate.";
end
modeState = struct( ...
    "StartPosition_deg", zeros(0, 2), ...
    "GoalPosition_deg", zeros(0, 2), ...
    "WaypointPositions_deg", zeros(0, 2), ...
    "RawObstacleStrokes_deg", {cell(0, 1)}, ...
    "LineObstaclePositions_deg", {cell(0, 1)}, ...
    "PolygonObstaclePositions_deg", {cell(0, 1)}, ...
    "CanonicalObstacles", azElObstacles.combineAzElObstacles(), ...
    "SegmentResults", repmat(emptySegmentRecord(), 0, 1), ...
    "CombinedTrajectory", emptyCombinedTrajectory(), ...
    "LastPlannerResult", struct(), ...
    "LastValidation", validateAzElTrajectory(), ...
    "GraphicsHandles", graphicsHandles, ...
    "InteractionState", "idle", ...
    "Status", instruction, ...
    "PlannerLog", strings(0, 1), ...
    "ResolvedControls", struct(), ...
    "SolvedSegmentCount", 0, ...
    "FirstFailureSegmentIndex", 0, ...
    "FirstFailureReason", "", ...
    "LatestValidatedArrival_s", zeros(0, 1), ...
    "LatestArrivalSearchResolution_s", zeros(0, 1));
end

function trajectory = emptyCombinedTrajectory()
% Define a sandbox-owned combined trajectory without planner-result claims.
trajectory = struct( ...
    "time_s", zeros(0, 1), ...
    "position_deg", zeros(0, 2), ...
    "velocity_deg_s", zeros(0, 2), ...
    "acceleration_deg_s2", zeros(0, 2), ...
    "jerk_deg_s3", zeros(0, 2), ...
    "SegmentStartIndices", zeros(0, 1));
end

function segment = emptySegmentRecord()
% Define one stable independently planned Free Mode segment record.
segment = struct( ...
    "RequestedStart_deg", [NaN NaN], ...
    "RequestedStop_deg", [NaN NaN], ...
    "StartTime_s", NaN, ...
    "ArrivalTime_s", NaN, ...
    "PlannerResult", struct(), ...
    "Validation", validateAzElTrajectory(), ...
    "Success", false, ...
    "TerminationReason", "", ...
    "LatestValidatedArrival_s", NaN, ...
    "LatestArrivalSearchResolution_s", NaN, ...
    "TestedArrivalTimes_s", zeros(0, 1), ...
    "TestedArrivalSuccess", false(0, 1), ...
    "SearchStatement", "No arrival candidates were tested.");
end

% --- Interaction State And Callbacks ------------------------------------

function handleAction(sourceHandle, ~)
% Dispatch every action while converting UI input errors into status text.
figureHandle = ancestor(sourceHandle, "figure");
request = get(sourceHandle, "UserData");
modeName = string(request.Mode);
actionName = string(request.Action);
try
    switch actionName
        case "AddObstacle"
            activateInteraction(figureHandle, modeName, "drawingObstacle");
        case "AddSegment"
            activateInteraction(figureHandle, modeName, "placingWaypoint");
        case "Run"
            executeGoalPlan(figureHandle);
        case "Recalculate"
            executeFreePlan(figureHandle, "Manual full-chain recalculation");
        case "Undo"
            undoFreeWaypoint(figureHandle);
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
    modeState.Status = "Input or planning error: " + string(exceptionText);
    modeState = appendLogLines( ...
        modeState, "[Sandbox error] " + string(exceptionText));
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

function exceptionText = formatSandboxException(exception)
%% Section 0: Header & Readme
% SYNTAX
%   exceptionText = formatSandboxException(exception)
%**************************************************************************
% PURPOSE
%   - Preserve the identifier and earliest source location of a UI failure.
%**************************************************************************
% INPUTS
%   - exception (MException scalar)
%       Exception caught by the shared sandbox action callback.
%**************************************************************************
% OUTPUTS
%   - exceptionText (character vector)
%       Actionable message with identifier and first stack location.
%**************************************************************************
% UNITS
%   - Not applicable.
%**************************************************************************
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
% Give one mouse-driven interaction exclusive ownership of callbacks.
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
    case "placingWaypoint"
        interactionState = "placingFreeWaypoint";
        modeState.Status = "Place the next requested endpoint with one left click.";
    case "placingInitialGoal"
        interactionState = "placingFreeGoal";
        modeState.Status = "Place the first requested endpoint with one left click.";
    case "drawingObstacle"
        interactionState = "drawing" + upperFirst(modeName) + "Obstacle";
        modeState.Status = "Hold the left mouse button, trace one obstacle, then release.";
end
applicationState.InteractionState = interactionState;
modeState.InteractionState = interactionState;
applicationState = setModeState(applicationState, modeName, modeState);
guidata(figureHandle, applicationState);
set(figureHandle, "WindowButtonDownFcn", @handleFigureMouseDown, "WindowButtonMotionFcn", "", "WindowButtonUpFcn", "");
refreshApplication(figureHandle);
end

function beginGuidedScene(figureHandle, modeName)
% Resume the next unfinished step in the automatic start-to-obstacle sequence.
applicationState = guidata(figureHandle);
modeState = getModeState(applicationState, modeName);

if isempty(modeState.StartPosition_deg)
    requestedState = "placingStart";
elseif modeName == "goal" && isempty(modeState.GoalPosition_deg)
    requestedState = "placingGoal";
elseif modeName == "free" && isempty(modeState.WaypointPositions_deg)
    requestedState = "placingInitialGoal";
elseif isempty(modeState.RawObstacleStrokes_deg)
    requestedState = "drawingObstacle";
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
% Begin a trace or commit one requested point on the active tab canvas.
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
if string(get(figureHandle, "SelectionType")) ~= "normal"
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
if contains(applicationState.InteractionState, "drawing")
    applicationState.ActiveStroke_deg = point_deg;
    applicationState.ActiveTraceHandle = plot( ...
        modeState.GraphicsHandles.Axes, point_deg(1), point_deg(2), ...
        "b-", "LineWidth", 1.4, "HandleVisibility", "off");
    guidata(figureHandle, applicationState);
    set(figureHandle, "WindowButtonMotionFcn", @handleFigureMouseMotion, "WindowButtonUpFcn", @handleFigureMouseUp);
    return;
end
shouldRecalculateFree = false;
nextInteraction = "";
switch applicationState.InteractionState
    case "placingGoalStart"
        modeState.StartPosition_deg = point_deg;
        modeState = clearModeSolution(modeState);
        modeState.Status = "Start point set. Click the goal next.";
        nextInteraction = "placingGoal";
    case "placingFreeStart"
        modeState.StartPosition_deg = point_deg;
        modeState = clearModeSolution(modeState);
        modeState.Status = "Start point set. Click the first endpoint next.";
        nextInteraction = "placingInitialGoal";
    case "placingGoalStop"
        modeState.GoalPosition_deg = point_deg;
        modeState = clearModeSolution(modeState);
        modeState.Status = "Goal point set. Draw obstacles, then click Run.";
        nextInteraction = "drawingObstacle";
    case "placingFreeGoal"
        modeState.WaypointPositions_deg = point_deg;
        modeState = clearModeSolution(modeState);
        modeState.Status = "First endpoint set. Draw obstacles, then click Recalculate.";
        nextInteraction = "drawingObstacle";
    case "placingFreeWaypoint"
        modeState.WaypointPositions_deg(end + 1, :) = point_deg;
        shouldRecalculateFree = true;
end
applicationState = setModeState(applicationState, modeName, modeState);
guidata(figureHandle, applicationState);
cancelInteraction(figureHandle);
if strlength(nextInteraction) > 0
    activateInteraction(figureHandle, modeName, nextInteraction);
elseif shouldRecalculateFree
    executeFreePlan(figureHandle, "Endpoint committed; replanning the chain");
else
    refreshApplication(figureHandle);
end
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
% Finish one trace, classify it as line or polygon, and replan if needed.
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
applicationState = guidata(figureHandle);
modeState = getModeState(applicationState, modeName);
shouldRecalculateFree = modeName == "free" && ...
    modeState.SolvedSegmentCount > 0 && ...
    ~isempty(modeState.StartPosition_deg) && ...
    ~isempty(modeState.WaypointPositions_deg) && ...
    size(simplifiedStroke_deg, 1) >= 2;
if shouldRecalculateFree
    executeFreePlan( figureHandle, "Obstacle completed; recalculating the full chain");
elseif size(simplifiedStroke_deg, 1) >= 2
    refreshApplication(figureHandle);
else
    % A rejected trace leaves no obstacle, so keep the initial draw step active.
    beginGuidedScene(figureHandle, modeName);
end
end

function cancelInteraction(figureHandle)
% Restore deterministic idle callbacks and discard any unfinished trace.
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
% Build Goal Mode inputs, call the public planner, and validate the result.
cancelInteraction(figureHandle);
applicationState = guidata(figureHandle);
modeState = applicationState.GoalMode;
if isempty(modeState.StartPosition_deg) || isempty(modeState.GoalPosition_deg)
    error("azElInteractiveSandbox:IncompleteGoalScene", "Goal Mode requires both a start point and a goal point.");
end
controls = readModeControls(applicationState, "goal");
obstacleTime_s = [0; controls.MissionTime_s];
canonicalObstacles = buildCanonicalObstacles( modeState, obstacleTime_s, controls);
[initialState, goalState, limits] = buildPlannerInputs( ...
    modeState.StartPosition_deg, modeState.GoalPosition_deg, ...
    0, controls.MissionTime_s, controls);
plannerOptions = applicationState.Options.PlannerOptions;
plannerOptions.PlannerMethod = controls.PlannerMethod;
plannerOptions.GoalTimeMode = "earliestArrival";
plannerOptions.Verbose = controls.Verbose;
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
    plannerOptions, "Goal Mode");
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
end

function executeFreePlan(figureHandle, reasonText)
% Replan every requested segment from the start and stop at first failure.
cancelInteraction(figureHandle);
applicationState = guidata(figureHandle);
modeState = applicationState.FreeMode;
if isempty(modeState.StartPosition_deg) || isempty(modeState.WaypointPositions_deg)
    error("azElInteractiveSandbox:IncompleteFreeScene", "Free Mode requires a start and at least one requested endpoint.");
end
controls = readModeControls(applicationState, "free");
requestedSegmentCount = size(modeState.WaypointPositions_deg, 1);
obstacleEndTime_s = requestedSegmentCount * controls.MaximumSegmentDuration_s;
canonicalObstacles = buildCanonicalObstacles( modeState, [0; obstacleEndTime_s], controls);
modeState = clearModeSolution(modeState);
modeState.CanonicalObstacles = canonicalObstacles;
modeState.ResolvedControls = controls;
modeState.PlannerLog = ["[Free Mode]"; string(reasonText)];
modeState.Status = "Planning segment 1/" + requestedSegmentCount + "...";
modeState.InteractionState = "planning";
applicationState.FreeMode = modeState;
applicationState.InteractionState = "planning";
guidata(figureHandle, applicationState);
refreshApplication(figureHandle);
drawnow;
currentStart_deg = modeState.StartPosition_deg;
currentStartTime_s = 0;

% Solve requested free-mode segments in order because each arrival starts the next segment.
for segmentIndex = 1:requestedSegmentCount
    applicationState = guidata(figureHandle);
    modeState = applicationState.FreeMode;
    modeState.Status = "Planning segment " + segmentIndex + "/" + requestedSegmentCount + "...";
    applicationState.FreeMode = modeState;
    guidata(figureHandle, applicationState);
    refreshApplication(figureHandle);
    drawnow;
    requestedStop_deg = modeState.WaypointPositions_deg(segmentIndex, :);
    [segmentRecord, segmentLog] = solveLatestValidatedSegment( ...
        canonicalObstacles, currentStart_deg, requestedStop_deg, ...
        currentStartTime_s, controls, applicationState.Options, ...
        segmentIndex);
    applicationState = guidata(figureHandle);
    modeState = applicationState.FreeMode;
    modeState.SegmentResults(end + 1, 1) = segmentRecord;
    modeState.PlannerLog = [modeState.PlannerLog; segmentLog];
    modeState.LastPlannerResult = segmentRecord.PlannerResult;
    modeState.LastValidation = segmentRecord.Validation;
    if ~segmentRecord.Success
        modeState.FirstFailureSegmentIndex = segmentIndex;
        modeState.FirstFailureReason = segmentRecord.TerminationReason;
        modeState.Status = "Segment " + segmentIndex + " failed: " + ...
            segmentRecord.TerminationReason + ...
            ". Downstream timing was not fabricated.";
        applicationState.FreeMode = modeState;
        guidata(figureHandle, applicationState);
        break;
    end
    modeState = appendLogLines(modeState, sprintf( ...
        "Selected segment %d: arrival=%.6g s, resolution=%.6g s. %s", ...
        segmentIndex, segmentRecord.LatestValidatedArrival_s, ...
        segmentRecord.LatestArrivalSearchResolution_s, ...
        segmentRecord.SearchStatement));
    modeState.SolvedSegmentCount = segmentIndex;
    modeState.LatestValidatedArrival_s(end + 1, 1) = segmentRecord.LatestValidatedArrival_s;
    modeState.LatestArrivalSearchResolution_s(end + 1, 1) = segmentRecord.LatestArrivalSearchResolution_s;
    modeState.CombinedTrajectory = appendSegmentTrajectory( modeState.CombinedTrajectory, segmentRecord.PlannerResult);
    modeState.Status = "Segment " + segmentIndex + " solved at t=" + sprintf("%.6g", segmentRecord.ArrivalTime_s) + " s.";
    applicationState.FreeMode = modeState;
    guidata(figureHandle, applicationState);
    currentStart_deg = requestedStop_deg;
    currentStartTime_s = segmentRecord.ArrivalTime_s;
end
applicationState = guidata(figureHandle);
modeState = applicationState.FreeMode;
if modeState.FirstFailureSegmentIndex == 0
    modeState.Status = formatFreeStatus( modeState, requestedSegmentCount);
end
modeState.InteractionState = "idle";
applicationState.FreeMode = modeState;
applicationState.InteractionState = "idle";
guidata(figureHandle, applicationState);
refreshApplication(figureHandle);
end

function [segment, logLines] = solveLatestValidatedSegment( ...
        obstacles, startPosition_deg, stopPosition_deg, startTime_s, ...
        controls, sandboxOptions, segmentIndex)
% Find the latest independently validated tested arrival in a finite span.
segment = emptySegmentRecord();
segment.RequestedStart_deg = startPosition_deg;
segment.RequestedStop_deg = stopPosition_deg;
segment.StartTime_s = startTime_s;
latestAllowedTime_s = startTime_s + controls.MaximumSegmentDuration_s;
plannerOptions = sandboxOptions.PlannerOptions;
plannerOptions.PlannerMethod = controls.PlannerMethod;
plannerOptions.Verbose = controls.Verbose;
logLines = "[Free Mode Segment " + segmentIndex + "]";
testedTimes_s = zeros(0, 1);
testedSuccess = false(0, 1);

% The finite upper-bound test is decisive when it validates: no arrival
% later than the user-supplied segment horizon is allowed by the sandbox.
[upperResult, upperValidation, upperLog] = runSegmentCandidate( ...
    obstacles, startPosition_deg, stopPosition_deg, startTime_s, ...
    latestAllowedTime_s, controls, plannerOptions, "fixedArrival", ...
    "upper bound");
logLines = [logLines; upperLog];
testedTimes_s(end + 1, 1) = latestAllowedTime_s;
testedSuccess(end + 1, 1) = upperResult.Success && upperValidation.Passed;
if testedSuccess(end)
    segment = populateSuccessfulSegment( ...
        segment, upperResult, upperValidation, 0, ...
        testedTimes_s, testedSuccess, ...
        "The finite upper bound passed; it is the latest allowed arrival.");
    return;
end

% Drawn sandbox obstacles are static across the finite planning horizon. An
% endpointBlocked result at the upper bound therefore applies to every
% candidate arrival, so retrying the same endpoint cannot add evidence.
if upperResult.TerminationReason == "endpointBlocked"
    segment.PlannerResult = upperResult;
    segment.Validation = upperValidation;
    segment.Success = false;
    segment.TerminationReason = upperResult.TerminationReason;
    segment.TestedArrivalTimes_s = testedTimes_s;
    segment.TestedArrivalSuccess = testedSuccess;
    segment.SearchStatement = "Static protected geometry blocks a segment endpoint.";
    return;
end

bestResult = struct();
bestValidation = validateAzElTrajectory();
failureResult = upperResult;
failureValidation = upperValidation;

% Earliest arrival supplies a validated fallback when the fixed-time samples
% miss a feasible time. It is not treated as a global earliest certificate.
[earliestResult, earliestValidation, earliestLog] = runSegmentCandidate( ...
    obstacles, startPosition_deg, stopPosition_deg, startTime_s, ...
    latestAllowedTime_s, controls, plannerOptions, "earliestArrival", ...
    "earliest fallback");
logLines = [logLines; earliestLog];
if earliestResult.Success && earliestValidation.Passed
    bestResult = earliestResult;
    bestValidation = earliestValidation;
end

maximumDuration_s = controls.MaximumSegmentDuration_s;
velocityLowerBound_s = max( abs(stopPosition_deg - startPosition_deg) ./ controls.MaxVelocity_deg_s);
minimumTestDuration_s = max( min(maximumDuration_s, velocityLowerBound_s), min(0.05, maximumDuration_s));
coarseCount = sandboxOptions.LatestArrivalCoarseCandidateCount;
coarseDurations_s = linspace( maximumDuration_s, minimumTestDuration_s, coarseCount + 1).';
coarseDurations_s = coarseDurations_s(2:end);
previousFailedTime_s = latestAllowedTime_s;
fixedResult = struct();
fixedValidation = validateAzElTrajectory();
fixedTime_s = NaN;

% Test coarse arrival candidates from latest to earliest and keep the first validated result.
for candidateIndex = 1:numel(coarseDurations_s)
    candidateTime_s = startTime_s + coarseDurations_s(candidateIndex);
    [candidateResult, candidateValidation, candidateLog] = runSegmentCandidate( ...
        obstacles, startPosition_deg, stopPosition_deg, startTime_s, ...
        candidateTime_s, controls, plannerOptions, "fixedArrival", ...
        "coarse " + candidateIndex + "/" + numel(coarseDurations_s));
    logLines = [logLines; candidateLog]; %#ok<AGROW>
    candidatePassed = candidateResult.Success && candidateValidation.Passed;
    testedTimes_s(end + 1, 1) = candidateTime_s; %#ok<AGROW>
    testedSuccess(end + 1, 1) = candidatePassed; %#ok<AGROW>
    if candidatePassed
        fixedResult = candidateResult;
        fixedValidation = candidateValidation;
        fixedTime_s = candidateTime_s;
        break;
    end
    failureResult = candidateResult;
    failureValidation = candidateValidation;
    previousFailedTime_s = candidateTime_s;
end

searchResolution_s = maximumDuration_s / coarseCount;
selectionStatement = "The public earliest-arrival fallback was the only validated call; " + ...
    "no fixed-time or global latest-arrival certificate is claimed.";
selectedResolution_s = NaN;
if ~isempty(fieldnames(fixedResult))
    refinementCount = sandboxOptions.LatestArrivalRefinementCandidateCount;
    if refinementCount > 0 && previousFailedTime_s > fixedTime_s
        refinementTimes_s = linspace( previousFailedTime_s, fixedTime_s, refinementCount + 2).';
        refinementTimes_s = refinementTimes_s(2:end - 1);
        searchResolution_s = (previousFailedTime_s - fixedTime_s) / (refinementCount + 1);

        % Refine only inside the failed-to-passed bracket established by the coarse search.
        for candidateIndex = 1:numel(refinementTimes_s)
            candidateTime_s = refinementTimes_s(candidateIndex);
            [candidateResult, candidateValidation, candidateLog] = runSegmentCandidate( ...
                obstacles, startPosition_deg, stopPosition_deg, ...
                startTime_s, candidateTime_s, controls, plannerOptions, ...
                "fixedArrival", "refinement " + candidateIndex + "/" + ...
                numel(refinementTimes_s));
            logLines = [logLines; candidateLog]; %#ok<AGROW>
            candidatePassed = candidateResult.Success && candidateValidation.Passed;
            testedTimes_s(end + 1, 1) = candidateTime_s; %#ok<AGROW>
            testedSuccess(end + 1, 1) = candidatePassed; %#ok<AGROW>
            if candidatePassed
                fixedResult = candidateResult;
                fixedValidation = candidateValidation;
                break;
            end
            failureResult = candidateResult;
            failureValidation = candidateValidation;
        end
    end
    bestResult = fixedResult;
    bestValidation = fixedValidation;
    selectionStatement = "Latest validated arrival among bounded descending fixed-time " + ...
        "samples; no global latest-arrival certificate.";
    selectedResolution_s = searchResolution_s;
end

if ~isempty(fieldnames(bestResult))
    segment = populateSuccessfulSegment( ...
        segment, bestResult, bestValidation, selectedResolution_s, ...
        testedTimes_s, testedSuccess, selectionStatement);
else
    segment.PlannerResult = failureResult;
    segment.Validation = failureValidation;
    segment.Success = false;
    segment.TerminationReason = failureResult.TerminationReason;
    segment.TestedArrivalTimes_s = testedTimes_s;
    segment.TestedArrivalSuccess = testedSuccess;
    segment.SearchStatement = "No tested arrival passed inside the finite segment horizon.";
end
end

function [result, validation, logLines] = runSegmentCandidate( ...
        obstacles, startPosition_deg, stopPosition_deg, startTime_s, ...
        goalTime_s, controls, plannerOptions, goalTimeMode, labelText)
% Run one explicitly labeled segment candidate through public interfaces.
[initialState, goalState, limits] = buildPlannerInputs( ...
    startPosition_deg, stopPosition_deg, startTime_s, goalTime_s, controls);
plannerOptions.GoalTimeMode = goalTimeMode;
[result, validation, plannerLog] = callPlanner( ...
    obstacles, initialState, goalState, limits, plannerOptions, ...
    "Candidate " + labelText + " at t=" + sprintf("%.6g", goalTime_s));
logLines = plannerLog;
end

function segment = populateSuccessfulSegment( ...
        segment, result, validation, resolution_s, testedTimes_s, ...
        testedSuccess, statement)
% Assemble one successful segment without relabeling it as a global solve.
segment.ArrivalTime_s = result.ArrivalTime_s;
segment.PlannerResult = result;
segment.Validation = validation;
segment.Success = true;
segment.TerminationReason = result.TerminationReason;
segment.LatestValidatedArrival_s = result.ArrivalTime_s;
segment.LatestArrivalSearchResolution_s = resolution_s;
segment.TestedArrivalTimes_s = testedTimes_s;
segment.TestedArrivalSuccess = testedSuccess;
segment.SearchStatement = statement;
end

function [result, validation, logLines] = callPlanner( obstacles, initialState, goalState, limits, options, labelText)
% Capture optional verbose output without suppressing planner diagnostics.
result = struct();
plannerText = "";
if options.Verbose
    plannerText = string(evalc( 'result = planAzElMotion(obstacles, initialState, goalState, limits, options);'));
else
    result = planAzElMotion( obstacles, initialState, goalState, limits, options);
end
if result.Success
    validation = validateAzElTrajectory(result);
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
    "Result: success=%d, validation=%d, reason=%s, arrival=%.6g s", ...
    result.Success, validation.Passed, result.TerminationReason, ...
    result.ArrivalTime_s);
end

function [initialState, goalState, limits] = buildPlannerInputs( ...
        startPosition_deg, stopPosition_deg, startTime_s, goalTime_s, controls)
% Build rest-to-rest endpoint states and explicit physical/workspace limits.
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

function combined = appendSegmentTrajectory(combined, result)
% Stitch validated sampled histories while removing duplicate knot samples.
if isempty(combined.time_s)
    firstRetainedIndex = 1;
    segmentStartIndex = 1;
else
    firstRetainedIndex = 2;
    segmentStartIndex = numel(combined.time_s) + 1;
end
retainedIndices = firstRetainedIndex:numel(result.time_s);
combined.time_s = [combined.time_s; result.time_s(retainedIndices)];
combined.position_deg = [ combined.position_deg; result.position_deg(retainedIndices, :)];
combined.velocity_deg_s = [ combined.velocity_deg_s; result.velocity_deg_s(retainedIndices, :)];
combined.acceleration_deg_s2 = [combined.acceleration_deg_s2; result.acceleration_deg_s2(retainedIndices, :)];
combined.jerk_deg_s3 = [ combined.jerk_deg_s3; result.jerk_deg_s3(retainedIndices, :)];
combined.SegmentStartIndices(end + 1, 1) = segmentStartIndex;
end

% --- Controls And Obstacle Canonicalization -----------------------------

function controls = readModeControls(applicationState, modeName)
% Read and validate one tab's independent physical and geometry controls.
modeState = getModeState(applicationState, modeName);
handles = modeState.GraphicsHandles.Controls;
workspaceAzimuthInterval_deg = readAxisPairControl( ...
    handles.WorkspaceAzimuthHandles, ...
    "Workspace azimuth interval (deg)", false);
workspaceElevationInterval_deg = readAxisPairControl( ...
    handles.WorkspaceElevationHandles, ...
    "Workspace elevation interval (deg)", false);
if workspaceAzimuthInterval_deg(2) <= workspaceAzimuthInterval_deg(1)
    error("azElInteractiveSandbox:InvalidAzimuthInterval", "Workspace azimuth lower bound must be below the upper bound.");
end
if workspaceElevationInterval_deg(2) <= workspaceElevationInterval_deg(1)
    error("azElInteractiveSandbox:InvalidElevationInterval", ...
        "Workspace elevation lower bound must be below the upper bound.");
end
maxVelocity_deg_s = readAxisPairControl( handles.VelocityHandles, "Maximum velocity (deg/s)", true);
maxAcceleration_deg_s2 = readAxisPairControl( handles.AccelerationHandles, "Maximum acceleration (deg/s^2)", true);
maxJerk_deg_s3 = readAxisPairControl( handles.JerkHandles, "Maximum jerk (deg/s^3)", true);
horizon_s = readScalarControl( handles.HorizonHandle, "Planning horizon (s)", true);
pathObstacleRadius_deg = readScalarControl( handles.PathRadiusHandle, "Line/capsule radius (deg)", true);
pathSafetyMargin_deg = readScalarControl( handles.PathMarginHandle, "Line safety margin (deg)", false);
obstacleSafetyMargin_deg = readScalarControl( handles.ObstacleMarginHandle, "Polygon safety margin (deg)", false);
controls = struct( ...
    "PlannerMethod", "hs3", ...
    "WorkspaceAzimuthInterval_deg", workspaceAzimuthInterval_deg, ...
    "WorkspaceElevationInterval_deg", workspaceElevationInterval_deg, ...
    "MaxVelocity_deg_s", maxVelocity_deg_s, ...
    "MaxAcceleration_deg_s2", maxAcceleration_deg_s2, ...
    "MaxJerk_deg_s3", maxJerk_deg_s3, ...
    "MissionTime_s", NaN, ...
    "MaximumSegmentDuration_s", NaN, ...
    "PathObstacleRadius_deg", pathObstacleRadius_deg, ...
    "PathSafetyMargin_deg", pathSafetyMargin_deg, ...
    "ObstacleSafetyMargin_deg", obstacleSafetyMargin_deg, ...
    "Verbose", logical(get(handles.VerboseHandle, "Value")));
if modeName == "goal"
    controls.MissionTime_s = horizon_s;
else
    controls.MaximumSegmentDuration_s = horizon_s;
end
end

function values = readAxisPairControl(handles, labelText, mustBePositive)
% Parse one finite two-value edit pair with a named error context.
values = [ str2double(get(handles.FirstHandle, "String")), str2double(get(handles.SecondHandle, "String"))];
validateattributes(values, {'numeric'}, {'real', 'finite', 'vector', 'numel', 2}, "azElInteractiveSandbox", labelText);
if mustBePositive && any(values <= 0)
    error("azElInteractiveSandbox:NonpositiveControl", "%s values must both be positive.", labelText);
end
values = reshape(double(values), 1, 2);
end

function value = readScalarControl(handle, labelText, mustBePositive)
% Parse one finite scalar edit value with positive/nonnegative policy.
value = str2double(get(handle, "String"));
validateattributes(value, {'numeric'}, {'real', 'finite', 'scalar'}, "azElInteractiveSandbox", labelText);
if mustBePositive && value <= 0
    error("azElInteractiveSandbox:NonpositiveControl", "%s must be positive.", labelText);
elseif ~mustBePositive && value < 0
    error("azElInteractiveSandbox:NegativeControl", "%s must be nonnegative.", labelText);
end
value = double(value);
end

function applyDefaultControls(handles, modeName, options)
% Restore one tab's editable values without touching the other tab.
writeAxisPair(handles.WorkspaceAzimuthHandles, options.WorkspaceAzimuthInterval_deg);
writeAxisPair(handles.WorkspaceElevationHandles, options.WorkspaceElevationInterval_deg);
writeAxisPair(handles.VelocityHandles, options.MaxVelocity_deg_s);
writeAxisPair(handles.AccelerationHandles, options.MaxAcceleration_deg_s2);
writeAxisPair(handles.JerkHandles, options.MaxJerk_deg_s3);
if modeName == "goal"
    horizon_s = options.MissionTime_s;
else
    horizon_s = options.MaximumSegmentDuration_s;
end
set(handles.HorizonHandle, "String", sprintf("%.8g", horizon_s));
set(handles.PathRadiusHandle, "String", sprintf("%.8g", options.PathObstacleRadius_deg));
set(handles.PathMarginHandle, "String", sprintf("%.8g", options.PathSafetyMargin_deg));
set(handles.ObstacleMarginHandle, "String", sprintf("%.8g", options.ObstacleSafetyMargin_deg));
set(handles.VerboseHandle, "Value", options.Verbose);
end

function writeAxisPair(handles, values)
% Write one two-value default into explicit edit handles.
set(handles.FirstHandle, "String", sprintf("%.8g", values(1)));
set(handles.SecondHandle, "String", sprintf("%.8g", values(2)));
end

function obstacles = buildCanonicalObstacles( modeState, obstacleTime_s, controls)
% Rebuild protected obstacles from retained raw-derived geometry once.
pathObstacleData = cell(numel(modeState.LineObstaclePositions_deg), 1);

% Convert every drawn line into one buffered capsule obstacle.
for lineIndex = 1:numel(modeState.LineObstaclePositions_deg)
    pathObstacleData{lineIndex} = pathToObstacleData( ...
        modeState.LineObstaclePositions_deg{lineIndex}, obstacleTime_s, ...
        controls.PathObstacleRadius_deg, ...
        controls.PathSafetyMargin_deg, lineIndex);
end
polygonObstacleData = obstaclePolygonsToData( ...
    modeState.PolygonObstaclePositions_deg, obstacleTime_s, ...
    controls.ObstacleSafetyMargin_deg);
obstacles = azElObstacles.combineAzElObstacles(pathObstacleData, polygonObstacleData);
end

function obstacleData = pathToObstacleData( path_deg, time_s, radius_deg, safetyMargin_deg, lineIndex)
% Convert one simplified freehand line into a continuous low-vertex capsule.
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
obstacleData = azElObstacles.makeAzElObstacleData( ...
    "drawn line obstacle " + lineIndex, time_s, ...
    vertices_deg(:, 1), vertices_deg(:, 2), safetyMargin_deg);
end

function pathShape = buildPathCapsuleShape(path_deg, radius_deg)
% Union conservative low-vertex segment capsules without geometry gaps.
endCapSegmentCount = 3;
arcIncrement_rad = pi / endCapSegmentCount;
% Enlarging the construction radius makes every polygonal cap contain the
% requested circular radius between cap vertices.
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

function obstacleData = obstaclePolygonsToData( polygonCollection_deg, time_s, safetyMargin_deg)
% Construct canonical protected polygons while applying margin once.
obstacleData = cell(numel(polygonCollection_deg), 1);

% Close and canonicalize every user-drawn polygon independently.
for polygonIndex = 1:numel(polygonCollection_deg)
    polygon_deg = polygonCollection_deg{polygonIndex};
    if ~isequal(polygon_deg(1, :), polygon_deg(end, :))
        polygon_deg = [polygon_deg; polygon_deg(1, :)]; %#ok<AGROW>
    end
    obstacleData{polygonIndex} = azElObstacles.makeAzElObstacleData( ...
        "drawn polygon obstacle " + polygonIndex, time_s, ...
        polygon_deg(:, 1), polygon_deg(:, 2), safetyMargin_deg);
end
end

function simplified_deg = simplifyFreehandBoundary(points_deg, tolerance_deg)
% Retain shape-defining turns while removing mouse-event oversampling.
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
% Render both independent tabs and synchronize status and control states.
if isempty(figureHandle) || ~isgraphics(figureHandle)
    return;
end
applicationState = guidata(figureHandle);
redrawMode(applicationState, "goal");
redrawMode(applicationState, "free");
updateModeStatusDisplay(applicationState.GoalMode);
updateModeStatusDisplay(applicationState.FreeMode);
updateControlEnablement(applicationState);
end

function redrawMode(applicationState, modeName)
% Redraw one tab exclusively from its retained mode and result state.
modeState = getModeState(applicationState, modeName);
axesHandle = modeState.GraphicsHandles.Axes;
if ~isgraphics(axesHandle)
    return;
end

% Raw strokes and temporary traces deliberately hide their handles from the
% legend. Delete every child explicitly because plain cla can preserve those
% hidden objects and leave a stale obstacle outline after Reset.
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

% Overlay polygon boundaries before rendering their canonical protected geometry.
for polygonIndex = 1:numel(modeState.PolygonObstaclePositions_deg)
    polygon_deg = modeState.PolygonObstaclePositions_deg{polygonIndex};
    fill(axesHandle, polygon_deg(:, 1), polygon_deg(:, 2), ...
        [0.25 0.35 0.85], "FaceAlpha", 0.10, ...
        "EdgeColor", [0.25 0.35 0.85], "LineStyle", ":", ...
        "HandleVisibility", "off");
end
renderCanonicalObstacles(axesHandle, modeState.CanonicalObstacles);

if ~isempty(modeState.StartPosition_deg)
    plot(axesHandle, modeState.StartPosition_deg(1), ...
        modeState.StartPosition_deg(2), "go", ...
        "MarkerFaceColor", "g", "MarkerSize", 8, ...
        "LineWidth", 1.2, "DisplayName", "Start");
end
if modeName == "goal"
    redrawGoalRequest(axesHandle, modeState);
else
    redrawFreeRequest(axesHandle, modeState);
end
if ~isempty(findobj(axesHandle, "-property", "DisplayName"))
    legend(axesHandle, "Location", "best");
end
if modeName == "goal"
    title(axesHandle, "Goal Mode: requested geometry and solved motion");
else
    title(axesHandle, "Free Mode: latest validated tested arrivals, not a global optimum");
end
end

function redrawGoalRequest(axesHandle, modeState)
% Show requested Goal Mode geometry, result, or retained partial route.
if ~isempty(modeState.GoalPosition_deg)
    plot(axesHandle, modeState.GoalPosition_deg(1), ...
        modeState.GoalPosition_deg(2), "ro", ...
        "MarkerFaceColor", "r", "MarkerSize", 8, ...
        "LineWidth", 1.2, "DisplayName", "Goal");
end
if ~isempty(modeState.StartPosition_deg) && ~isempty(modeState.GoalPosition_deg)
    request_deg = [modeState.StartPosition_deg; modeState.GoalPosition_deg];
    plot(axesHandle, request_deg(:, 1), request_deg(:, 2), ...
        "--", "Color", [0.35 0.55 0.85], "LineWidth", 1.2, ...
        "DisplayName", "Requested direct geometry");
end
result = modeState.LastPlannerResult;
if isempty(fieldnames(result))
    return;
end
if result.Success
    plot(axesHandle, result.position_deg(:, 1), result.position_deg(:, 2), ...
        "k-", "LineWidth", 2.4, "DisplayName", "Solved motion");
elseif result.SearchDiagnostics.BestPartialSeedIndex > 0
    partialIndex = result.SearchDiagnostics.BestPartialSeedIndex;
    partialRoute_deg = result.Seeds(partialIndex).position_deg;
    plot(axesHandle, partialRoute_deg(:, 1), partialRoute_deg(:, 2), ...
        "-.", "Color", [0.90 0.55 0.10], "LineWidth", 1.8, ...
        "DisplayName", "Best partial route");
end
end

function redrawFreeRequest(axesHandle, modeState)
% Distinguish requested endpoints, solved chain, and first failed segment.
if ~isempty(modeState.StartPosition_deg)
    request_deg = [ modeState.StartPosition_deg; modeState.WaypointPositions_deg];
    if size(request_deg, 1) > 1
        plot(axesHandle, request_deg(:, 1), request_deg(:, 2), ...
            "o--", "Color", [0.25 0.45 0.85], ...
            "MarkerFaceColor", [0.75 0.85 1], "LineWidth", 1.2, ...
            "DisplayName", "User-requested segment chain");
    end
end
if ~isempty(modeState.CombinedTrajectory.time_s)
    motion_deg = modeState.CombinedTrajectory.position_deg;
    plot(axesHandle, motion_deg(:, 1), motion_deg(:, 2), "k-", "LineWidth", 2.4, "DisplayName", "Cumulative solved motion");
end
failureIndex = modeState.FirstFailureSegmentIndex;
if failureIndex > 0 && failureIndex <= size(modeState.WaypointPositions_deg, 1)
    if failureIndex == 1
        failureStart_deg = modeState.StartPosition_deg;
    else
        failureStart_deg = modeState.WaypointPositions_deg(failureIndex - 1, :);
    end
    failureStop_deg = modeState.WaypointPositions_deg(failureIndex, :);
    failureSegment_deg = [failureStart_deg; failureStop_deg];
    plot(axesHandle, failureSegment_deg(:, 1), failureSegment_deg(:, 2), ...
        "r--", "LineWidth", 2.6, ...
        "DisplayName", "Failed/unresolved segment");
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
% Present concise status and the complete retained mode-specific log.
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
% Disable invalid actions and all geometry edits during synchronous plans.
isPlanning = applicationState.InteractionState == "planning";
modeNames = ["goal" "free"];

% Synchronize both tabs so a synchronous plan disables every competing edit.
for modeName = modeNames
    modeState = getModeState(applicationState, modeName);
    actionNames = string(fieldnames(modeState.GraphicsHandles.Actions));

    % Start from the common planning lock before applying mode-specific availability.
    for actionName = reshape(actionNames, 1, [])
        set(modeState.GraphicsHandles.Actions.(actionName), "Enable", onOff(~isPlanning));
    end
    controlHandles = findall( modeState.GraphicsHandles.ControlPanel, "Type", "uicontrol");
    set(controlHandles, "Enable", onOff(~isPlanning));
    set(modeState.GraphicsHandles.Controls.VerboseHandle, "Enable", onOff(~isPlanning));
    if isPlanning
        continue;
    end
    if modeName == "goal"
        canRun = ~isempty(modeState.StartPosition_deg) && ~isempty(modeState.GoalPosition_deg);
        set(modeState.GraphicsHandles.Actions.AddObstacle, "Enable", onOff(canRun));
        set(modeState.GraphicsHandles.Actions.Run, "Enable", onOff(canRun));
    else
        hasStart = ~isempty(modeState.StartPosition_deg);
        hasSegments = hasStart && ~isempty(modeState.WaypointPositions_deg);
        set(modeState.GraphicsHandles.Actions.AddObstacle, "Enable", onOff(hasSegments));
        set(modeState.GraphicsHandles.Actions.AddSegment, "Enable", onOff(hasStart));
        set(modeState.GraphicsHandles.Actions.Recalculate, "Enable", onOff(hasSegments));
        set(modeState.GraphicsHandles.Actions.Undo, "Enable", onOff(~isempty(modeState.WaypointPositions_deg)));
    end
    hasResult = ~isempty(fieldnames(modeState.LastPlannerResult));
    hasSceneData = ~isempty(modeState.StartPosition_deg) || ...
        ~isempty(modeState.GoalPosition_deg) || ...
        ~isempty(modeState.WaypointPositions_deg) || ...
        ~isempty(modeState.RawObstacleStrokes_deg);
    set(modeState.GraphicsHandles.Actions.Diagnostics, "Enable", onOff(hasResult));
    set(modeState.GraphicsHandles.Actions.Export, ...
        "Enable", onOff(hasResult || hasSceneData));
end
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
% Report Goal Mode success/failure without hiding unavailable quantities.
status = [ ...
    "Success: " + result.Success; ...
    "TerminationReason: " + result.TerminationReason; ...
    "ArrivalTime_s: " + sprintf("%.6g", result.ArrivalTime_s); ...
    "TrajectoryDuration_s: " + ...
        sprintf("%.6g", result.TrajectoryDuration_s); ...
    "SelectedMotionSource: " + result.SelectedMotionSource; ...
    "Independent validation: " + validation.Passed];
end

function status = formatFreeStatus(modeState, requestedSegmentCount)
% Report cumulative Free Mode results and the bounded-search limitation.
if isempty(modeState.CombinedTrajectory.time_s)
    totalElapsedTime_s = NaN;
    currentEnd_deg = [NaN NaN];
else
    totalElapsedTime_s = modeState.CombinedTrajectory.time_s(end) - modeState.CombinedTrajectory.time_s(1);
    currentEnd_deg = modeState.CombinedTrajectory.position_deg(end, :);
end
status = [ ...
    "SolvedSegmentCount: " + modeState.SolvedSegmentCount; ...
    "RequestedSegmentCount: " + requestedSegmentCount; ...
    "TotalElapsedMotionTime_s: " + sprintf("%.6g", totalElapsedTime_s); ...
    "CurrentEndPosition_deg: [" + sprintf("%.6g %.6g", currentEnd_deg) + "]"; ...
    "LatestValidatedArrival_s: [" + formatNumericVector( ...
        modeState.LatestValidatedArrival_s) + "]"; ...
    "LatestArrivalSearchResolution_s: [" + formatNumericVector( ...
        modeState.LatestArrivalSearchResolution_s) + "]"; ...
    "FirstFailureSegmentIndex: " + modeState.FirstFailureSegmentIndex; ...
    "FirstFailureReason: " + modeState.FirstFailureReason; ...
    "Arrival claim: latest validated tested time within each finite horizon."];
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
% Invalidate solved data after requested geometry or controls can change.
modeState.CanonicalObstacles = azElObstacles.combineAzElObstacles();
modeState.SegmentResults = repmat(emptySegmentRecord(), 0, 1);
modeState.CombinedTrajectory = emptyCombinedTrajectory();
modeState.LastPlannerResult = struct();
modeState.LastValidation = validateAzElTrajectory();
modeState.ResolvedControls = struct();
modeState.SolvedSegmentCount = 0;
modeState.FirstFailureSegmentIndex = 0;
modeState.FirstFailureReason = "";
modeState.LatestValidatedArrival_s = zeros(0, 1);
modeState.LatestArrivalSearchResolution_s = zeros(0, 1);
end

function resetMode(figureHandle, modeName)
% Restore one tab to a fresh state while leaving the other tab untouched.
cancelInteraction(figureHandle);
applicationState = guidata(figureHandle);
modeState = getModeState(applicationState, modeName);
graphicsHandles = modeState.GraphicsHandles;
applyDefaultControls( graphicsHandles.Controls, modeName, applicationState.Options);
modeState = emptyModeState(modeName, graphicsHandles);
applicationState = setModeState(applicationState, modeName, modeState);
guidata(figureHandle, applicationState);
beginGuidedScene(figureHandle, modeName);
end

function undoFreeWaypoint(figureHandle)
% Remove only the last requested endpoint and recompute retained timing.
cancelInteraction(figureHandle);
applicationState = guidata(figureHandle);
modeState = applicationState.FreeMode;
if isempty(modeState.WaypointPositions_deg)
    return;
end
modeState.WaypointPositions_deg(end, :) = [];
modeState = clearModeSolution(modeState);
applicationState.FreeMode = modeState;
guidata(figureHandle, applicationState);
if isempty(modeState.WaypointPositions_deg)
    modeState.Status = "Last endpoint removed. Add a new segment when ready.";
    applicationState.FreeMode = modeState;
    guidata(figureHandle, applicationState);
    refreshApplication(figureHandle);
else
    executeFreePlan(figureHandle, "Undo completed; replanning retained segments");
end
end

function openDiagnostics(figureHandle, modeName)
% Plot retained planner diagnostics without rerunning any planner call.
applicationState = guidata(figureHandle);
modeState = getModeState(applicationState, modeName);
result = modeState.LastPlannerResult;
if isempty(fieldnames(result))
    error("azElInteractiveSandbox:NoDiagnosticResult", "No planner result is available for %s mode.", modeName);
end
plotOptions = struct( ...
    "FigureVisible", applicationState.Options.FigureVisible, ...
    "Title", upperFirst(modeName) + " Mode diagnostics", ...
    "ShowAnimation", false);
modeState.GraphicsHandles.DiagnosticPlotHandles = azElPlotting.plotMotion(result, plotOptions);
applicationState = setModeState(applicationState, modeName, modeState);
guidata(figureHandle, applicationState);
end

function exportModeDiagnosis(figureHandle, modeName)
% Save exact retained scene, input, result, and validation evidence.
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
%% Section 0: Header & Readme
% SYNTAX
%   exportInfo = exportCurrentSandboxDiagnosis( ...
%       figureHandle, filePath, modeName)
%**************************************************************************
% PURPOSE
%   - Export the current guidata-backed sandbox state to an explicit path.
%**************************************************************************
% INPUTS
%   - figureHandle (scalar graphics handle)
%       Live sandbox figure owning the current application state.
%   - filePath (scalar text)
%       Destination MAT path; a missing .mat extension is added.
%   - modeName (scalar text)
%       Independent sandbox mode, either "goal" or "free".
%**************************************************************************
% OUTPUTS
%   - exportInfo (scalar struct)
%       Verified path, byte count, planner status, mode, and schema.
%**************************************************************************
% UNITS
%   - Bundle payload units follow exportAzElSandboxDiagnosis.
%**************************************************************************
if isempty(figureHandle) || ~isgraphics(figureHandle)
    error("azElInteractiveSandbox:ClosedFigure", ...
        "The sandbox figure must remain open while exporting a bundle.");
end
applicationState = guidata(figureHandle);
applicationState = prepareSandboxStateForExport( ...
    applicationState, modeName);
exportInfo = exportAzElSandboxDiagnosis( ...
    filePath, applicationState, modeName);
end

function applicationState = prepareSandboxStateForExport( ...
        applicationState, modeName)
%% Section 0: Header & Readme
% SYNTAX
%   applicationState = prepareSandboxStateForExport( ...
%       applicationState, modeName)
%**************************************************************************
% PURPOSE
%   - Capture current controls and geometry without invoking the planner.
%**************************************************************************
% INPUTS
%   - applicationState (scalar sandbox state struct)
%       Live guidata state containing both independent mode records.
%   - modeName (scalar text)
%       Mode to prepare, either "goal" or "free".
%**************************************************************************
% OUTPUTS
%   - applicationState (scalar sandbox state struct)
%       Copy with current canonical geometry and a pre-run export request.
%**************************************************************************
% UNITS
%   - Positions are degrees and horizons are seconds.
%**************************************************************************
modeState = getModeState(applicationState, modeName);
controls = readModeControls(applicationState, modeName);
plannerOptions = applicationState.Options.PlannerOptions;
plannerOptions.PlannerMethod = controls.PlannerMethod;
plannerOptions.Verbose = controls.Verbose;
plannerInputs = struct();
if modeName == "goal"
    obstacleEndTime_s = controls.MissionTime_s;
    hasCompleteScene = ~isempty(modeState.StartPosition_deg) && ...
        ~isempty(modeState.GoalPosition_deg);
    plannerOptions.GoalTimeMode = "earliestArrival";
    if hasCompleteScene
        [initialState, goalState, limits] = buildPlannerInputs( ...
            modeState.StartPosition_deg, modeState.GoalPosition_deg, ...
            0, controls.MissionTime_s, controls);
        plannerInputs = struct( ...
            "obstacles", azElObstacles.combineAzElObstacles(), ...
            "initialState", initialState, ...
            "goalState", goalState, ...
            "limits", limits);
    end
else
    requestedSegmentCount = size(modeState.WaypointPositions_deg, 1);
    obstacleEndTime_s = max(1, requestedSegmentCount) * ...
        controls.MaximumSegmentDuration_s;
    hasCompleteScene = ~isempty(modeState.StartPosition_deg) && ...
        requestedSegmentCount > 0;
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
    "RequestedGoal_deg", modeState.GoalPosition_deg, ...
    "RequestedWaypoints_deg", modeState.WaypointPositions_deg);
applicationState = setModeState(applicationState, modeName, modeState);
end

function modeState = getModeState(applicationState, modeName)
% Read one named mode without duplicating dynamic-field spelling.
if modeName == "goal"
    modeState = applicationState.GoalMode;
else
    modeState = applicationState.FreeMode;
end
end

function applicationState = setModeState( applicationState, modeName, modeState)
% Write one named mode while preserving the independent peer record.
if modeName == "goal"
    applicationState.GoalMode = modeState;
else
    applicationState.FreeMode = modeState;
end
end

function snapshot = publicStateSnapshot(figureHandle)
% Return current plain state without copying callbacks into guidata.
if isempty(figureHandle) || ~isgraphics(figureHandle)
    snapshot = struct( "FigureHandle", gobjects(0), "Status", "The sandbox figure is closed.");
    return;
end
snapshot = guidata(figureHandle);
snapshot.ExportBundle = @(filePath, modeName) ...
    exportCurrentSandboxDiagnosis(figureHandle, filePath, modeName);
end
