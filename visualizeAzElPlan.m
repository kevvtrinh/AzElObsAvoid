function figureHandle = visualizeAzElPlan(request, result, queryTime_s)
%% Section 0: Header & Readme
% SYNTAX
%   figureHandle = visualizeAzElPlan(request, result)
%   figureHandle = visualizeAzElPlan(request, result, queryTime_s)
%**************************************************************************
% PURPOSE
%   - Visualize an already validated command and the canonical obstacle
%     geometry evaluated by the same interpolation authority as validation.
%   - Support understanding only; never certify safety or optimality.
%**************************************************************************
% INPUTS
%   - request (scalar planning request)
%   - result (successful scalar planner result)
%   - queryTime_s (optional finite scalar)
%       Obstacle display time, defaulting to first valid arrival.
%**************************************************************************
% OUTPUTS
%   - figureHandle (MATLAB figure)
%**************************************************************************
% UNITS
%   - Axes are azimuth and elevation in degrees; time is seconds.

%% Section 1: Normalize Inputs
if ~isstruct(result) || ~isscalar(result) || ~result.success || ...
        ~result.validation.isValid
    error("visualizeAzElPlan:UnvalidatedResult", ...
        "Only an independently validated successful result may be shown.");
end
request = normalizeAzElPlannerRequest(request);
if nargin < 3 || isempty(queryTime_s)
    queryTime_s = result.arrivalTime_s;
end
validateattributes(queryTime_s, "numeric", ...
    ["scalar", "real", "finite"], mfilename, "queryTime_s");
queryTime_s = min(max(queryTime_s, result.command.time_s(1)), ...
    result.command.time_s(end));

%% Section 2: Evaluate Shared Geometry & Command State
displayTime_s = linspace(result.command.time_s(1), ...
    result.command.time_s(end), 401).';
displayState = sampleAzElCommand(result.command, displayTime_s);
currentState = sampleAzElCommand(result.command, queryTime_s);
[regionRecords, ~] = azElObstacleRegionsAtTime(request.obstacles, ...
    queryTime_s, request.options.temporalPadding_s);

%% Section 3: Draw The Validated Result
figureHandle = figure("Name", "Validated Az/El Plan", ...
    "Color", "white");
axesHandle = axes(figureHandle);
hold(axesHandle, "on");
referenceAzimuth_deg = median(displayState.unwrappedPosition_deg(:, 1));
obstacleHandle = gobjects(0);
for regionIndex = 1:numel(regionRecords)
    vertices_deg = regionRecords(regionIndex).vertices_deg;
    if request.options.azimuthWrap
        span_deg = diff(request.limits.azimuth_deg);
        shiftCount = round((referenceAzimuth_deg - ...
            mean(vertices_deg(:, 1))) ./ span_deg);
        vertices_deg(:, 1) = vertices_deg(:, 1) + shiftCount .* span_deg;
    end
    newObstacleHandle = patch(axesHandle, ...
        vertices_deg(:, 1), vertices_deg(:, 2), ...
        [0.85, 0.25, 0.2], "FaceAlpha", 0.22, ...
        "EdgeColor", [0.65, 0.1, 0.08], "LineWidth", 1.2);
    if isempty(obstacleHandle)
        obstacleHandle = newObstacleHandle;
    else
        newObstacleHandle.HandleVisibility = "off";
    end
end
commandHandle = plot(axesHandle, ...
    displayState.unwrappedPosition_deg(:, 1), ...
    displayState.unwrappedPosition_deg(:, 2), "b-", "LineWidth", 2);
knotHandle = plot(axesHandle, ...
    result.command.unwrappedPosition_deg(:, 1), ...
    result.command.unwrappedPosition_deg(:, 2), "bo", ...
    "MarkerFaceColor", "white");
stateHandle = plot(axesHandle, currentState.unwrappedPosition_deg(1), ...
    currentState.unwrappedPosition_deg(2), "ko", ...
    "MarkerFaceColor", [1, 0.75, 0], "MarkerSize", 8);
xlabel(axesHandle, "Continuous azimuth (deg)");
ylabel(axesHandle, "Elevation (deg)");
title(axesHandle, sprintf( ...
    "Validated command; obstacle geometry at t = %.3f s", queryTime_s));
grid(axesHandle, "on");
axis(axesHandle, "equal");
if isempty(obstacleHandle)
    legend(axesHandle, [commandHandle, knotHandle, stateHandle], ...
        ["Command", "Command knots", "Displayed state"], ...
        "Location", "best");
else
    legend(axesHandle, [obstacleHandle, commandHandle, knotHandle, ...
        stateHandle], ["Obstacle", "Command", "Command knots", ...
        "Displayed state"], "Location", "best");
end
hold(axesHandle, "off");
end
