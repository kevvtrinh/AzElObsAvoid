function handles = plotAzElHs3Motion(result, optionOverrides)
%% Section 0: Header & Readme
% SYNTAX
%   handles = plotAzElHs3Motion(result)
%   handles = plotAzElHs3Motion(result, optionOverrides)
%**************************************************************************
% PURPOSE
%   - Plot a combined sandbox motion profile and its per-segment limits.
%**************************************************************************
% INPUTS
%   - result (scalar solveAzElHs3Segments result struct)
%       Must contain successful position, velocity, acceleration, jerk,
%       limit, time, and segment-index histories.
%   - optionOverrides (scalar struct, optional; default struct())
%       .FigureVisible is "on" or "off" (default "on").
%       .Title is scalar text (default "HS-3 motion profile").
%**************************************************************************
% OUTPUTS
%   - handles (scalar struct)
%       Figure and four axes handles.
%**************************************************************************
% UNITS
%   - Time is seconds. Position is degrees. Derivative units are stated on
%     each axis. Histories use [azimuth elevation] order.
%**************************************************************************

%% Section 1: Validate Inputs & Apply Defaults

if nargin < 2 || isempty(optionOverrides)
    optionOverrides = struct();
end
defaults = struct( ...
    "FigureVisible", "on", ...
    "Title", "HS-3 motion profile");
[options, unknownOptions] = azElInternal.resolveOptions( ...
    defaults, optionOverrides);
if ~isempty(unknownOptions)
    warning("plotAzElHs3Motion:UnknownOptions", ...
        "Ignored unknown option fields: %s. No behavior changed.", ...
        strjoin(unknownOptions, ", "));
end
if ~isstruct(result) || ~isscalar(result) || ...
        ~isfield(result, "Success") || ~result.Success
    error("plotAzElHs3Motion:UnsuccessfulResult", ...
        "result must be a successful solveAzElHs3Segments result.");
end
options.FigureVisible = lower(string(options.FigureVisible));
if ~isscalar(options.FigureVisible) || ~any( ...
        options.FigureVisible == ["on" "off"])
    error("plotAzElHs3Motion:InvalidFigureVisible", ...
        "FigureVisible must be on or off.");
end
options.Title = string(options.Title);
if ~isscalar(options.Title)
    error("plotAzElHs3Motion:InvalidTitle", ...
        "Title must be scalar text.");
end

%% Section 2: Plot Motion And Limits

figureHandle = figure( ...
    "Visible", options.FigureVisible, ...
    "Name", options.Title);
layout = tiledlayout(figureHandle, 4, 1, ...
    "TileSpacing", "compact", ...
    "Padding", "compact");
axesHandles = gobjects(4, 1);
valueFields = ["position_deg" "velocity_deg_s" ...
    "acceleration_deg_s2" "jerk_deg_s3"];
limitFields = ["" "maxVelocity_deg_s" ...
    "maxAcceleration_deg_s2" "maxJerk_deg_s3"];
yLabels = ["Position (deg)" "Velocity (deg/s)" ...
    "Acceleration (deg/s^2)" "Jerk (deg/s^3)"];
for quantityIndex = 1:4
    axesHandle = nexttile(layout);
    axesHandles(quantityIndex) = axesHandle;
    hold(axesHandle, "on");
    plot(axesHandle, result.time_s, ...
        result.(valueFields(quantityIndex))(:, 1), ...
        "LineWidth", 1.3, "DisplayName", "Azimuth");
    plot(axesHandle, result.time_s, ...
        result.(valueFields(quantityIndex))(:, 2), ...
        "LineWidth", 1.3, "DisplayName", "Elevation");
    if limitFields(quantityIndex) ~= ""
        limitValues = result.(limitFields(quantityIndex));
        plot(axesHandle, result.time_s, limitValues(:, 1), "--", ...
            "Color", [0.3 0.3 0.3], ...
            "HandleVisibility", "off");
        plot(axesHandle, result.time_s, -limitValues(:, 1), "--", ...
            "Color", [0.3 0.3 0.3], ...
            "HandleVisibility", "off");
        plot(axesHandle, result.time_s, limitValues(:, 2), ":", ...
            "Color", [0.45 0.45 0.45], ...
            "HandleVisibility", "off");
        plot(axesHandle, result.time_s, -limitValues(:, 2), ":", ...
            "Color", [0.45 0.45 0.45], ...
            "HandleVisibility", "off");
    end
    boundaryIndices = find(diff(result.SegmentIndex) > 0);
    for boundaryIndex = boundaryIndices(:).'
        xline(axesHandle, result.time_s(boundaryIndex + 1), "-.", ...
            "Color", [0.55 0.55 0.55], ...
            "HandleVisibility", "off");
    end
    ylabel(axesHandle, yLabels(quantityIndex));
    grid(axesHandle, "on");
    box(axesHandle, "on");
end
xlabel(axesHandles(end), "Time (s)");
legend(axesHandles(1), "Location", "best");
title(layout, options.Title);

%% Section 3: Return Handles

handles = struct( ...
    "Figure", figureHandle, ...
    "Axes", axesHandles);
end
