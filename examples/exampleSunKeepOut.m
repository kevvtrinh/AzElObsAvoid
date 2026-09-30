function result = exampleSunKeepOut(exampleOverrides)
%% Section 0: Header & Readme
% SYNTAX
%   result = exampleSunKeepOut()
%   result = exampleSunKeepOut(exampleOverrides)
%**************************************************************************
% PURPOSE
%   - Rebuild the owner's Sun keep-out scenario without STK and plan the
%     original fixed-arrival slew. Print normalization and validation results.
%   - This is the owner's reported crash case: a native access violation in
%     libmwpolyfun on R2024b 24.2.0.2712019. On R2024b Update 4
%     (24.2.0.2833386), it plans successfully in about 250 s at about 3.3 GB
%     peak memory. Runtime and memory depend on the machine.
%   - The Sun stays below the 20 deg elevation limit for the whole window.
%**************************************************************************
% INPUTS
%   - exampleOverrides (scalar struct, optional; default struct())
%       Uniform display controls and public planner option overrides.
%       FigureVisible = "off" allows plotting in a batch run.
%**************************************************************************
% OUTPUTS
%   - result (scalar struct)
%       Unmodified public planner result. Ordinary planning failure returns
%       Success = false; invalid input throws an error.
%**************************************************************************
% UNITS
%   - Positions are [azimuth elevation] in degrees in the sensor frame.
%     Time is elapsed seconds; velocity, acceleration, and jerk use deg/s,
%     deg/s^2, and deg/s^3. Sun direction vectors are dimensionless.
%**************************************************************************

%% Section 1: Resolve Example Controls
if nargin < 1 || isempty(exampleOverrides)
    exampleOverrides = struct();
end
scenarioDefaults = struct( ...
    "GoalTimeMode", "fixedArrival", ...
    "SampleTime_s", 1, ...
    "Title",        "Owner's Sun keep-out slew (STK-free)");
[options, displayOptions] = resolveExampleOptions( ...
    exampleOverrides, scenarioDefaults, [2 2]);

%% Section 2: Create Obstacles
% These 17 Sun-center anchors come from the owner's STK output for a 900 km
% sun-synchronous satellite. PCHIP on the original 0.25 s grid reproduces
% that output to about 1e-7 deg; this is a recorded-window reconstruction,
% not an orbit propagator or a Sun ephemeris for other times.
sunAnchors = [ ...
    7820.0, 25.851430421,  -3.257397244;
    7835.0, 25.875735989,  -4.042770596;
    7850.0, 25.905314939,  -4.827965390;
    7865.0, 25.940193930,  -5.612946254;
    7880.0, 25.980404459,  -6.397677500;
    7895.0, 26.025982924,  -7.182123072;
    7910.0, 26.076970689,  -7.966246486;
    7925.0, 26.133414164,  -8.750010781;
    7940.0, 26.195364894,  -9.533378455;
    7955.0, 26.262879656, -10.316311413;
    7970.0, 26.336020566, -11.098770899;
    7985.0, 26.414855203, -11.880717439;
    8000.0, 26.499456731, -12.662110775;
    8015.0, 26.589904048, -13.442909798;
    8030.0, 26.686281934, -14.223072477;
    8045.0, 26.788681216, -15.002555790;
    8060.0, 26.897198946, -15.781315648];
time_s    = (7820:0.25:8060).';
sunAz_deg = pchip(sunAnchors(:, 1), sunAnchors(:, 2), time_s);
sunEl_deg = pchip(sunAnchors(:, 1), sunAnchors(:, 3), time_s);
sunUnitSensor_1 = [cosd(sunEl_deg) .* cosd(sunAz_deg), ...
    cosd(sunEl_deg) .* sind(sunAz_deg), sind(sunEl_deg)];
sunKeepOut = buildSunKeepOutAzEl(time_s, sunUnitSensor_1, sunAz_deg, 4, 73);

% Preserve the owner's source-index correspondence and zero safety margin.
% At 8052.5 s, closure roundoff makes a crossing seam sliver. Normalization
% removes its two vertices and keeps 71; no slice is emptied. The adjacent
% 72-to-71 and 71-to-72 intervals explicitly use endpoint convex hulls because
% their unequal counts cannot support source-index vertex correspondence.
obstacles = obstacleAvoidance.obstacles.createObstacle( ...
    'Sun keep-out', sunKeepOut.time_s, sunKeepOut.az_deg, sunKeepOut.el_deg, ...
    0, struct('vertexCorrespondence', 'sourceIndex'));
vertexCountBySample = cellfun(@numel, obstacles.x_units);
normalization       = obstacles.NormalizationDiagnostics;
shortSliceTimes_s   = obstacles.time_s(vertexCountBySample < 3);
fprintf('Sun samples: %d, time [%.2f %.2f] s, step %.2f s.\n', ...
    numel(time_s), time_s(1), time_s(end), time_s(2) - time_s(1));
fprintf('Sun center: az [%.9f %.9f] deg, el [%.9f %.9f] deg.\n', ...
    min(sunAz_deg), max(sunAz_deg), min(sunEl_deg), max(sunEl_deg));
fprintf('Normalized vertices per slice: min=%d, max=%d.\n', ...
    min(vertexCountBySample), max(vertexCountBySample));

% Diagnostics list protected and original geometry separately. With zero
% margin they match; printing each role avoids counting removals twice.
for roleIndex = 1:numel(normalization.Roles)
    fprintf('Normalization (%s): removed duplicates=%d, removed zigzags=%d.\n', ...
        normalization.Roles(roleIndex), ...
        normalization.RemovedDuplicateVertexCount(roleIndex), ...
        sum(normalization.RemovedZigzagVertexCountBySample(:, roleIndex)));
end
fprintf('Slice times with fewer than 3 vertices (s): %s.\n', ...
    mat2str(shortSliceTimes_s.', 12));

%% Section 3: Create Planner Inputs
initialState = struct("time_s", 7880, "position_units", [80 25]);
goalState    = struct("time_s", 8000, "position_units", [0 80]);
limits       = struct( ...
    "xInterval_units",          [-150 150], ...
    "yInterval_units",          [20 90], ...
    "maxVelocity_units_s",      [2 2], ...
    "maxAcceleration_units_s2", [0.75 0.75], ...
    "maxJerk_units_s3",         displayOptions.MaxJerk_units_s3);

%% Section 4: Run Planner
% Time the public planner call alone, including its obstacle preparation.
planningTimer  = tic;
result         = planner(obstacles, initialState, goalState, limits, options);
planningTime_s = toc(planningTimer);
fprintf('Planner Success: %d\n', result.Success);
fprintf('Planner Message: %s\n', result.Message);
fprintf('Planner TerminationReason: %s\n', result.TerminationReason);
fprintf('Planning wall time: %.3f s\n', planningTime_s);

%% Section 5: Validate Result
% Recheck the returned motion with the public independent validator through
% the same example helper used by other maintained examples.
exampleValidation = validateExampleResult(result, "Owner's Sun keep-out slew");
trajectoryValidation = exampleValidation.TrajectoryValidation;
fprintf('Independent validation Passed: %d\n', trajectoryValidation.Passed);
fprintf('Independent validation Message: %s\n', trajectoryValidation.Message);
fprintf('Example validation Passed: %d\n', exampleValidation.Passed);
fprintf('Example validation Message: %s\n', exampleValidation.Message);
if ~exampleValidation.Passed
    warning("exampleSunKeepOut:ValidationFailed", "%s", exampleValidation.Message);
end

%% Section 6: Plot Diagnostics And Motion
if displayOptions.PlotOutputs
    obstacleAvoidance.plotting.plotTrajectory(result, displayOptions.PlotOptions);

    % Show source rings so a normalization loss cannot hide the supplied
    % keep-out geometry. Each ring is a snapshot, not a swept obstacle region.
    figureHandle = figure('Name', "Sun keep-out in sensor azimuth/elevation", ...
        'Visible', displayOptions.FigureVisible);
    axesHandle = axes('Parent', figureHandle);
    hold(axesHandle, 'on');
    plot(axesHandle, sunAz_deg, sunEl_deg, 'Color', [0.85 0.45 0], ...
        'LineWidth', 1.5, 'DisplayName', 'Sun center path');
    ringTimes_s = [7820 7880 7940 8000 8052.5 8060];
    ringColors  = parula(numel(ringTimes_s));
    for ringIndex = 1:numel(ringTimes_s)
        sampleIndex = find(time_s == ringTimes_s(ringIndex), 1);
        plot(axesHandle, sunKeepOut.az_deg{sampleIndex}, ...
            sunKeepOut.el_deg{sampleIndex}, 'Color', ringColors(ringIndex, :), ...
            'DisplayName', sprintf('4 deg ring at %.2f s', ringTimes_s(ringIndex)));
    end

    azLimits_deg = limits.xInterval_units;
    elLimits_deg = limits.yInterval_units;
    plot(axesHandle, azLimits_deg([1 2 2 1 1]), ...
        elLimits_deg([1 1 2 2 1]), 'k--', 'LineWidth', 1.2, ...
        'DisplayName', 'Sensor limits');
    if ~isempty(result.position_units)
        plot(axesHandle, result.position_units(:, 1), result.position_units(:, 2), ...
            'b-', 'LineWidth', 1.8, 'DisplayName', 'Planned path');
    end
    plot(axesHandle, initialState.position_units(1), initialState.position_units(2), ...
        'go', 'MarkerFaceColor', 'g', 'DisplayName', 'Start: 7880 s');
    plot(axesHandle, goalState.position_units(1), goalState.position_units(2), ...
        'rp', 'MarkerFaceColor', 'r', 'MarkerSize', 10, 'DisplayName', 'Goal: 8000 s');
    xlabel(axesHandle, 'Sensor azimuth (deg)');
    ylabel(axesHandle, 'Sensor elevation (deg)');
    title(axesHandle, displayOptions.Title);
    axis(axesHandle, 'equal');
    grid(axesHandle, 'on');
    legend(axesHandle, 'Location', 'eastoutside');
end
end

%% Section 7: Local Functions

function sunKeepOut = buildSunKeepOutAzEl( ...
        time_s, sunUnitSensor_1, sunAz_deg, exclusionAngle_deg, boundaryPointCount)
    % Preserve the owner's cone construction and repeated closing point.
    % Known limits: near zenith, this az/el polygon loses cap interiors when
    % the cap contains the pole. The reference-basis switch changes the
    % boundary phase, so matching source indices can then join different points.
    numberOfTimes  = numel(time_s);
    boundaryAz_deg = cell(numberOfTimes, 1);
    boundaryEl_deg = cell(numberOfTimes, 1);
    phase_rad      = linspace(0, 2 * pi, boundaryPointCount).';
    coneAngle_rad  = deg2rad(exclusionAngle_deg);
    for sampleIndex = 1:numberOfTimes
        sunDirection = sunUnitSensor_1(sampleIndex, :).';
        if abs(dot(sunDirection, [0; 0; 1])) < 0.9
            reference = [0; 0; 1];
        else
            reference = [0; 1; 0];
        end
        basisU = cross(reference, sunDirection);
        basisU = basisU / norm(basisU);
        basisV = cross(sunDirection, basisU);
        basisV = basisV / norm(basisV);
        coneDirections = ...
            cos(coneAngle_rad) * sunDirection.' + ...
            sin(coneAngle_rad) * ( ...
                cos(phase_rad) * basisU.' + ...
                sin(phase_rad) * basisV.');
        coneDirections = coneDirections ./ vecnorm(coneDirections, 2, 2);
        rawAz_deg = atan2d(coneDirections(:, 2), coneDirections(:, 1));
        el_deg = atan2d(coneDirections(:, 3), ...
            hypot(coneDirections(:, 1), coneDirections(:, 2)));

        % Keep azimuth continuous around the center: for center = 179 deg,
        % a boundary point at -179 deg becomes 181 deg, only 2 deg away.
        localDeltaAz_deg = mod(rawAz_deg - sunAz_deg(sampleIndex) + 180, 360) - 180;
        boundaryAz_deg{sampleIndex} = sunAz_deg(sampleIndex) + localDeltaAz_deg;
        boundaryEl_deg{sampleIndex} = el_deg;
    end
    sunKeepOut = struct( ...
        'targetName',        'Sun keep-out', ...
        'time_s',            double(time_s(:)), ...
        'centerAz_deg',      double(sunAz_deg(:)), ...
        'centerEl_deg',      atan2d(sunUnitSensor_1(:, 3), ...
            hypot(sunUnitSensor_1(:, 1), sunUnitSensor_1(:, 2))), ...
        'angularRadius_deg', double(exclusionAngle_deg), ...
        'az_deg',            {boundaryAz_deg}, ...
        'el_deg',            {boundaryEl_deg});
end
