function [result, runtimeTable] = exampleVietnamSunRuntime(exampleOverrides)
%% Section 0: Header & Readme
% SYNTAX
%   result = exampleVietnamSunRuntime()
%   [result, runtimeTable] = exampleVietnamSunRuntime( ...
%       struct('runRuntimeStudy', true, 'FigureVisible', 'off'))
%**************************************************************************
% PURPOSE
%   - Graph/route dominated: 817 of 840 s; BMTP 19 s. Latest owner: 818.721 s.
%   - Recommended: Vietnam 50 vertices at 3 s, Sun 16-gon at 25 s: 20.531 s
%     median (97.5% saved). Parent: owner 961 x 73 = 840 s; 10 s / 12 x 16 = 46.6 s
%     (94% saved); 25 s / 6 x 16 = 28.5 s (97% saved); 1 x 18 static hull = 31.9 s.
%     All were valid, with 0 true-cone points outside at 0.25 s / 720 points.
%   - Circumscription adds about 2% of the 4 deg radius. Az/el bends the edges;
%     add a 0.05 deg pad (vertex radius 4.128 deg). Small added blocked area,
%     no longer path. Refuse lean geometry if its fine-grid check fails.
%   - Further LOSSY step: 24 Vietnam lines add 1649 deg^2, path 131 -> 145 deg;
%     latest plan 1.014 s (old aligned Sun: 1.994 s). Both rows run twice below.
%   - Old Sun 1/3/6 s: 69.619/39.266/37.225 s; aligned 3 s saved 39% (24.003 s).
%     Vietnam at 6 s: 20.905 s. Clipping/output sampling did not help; temporal
%     tuning saved little within variation. Keep defaults and full validation.
%   - Proved Sun omission: 26.200 s; with y >= -50, 6.478 -> 2.145 s. Recompute
%     the speed proof for each request; distance from a plotted path is no proof.
%   - Unsafe: inward simplification, relaxed tolerances, skipped validation.
%     Not recommended: Vietnam box (+2525 deg^2, 166 deg path) or enclosing
%     16-gon (+10164 deg^2, blocked a feasible request). All old successes
%     passed full validation; coarse timing still changes geometry.
%   - Full Vietnam: 0.25 s PCHIP of all 38 samples; original 4829-sample history
%     unavailable. Fewer anchors missed by 0.026-0.144 deg. Other MATLAB jobs
%     were running and the parent owner row ran cold. Compare medians below.
%   - Previous 19-row evidence, verbatim: two-run PROFILED medians.
% Variant | Samples(Vietnam Sun) | Vertices(Vietnam Sun) | AddedArea_deg2 | MotionLength_deg | Create_s | Plan_s | FullCheck_s | Total_s | ProfileCreate_s | Prep_s | Graph_s | BMTP_s | PlannerValid_s | Other_s | Success | FullValid
% owner 3 s | [38;961] | [50;73] | 0.000 | 131.128 | 0.400 | 840.320 | 10.211 | 850.931 | 0.369 | 2.233 | 817.097 | 18.557 | 2.197 | 0.268 | 1 | 1
% time 0.25 s | [441;82] | [50;73] | 0.000 | 131.312 | 0.230 | 1084.943 | 11.856 | 1097.029 | 0.187 | 2.344 | 1013.420 | 65.994 | 2.904 | 0.326 | 1 | 1
% time 1 s | [111;82] | [50;73] | 0.000 | 131.774 | 0.174 | 93.331 | 9.702 | 103.208 | 0.128 | 0.894 | 69.883 | 21.372 | 0.944 | 0.285 | 1 | 1
% time 6 s | [20;82] | [50;73] | 0.000 | 130.823 | 0.188 | 20.905 | 10.173 | 31.266 | 0.135 | 0.957 | 8.201 | 10.735 | 0.732 | 0.334 | 1 | 1
% clip 3 s | [35;35] | [50;73] | 0.000 | 130.822 | 0.211 | 42.505 | 19.935 | 62.651 | 0.152 | 0.529 | 11.744 | 29.211 | 0.719 | 0.362 | 1 | 1
% box 4 vertices | [38;82] | [4;73] | 2525.492 | 166.252 | 0.337 | 11.327 | 13.161 | 24.825 | 0.191 | 0.414 | 0.416 | 9.841 | 0.311 | 0.493 | 1 | 1
% ring 16 vertices | [38;82] | [16;73] | 10163.612 | NaN | 0.343 | 0.725 | 0.001 | 1.069 | 0.175 | 0.513 | 0.000 | 0.000 | 0.000 | 0.381 | 0 | 0
% output 1 s | [38;82] | [50;73] | 0.000 | 130.822 | 0.280 | 27.762 | 11.630 | 39.672 | 0.215 | 0.591 | 9.031 | 17.177 | 0.578 | 0.452 | 1 | 1
% temporal 1 s | [38;82] | [50;73] | 0.000 | 130.822 | 0.178 | 22.758 | 9.378 | 32.315 | 0.127 | 0.435 | 8.505 | 13.035 | 0.524 | 0.311 | 1 | 1
% support 24 vertices | [38;82] | [24;73] | 1649.364 | 145.099 | 0.231 | 1.994 | 8.895 | 11.119 | 0.120 | 0.244 | 0.219 | 1.131 | 0.216 | 0.295 | 1 | 1
% 24 vertices + clip + Sun full | [35;401] | [24;73] | 1649.364 | 145.099 | 0.292 | 5.285 | 8.915 | 14.492 | 0.205 | 1.481 | 0.195 | 1.779 | 1.650 | 0.267 | 1 | 1
% Sun omitted: reachability proved | 38 | 50 | 0.000 | 130.846 | 0.125 | 26.200 | 10.499 | 36.824 | 0.095 | 0.276 | 10.412 | 14.852 | 0.404 | 0.287 | 1 | 1
% Sun 3 s aligned: control | [38;82] | [50;73] | 0.000 | 130.822 | 0.197 | 24.003 | 9.702 | 33.902 | 0.143 | 0.478 | 8.699 | 14.026 | 0.510 | 0.344 | 1 | 1
% Sun time 1 s | [38;241] | [50;73] | 0.000 | 131.128 | 0.211 | 69.619 | 9.581 | 79.411 | 0.153 | 0.505 | 55.092 | 13.157 | 0.629 | 0.295 | 1 | 1
% Sun time 3 s | [38;81] | [50;73] | 0.000 | 130.835 | 0.176 | 39.266 | 9.075 | 48.518 | 0.128 | 0.427 | 25.063 | 13.079 | 0.450 | 0.295 | 1 | 1
% Sun time 6 s | [38;41] | [50;73] | 0.000 | 130.873 | 0.183 | 37.225 | 10.553 | 47.961 | 0.133 | 0.587 | 18.742 | 17.097 | 0.500 | 0.350 | 1 | 1
% outside Sun retained: y >= -50 | [35;401] | [24;73] | 1649.364 | 145.099 | 0.354 | 6.478 | 10.776 | 17.608 | 0.241 | 1.888 | 0.218 | 2.171 | 1.957 | 0.356 | 1 | 1
% outside Sun omitted: y >= -50 | 35 | 24 | 1649.364 | 145.099 | 0.211 | 2.145 | 10.762 | 13.119 | 0.106 | 0.143 | 0.255 | 1.326 | 0.188 | 0.339 | 1 | 1
% recommended | [35;35] | [24;73] | 1649.364 | 145.099 | 0.255 | 2.256 | 9.825 | 12.336 | 0.132 | 0.325 | 0.256 | 1.251 | 0.208 | 0.340 | 1 | 1
%**************************************************************************
% INPUTS
%   - exampleOverrides (scalar struct, optional; default struct())
%       Shared display/planner controls; runRuntimeStudy defaults to false.
%       This recorded window requires fixed arrival and unwrapped axes.
%**************************************************************************
% OUTPUTS
%   - result: unmodified public planner result; expected failure has Success
%       = false, invalid inputs throw. FullValid must also pass before use.
%   - runtimeTable: four comparison rows, otherwise empty. Slices/Vertices
%       are [Vietnam Sun]; area is the sum of their maximum added areas.
%**************************************************************************
% UNITS
%   - Sensor [azimuth elevation] in deg, elapsed time in s; velocity,
%     acceleration and jerk in deg/s, deg/s^2, deg/s^3; az/el area in deg^2.
%     Path length is the planner's sampled estimate, not an exact integral.
%**************************************************************************

%% Section 1: Resolve Example Controls
if nargin < 1 || isempty(exampleOverrides)
    exampleOverrides = struct();
end
validateattributes(exampleOverrides, {'struct'}, {'scalar'});
runRuntimeStudy = false;
if isfield(exampleOverrides, 'runRuntimeStudy')
    runRuntimeStudy = obstacleAvoidance.input.normalizeLogicalScalar(exampleOverrides.runRuntimeStudy, ...
        'runRuntimeStudy', 'exampleVietnamSunRuntime:InvalidStudyFlag');
    exampleOverrides = rmfield(exampleOverrides, 'runRuntimeStudy');
end
scenarioDefaults = struct( ...
    'GoalTimeMode', 'fixedArrival', ...
    'SampleTime_s', 0.05, ...
    'ShowAnimation', false, ...
    'Title', 'Vietnam and lean Sun keep-out');
[options, displayOptions] = resolveExampleOptions(exampleOverrides, scenarioDefaults, [2 2]);
if options.GoalTimeMode ~= "fixedArrival" || options.WrapX || options.WrapY
    error('exampleVietnamSunRuntime:UnsupportedRequestMode', 'Use fixedArrival and unwrapped axes.');
end
runtimeTable = table();

%% Section 2: Create Obstacles
[vietnamTime_s, sourceX_deg, sourceY_deg, sunAnchors] = sourceHistory();
fullTime_s = (2765:0.25:2875).';
fullX_deg  = pchip(vietnamTime_s, sourceX_deg.', fullTime_s).';
fullY_deg  = pchip(vietnamTime_s, sourceY_deg.', fullTime_s).';
vietnam = makeObstacle('Vietnam', vietnamTime_s, sourceX_deg, sourceY_deg);
fullVietnam = makeObstacle('Vietnam', fullTime_s, fullX_deg, fullY_deg);
sunTime_s = (2700:0.25:2940).';
[ownerX_deg, ownerY_deg] = sunBoundary(sunAnchors, sunTime_s, 73, 4, true);
fullSun = makeObstacle('Sun keep-out', sunTime_s, ownerX_deg, ownerY_deg);
fullObstacles = [fullVietnam; fullSun];

% Bracket the request. Planner vertices interpolate linearly between slices.
leanTime_s = (2750:25:2875).';
[leanX_deg, leanY_deg] = sunBoundary(sunAnchors, leanTime_s, 16, 4 / cos(pi / 16) + 0.05, false);
leanSun = makeObstacle('Sun keep-out', leanTime_s, leanX_deg, leanY_deg);
proofTime_s = (2770:0.25:2870).';
[coneX_deg, coneY_deg] = sunBoundary(sunAnchors, proofTime_s, 720, 4, false);
checkX_deg = interp1(leanSun.time_s, cell2mat(leanSun.x_units.').', proofTime_s);
checkY_deg = interp1(leanSun.time_s, cell2mat(leanSun.y_units.').', proofTime_s);
for sampleIndex = 1:numel(proofTime_s)
    inside = inpolygon(coneX_deg(sampleIndex, :), coneY_deg(sampleIndex, :), checkX_deg(sampleIndex, :), checkY_deg(sampleIndex, :));
    if ~all(inside)
        error('exampleVietnamSunRuntime:ContainmentFailed', 'Lean Sun misses the true cone; refuse this input.');
    end
end
sunAddedArea_deg2 = max(polyarea(checkX_deg, checkY_deg, 2) - polyarea(coneX_deg, coneY_deg, 2));
fprintf('Lean Sun containment: 0 outside points (%d times x 720 cone points).\n', numel(proofTime_s));

%% Section 3: Create Planner Inputs
initialState = struct('time_s', 2770, 'position_units', [80 0]);
goalState    = struct('time_s', 2870, 'position_units', [0 80]);
limits = struct( ...
    'xInterval_units', [-180 180], ...
    'yInterval_units', [-90 90], ...
    'maxVelocity_units_s', [2 2], ...
    'maxAcceleration_units_s2', [0.75 0.75], ...
    'maxJerk_units_s3', displayOptions.MaxJerk_units_s3);
% Travel distance <= speed x time. The SAME endpoint-box side must exclude
% the Sun at both ends of each affine interval to prove continuous exclusion.
active = sunTime_s >= initialState.time_s & sunTime_s <= goalState.time_s;
activeTime_s = sunTime_s(active);
fromStart_deg = (activeTime_s - initialState.time_s) * limits.maxVelocity_units_s;
toGoal_deg = (goalState.time_s - activeTime_s) * limits.maxVelocity_units_s;
sunLower_deg = [min(ownerX_deg(active, :), [], 2), min(ownerY_deg(active, :), [], 2)];
sunUpper_deg = [max(ownerX_deg(active, :), [], 2), max(ownerY_deg(active, :), [], 2)];
separated = [sunUpper_deg < initialState.position_units - fromStart_deg, ...
    sunLower_deg > initialState.position_units + fromStart_deg, ...
    sunUpper_deg < goalState.position_units - toGoal_deg, ...
    sunLower_deg > goalState.position_units + toGoal_deg];
sunExcluded = all(any(separated(1:end - 1, :) & separated(2:end, :), 2));
fprintf('Sun excluded by continuous forward/backward speed bounds: %d.\n', sunExcluded);

%% Section 4: Run Planner
obstacles = [vietnam; leanSun];
planningTimer  = tic;
result         = planner(obstacles, initialState, goalState, limits, options);
planningTime_s = toc(planningTimer);
fprintf('Default: Success=%d, plan %.3f s, reason=%s.\n', result.Success, planningTime_s, result.TerminationReason);

%% Section 5: Validate Result
validationTimer = tic;
fullValidation  = validateFullHistory(result, fullObstacles);
fullCheckTime_s = toc(validationTimer);
fprintf('Default: FullValid=%d, full-check %.3f s. %s\n', ...
    fullValidation.Passed, fullCheckTime_s, fullValidation.Message);
fprintf('Default: added area %.3f deg^2, path length %.3f deg.\n', sunAddedArea_deg2, result.MotionLength_units);
if ~fullValidation.Passed
    warning('exampleVietnamSunRuntime:ValidationFailed', 'Do not use this motion. %s', fullValidation.Message);
end

%% Section 6: Plot Diagnostics And Motion
if displayOptions.PlotOutputs
    obstacleAvoidance.plotting.plotTrajectory(result, displayOptions.PlotOptions);
    figureHandle = figure('Name', 'Vietnam and Sun', 'Visible', displayOptions.FigureVisible);
    axesHandle = axes('Parent', figureHandle);
    hold(axesHandle, 'on');
    snapshotTimes_s = [2770 2800 2840 2870];
    sampleIndices = find(ismember(fullTime_s, snapshotTimes_s));
    vietnamHandles = plot(axesHandle, fullX_deg(sampleIndices, [1:end 1]).', fullY_deg(sampleIndices, [1:end 1]).');
    set(vietnamHandles, {'DisplayName'}, cellstr(compose('Vietnam %.0f s', snapshotTimes_s.')));
    coneIndices = find(ismember(proofTime_s, snapshotTimes_s));
    sunHandles = plot(axesHandle, coneX_deg(coneIndices, [1:end 1]).', coneY_deg(coneIndices, [1:end 1]).', '--');
    set(sunHandles, {'DisplayName'}, cellstr(compose('True Sun %.0f s', snapshotTimes_s.')));
    plot(axesHandle, limits.xInterval_units([1 2 2 1 1]), limits.yInterval_units([1 1 2 2 1]), 'k--', 'DisplayName', 'Sensor limits');
    plot(axesHandle, result.position_units(:, 1), result.position_units(:, 2), 'b-', 'DisplayName', 'Planned path');
    plot(axesHandle, [80 0], [0 80], 'ko', 'DisplayName', 'Start / goal');
    xlabel(axesHandle, 'Sensor azimuth (deg)');
    ylabel(axesHandle, 'Sensor elevation (deg)');
    title(axesHandle, displayOptions.Title);
    set(axesHandle, 'DataAspectRatio', [1 1 1], 'XGrid', 'on', 'YGrid', 'on');
    legend(axesHandle, 'Location', 'eastoutside');
end

%% Section 7: Compare Input Choices
if runRuntimeStudy
    % Fixed support directions give a convex enclosure. Add the largest
    % missed dense projection to every line, plus an outward arithmetic pad.
    angles_rad = (0:23) * (2 * pi / 24);
    normalX_1 = cos(angles_rad);
    normalY_1 = sin(angles_rad);
    fullSupport_deg = zeros(numel(fullTime_s), 24);
    for directionIndex = 1:24
        fullSupport_deg(:, directionIndex) = max(fullX_deg * normalX_1(directionIndex) + fullY_deg * normalY_1(directionIndex), [], 2);
    end
    support_deg = interp1(fullTime_s, fullSupport_deg, vietnamTime_s);
    missed_deg = fullSupport_deg - interp1(vietnamTime_s, support_deg, fullTime_s);
    support_deg = support_deg + max(0, max(missed_deg, [], 'all')) + 1e-9;
    neighborIndices = [2:24 1];
    reducedX_deg = (support_deg .* normalY_1(neighborIndices) - ...
        support_deg(:, neighborIndices) .* normalY_1) / sin(2 * pi / 24);
    reducedY_deg = (support_deg(:, neighborIndices) .* normalX_1 - ...
        support_deg .* normalX_1(neighborIndices)) / sin(2 * pi / 24);
    reducedVietnam = makeObstacle('Vietnam', vietnamTime_s, reducedX_deg, reducedY_deg);
    denseX_deg = interp1(vietnamTime_s, reducedX_deg, fullTime_s);
    denseY_deg = interp1(vietnamTime_s, reducedY_deg, fullTime_s);
    vietnamAddedArea_deg2 = max(polyarea(denseX_deg, denseY_deg, 2) - polyarea(fullX_deg, fullY_deg, 2));
    % Fixed normals and affine offsets also contain the reference between samples.
    if ~sunExcluded
        error('exampleVietnamSunRuntime:UnprovedSunOmission', 'Cannot run the omission row without proof.');
    end
    variantNames = ["owner inputs"; "Sun omitted by proof"; "lossy Vietnam 24"; "recommended lean Sun"];
    comparisonObstacles = {[vietnam; fullSun], vietnam, [reducedVietnam; leanSun], [vietnam; leanSun]};
    addedAreas_deg2 = [0; 0; vietnamAddedArea_deg2 + sunAddedArea_deg2; sunAddedArea_deg2];
    fprintf('Comparison: unprofiled two-run medians; slices/vertices = [Vietnam Sun].\n');
    fprintf('Variant | Slices | Vertices | Plan_s | FullCheck_s | AddedArea_deg2 | Path_deg | Success | FullValid\n');
    for variantIndex = 1:4
        trialObstacles = comparisonObstacles{variantIndex};
        measurements = zeros(2, 5);
        for repeatIndex = 1:2
            timer = tic;
            trialResult = planner(trialObstacles, initialState, goalState, limits, options);
            measurements(repeatIndex, 1) = toc(timer);
            timer = tic;
            trialValidation = validateFullHistory(trialResult, fullObstacles);
            measurements(repeatIndex, 2) = toc(timer);
            measurements(repeatIndex, 3:5) = [trialResult.MotionLength_units, trialResult.Success, trialValidation.Passed];
        end
        slices = string(mat2str(arrayfun(@(item) numel(item.time_s), trialObstacles).'));
        vertices = string(mat2str(arrayfun(@(item) max(cellfun(@numel, item.x_units)), trialObstacles).'));
        measured = median(measurements(:, 1:3), 1);
        passed = all(measurements(:, 4:5), 1);
        runtimeTable = [runtimeTable; table(variantNames(variantIndex), slices, vertices, ...
            measured(1), measured(2), addedAreas_deg2(variantIndex), measured(3), ...
            passed(1), passed(2), 'VariableNames', ...
            {'Variant', 'Slices', 'Vertices', 'Plan_s', 'FullCheck_s', 'AddedArea_deg2', 'Path_deg', 'Success', 'FullValid'})]; %#ok<AGROW>
        fprintf('%s | %s | %s | %.3f | %.3f | %.3f | %.3f | %d | %d\n', ...
            variantNames(variantIndex), slices, vertices, measured(1:2), ...
            addedAreas_deg2(variantIndex), measured(3), passed(1), passed(2));
    end
end
end

%% Section 8: Local Functions
function obstacle = makeObstacle(name, time_s, x_deg, y_deg)
    obstacle = obstacleAvoidance.obstacles.createObstacle(name, time_s, ...
        num2cell(x_deg.', 1).', num2cell(y_deg.', 1).', 0, struct('vertexCorrespondence', 'sourceIndex'));
    if any(cellfun(@numel, obstacle.x_units) < 3)
        error('exampleVietnamSunRuntime:LostSlice', 'Normalization emptied a supplied polygon.');
    end
end

function [x_deg, y_deg] = sunBoundary(anchors, time_s, cornerCount, radius_deg, useOwnerRing)
    centerAz_deg = pchip(anchors(:, 1), anchors(:, 2), time_s);
    centerEl_deg = pchip(anchors(:, 1), anchors(:, 3), time_s);
    phase_rad = (0:cornerCount - 1).' * (2 * pi / cornerCount);
    if useOwnerRing
        phase_rad = linspace(0, 2 * pi, cornerCount).';
    end
    x_deg = zeros(numel(time_s), cornerCount);
    y_deg = x_deg;
    for sampleIndex = 1:numel(time_s)
        center_1 = [cosd(centerEl_deg(sampleIndex)) * cosd(centerAz_deg(sampleIndex)); ...
            cosd(centerEl_deg(sampleIndex)) * sind(centerAz_deg(sampleIndex)); sind(centerEl_deg(sampleIndex))];
        reference_1 = [0; 0; 1];
        if useOwnerRing && abs(center_1(3)) >= 0.9
            reference_1 = [0; 1; 0];
        end
        basisU_1 = cross(reference_1, center_1);
        % Near zenith the basis degenerates; an az/el ring cannot cover a pole.
        if norm(basisU_1) < 1e-8 || abs(centerEl_deg(sampleIndex)) + radius_deg >= 90
            error('exampleVietnamSunRuntime:SunNearPole', 'Sun basis or az/el polygon is undefined near a pole.');
        end
        basisU_1 = basisU_1 / norm(basisU_1);
        basisV_1 = cross(center_1, basisU_1);
        % Tangent-plane circumscription plus projection pad; check in az/el.
        points_1 = cosd(radius_deg) * center_1.' + sind(radius_deg) * ...
            (cos(phase_rad) * basisU_1.' + sin(phase_rad) * basisV_1.');
        rawAz_deg = atan2d(points_1(:, 2), points_1(:, 1));
        % Center 179, point -179 becomes 181: keep the local ring continuous.
        x_deg(sampleIndex, :) = centerAz_deg(sampleIndex) + mod(rawAz_deg - centerAz_deg(sampleIndex) + 180, 360).' - 180;
        y_deg(sampleIndex, :) = atan2d(points_1(:, 3), hypot(points_1(:, 1), points_1(:, 2))).';
    end
    % Few exact corners, no repeated closure: avoid the roundoff crossing repair that emptied a Sun slice in exampleSunKeepOut.
end

function validation = validateFullHistory(result, fullObstacles)
    % Re-prove the unchanged polynomial, then run the public independent validator.
    validation = struct('Passed', false, 'Message', 'No successful motion to check.');
    if ~result.Success
        return
    end
    interval_s = [result.Inputs.initialState.time_s, result.ArrivalTime_s];
    preparedObstacles = obstacleAvoidance.obstacles.prepareObstacles(fullObstacles, interval_s);
    cells = obstacleAvoidance.obstacles.createTimeCells(preparedObstacles, interval_s(1), interval_s(2));
    coverage = struct('ExactRegionCount', numel(cells.Regions_units), ...
        'ActiveTimeInterval_s', cells.ActiveTimeInterval_s, 'EndRegions_units', {cells.EndRegions_units}, ...
        'BreakTime_s', cells.BreakTime_s);
    solverRequest = struct('InitialState', result.Inputs.initialState, 'Regions_units', {cells.Regions_units}, ...
        'Coverage', coverage, 'Limits', result.Diagnostics.Limits, 'Options', result.Options);
    polynomial = result.Diagnostics.Polynomial;
    curveControls_deg = bmtpEngine.motion.powerToBernstein(polynomial.positionPower_units);
    preparedMotion = struct('ControlPoint_units', curveControls_deg, ...
        'SegmentTime_s', polynomial.SegmentDuration_s, 'FinalTime_s', polynomial.FinalTime_s, ...
        'GivenPower_units', polynomial.positionPower_units);
    [~, reserve_deg] = bmtpEngine.validation.createCoordinateTolerances(result.Diagnostics.Route_units, ...
        solverRequest.Limits.xInterval_units, solverRequest.Limits.yInterval_units, cells.Regions_units, cells.EndRegions_units);
    target_deg = (1 + 2 ^ 20 * eps) * result.Options.CollisionClearanceTolerance_units + reserve_deg;
    proof = bmtpEngine.validation.checkFinalMotion(solverRequest, preparedMotion, reserve_deg, target_deg);
    proofResult = result;
    proofResult.Inputs.obstacles = fullObstacles;
    proofResult.Diagnostics.PreparedObstacles = preparedObstacles;
    proofResult.Diagnostics.SeparationProof = proof;
    validation = obstacleAvoidance.validateTrajectory(proofResult);
end

function [time_s, x_deg, y_deg, sunAnchors] = sourceHistory()
    % Exact owner samples: each row is [50 azimuths, 50 elevations].
    time_s = [2765:3:2873 2875].';
    coordinates_deg = [ ...
        -10.170202 -1.9991 5.890911 13.257609 19.953743 25.925914 31.190634 35.82717 40.79264 42.858581 44.655314 46.234531 47.636097 50.15493 53.008346 56.244042 59.911823 59.2293 58.435005 57.486188 56.315906 48.154203 40.847985 34.481551 29.023733 23.588651 17.344251 10.258845 2.3888834 -6.0854939 -14.858285 -21.411521 -26.797167 -31.231747 -34.908184 -32.143092 -29.841523 -27.897206 -26.232889 -24.791518 -23.530265 -20.474297 -17.11664 -18.225695 -19.503875 -20.996007 -22.763981 -19.231752 -15.143555 -10.408019 51.56021 51.368068 50.709636 49.658089 48.314724 46.786542 45.169538 46.073837 46.837326 44.947364 43.223476 41.65723 40.23819 41.548147 42.928944 44.363195 45.824814 48.144741 50.756016 53.688265 56.968102 55.807699 54.238059 52.398263 50.416596 52.242991 53.947728 55.423909 56.553727 57.227694 57.370342 54.948957 52.466827 50.034491 47.721194 45.253983 43.043894 41.078725 39.34015 37.807543 36.460164 37.38028 38.328979 39.90821 41.702424 43.736705 46.036175 47.832618 49.680808 51.532069; ...
        -9.4137529 -1.0989041 6.8814042 14.283868 20.970383 26.901039 32.105258 36.794179 41.798092 43.768122 45.480696 46.985821 48.321876 50.890147 53.793035 57.076051 60.785717 60.218433 59.565394 58.793739 57.852054 49.671331 42.282288 35.796389 30.205263 24.847811 18.654845 11.575174 3.6445412 -4.9701863 -13.958477 -20.788465 -26.389886 -30.988128 -34.787522 -32.002343 -29.694023 -27.750912 -26.092518 -24.659801 -23.40871 -20.296594 -16.875791 -17.961507 -19.215432 -20.683093 -22.427796 -18.781458 -14.55579 -9.65747 52.066255 51.802693 51.06129 49.925111 48.503498 46.908281 45.237211 46.103356 46.821714 44.916362 43.182295 41.609687 40.187062 41.474935 42.827863 44.227539 45.647113 47.950636 50.545518 53.462479 56.729994 55.683933 54.213286 52.451437 50.526033 52.423349 54.214657 55.79049 57.026477 57.802453 58.030234 55.608033 53.099946 50.626892 48.265643 45.726074 43.452069 41.431132 39.644226 38.069866 36.686446 37.621943 38.583693 40.203012 42.043849 44.132333 46.494656 48.316804 50.182732 52.039385; ...
        -8.6183901 -0.16073286 7.9045862 15.335135 22.004018 27.886157 33.024403 37.762104 42.799952 44.672826 46.300551 47.731236 49.001649 51.61737 54.567162 57.894252 61.641811 61.186798 60.671156 60.071526 59.351432 51.168134 43.710462 37.115183 31.39681 26.126012 19.996135 12.935483 4.9564118 -3.7915901 -12.996991 -20.120284 -25.95111 -30.723164 -34.652735 -31.84842 -29.534641 -27.594076 -25.942865 -24.519943 -23.280027 -20.110173 -16.624499 -17.685106 -18.912632 -20.353215 -22.071481 -18.305639 -13.936164 -8.868143 52.574621 52.234668 51.406254 50.182877 48.681944 47.01979 45.295488 46.122304 46.794784 44.876022 43.133418 41.555802 40.130704 41.395859 42.720346 44.085019 45.462392 47.747434 50.323145 53.221056 56.471076 55.538099 54.167632 52.486442 50.620616 52.587721 54.465596 56.14284 57.48897 58.37302 58.693113 56.275848 53.744961 51.232507 48.823446 46.20948 43.869785 41.791573 39.955075 38.337916 36.917589 37.868373 38.842868 40.503054 42.391455 44.535272 46.961793 48.808576 50.690269 52.54918; ...
        -7.7822609 0.81650838 8.9606734 16.410941 23.053808 28.880325 33.947184 38.729902 43.797089 45.571821 47.114195 48.470239 49.674987 52.336129 55.330237 58.698166 62.479699 62.133826 61.751465 61.318302 60.812047 52.641844 45.129774 38.435694 32.596748 27.42156 21.366719 14.339269 6.3255835 -2.5467087 -11.96946 -19.403496 -25.478381 -30.435222 -34.502771 -31.680459 -29.362682 -27.426141 -25.783487 -24.371588 -23.143923 -19.914692 -16.362375 -17.395995 -18.59483 -20.005522 -21.693896 -17.80287 -13.282977 -8.0381743 53.084495 52.663022 51.743582 50.430607 48.849509 47.120729 45.344202 46.130619 46.756613 44.826451 43.076962 41.495689 40.069224 41.311084 42.606629 43.935965 45.271091 47.535696 50.089619 52.964934 56.192583 55.370907 54.101308 52.503122 50.699986 52.735442 54.699498 56.479521 57.939504 58.937696 59.357617 56.951692 54.401618 51.85136 49.394785 46.704354 44.297167 42.160149 40.272778 38.611757 37.153646 38.119592 39.106483 40.808307 42.745201 44.945461 47.4375 49.307678 51.202921 53.06065; ...
        -6.9034799 1.8338415 10.049751 17.510688 24.11882 29.882543 34.872689 39.696521 44.788392 46.464253 47.92097 49.202312 50.34148 53.045981 56.081803 59.487361 63.299039 63.059039 62.805629 62.533025 62.232252 54.089917 46.537563 39.755657 33.803382 28.732607 22.764941 15.785702 7.752856 -1.2326179 -10.871278 -18.634338 -24.969022 -30.122524 -34.336489 -31.497539 -29.177414 -27.246527 -25.61392 -24.214362 -23.000099 -19.709801 -16.089021 -17.093662 -18.261356 -19.639124 -21.293831 -17.271649 -12.594457 -7.165665 53.59499 53.086726 52.072308 50.667533 49.005669 47.210799 45.383218 46.12828 46.707326 44.76779 43.013068 41.429484 40.002749 41.220792 42.486967 43.78072 45.073656 47.315995 49.845681 52.695083 55.895799 55.183162 54.014625 52.50141 50.763843 52.865909 54.915362 56.799099 58.376309 59.494655 60.022228 57.634746 55.069595 52.483446 49.979829 47.210843 44.734338 42.536963 40.597421 38.89146 37.394674 38.375628 39.374517 41.118741 43.105038 45.362829 47.92167 49.813813 51.720133 53.572919; ...
        -5.9801258 2.8922101 11.171768 18.633648 25.198027 30.891763 35.79999 40.660909 45.772771 47.349293 48.720232 49.926953 51.00073 53.746504 56.821434 60.261447 64.099539 63.962034 63.833076 63.71485 63.610733 55.510043 47.931268 41.0728 35.014955 30.05717 24.188903 17.273612 9.2387005 0.15349431 -9.6975892 -17.808723 -24.420097 -29.783116 -34.152639 -31.298656 -28.978045 -27.05461 -25.433672 -24.047875 -22.848238 -19.495129 -15.80402 -16.77757 -17.911506 -19.253076 -20.869998 -16.710382 -11.868751 -6.2486765 54.105125 53.504686 52.391429 50.892884 49.149915 47.289715 45.412421 46.115291 46.647067 44.700194 42.941889 41.357329 39.931405 41.125165 42.361613 43.619626 44.870527 47.088898 49.592071 52.412477 55.582027 54.975737 53.907978 52.481309 50.811938 52.978569 55.112223 57.100135 58.797543 60.041918 60.685241 58.324058 55.74849 53.128709 50.578723 47.729072 45.181404 42.922103 40.929075 39.177083 37.640719 38.63649 39.64693 41.434301 43.470894 45.787272 48.414155 50.326626 52.241274 54.085015; ...
        -5.0102684 3.9924468 12.326514 19.778953 26.290313 31.906891 36.72814 41.622019 46.749168 48.226136 49.511365 50.643679 51.652357 54.437298 57.548739 61.020082 64.880966 64.842492 64.833357 64.86312 64.94649 56.900163 49.308445 42.38486 36.229661 31.393135 25.636459 18.801455 10.783187 1.6142394 -8.4433387 -16.922242 -23.828401 -29.414861 -33.949857 -31.082733 -28.763737 -26.849735 -25.242227 -23.871716 -22.688013 -19.270294 -15.506945 -16.447164 -17.544547 -18.846391 -20.421032 -16.117396 -11.103934 -5.2852576 54.613839 53.915752 52.699932 51.105904 49.281769 47.357232 45.431728 46.09169 46.576021 44.623845 42.863594 41.279377 39.85533 41.024398 42.230832 43.453031 44.662148 46.854975 49.329532 52.118101 55.252592 54.749576 53.781852 52.442905 50.844084 53.072941 55.289181 57.381217 59.201315 60.577377 61.344767 59.01854 56.437816 53.787049 51.191591 48.259151 45.638462 43.31565 41.267808 39.468683 37.89183 38.902189 39.923681 41.754927 43.842679 46.218662 48.914771 50.845712 52.765642 54.595887; ...
        -3.9919823 5.1352538 13.513609 20.945596 27.394474 32.926791 37.656182 42.578811 47.716557 49.094011 50.293776 51.352031 52.295999 55.117991 58.263358 61.762963 65.643137 65.700168 65.806135 65.97736 66.238823 58.258469 50.666792 43.689608 37.44566 32.73828 27.105224 20.3673 12.385929 3.1519488 -7.1033045 -15.970138 -23.190425 -29.015415 -33.726649 -30.848606 -28.533589 -26.631207 -25.039043 -23.685458 -22.519084 -19.034901 -15.197355 -16.10187 -17.159716 -18.41803 -19.945484 -15.490925 -10.298015 -4.2734573 55.119978 54.318716 52.996786 51.305857 49.400789 47.413139 45.441084 46.057548 46.494404 44.538947 42.77837 41.195795 39.77467 40.918692 42.094896 43.281286 44.448954 46.614792 49.058804 51.812936 54.90882 54.505675 53.63681 52.386362 50.860156 53.148616 55.445404 57.640969 59.585701 61.098786 61.998713 59.71695 57.136988 54.458311 51.81853 48.801173 46.105593 43.717677 41.613683 39.766311 38.148049 39.172727 40.204715 42.080542 44.220286 46.656844 49.423297 51.370608 53.292459 55.104389; ...
        -2.9233643 6.3211817 14.732491 22.132426 28.509225 33.95029 38.583151 43.530261 48.673952 49.952174 51.066896 52.051568 52.931307 55.788231 58.964964 62.48983 66.385919 66.534887 66.751176 67.057254 67.487303 59.583404 52.004162 44.984862 38.661091 34.090284 28.592593 21.968822 14.046028 4.7685823 -5.6721528 -14.947287 -22.502323 -28.582195 -33.481375 -30.595011 -28.286638 -26.398285 -24.823546 -23.488651 -22.341092 -18.788536 -14.874793 -15.741089 -16.75621 -17.966897 -19.441813 -14.829111 -9.4489336 -3.2113416 55.622289 54.712315 53.28095 51.492024 49.506559 47.457251 45.440457 46.012963 46.402454 44.445719 42.686411 41.106754 39.689572 40.80825 41.954072 43.104737 44.231373 46.368903 48.780616 51.497952 54.552023 54.245068 53.473483 52.311915 50.860086 53.205259 55.580136 57.878063 59.948751 61.60377 62.64477 60.417875 57.845311 55.142275 52.459603 49.355201 46.58286 44.128239 41.966748 40.07001 38.409412 39.448094 40.489961 42.411049 44.603579 47.101627 49.939459 51.900781 53.820857 55.609278; ...
        -1.802574 7.5505906 15.982392 23.33815 29.633201 34.976189 39.50808 44.475365 49.620412 50.799921 51.830191 52.741875 53.557957 56.447695 59.653267 63.200465 67.109226 67.346543 67.668352 68.102648 68.691757 60.873666 53.318578 46.268511 39.874085 35.446753 30.095748 23.603286 15.762005 6.4655979 -4.1445449 -13.848217 -21.759898 -28.112373 -33.21224 -30.320588 -28.021857 -26.150189 -24.595135 -23.280827 -22.153668 -18.530774 -14.538791 -15.364206 -16.333204 -17.491848 -18.908394 -14.130013 -8.5545861 -2.0970337 56.119434 55.095244 53.551392 51.663725 49.598714 47.489431 45.429854 45.958071 46.300444 44.344406 42.587932 41.012438 39.600198 40.693287 41.808641 42.923735 44.009831 46.117859 48.495689 51.174109 54.183501 53.968828 53.292576 52.219881 50.843879 53.242621 55.692716 58.09125 60.288526 62.089845 63.280414 61.119724 58.561974 55.838656 53.114841 49.921278 47.070311 44.547385 42.327049 40.379821 38.675955 39.72828 40.779344 42.746342 44.992405 47.552792 50.462941 52.435633 54.349888 56.109223; ...
        -0.62784781 8.8236358 17.262344 24.561336 30.76497 36.003264 40.430006 45.413143 50.555039 51.636582 52.583148 53.422555 54.175636 57.096082 60.328002 63.894687 67.813013 68.135089 68.557619 69.113521 69.852229 62.128193 54.608239 47.538527 41.082781 36.805236 31.611691 25.26757 17.531766 8.243852 -2.5152262 -12.667078 -20.958546 -27.602819 -32.917258 -30.023854 -27.73814 -25.886086 -24.353175 -23.061493 -21.956426 -18.26117 -14.188864 -14.970579 -15.889829 -16.99168 -18.343495 -13.391591 -7.6128201 -0.92872764 56.609971 55.46615 53.80708 51.820306 49.676922 47.509572 45.4093 45.893028 46.188666 44.235262 42.483153 40.913036 39.506705 40.574015 41.658877 42.738624 43.784738 45.862193 48.204724 50.842341 53.804521 53.678043 53.09484 52.110636 50.811593 53.260531 55.782578 58.279364 60.603106 62.554427 63.902885 61.820698 59.286025 56.547088 53.784228 50.499412 47.567967 44.975142 42.694613 40.695769 38.947701 40.013257 41.072767 43.086286 45.386578 48.010074 50.993367 52.974488 54.878505 56.60279; ...
        0.6024492 10.140227 18.571155 25.800413 31.903036 37.030277 41.347973 46.342644 51.47699 52.461528 53.32529 54.09324 54.784056 57.733117 60.98894 64.572352 68.497279 68.900538 69.419021 70.089981 70.968969 63.346156 55.871534 48.792981 42.285338 38.163251 33.137268 26.958164 19.352551 10.10345 -0.7791962 -11.397688 -20.093246 -27.050092 -32.59425 -29.703208 -27.434306 -25.605095 -24.096998 -22.830136 -21.748963 -17.979265 -13.824516 -14.559549 -15.425187 -16.465133 -17.745294 -12.611724 -6.621464 0.2952606 57.092369 55.823652 54.047003 51.961164 49.740901 47.517607 45.378856 45.818023 46.067439 44.118561 42.372305 40.808744 39.409261 40.450652 41.505059 42.549745 43.556499 45.602429 47.908409 50.503562 53.416316 53.373812 52.88108 51.984629 50.763355 53.258909 55.849265 58.441348 60.890631 62.994857 64.509199 62.518787 60.016367 57.267115 54.467706 51.08958 48.075828 45.411523 43.069461 41.017878 39.224673 40.302991 41.370125 43.430734 45.785888 48.473174 51.530305 53.51659 55.405566 57.088453; ...
        1.8897862 11.500002 19.907406 27.053681 33.04585 38.05598 42.261036 47.262953 52.385472 53.274168 54.056169 54.753583 55.382945 58.35855 61.635883 65.233354 69.162059 69.642956 70.252677 71.03225 72.042399 64.526947 57.107038 50.030058 43.47995 39.518305 34.669196 28.671205 21.220912 12.043617 1.0681107 -10.033558 -19.158516 -26.450394 -32.240807 -29.356908 -27.10909 -25.306281 -23.825901 -22.586216 -21.53086 -17.684584 -13.445235 -14.130435 -14.938349 -15.910893 -17.111862 -11.788209 -5.5783432 1.5764562 57.565003 56.166344 54.27018 52.085741 49.79042 47.513512 45.338609 45.733273 45.937101 43.99459 42.255631 40.699767 39.308037 40.32342 41.347464 42.357438 43.325506 45.339075 47.607408 50.158656 53.020079 53.057241 52.65214 51.842366 50.699351 53.237763 55.892434 58.576268 61.14932 63.408434 65.096144 63.211742 60.751737 57.998184 55.165162 51.691718 48.593864 45.856522 43.451599 41.346159 39.506885 40.597439 41.671297 43.779516 46.190093 48.941745 52.073257 54.061101 55.92983 57.564591; ...
        3.2354245 12.902304 21.269456 28.319319 34.191821 39.079125 43.168267 48.173187 53.279745 54.073952 54.775366 55.403262 55.972049 58.972153 62.268658 65.877618 69.807422 70.362452 71.058772 71.940647 73.07309 65.670159 58.313514 51.248063 44.664863 40.86792 36.204109 30.402515 23.132707 14.062575 3.0306061 -8.5679373 -18.148378 -25.799522 -31.85426 -28.983058 -26.76113 -24.988649 -23.539141 -22.32917 -21.301681 -17.376633 -13.050498 -13.682535 -14.428344 -15.327583 -16.441162 -10.918761 -4.4812986 2.9161836 58.02616 56.492801 54.475662 52.193532 49.825293 47.497294 45.288668 45.639014 45.798006 43.863642 42.133375 40.586306 39.203202 40.192537 41.186365 42.162027 43.092137 45.072619 47.302363 49.808472 52.616952 52.729419 52.408893 51.684404 50.619825 53.197182 55.911857 58.683326 61.377501 63.792441 65.66029 63.897064 61.490684 58.739629 55.876419 52.305715 49.122015 46.310108 43.841013 41.68061 39.79434 40.89654 41.976139 44.132433 46.598916 49.415392 52.621653 54.607083 56.449949 58.029493; ...
        4.6403479 14.346141 22.655428 29.595392 35.339324 40.098469 44.06876 49.072511 54.159131 54.860375 55.482499 56.041979 56.551136 59.573726 62.887126 66.505103 70.433471 71.059186 71.837558 72.815581 74.061745 66.77558 59.489914 52.445433 45.838378 42.209651 37.738582 32.147636 25.083105 16.157402 5.1112513 -6.9939356 -17.056357 -25.092845 -31.431665 -28.579606 -26.388974 -24.651147 -23.235941 -22.05841 -21.060974 -17.054906 -12.63977 -13.215128 -13.894178 -14.713776 -15.731061 -10.001042 -3.3282337 4.3154971 58.47405 56.801607 54.662554 52.2841 49.845395 47.469012 45.229181 45.535514 45.65053 43.726031 42.005798 40.468579 39.094936 40.05823 41.022039 41.963843 42.85676 44.803539 46.993894 49.453832 52.208034 52.391433 52.152239 51.511355 50.525082 53.137352 55.907438 58.761885 61.573651 64.144199 66.198018 64.571992 62.23156 59.490659 56.601235 52.931418 49.660188 46.772229 44.23768 42.021223 40.08704 41.200227 42.284499 44.489271 47.012049 49.893672 53.174852 55.153512 56.964476 58.481369; ...
        6.1052247 15.830179 24.063226 30.879872 36.486714 41.112785 44.961633 49.960126 55.023002 55.632971 56.177213 56.669462 57.11999 60.163087 63.491172 67.115796 71.040336 71.733351 72.589334 73.657535 75.009167 67.843165 60.635367 53.620744 46.998872 43.541116 39.269185 33.9019 27.066629 18.323957 7.3117786 -5.304617 -15.875434 -24.325231 -30.969739 -28.144307 -25.991053 -24.292653 -22.915478 -21.773318 -20.808264 -16.718872 -12.212501 -12.727474 -13.334817 -14.067981 -14.979309 -9.0326525 -2.1171314 5.7751423 58.906806 57.09135 54.830012 52.357062 49.850647 47.428751 45.160307 45.423052 45.495054 43.582069 41.873153 40.346795 38.983414 39.920717 40.854752 41.763195 42.619728 44.532287 46.682593 49.095512 51.794363 52.044338 51.883088 51.323869 50.415474 53.058531 55.879193 58.811462 61.736409 64.4611 66.705538 65.233483 62.972486 60.250343 57.339288 53.568609 50.208246 47.242803 44.64155 42.367969 40.384968 41.508414 42.5962 44.849781 47.429141 50.376084 53.732127 55.699253 57.471852 58.918348; ...
        7.6303293 17.352708 25.490536 32.170645 37.632334 42.120862 45.846033 50.835285 55.870793 56.391317 56.859187 57.28546 57.678416 60.740083 64.080708 67.709712 71.628172 72.385179 73.314449 74.467055 75.916251 68.873025 61.74918 54.77271 48.144801 44.860007 40.792508 35.660482 29.077208 20.556794 9.6323282 -3.4932048 -14.598066 -23.491014 -30.464846 -27.674723 -25.565692 -23.911981 -22.576887 -21.47325 -20.543062 -16.367991 -11.768134 -12.218815 -12.749204 -13.38866 -14.183561 -8.0111653 -0.84611372 7.2954772 59.322505 57.360652 54.977264 52.412113 49.841029 47.376641 45.08224 45.301935 45.331977 43.432079 41.735708 40.221173 38.868813 39.780221 40.684771 41.560394 42.38138 44.259301 46.369026 48.734258 51.376925 51.689167 51.602363 51.122632 50.291407 52.961065 55.82727 58.831753 61.864622 64.740668 67.178949 65.878207 63.71134 61.01759 58.090168 54.217013 50.766013 47.721721 45.052558 42.72081 40.688104 41.821001 42.911047 45.213691 47.849805 50.862068 54.292665 56.24307 57.970416 59.338503; ...
        9.2154836 18.911633 26.93484 33.465528 38.774529 43.121521 46.721138 51.697286 56.701995 57.135034 57.528131 57.88975 58.226237 61.304578 64.655673 68.286894 72.197163 73.014931 74.013295 75.244745 76.78396 69.865412 62.830825 55.900192 49.274712 46.164119 42.30521 37.418471 31.108257 22.849145 12.071106 -1.5533093 -13.216198 -22.583935 -29.912934 -27.168195 -25.111089 -23.50787 -22.219261 -21.157534 -20.264858 -16.001701 -11.306101 -11.688377 -12.13625 -12.674222 -13.341372 -6.9341455 0.48651365 8.8764132 59.719177 57.608176 55.103617 52.449019 49.816578 47.312851 44.995194 45.172484 45.161705 43.276391 41.59373 40.091931 38.751316 39.636961 40.512357 41.355738 42.142042 43.984997 46.053735 48.370776 50.956652 51.326924 51.310986 50.908363 50.153335 52.845377 55.751937 58.822633 61.95736 64.980615 67.61429 66.502552 64.44573 61.791136 58.853364 54.876283 51.333262 48.208839 45.470615 43.079693 40.996416 42.137875 43.228826 45.580699 48.273611 51.351005 54.85556 56.783615 58.458406 59.739854; ...
        10.86 20.504472 28.393438 34.762295 39.911659 44.113613 47.586163 52.545476 57.516153 57.863781 58.183784 58.482128 58.763291 61.856459 65.216025 68.847406 72.747506 73.622893 74.686292 75.991245 77.613307 70.820693 63.879928 57.00219 50.38725 47.451357 43.804054 39.170945 33.152794 25.192958 14.624056 0.52078291 -11.72129 -21.597077 -29.30948 -26.621819 -24.625309 -23.078982 -21.841641 -20.825465 -19.973121 -15.619424 -10.825822 -11.135371 -11.49484 -11.923028 -12.4502 -5.7991722 1.8822108 10.517355 60.094825 57.832645 55.208457 52.467618 49.77738 47.237577 44.8994 45.035031 44.984645 43.115332 41.447482 39.959284 38.631096 39.491153 40.337764 41.149512 41.902022 43.70977 45.737227 48.005729 50.534414 50.958568 51.00987 50.681799 50.001745 52.711953 55.653573 58.784154 62.013933 65.178885 68.007616 67.102615 65.172968 62.569516 59.628254 55.545992 51.909713 48.703975 45.895603 43.444541 41.309854 42.458899 43.549297 45.950469 48.700079 51.842203 55.419801 57.319422 58.933957 60.12039; ...
        12.5626 22.128342 29.863464 36.058685 41.042108 45.096029 48.44036 53.379255 58.312873 58.577261 58.825919 59.062418 59.28944 62.395636 65.761749 69.391337 73.279425 74.209381 75.333899 76.70724 78.405349 71.739348 64.896261 58.077848 51.481162 48.71976 45.285936 40.913036 35.203543 27.578965 17.284545 2.7337066 -10.104436 -20.522836 -28.649448 -26.032439 -24.106282 -22.623901 -21.443026 -20.476312 -19.667303 -15.220569 -10.326716 -10.559 -10.823846 -11.133407 -11.507434 -4.603892 3.342121 12.217121 60.447447 58.032866 55.291277 52.467833 49.723582 47.15106 44.795117 44.889931 44.801219 42.949241 41.297239 39.823456 38.508339 39.343014 40.161244 40.942 41.661618 43.434 45.41999 47.639745 50.11103 50.585031 50.699925 50.4437 49.837172 52.561358 55.532681 58.716555 62.033923 65.333727 68.355097 67.674242 65.89005 63.351044 60.414088 56.225628 52.495027 49.20691 46.327382 43.815266 41.628364 42.783926 43.872202 46.322635 49.128689 52.334906 55.984278 57.848917 59.395122 60.478092; ...
        14.321382 23.77999 31.341917 37.352429 42.164295 46.067706 49.283021 54.198073 59.09181 59.275213 59.454335 59.630461 59.804558 62.922034 66.29285 69.918794 73.793154 74.774723 75.956589 77.393435 79.161164 72.621941 65.879729 59.126445 52.5553 49.967503 46.747923 42.640008 37.253085 29.996859 20.04319 5.0886147 -8.3564597 -19.352837 -27.927194 -25.3966 -23.551779 -22.141121 -21.022358 -20.109306 -19.346831 -14.804523 -9.8081915 -9.958452 -10.122114 -10.303649 -10.510387 -3.3460414 4.8670231 13.973904 60.775055 58.207735 55.351661 52.44966 49.655377 47.053563 44.682614 44.73754 44.611838 42.778446 41.143266 39.684659 38.383216 39.192754 39.983037 40.733464 41.421109 43.158041 45.102478 47.273411 49.687258 50.207194 50.382035 50.194833 49.660173 52.394203 55.389856 58.620244 62.017174 65.443724 68.653112 68.21304 66.593631 64.133786 61.209969 56.914579 53.088798 49.717378 46.765778 44.191753 41.951868 43.112782 44.197254 46.696795 49.558863 52.82828 56.547761 58.370403 59.839868 60.810952; ...
        16.133755 25.455795 32.825687 38.641267 43.276687 47.027629 50.11348 55.00143 59.852676 59.957418 60.068862 60.186123 60.308539 63.435602 66.80935 70.429903 74.288943 75.319269 76.554863 78.050558 79.881851 73.469113 66.83036 60.147397 53.608627 51.192917 48.187271 44.347315 39.293992 32.435485 22.887758 7.5866482 -6.4681469 -18.077928 -27.136415 -24.71054 -22.959416 -21.629053 -20.578531 -19.723652 -19.011114 -14.370662 -9.2696576 -9.3329147 -9.3884877 -9.4320285 -9.456337 -2.023516 6.4572378 15.785205 61.075709 58.356265 55.389307 52.413175 49.573013 46.945384 44.562182 44.578228 44.416924 42.603282 40.98583 39.543113 38.255906 39.040579 39.803379 40.524164 41.180763 42.88223 44.78512 46.907275 49.263805 49.825899 50.057069 49.935971 49.471338 52.211158 55.225796 58.495799 61.963813 65.507854 68.898371 68.714447 67.280012 64.915531 62.014842 57.612125 53.690548 50.235064 47.210585 44.573867 42.28028 43.445274 44.524144 47.072512 49.989977 53.321415 57.108909 58.882068 60.266098 61.117003; ...
        17.996412 27.151805 34.311588 39.92297 44.377806 47.974839 50.931116 55.788879 60.595232 60.623694 60.669358 60.729293 60.801291 63.936304 67.311294 70.924808 74.767057 75.843384 77.129232 78.679353 80.568512 74.281572 67.748289 61.140244 54.640216 52.394486 49.601453 46.030664 41.318971 34.883113 25.803263 10.226412 -4.4305266 -16.688144 -26.270056 -23.970159 -22.326639 -21.086015 -20.110386 -19.31852 -18.659542 -13.918349 -8.7105218 -8.6815735 -8.6218049 -8.5168145 -8.3425438 -0.63442526 8.1125557 17.6478 61.347542 58.477598 55.404025 52.358537 49.476785 46.826848 44.434125 44.412373 44.216892 42.424081 40.825199 39.399032 38.126585 38.886696 39.622502 40.314348 40.940831 42.606886 44.468319 46.54185 48.841323 49.441943 49.725874 49.667892 49.271279 52.01294 55.041285 58.34396 61.874237 65.525508 69.088028 69.173798 67.945124 65.693762 62.827463 58.317425 54.29972 50.759602 47.661563 44.96145 42.613493 43.78119 44.852536 47.44931 50.42135 53.813322 57.666256 59.381991 60.671668 61.394349; ...
        19.905317 28.863783 35.796397 41.195355 45.466239 48.908431 51.735351 56.560021 61.319289 61.273891 61.255703 61.259875 61.282739 64.424121 67.798738 71.403665 75.227766 76.347437 77.680218 79.280567 81.222248 75.06007 68.63375 62.104645 55.649251 53.570855 50.988173 47.686067 43.321005 37.327737 28.772266 13.003457 -2.2352473 -15.172707 -25.320208 -23.170983 -21.650711 -20.510225 -19.616704 -18.893043 -18.291477 -13.446927 -8.1301916 -8.003614 -7.8209062 -7.556279 -7.1662796 0.82284602 9.8321669 19.557732 61.58879 58.571014 55.395736 52.28597 49.36703 46.698302 44.298756 44.24035 44.01215 42.241166 40.66163 39.252623 37.99542 38.731297 39.440622 40.104248 40.701548 42.332303 44.152447 46.177606 48.42041 49.056076 49.38926 49.391361 49.06062 51.800296 54.837178 58.165602 61.749101 65.496515 69.219788 69.586418 68.584524 66.465616 63.646381 59.029504 54.915667 51.290566 48.11843 45.354314 42.951381 44.120288 45.182064 47.826674 50.852244 54.302926 58.218208 59.868139 61.054401 61.641192; ...
        21.855696 30.58724 37.276881 42.456306 46.540645 49.827562 52.525654 57.31451 62.024704 61.907898 61.82781 61.777799 61.752825 64.899053 68.271762 71.866649 75.671356 76.831817 78.208357 79.854959 81.844155 75.805406 69.487067 63.040375 56.635023 54.720834 52.345372 49.309874 45.293468 39.757382 31.775316 15.909778 0.12487705 -13.520109 -24.278042 -22.308157 -20.928717 -19.899811 -19.096219 -18.44633 -17.906268 -12.955735 -7.5280847 -7.298237 -6.9846548 -6.5487333 -5.9248894 2.3495392 11.614565 21.510287 61.79783 58.635958 55.364486 52.195786 49.244133 46.560122 44.156407 44.062548 43.803112 42.054865 40.495386 39.104101 37.862586 38.574583 39.257957 39.894092 40.463142 42.058763 43.837856 45.814988 48.001621 48.669009 49.048018 49.107146 48.840007 51.574014 54.614403 57.961744 61.589319 65.421152 69.292012 69.94776 69.193417 67.227858 64.469908 59.747236 55.537649 51.82747 48.580864 45.752247 43.293804 44.462307 45.512338 48.204052 51.281865 54.789071 58.763047 60.338387 61.412125 61.855871; ...
        23.842085 32.317514 38.749842 43.703791 47.599763 50.731451 53.301538 58.052046 62.711377 62.525631 62.385611 62.283007 62.211501 65.361111 68.730452 72.313939 76.098111 77.29691 78.714182 80.403278 82.43531 76.518404 70.308634 63.947311 57.596929 55.84339 53.671243 50.89881 47.230238 42.160432 34.791633 18.933512 2.6552779 -11.718193 -23.133669 -21.376396 -20.157539 -19.252793 -18.547602 -17.977446 -17.503238 -12.444096 -6.903623 -6.5646521 -6.1119338 -5.4925313 -4.6158198 3.946462 13.457504 23.500046 61.9732 58.672032 55.310427 52.088352 49.10851 46.412694 44.007409 43.879348 43.590173 41.86549 40.326718 38.953665 37.728245 38.416737 39.074709 39.68409 40.225818 41.786524 43.524867 45.454394 47.585456 48.281398 48.702896 48.81599 48.610087 51.334898 54.373934 57.733503 61.396022 65.300124 69.303779 70.253528 69.76668 67.976835 65.296087 60.469335 56.16482 52.369756 49.048497 46.155001 43.640593 44.806953 45.842936 48.580846 51.709357 55.270511 59.298922 60.79052 61.742684 62.036886; ...
        25.858364 34.049816 40.212144 44.935875 48.642413 51.61938 54.062564 58.772375 63.379251 63.127043 62.929062 62.775464 62.658738 65.810323 69.174912 72.745732 76.508327 77.743111 79.198234 80.926277 82.996776 77.199913 71.098914 64.825428 58.534472 56.937653 54.964225 52.449991 49.125779 44.525914 37.799859 22.058778 5.3588451 -9.7543984 -21.876077 -20.369983 -19.333867 -18.567096 -17.969473 -17.48543 -17.081691 -11.911328 -6.256245 -5.8020954 -5.2016712 -4.386113 -3.2367008 5.6138761 15.357924 25.520913 62.113643 58.679015 55.23383 51.964112 48.960619 46.256428 43.852108 43.691136 43.373726 41.673356 40.155875 38.801517 37.592561 38.257943 38.891076 39.474448 39.989773 41.515828 43.213782 45.096198 47.172374 47.893863 48.354615 48.518627 48.371513 51.083772 54.116794 57.482103 61.170548 65.134552 69.254937 70.49985 70.298925 68.708453 66.122656 61.19433 56.796221 52.916793 49.520912 46.562299 43.991562 45.15391 46.173406 48.956419 52.133809 55.745916 59.823855 61.222257 62.043985 62.182935; ...
        27.897845 35.779306 41.660753 46.150733 49.667509 52.490695 54.808337 59.475291 64.028307 63.712111 63.458144 63.25515 63.094518 66.246732 69.605261 73.162231 76.902304 78.170821 79.661054 81.424698 83.529594 77.850797 71.858427 65.674788 59.447252 58.002902 56.223003 53.960936 50.975198 46.843749 40.778884 25.265828 8.2351011 -7.6160775 -20.493032 -19.282748 -18.454189 -17.840547 -17.360397 -16.969289 -16.640911 -11.356742 -5.5854068 -5.0098339 -4.2528515 -3.2280303 -1.7854129 7.3514099 17.31192 27.566202 62.218127 58.656865 55.135078 51.823565 48.800946 46.091747 43.690852 43.498293 43.154158 41.478766 39.983102 38.647856 37.455695 38.09838 38.707248 39.265356 39.755192 41.246902 42.904877 44.74074 46.762791 47.506978 48.00386 48.215774 48.124942 50.821473 53.844038 57.208842 60.91441 64.925936 69.14611 70.683429 70.78459 69.418144 66.947006 61.920552 57.430768 53.467874 49.99764 46.973828 44.3465 45.502831 46.503269 49.330094 52.554246 56.21387 60.335745 61.631269 62.314026 62.292943; ...
        29.953384 37.501166 43.092762 47.346663 50.674051 53.344807 55.538506 60.160626 64.658562 64.280839 63.972854 63.72206 63.518834 66.670389 70.02162 73.563645 77.28034 78.580435 80.103173 81.899271 84.034774 78.471921 72.587734 66.495532 60.33496 59.038566 57.446502 55.429563 52.77429 49.104955 43.708664 28.53153 11.279356 -5.2909785 -18.971001 -18.108053 -17.514792 -17.070867 -16.718881 -16.427995 -16.180163 -10.779645 -4.8905846 -4.1871698 -3.2645274 -2.0169766 -0.26016261 9.1579737 19.314733 29.628744 62.285863 58.605711 55.014648 51.667263 48.629998 45.919081 43.523989 43.301194 42.931836 41.282012 39.808631 38.492867 37.317798 37.938214 38.523402 39.056994 39.52224 40.979948 42.598403 44.388326 46.357077 47.121272 47.651273 47.908117 47.87102 50.548833 53.556735 56.915075 60.629261 64.6761 68.978656 70.8017 71.218064 70.100845 67.766137 62.646112 58.067243 54.0222 50.478154 47.389235 44.705167 45.853334 46.83201 49.701144 52.969634 56.672873 60.832372 62.015199 62.550929 62.366079; ...
        32.017483 39.210659 44.50542 48.522092 51.661136 54.181193 56.252768 60.82826 65.270068 64.833261 64.473212 64.176208 63.931695 67.081362 70.42413 73.950194 77.642743 78.972357 80.52513 82.350722 84.51331 79.064162 73.287443 67.287876 61.197382 60.044215 58.633875 56.854185 54.519543 51.301775 46.570898 31.830111 14.481873 -2.7679754 -17.295168 -16.838826 -16.511781 -16.255695 -16.043391 -15.860496 -15.698695 -10.179349 -4.1712885 -3.3334623 -2.2358528 -0.75184462 1.3403989 11.031646 21.360745 31.700996 62.316343 58.52587 54.873128 51.495815 48.448317 45.738877 43.351879 43.100216 42.707129 41.083389 39.632699 38.336743 37.179027 37.777616 38.339716 38.849536 39.291081 40.715164 42.294596 44.039241 45.955574 46.737239 47.297472 47.596328 47.610395 50.266696 53.255978 56.602209 60.316878 64.387163 68.754629 70.852964 71.593856 70.751004 68.576606 63.368882 58.704287 54.578888 50.961871 47.808131 45.067301 46.205013 47.159089 50.068807 53.378884 57.121347 61.311412 62.371704 62.752986 62.401789; ...
        34.082458 40.903208 45.896158 49.675584 52.627954 54.999389 56.950857 61.478106 65.862905 65.36943 64.959255 64.617617 64.333117 67.479725 70.812935 74.3221 77.989814 79.34698 80.927447 82.779753 84.966158 79.628386 73.958184 68.052095 62.034381 61.019546 59.784495 58.23349 56.208136 53.427762 49.349602 35.134268 17.827331 -0.037929451 -15.449449 -15.467556 -15.441074 -15.392577 -15.332341 -15.265708 -15.195737 -9.5551637 -3.4270579 -2.4481243 -1.1660869 0.56825045 3.0171003 12.969628 23.443545 33.775202 62.309328 58.417815 54.711183 51.309863 48.25645 45.551579 43.174871 42.895723 42.480382 40.883169 39.455524 38.179659 37.039524 37.61674 38.156349 38.64314 39.061855 40.452722 41.993665 43.693734 45.558578 46.355327 46.943027 47.281047 47.343696 49.975886 52.942852 56.271668 59.979112 64.061461 68.476665 70.836464 71.906791 71.362601 69.374474 64.086475 59.340384 55.136951 51.44814 48.230079 45.432606 46.557422 47.48393 50.432276 53.780847 57.55764 61.770446 62.698475 62.918685 62.399793; ...
        36.140569 42.574447 47.262602 50.805846 53.57379 55.798997 57.632553 62.110116 66.437183 65.889421 65.431036 65.046326 64.72313 67.865564 71.188188 74.679592 78.321858 79.704701 81.310646 83.187057 85.394246 80.165453 74.600617 68.788521 62.845902 61.964382 60.897935 59.566525 57.837914 55.477789 52.031448 38.416352 21.29454 2.9051204 -13.416691 -13.986381 -14.298444 -14.478989 -14.584106 -14.64253 -14.670504 -8.9064127 -2.657477 -1.530646 -0.05463165 1.9439047 4.770114 14.968154 25.555986 35.843529 62.264871 58.282189 54.529567 51.110093 48.054967 45.357641 42.993319 42.688073 42.251936 40.681624 39.277324 38.021792 36.899434 37.455741 37.973459 38.437956 38.8347 40.192783 41.695802 43.352035 45.166356 45.975949 46.588479 46.962889 47.071545 49.677223 52.618443 55.924894 59.617876 63.701505 68.147898 70.752443 72.152238 71.92921 70.155247 64.796222 59.973855 55.695303 51.936244 48.654602 45.800759 46.910086 47.80593 50.790703 54.174324 57.980032 62.206987 62.993284 63.046755 62.360106; ...
        38.184181 44.220278 48.602593 51.911724 54.498019 56.57968 58.297675 62.724279 66.993041 66.393333 65.888626 65.462389 65.101774 68.238975 71.550051 75.022903 78.63918 80.045914 81.675241 83.573313 85.798477 80.676216 75.215419 69.497539 63.631957 62.878659 61.973958 60.852667 59.407351 57.448011 54.605938 41.649653 24.856698 6.0626241 -11.179008 -12.38717 -13.079547 -13.512349 -13.797032 -13.989837 -14.122203 -8.232432 -1.8621788 -0.58060324 1.0989487 3.3753802 6.59885 17.022469 27.690301 37.898231 62.183311 58.119784 54.329101 50.89722 47.844448 45.157518 42.807576 42.47762 42.022117 40.479015 39.09831 37.863313 36.758896 37.294764 37.791194 38.234128 38.60974 39.935497 41.401184 43.014348 44.779143 45.599485 46.234334 46.642443 46.79455 49.37151 52.283821 55.563325 59.235116 63.309926 67.771831 70.602122 72.326343 72.444107 70.913825 65.495152 60.602847 56.252749 52.425397 49.081174 46.171402 47.262493 48.124455 51.143204 54.558066 58.386752 62.618501 63.254025 63.136189 62.283034; ...
        40.205919 45.836922 49.914197 52.992211 55.400108 57.341153 58.946078 63.32061 67.530633 66.881277 66.332107 65.865866 65.469095 68.60006 71.898688 75.352265 78.942078 80.371003 82.02173 83.939173 86.179712 81.161508 75.803276 70.179568 64.392626 63.762413 63.012497 62.091597 60.915496 59.3358 57.065389 44.809572 28.482215 9.4300862 -8.718333 -10.661657 -11.779961 -12.490028 -12.969435 -13.306489 -13.550025 -7.5325741 -1.0408488 0.40233279 2.2949072 4.8625594 8.5018523 19.126831 29.838247 39.931804 62.065258 57.931521 54.110664 50.671978 47.625473 44.951657 42.617982 42.264699 41.791233 40.275588 38.918678 37.704381 36.61804 37.133947 37.609692 38.031783 38.387085 39.680993 41.109962 42.68085 44.397142 45.226273 45.881054 46.320263 46.513294 49.059526 51.940033 55.18838 58.832772 62.889413 67.352221 70.387627 72.426243 72.900436 71.644451 66.17998 61.225325 56.807978 52.914736 49.509216 46.544144 47.614094 48.438841 51.488854 54.930781 58.775981 63.002434 63.478745 63.186268 62.169156; ...
        42.198785 47.420939 51.195703 54.046439 56.279615 58.083195 59.577657 63.899161 68.050143 67.353387 66.76158 66.256834 65.825152 68.948933 72.234276 75.667918 79.230858 80.680355 82.350613 84.285285 86.538794 81.622153 76.364889 70.835072 65.128048 64.615777 64.013641 63.283268 62.361924 61.139627 59.404768 47.874522 32.136028 12.995618 -6.0173753 -8.8016844 -10.395276 -11.409396 -12.099628 -12.591343 -12.953161 -6.8062224 -0.19324334 1.4183691 3.533271 6.4048707 10.476667 21.274515 31.99125 41.937114 61.911595 57.718454 53.875195 50.435129 47.398637 44.740514 42.424887 42.049648 41.559589 40.071587 38.738627 37.545158 36.476998 36.973429 37.429088 37.831048 38.166842 39.429396 40.822281 42.351705 44.020535 44.856628 45.529083 45.996884 46.228352 48.742037 51.58811 54.801462 58.41278 62.442688 66.892987 70.111899 72.450248 73.291445 72.340688 66.847087 61.839065 57.359563 53.403322 49.938101 46.918559 47.964311 48.7484 51.8267 55.291142 59.145877 63.356258 63.665706 63.19659 62.019331; ...
        44.156296 48.96927 52.445633 55.073678 57.136177 58.805633 60.192337 64.460006 68.551768 67.809808 67.177151 66.635374 66.170008 69.28571 72.556987 75.970098 79.505812 80.974345 82.662367 84.612265 86.876524 82.058951 76.900953 71.464535 65.838413 65.438964 64.977615 64.427878 63.74667 62.858948 61.621441 50.826604 35.781458 16.738924 -3.0608619 -6.7994365 -8.9211528 -10.267833 -11.185919 -11.843247 -12.330795 -6.0527874 0.68081468 2.4675913 4.8138337 8.0012626 12.519786 23.457919 34.140606 43.907538 61.723436 57.481729 53.623663 50.187434 47.164521 44.52453 42.22862 41.832783 41.327465 39.867237 38.558338 37.38579 36.335892 36.813331 37.249503 37.632032 37.949102 39.180811 40.538261 42.02705 43.649471 44.490824 45.178817 45.6728 45.940268 48.419772 51.22904 54.403928 57.977032 61.97244 66.398086 69.778529 72.397941 73.610767 72.995413 67.492519 62.441648 57.905952 53.890136 50.367148 47.294184 48.31252 49.052415 52.155754 55.637791 59.494584 63.677498 63.813406 63.167068 61.834653; ...
        46.072566 50.479239 53.662737 56.073334 57.969515 59.508346 60.790081 65.003246 69.035723 68.250698 67.578942 67.00158 66.503738 69.610514 72.867005 76.259046 79.767237 81.253344 82.957466 84.92072 87.193678 82.472686 77.412171 72.068473 66.523961 66.232263 65.904769 65.525835 65.070174 64.494064 63.71484 53.651879 39.38219 20.630781 0.16264127 -4.6478527 -7.3534524 -9.0627855 -10.226643 -11.061063 -11.682114 -5.2717231 1.5814167 3.5499355 6.1361117 9.6501346 14.626556 25.668648 36.277649 45.837045 61.502116 57.222588 53.357073 49.92967 46.923713 44.304144 42.029512 41.614413 41.095135 39.662758 38.377991 37.226425 36.19484 36.653777 37.071055 37.434841 37.733951 38.935336 40.258013 41.707005 43.28408 44.129113 44.830629 45.348484 45.649572 48.09344 50.863786 53.997099 57.527374 61.481315 65.871455 69.391613 72.270217 73.852762 73.600877 68.111991 63.03046 58.445473 54.374075 50.795616 47.670517 48.658069 49.350145 52.475009 55.969354 59.820258 63.963791 63.92063 63.097938 61.616446; ...
        47.324563 51.463432 54.455385 56.724211 58.512069 59.965825 61.179162 65.355687 69.348657 68.536083 67.839212 67.238916 66.720083 69.820464 73.066723 76.444444 79.934148 81.431196 83.145175 85.116355 87.394054 82.736074 77.739524 72.45718 66.967343 66.744699 66.502637 66.232176 65.91886 65.537838 65.042463 55.459869 41.740922 23.289576 2.4629554 -3.1275438 -6.2544333 -8.2229384 -9.5610055 -10.520112 -11.234644 -4.7354162 2.1965692 4.2897961 7.0404327 10.777485 16.063607 27.153211 37.691321 47.097979 61.336893 57.038017 53.171487 49.752617 46.759743 44.154988 41.895353 41.468146 40.940262 39.526472 38.257811 37.120253 36.100894 36.547768 36.952776 37.304444 37.591993 38.773458 40.073325 41.496257 43.043693 43.890359 44.599829 45.132365 45.454574 47.873964 50.617304 53.721345 57.220758 61.14357 65.504642 69.106105 72.144178 73.968827 73.973089 68.50829 63.413974 58.800455 54.694522 51.080551 47.921533 48.886617 49.544767 52.681893 56.181333 60.023682 64.134138 63.969168 63.03011 61.453089; ...
        ];
    x_deg = coordinates_deg(:, 1:50);
    y_deg = coordinates_deg(:, 51:100);
    sunAnchors = [ ...
        2700.0 -0.001884567 -55.133167072; 2715.0 -0.001938758 -55.693207767; 2730.0 -0.001994496 -56.241618762; 2745.0 -0.002051718 -56.777816268; 2760.0 -0.002110341 -57.301191809; 2775.0 -0.002170260 -57.811112523; ...
        2790.0 -0.002231344 -58.306921777; 2805.0 -0.002293438 -58.787940119; 2820.0 -0.002356352 -59.253466629; 2835.0 -0.002419870 -59.702780717; 2850.0 -0.002483740 -60.135144416; 2865.0 -0.002547675 -60.549805202; ...
        2880.0 -0.002611356 -60.945999405; 2895.0 -0.002674426 -61.322956211; 2910.0 -0.002736499 -61.679902294; 2925.0 -0.002797154 -62.016067068; 2940.0 -0.002855948 -62.330688535; ...
        ];
end
