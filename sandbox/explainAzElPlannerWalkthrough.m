function walkthrough = explainAzElPlannerWalkthrough( ...
        scenarioSource, optionOverrides)
%% Section 0: Header & Readme
% SYNTAX
%   walkthrough = explainAzElPlannerWalkthrough()
%   walkthrough = explainAzElPlannerWalkthrough(scenarioSource)
%   walkthrough = explainAzElPlannerWalkthrough( ...
%       scenarioSource, optionOverrides)
%**************************************************************************
% PURPOSE
%   - Run a compatible Az/El example once and explain its seed, geometry,
%     spline controls, quintic coefficients, selection, and validation in
%     one six-tab figure.
%   - Run the maintained static single-U example automatically when no
%     scenario source is supplied.
%**************************************************************************
% INPUTS
%   - scenarioSource (optional; default "exampleUShapedAzElTimeSpace")
%       Example function name, function handle, or compatible scalar
%       planner result struct.
%   - optionOverrides (scalar struct, optional; default struct())
%       FigureVisible: "on" or "off"; default "on".
%       SaveFigures: logical scalar; default false.
%       OutputDirectory: scalar text; default tmp/single_u_walkthrough.
%       RandomSeed: finite integer scalar; default 0.
%       ExportResolution_dpi: positive integer scalar; default 180.
%       ScenarioOverrides: scalar struct forwarded to an example function.
%**************************************************************************
% OUTPUTS
%   - walkthrough (scalar struct)
%       Result, options, figure and axes handles, reconstructed explanatory
%       corridor records, protected envelope, convex regions, and saved files.
%**************************************************************************
% UNITS
%   - Position and clearance are degrees. Time is seconds. Derivatives use
%     deg/s, deg/s^2, and deg/s^3.
%**************************************************************************

%% Section 1: Resolve Walkthrough Controls

if nargin < 1 || isempty(scenarioSource)
    scenarioSource = "exampleUShapedAzElTimeSpace";
end
if nargin < 2 || isempty(optionOverrides)
    optionOverrides = struct();
end
if ~isstruct(optionOverrides) || ~isscalar(optionOverrides)
    error("explainAzElPlannerWalkthrough:InvalidOptions", ...
        "optionOverrides must be a scalar struct.");
end
optionOverrides.ScenarioSource = scenarioSource;
projectRoot = fileparts(fileparts(mfilename("fullpath")));
options = resolveWalkthroughOptions(optionOverrides, projectRoot);
originalPath = path;
pathCleanup = onCleanup(@() path(originalPath));
addpath(projectRoot, fullfile(projectRoot, "examples"));

if options.SaveFigures && ~isfolder(options.OutputDirectory)
    mkdir(options.OutputDirectory);
end

%% Section 2: Run And Validate The Requested Scenario Once

fprintf("\nAz/El planner walkthrough\n");
fprintf("1. Running scenario source %s once with random seed %d.\n", ...
    scenarioSourceLabel(options.ScenarioSource), options.RandomSeed);
result = runScenarioSource(options);
if ~isCompatiblePlannerResult(result)
    error("explainAzElPlannerWalkthrough:IncompatibleResult", ...
        "ScenarioSource must return a compatible planner result struct.");
end
independentValidationPassed = result.Validation.Passed;
if isfield(result, "ExampleValidation")
    independentValidationPassed = independentValidationPassed && ...
        result.ExampleValidation.Passed;
end
if ~result.Success || ~independentValidationPassed
    error("explainAzElPlannerWalkthrough:ExampleFailed", ...
        "The requested scenario did not produce a validated motion: %s", ...
        result.Message);
end

smoothedPathLength_deg = sum(vecnorm(diff(result.position_deg), 2, 2));

selectedSeedIndex = result.SelectedSeedIndex;
selectedSeed = result.Seeds(selectedSeedIndex);
selectedSummary = result.SeedSummaries(selectedSeedIndex);
selectedSeedSpanWeight = diff(selectedSeed.tau(:));
initialControlPoint_deg = [ ...
    repmat(selectedSeed.position_deg(1, :), 3, 1); ...
    selectedSeed.position_deg(2:end - 1, :); ...
    repmat(selectedSeed.position_deg(end, :), 3, 1)];
primaryMovableControlCount = max(0, ...
    size(initialControlPoint_deg, 1) - 6);
startControlOverlapCount = nnz(vecnorm( ...
    initialControlPoint_deg - selectedSeed.position_deg(1, :), ...
    2, 2) <= 1e-12);
goalControlOverlapCount = nnz(vecnorm( ...
    initialControlPoint_deg - selectedSeed.position_deg(end, :), ...
    2, 2) <= 1e-12);
holdRecoveryUsed = false;
holdMultiplier = 1;
holdTrialCount = 0;
if isfield(selectedSummary, "SolverDiagnostics")
    selectedSolverDiagnostics = selectedSummary.SolverDiagnostics;
    if isfield(selectedSolverDiagnostics, "HoldRecoveryUsed")
        holdRecoveryUsed = selectedSolverDiagnostics.HoldRecoveryUsed;
        holdMultiplier = selectedSolverDiagnostics.HoldMultiplier;
        holdTrialCount = selectedSolverDiagnostics.HoldTrialCount;
    end
end
compactAttempted = false;
compactAccepted = false;
compactTrialCount = 0;
compactQpCount = 0;
if isfield(result.SearchDiagnostics, "CompactC3")
    compactDiagnostics = result.SearchDiagnostics.CompactC3;
    compactAttempted = compactDiagnostics.Attempted;
    compactAccepted = compactDiagnostics.Accepted;
    compactTrialCount = compactDiagnostics.TrialCount;
    compactQpCount = compactDiagnostics.QpCount;
end
initial_deg = result.Inputs.initialState.position_deg;
goal_deg = azElInternal.goalPositionAtTime( ...
    result.Inputs.goalState, result.ArrivalTime_s);
preparedObstacles = azElInternal.obstacles.prepareDynamic( ...
    result.Inputs.obstacles);
initialTime_s = result.Inputs.initialState.time_s;
[originalShape, protectedShape] = combinedObstacleShapesAtTime( ...
    preparedObstacles, initialTime_s);
geometryIsStatic = obstacleHistoryIsStatic(preparedObstacles);
envelopePadding_deg = 1e-6;
envelopeBoundary_deg = zeros(0, 2);
convexRegions = polyshape.empty(0, 1);
if geometryIsStatic
    envelopeBoundary_deg = ...
        azElPlannerMethods.corridor.internal.obstacles.buildEnvelopeBoundary( ...
        preparedObstacles, envelopePadding_deg);
    protectedShape = polyshape( ...
        envelopeBoundary_deg(:, 1), envelopeBoundary_deg(:, 2), ...
        "Simplify", true);
    convexRegions = azElInternal.convexPolygonRegions(protectedShape);
end

% The production solver constructs supports from its reduced route using a
% uniform segment parameter. Rebuild only that geometric evidence here; the
% final compact C3 trajectory remains the independently validated result.
explanatorySeed = selectedSeed;
explanatorySeed.tau = linspace( ...
    0, 1, size(selectedSeed.position_deg, 1)).';
explanatorySeed.CorridorBoundary_deg = envelopeBoundary_deg;
segmentCount = result.Polynomial.SegmentCount;
representativeSegmentIndex = max(1, ceil(segmentCount / 2));
representativePositionPower_deg = squeeze( ...
    result.Polynomial.positionPower_deg( ...
    representativeSegmentIndex, :, :));
corridor = struct([]);
if geometryIsStatic
    corridor = ...
        azElInternal.buildSeedCorridor( ...
        explanatorySeed, segmentCount);
end
expectedCorridorRecordCount = segmentCount * numel(convexRegions);
if numel(corridor) ~= expectedCorridorRecordCount
    error("explainAzElPlannerWalkthrough:IncompleteCorridor", ...
        "Expected %d support records but reconstructed %d.", ...
        expectedCorridorRecordCount, numel(corridor));
end

plotBounds_deg = walkthroughPlotBounds( ...
    originalShape, protectedShape, result, initial_deg, goal_deg);
palette = struct( ...
    "Navy", [0.09 0.21 0.36], ...
    "Blue", [0.18 0.45 0.71], ...
    "Cyan", [0.00 0.67 0.78], ...
    "Green", [0.13 0.55 0.13], ...
    "Gold", [0.75 0.45 0.03], ...
    "Red", [0.72 0.14 0.10], ...
    "Gray", [0.55 0.58 0.62], ...
    "LightBlue", [0.78 0.87 0.93]);
figureHandle = figure( ...
    "Name", "Az/El planner walkthrough", ...
    "Visible", options.FigureVisible, ...
    "Color", "w", ...
    "Position", [40 40 1500 960]);
tabGroupHandle = uitabgroup(figureHandle);
tabHandles = gobjects(6, 1);
axesHandles = cell(6, 1);
figureFiles = strings(0, 1);

%% Section 3: Explain The Problem And Candidate Seeds

fprintf("2. Tab 1 explains the geometry, search graph, three seeds, " + ...
    "and candidate gates.\n");
[tabHandles(1), layoutHandle] = createWalkthroughTab( ...
    tabGroupHandle, "1 Problem and seeds", [2 4]);
axesHandles{1} = gobjects(8, 1);

axesHandles{1}(1) = nexttile(layoutHandle);
drawProblemGeometry(axesHandles{1}(1), originalShape, polyshape(), ...
    initial_deg, goal_deg, plotBounds_deg, palette);
title(axesHandles{1}(1), "1. Original obstacle geometry");

axesHandles{1}(2) = nexttile(layoutHandle);
drawProblemGeometry(axesHandles{1}(2), originalShape, protectedShape, ...
    initial_deg, goal_deg, plotBounds_deg, palette);
title(axesHandles{1}(2), "2. Protected geometry at initial time");
legend(axesHandles{1}(2), "Location", "southoutside", ...
    "NumColumns", 2, "FontSize", 7);

axesHandles{1}(3) = nexttile(layoutHandle);
drawProblemGeometry(axesHandles{1}(3), originalShape, protectedShape, ...
    initial_deg, goal_deg, plotBounds_deg, palette);
drawSearchEdges(axesHandles{1}(3), result.SearchDiagnostics.Grid, palette);
title(axesHandles{1}(3), "3. Visibility tests");

displayedSeedIndex = unique( ...
    [1, selectedSeedIndex, numel(result.Seeds)], "stable");
remainingSeedIndex = setdiff( ...
    1:numel(result.Seeds), displayedSeedIndex, "stable");
displayedSeedIndex = [displayedSeedIndex, remainingSeedIndex];
displayedSeedIndex = displayedSeedIndex( ...
    1:min(3, numel(displayedSeedIndex)));
for routePaneIndex = 1:3
    paneIndex = routePaneIndex + 3;
    axesHandles{1}(paneIndex) = nexttile(layoutHandle);
    if routePaneIndex > numel(displayedSeedIndex)
        axis(axesHandles{1}(paneIndex), "off");
        title(axesHandles{1}(paneIndex), ...
            sprintf("%d. No additional seed", paneIndex));
        continue;
    end
    seedIndex = displayedSeedIndex(routePaneIndex);
    drawProblemGeometry(axesHandles{1}(paneIndex), originalShape, ...
        protectedShape, initial_deg, goal_deg, plotBounds_deg, palette);
    seed = result.Seeds(seedIndex);
    selected = seedIndex == selectedSeedIndex;
    drawSeed(axesHandles{1}(paneIndex), seed, selected, palette);
    title(axesHandles{1}(paneIndex), sprintf( ...
        "%d. Seed %d: %s", paneIndex, seedIndex, seed.Source), ...
        "Interpreter", "none");
end

axesHandles{1}(7) = nexttile(layoutHandle);
hold(axesHandles{1}(7), "on");
seedColors = lines(numel(result.Seeds));
for seedIndex = 1:numel(result.Seeds)
    seed = result.Seeds(seedIndex);
    plot(axesHandles{1}(7), 1:numel(seed.tau), seed.tau, "-o", ...
        "Color", seedColors(seedIndex, :), ...
        "DisplayName", "Seed " + seedIndex);
end
grid(axesHandles{1}(7), "on");
box(axesHandles{1}(7), "on");
xlabel(axesHandles{1}(7), "Route vertex index");
ylabel(axesHandles{1}(7), "Normalized seed time, \tau");
ylim(axesHandles{1}(7), [0 1]);
title(axesHandles{1}(7), "7. Seed timing proposals");
legend(axesHandles{1}(7), "Location", "northwest", "FontSize", 7);

axesHandles{1}(8) = nexttile(layoutHandle);
summary = result.SeedSummaries;
gateValues = [ ...
    [summary.OptimizerFeasible].', ...
    [summary.ValidationPassed].', ...
    [summary.CollisionResolved].', ...
    (1:numel(summary)).' == selectedSeedIndex];
imagesc(axesHandles{1}(8), gateValues, [0 1]);
colormap(axesHandles{1}(8), [0.93 0.77 0.77; 0.75 0.90 0.77]);
xticks(axesHandles{1}(8), 1:4);
xticklabels(axesHandles{1}(8), ...
    ["Optimizer", "Validation", "Resolved", "Selected"]);
xtickangle(axesHandles{1}(8), 25);
yticks(axesHandles{1}(8), 1:numel(summary));
yticklabels(axesHandles{1}(8), "Seed " + (1:numel(summary)));
for rowIndex = 1:size(gateValues, 1)
    for columnIndex = 1:size(gateValues, 2)
        text(axesHandles{1}(8), columnIndex, rowIndex, ...
            string(gateValues(rowIndex, columnIndex)), ...
            "HorizontalAlignment", "center", "FontWeight", "bold");
    end
end
title(axesHandles{1}(8), "8. Candidate success gates");
figureFiles = exportWalkthroughFigure( ...
    layoutHandle, "01_problem_and_seeds", options, figureFiles);

%% Section 4: Explain How The Corridor Is Formed

if geometryIsStatic
    fprintf("3. Tab 2 derives the static corridor: protect, decompose, " + ...
        "sample midpoints, support, and intersect.\n");
    corridorTabTitle = "2 Static corridor formation";
else
    fprintf("3. Tab 2 shows the protected obstacle evolving at physical " + ...
        "time; a static corridor is not claimed.\n");
    corridorTabTitle = "2 Moving geometry";
end
[tabHandles(2), layoutHandle] = createWalkthroughTab( ...
    tabGroupHandle, corridorTabTitle, [2 3]);
axesHandles{2} = gobjects(6, 1);
segmentMidpointTau = ((1:segmentCount).' - 0.5) / segmentCount;
segmentMidpoint_deg = interp1( ...
    explanatorySeed.tau, explanatorySeed.position_deg, ...
    segmentMidpointTau, "linear");
representativeSegment = 1;

if geometryIsStatic
axesHandles{2}(1) = nexttile(layoutHandle);
drawProblemGeometry(axesHandles{2}(1), originalShape, protectedShape, ...
    initial_deg, goal_deg, plotBounds_deg, palette);
title(axesHandles{2}(1), "1. Original and protected geometry");

axesHandles{2}(2) = nexttile(layoutHandle);
configureSpatialAxes(axesHandles{2}(2), plotBounds_deg);
drawConvexRegions(axesHandles{2}(2), convexRegions);
plot(axesHandles{2}(2), originalShape, "FaceColor", "none", ...
    "EdgeColor", palette.Navy, "LineWidth", 1.5);
title(axesHandles{2}(2), sprintf( ...
    "2. Exact convex decomposition: %d regions", numel(convexRegions)));

axesHandles{2}(3) = nexttile(layoutHandle);
drawProblemGeometry(axesHandles{2}(3), originalShape, protectedShape, ...
    initial_deg, goal_deg, plotBounds_deg, palette);
plot(axesHandles{2}(3), explanatorySeed.position_deg(:, 1), ...
    explanatorySeed.position_deg(:, 2), "-o", ...
    "Color", palette.Navy, "LineWidth", 1.3, ...
    "DisplayName", "Selected seed");
scatter(axesHandles{2}(3), segmentMidpoint_deg(:, 1), ...
    segmentMidpoint_deg(:, 2), 42, palette.Gold, "filled", ...
    "DisplayName", "Span midpoint");
for segmentIndex = 1:segmentCount
    text(axesHandles{2}(3), segmentMidpoint_deg(segmentIndex, 1), ...
        segmentMidpoint_deg(segmentIndex, 2), "  s" + segmentIndex, ...
        "FontSize", 7, "Color", palette.Navy);
end
title(axesHandles{2}(3), "3. One seed midpoint s_k per span");

axesHandles{2}(4) = nexttile(layoutHandle);
drawProblemGeometry(axesHandles{2}(4), originalShape, protectedShape, ...
    initial_deg, goal_deg, plotBounds_deg, palette);
point_deg = segmentMidpoint_deg(representativeSegment, :);
distance_deg = zeros(numel(convexRegions), 1);
nearestPoint_deg = zeros(numel(convexRegions), 2);
for regionIndex = 1:numel(convexRegions)
    [distance_deg(regionIndex), nearestPoint_deg(regionIndex, :)] = ...
        azElInternal.geometry.pointPolygonClearance( ...
        convexRegions(regionIndex), point_deg);
end
[~, nearestRegionOrder] = sort(distance_deg, "ascend");
nearestRegionOrder = nearestRegionOrder(1:min(4, numel(nearestRegionOrder)));
for regionIndex = reshape(nearestRegionOrder, 1, [])
    plot(axesHandles{2}(4), ...
        [nearestPoint_deg(regionIndex, 1), point_deg(1)], ...
        [nearestPoint_deg(regionIndex, 2), point_deg(2)], "-", ...
        "Color", palette.Red, "LineWidth", 1.2);
    scatter(axesHandles{2}(4), nearestPoint_deg(regionIndex, 1), ...
        nearestPoint_deg(regionIndex, 2), 30, palette.Red, "filled");
end
scatter(axesHandles{2}(4), point_deg(1), point_deg(2), ...
    55, palette.Gold, "filled");
xlim(axesHandles{2}(4), point_deg(1) + [-5 5]);
ylim(axesHandles{2}(4), point_deg(2) + [-5 5]);
title(axesHandles{2}(4), sprintf( ...
    "4. Closest obstacle points to s_%d", representativeSegment));

axesHandles{2}(5) = nexttile(layoutHandle);
drawProblemGeometry(axesHandles{2}(5), originalShape, protectedShape, ...
    initial_deg, goal_deg, plotBounds_deg, palette);
selectedRegionIndex = nearestRegionOrder(1);
selectedRecordIndex = find( ...
    [corridor.SegmentIndex] == representativeSegment & ...
    [corridor.RegionIndex] == selectedRegionIndex, 1, "first");
drawSingleHalfspaceExplanation(axesHandles{2}(5), ...
    convexRegions(selectedRegionIndex), corridor(selectedRecordIndex), ...
    nearestPoint_deg(selectedRegionIndex, :), point_deg, ...
    plotBounds_deg, palette);
title(axesHandles{2}(5), "5. Support line and retained half-space");

axesHandles{2}(6) = nexttile(layoutHandle);
drawProblemGeometry(axesHandles{2}(6), originalShape, protectedShape, ...
    initial_deg, goal_deg, plotBounds_deg, palette);
safeCell_deg = safeCellForSegment( ...
    corridor, representativeSegment, plotBounds_deg);
if ~isempty(safeCell_deg)
    patch(axesHandles{2}(6), safeCell_deg(:, 1), safeCell_deg(:, 2), ...
        palette.Green, "FaceAlpha", 0.16, "EdgeColor", palette.Green, ...
        "LineWidth", 1.8, "DisplayName", "Safe cell");
end
plot(axesHandles{2}(6), explanatorySeed.position_deg(:, 1), ...
    explanatorySeed.position_deg(:, 2), "-", ...
    "Color", palette.Navy, "LineWidth", 1.2);
scatter(axesHandles{2}(6), point_deg(1), point_deg(2), ...
    55, palette.Gold, "filled");
drawSupportRecords(axesHandles{2}(6), corridor, ...
    representativeSegment, plotBounds_deg, palette);
title(axesHandles{2}(6), ...
    "6. Intersect retained half-spaces = safe cell");
else
    geometryTime_s = linspace(initialTime_s, result.ArrivalTime_s, 6);
    for paneIndex = 1:6
        axesHandles{2}(paneIndex) = nexttile(layoutHandle);
        time_s = geometryTime_s(paneIndex);
        [originalAtTime, protectedAtTime] = ...
            combinedObstacleShapesAtTime(preparedObstacles, time_s);
        goalAtTime_deg = azElInternal.goalPositionAtTime( ...
            result.Inputs.goalState, time_s);
        drawProblemGeometry(axesHandles{2}(paneIndex), ...
            originalAtTime, protectedAtTime, initial_deg, goalAtTime_deg, ...
            plotBounds_deg, palette);
        plot(axesHandles{2}(paneIndex), result.position_deg(:, 1), ...
            result.position_deg(:, 2), "-", "Color", palette.Gray, ...
            "LineWidth", 1.2, "DisplayName", "Validated motion");
        title(axesHandles{2}(paneIndex), sprintf( ...
            "%d. Exact geometry at t = %.2f s", paneIndex, time_s));
    end
end
figureFiles = exportWalkthroughFigure( ...
    layoutHandle, "02_corridor_construction", options, figureFiles);

%% Section 5: Show Every Span Cell

if geometryIsStatic
    fprintf("4. Tab 3 shows reconstructed safe cells across the " + ...
        "quintic spans.\n");
    spanTabTitle = "3 Span corridor cells";
else
    fprintf("4. Tab 3 shows time-local protected geometry along the " + ...
        "validated motion.\n");
    spanTabTitle = "3 Time-local motion slices";
end
[tabHandles(3), layoutHandle] = createWalkthroughTab( ...
    tabGroupHandle, spanTabTitle, [2 4]);
axesHandles{3} = gobjects(8, 1);
if geometryIsStatic
displayedSegmentIndex = unique(round(linspace(1, segmentCount, 8)), "stable");
if numel(displayedSegmentIndex) < 8
    displayedSegmentIndex = [displayedSegmentIndex, ...
        repmat(displayedSegmentIndex(end), 1, 8 - numel(displayedSegmentIndex))];
end
for paneIndex = 1:8
    segmentIndex = displayedSegmentIndex(paneIndex);
    axesHandles{3}(paneIndex) = nexttile(layoutHandle);
    drawProblemGeometry(axesHandles{3}(paneIndex), originalShape, ...
        protectedShape, initial_deg, goal_deg, plotBounds_deg, palette);
    safeCell_deg = safeCellForSegment( ...
        corridor, segmentIndex, plotBounds_deg);
    if ~isempty(safeCell_deg)
        patch(axesHandles{3}(paneIndex), ...
            safeCell_deg(:, 1), safeCell_deg(:, 2), palette.Green, ...
            "FaceAlpha", 0.16, "EdgeColor", palette.Green, ...
            "LineWidth", 1.5);
    end
    plot(axesHandles{3}(paneIndex), ...
        explanatorySeed.position_deg(:, 1), ...
        explanatorySeed.position_deg(:, 2), "-", ...
        "Color", palette.Navy, "LineWidth", 1.0);
    scatter(axesHandles{3}(paneIndex), ...
        segmentMidpoint_deg(segmentIndex, 1), ...
        segmentMidpoint_deg(segmentIndex, 2), ...
        48, palette.Gold, "filled");
    recordCount = sum([corridor.SegmentIndex] == segmentIndex);
    title(axesHandles{3}(paneIndex), sprintf( ...
        "Span %d | %d supports", segmentIndex, recordCount));
end
else
    geometryTime_s = linspace(initialTime_s, result.ArrivalTime_s, 8);
    for paneIndex = 1:8
        axesHandles{3}(paneIndex) = nexttile(layoutHandle);
        time_s = geometryTime_s(paneIndex);
        [originalAtTime, protectedAtTime] = ...
            combinedObstacleShapesAtTime(preparedObstacles, time_s);
        goalAtTime_deg = azElInternal.goalPositionAtTime( ...
            result.Inputs.goalState, time_s);
        drawProblemGeometry(axesHandles{3}(paneIndex), ...
            originalAtTime, protectedAtTime, initial_deg, goalAtTime_deg, ...
            plotBounds_deg, palette);
        reached = result.time_s <= time_s + 1e-10;
        plot(axesHandles{3}(paneIndex), result.position_deg(:, 1), ...
            result.position_deg(:, 2), "-", "Color", palette.Gray, ...
            "LineWidth", 1.0);
        plot(axesHandles{3}(paneIndex), result.position_deg(reached, 1), ...
            result.position_deg(reached, 2), "-", ...
            "Color", palette.Cyan, "LineWidth", 2.4);
        title(axesHandles{3}(paneIndex), sprintf( ...
            "Time-local slice | t = %.2f s", time_s));
    end
end
figureFiles = exportWalkthroughFigure( ...
    layoutHandle, "03_span_corridor_cells", options, figureFiles);

%% Section 6: Animate The Quintic Evolution Across Static Panels

fprintf("5. Tab 4 freezes eight times to show how the validated quintic " + ...
    "motion evolves.\n");
[tabHandles(4), layoutHandle] = createWalkthroughTab( ...
    tabGroupHandle, "4 Motion evolution", [2 4]);
axesHandles{4} = gobjects(8, 1);
snapshotIndex = unique(round(linspace(1, numel(result.time_s), 8)), ...
    "stable");
if numel(snapshotIndex) < 8
    snapshotIndex = [snapshotIndex, ...
        repmat(snapshotIndex(end), 1, 8 - numel(snapshotIndex))];
end
for paneIndex = 1:8
    sampleIndex = snapshotIndex(paneIndex);
    axesHandles{4}(paneIndex) = nexttile(layoutHandle);
    sampleTime_s = result.time_s(sampleIndex);
    if geometryIsStatic
        originalAtTime = originalShape;
        protectedAtTime = protectedShape;
    else
        [originalAtTime, protectedAtTime] = ...
            combinedObstacleShapesAtTime(preparedObstacles, sampleTime_s);
    end
    goalAtTime_deg = azElInternal.goalPositionAtTime( ...
        result.Inputs.goalState, sampleTime_s);
    drawProblemGeometry(axesHandles{4}(paneIndex), originalAtTime, ...
        protectedAtTime, initial_deg, goalAtTime_deg, ...
        plotBounds_deg, palette);
    plot(axesHandles{4}(paneIndex), result.position_deg(:, 1), ...
        result.position_deg(:, 2), "-", "Color", [0.76 0.79 0.82], ...
        "LineWidth", 1.4, "DisplayName", "Complete quintic");
    plot(axesHandles{4}(paneIndex), ...
        result.position_deg(1:sampleIndex, 1), ...
        result.position_deg(1:sampleIndex, 2), "-", ...
        "Color", palette.Cyan, "LineWidth", 3.0, ...
        "DisplayName", "Elapsed motion");
    scatter(axesHandles{4}(paneIndex), ...
        result.position_deg(sampleIndex, 1), ...
        result.position_deg(sampleIndex, 2), 54, palette.Red, ...
        "filled", "MarkerEdgeColor", "w", "LineWidth", 0.8, ...
        "DisplayName", "Current state");
    title(axesHandles{4}(paneIndex), sprintf( ...
        "t = %.2f s | %.0f%%", result.time_s(sampleIndex), ...
        100 * (sampleIndex - 1) / (numel(result.time_s) - 1)));
end
figureFiles = exportWalkthroughFigure( ...
    layoutHandle, "04_quintic_evolution", options, figureFiles);

%% Section 7: Explain The Piecewise Quintic And Its Derivatives

fprintf("6. Tab 5 exposes the polynomial spans, time allocation, and " + ...
    "continuous derivative limits, plus the seed-to-spline map.\n");
[tabHandles(5), layoutHandle] = createWalkthroughTab( ...
    tabGroupHandle, "5 Seed to quintic", [3 3]);
axesHandles{5} = gobjects(9, 1);
spanColors = turbo(segmentCount);

axesHandles{5}(1) = nexttile(layoutHandle);
drawProblemGeometry(axesHandles{5}(1), originalShape, protectedShape, ...
    initial_deg, goal_deg, plotBounds_deg, palette);
for segmentIndex = 1:segmentCount
    startTime_s = result.Polynomial.SegmentStartTime_s(segmentIndex);
    endTime_s = startTime_s + ...
        result.Polynomial.SegmentDuration_s(segmentIndex);
    belongsToSegment = result.time_s >= startTime_s - 1e-10 & ...
        result.time_s <= endTime_s + 1e-10;
    plot(axesHandles{5}(1), result.position_deg(belongsToSegment, 1), ...
        result.position_deg(belongsToSegment, 2), "-", ...
        "Color", spanColors(segmentIndex, :), "LineWidth", 2.4, ...
        "DisplayName", "Span " + segmentIndex);
end
title(axesHandles{5}(1), sprintf( ...
    "1. %d colored quintic spans", segmentCount));

axesHandles{5}(2) = nexttile(layoutHandle);
bar(axesHandles{5}(2), 1:segmentCount, ...
    result.Polynomial.SegmentDuration_s, ...
    "FaceColor", "flat", "CData", spanColors);
grid(axesHandles{5}(2), "on");
box(axesHandles{5}(2), "on");
xlabel(axesHandles{5}(2), "Span index");
ylabel(axesHandles{5}(2), "Duration (s)");
title(axesHandles{5}(2), "2. Physical time assigned per span");

axesHandles{5}(3) = nexttile(layoutHandle);
drawProblemGeometry(axesHandles{5}(3), originalShape, protectedShape, ...
    initial_deg, goal_deg, plotBounds_deg, palette);
plot(axesHandles{5}(3), selectedSeed.position_deg(:, 1), ...
    selectedSeed.position_deg(:, 2), "--o", ...
    "Color", palette.Gray, "LineWidth", 1.2, ...
    "DisplayName", "Selected seed Q");
plot(axesHandles{5}(3), initialControlPoint_deg(:, 1), ...
    initialControlPoint_deg(:, 2), ":s", ...
    "Color", palette.Gold, "LineWidth", 1.4, ...
    "MarkerFaceColor", palette.Gold, ...
    "DisplayName", "Initial controls P");
plot(axesHandles{5}(3), result.position_deg(:, 1), ...
    result.position_deg(:, 2), "-", ...
    "Color", palette.Cyan, "LineWidth", 2.2, ...
    "DisplayName", "Final polynomial");
text(axesHandles{5}(3), initial_deg(1), initial_deg(2), ...
    "  " + startControlOverlapCount + " controls overlap", ...
    "Color", palette.Navy, "FontSize", 7);
text(axesHandles{5}(3), goal_deg(1), goal_deg(2), ...
    "  " + goalControlOverlapCount + " controls overlap", ...
    "Color", palette.Navy, "FontSize", 7);
title(axesHandles{5}(3), "3. Seed, initial controls, final motion");
legend(axesHandles{5}(3), "Location", "best", "FontSize", 6);

quantityNames = [ ...
    "position_deg", "velocity_deg_s", ...
    "acceleration_deg_s2", "jerk_deg_s3"];
yLabels = [ ...
    "Position (deg)", "Velocity (deg/s)", ...
    "Acceleration (deg/s^2)", "Jerk (deg/s^3)"];
limits = {[], result.Inputs.limits.maxVelocity_deg_s, ...
    result.Inputs.limits.maxAcceleration_deg_s2, ...
    result.Inputs.limits.maxJerk_deg_s3};
for quantityIndex = 1:4
    paneIndex = quantityIndex + 3;
    axesHandles{5}(paneIndex) = nexttile(layoutHandle);
    hold(axesHandles{5}(paneIndex), "on");
    values = result.(quantityNames(quantityIndex));
    plot(axesHandles{5}(paneIndex), result.time_s, values(:, 1), ...
        "Color", palette.Blue, "LineWidth", 1.3, ...
        "DisplayName", "Azimuth");
    plot(axesHandles{5}(paneIndex), result.time_s, values(:, 2), ...
        "Color", palette.Gold, "LineWidth", 1.3, ...
        "DisplayName", "Elevation");
    if ~isempty(limits{quantityIndex})
        for axisIndex = 1:2
            yline(axesHandles{5}(paneIndex), ...
                limits{quantityIndex}(axisIndex), "--", ...
                "Color", palette.Red, "HandleVisibility", "off");
            yline(axesHandles{5}(paneIndex), ...
                -limits{quantityIndex}(axisIndex), "--", ...
                "Color", palette.Red, "HandleVisibility", "off");
        end
    end
    for knotIndex = 2:segmentCount
        xline(axesHandles{5}(paneIndex), ...
            result.Polynomial.SegmentStartTime_s(knotIndex), ":", ...
            "Color", palette.Gray, "HandleVisibility", "off");
    end
    grid(axesHandles{5}(paneIndex), "on");
    box(axesHandles{5}(paneIndex), "on");
    xlabel(axesHandles{5}(paneIndex), "Time (s)");
    ylabel(axesHandles{5}(paneIndex), yLabels(quantityIndex));
    title(axesHandles{5}(paneIndex), ...
        (paneIndex) + ". " + erase(yLabels(quantityIndex), ...
        [" (deg)", " (deg/s)", " (deg/s^2)", " (deg/s^3)"]));
end
legend(axesHandles{5}(4), "Location", "best", "FontSize", 7);

axesHandles{5}(8) = nexttile(layoutHandle);
bar(axesHandles{5}(8), 0:5, representativePositionPower_deg.', ...
    "grouped");
grid(axesHandles{5}(8), "on");
box(axesHandles{5}(8), "on");
xlabel(axesHandles{5}(8), "Local power j");
ylabel(axesHandles{5}(8), "Position coefficient c_j (deg)");
xticks(axesHandles{5}(8), 0:5);
legend(axesHandles{5}(8), ["Azimuth", "Elevation"], ...
    "Location", "best", "FontSize", 7);
title(axesHandles{5}(8), sprintf( ...
    "8. Span %d: p(\\tau)=\\Sigma c_j\\tau^j", ...
    representativeSegmentIndex));

axesHandles{5}(9) = nexttile(layoutHandle);
axis(axesHandles{5}(9), "off");
representationName = "Primary seed spline";
finalControlPointCount = size(initialControlPoint_deg, 1);
if compactAccepted
    representationName = "Compact C3 replacement";
    finalControlPointCount = 2 * segmentCount + 4;
end
weightText = strjoin(compose("%.3f", selectedSeedSpanWeight), ", ");
constructionLines = [ ...
    "SEED -> CONTROL -> QUINTIC"; ...
    ""; ...
    "Seed source: " + string(selectedSeed.Source); ...
    "Seed vertices / spans: " + size(selectedSeed.position_deg, 1) + ...
    " / " + numel(selectedSeedSpanWeight); ...
    "Seed span weights \Delta\tau: [" + weightText + "]"; ...
    "Initial controls: " + size(initialControlPoint_deg, 1); ...
    "Movable offsets (az+el): " + 2 * primaryMovableControlCount; ...
    "Spline degree: 5"; ...
    ""; ...
    "Final representation: " + representationName; ...
    "Final spans / controls: " + segmentCount + " / " + ...
    finalControlPointCount; ...
    "Hold recovery used: " + string(holdRecoveryUsed); ...
    "Hold multiplier / trials: " + compose("%.5g", holdMultiplier) + ...
    " / " + holdTrialCount; ...
    "Compact attempted / accepted: " + string(compactAttempted) + ...
    " / " + string(compactAccepted); ...
    "Compact duration trials / QPs: " + compactTrialCount + ...
    " / " + compactQpCount; ...
    ""; ...
    "Displayed coefficient span: " + representativeSegmentIndex; ...
    "t_k / h_k: " + compose("%.4f", ...
    result.Polynomial.SegmentStartTime_s( ...
    representativeSegmentIndex)) + " / " + compose("%.4f", ...
    result.Polynomial.SegmentDuration_s( ...
    representativeSegmentIndex)) + " s"; ...
    "\tau=(t-t_k)/h_k,  0 <= \tau <= 1"; ...
    "v, a, j follow by exact differentiation."; ...
    "Interior seed points guide the curve;"; ...
    "they are not mandatory crossings."];
text(axesHandles{5}(9), 0.02, 0.98, ...
    strjoin(constructionLines, newline), ...
    "Units", "normalized", "VerticalAlignment", "top", ...
    "FontName", "Consolas", "FontSize", 7.5, ...
    "Color", palette.Navy, "Interpreter", "tex");
title(axesHandles{5}(9), "9. Construction record");
figureFiles = exportWalkthroughFigure( ...
    layoutHandle, "05_quintic_spans_and_derivatives", ...
    options, figureFiles);

%% Section 8: Explain Candidate Selection And Independent Validation

fprintf("7. Tab 6 closes the loop with candidate ranking, clearance, " + ...
    "continuity, limits, and validation gates.\n");
[tabHandles(6), layoutHandle] = createWalkthroughTab( ...
    tabGroupHandle, "6 Selection and validation", [2 4]);
axesHandles{6} = gobjects(8, 1);
summary = result.SeedSummaries;
seedIndex = (1:numel(summary)).';

axesHandles{6}(1) = nexttile(layoutHandle);
arrivalTime_s = [summary.ArrivalTime_s].';
bar(axesHandles{6}(1), seedIndex, arrivalTime_s, ...
    "FaceColor", palette.Blue);
grid(axesHandles{6}(1), "on");
box(axesHandles{6}(1), "on");
xlabel(axesHandles{6}(1), "Seed index");
ylabel(axesHandles{6}(1), "Arrival time (s)");
title(axesHandles{6}(1), "1. Candidate arrival / failure time");

axesHandles{6}(2) = nexttile(layoutHandle);
jerkCost = [summary.IntegratedSquaredJerk_deg2_s5].';
finiteQuality = isfinite(arrivalTime_s) & isfinite(jerkCost);
finiteQuality = finiteQuality & [summary.ValidationPassed].';
scatter(axesHandles{6}(2), arrivalTime_s(finiteQuality), ...
    jerkCost(finiteQuality), 65, seedIndex(finiteQuality), "filled");
hold(axesHandles{6}(2), "on");
for index = reshape(find(finiteQuality), 1, [])
    text(axesHandles{6}(2), arrivalTime_s(index), jerkCost(index), ...
        "  Seed " + index, "FontSize", 8);
end
grid(axesHandles{6}(2), "on");
box(axesHandles{6}(2), "on");
xlabel(axesHandles{6}(2), "Arrival time (s)");
ylabel(axesHandles{6}(2), "Integrated squared jerk");
title(axesHandles{6}(2), "2. Validated time-jerk quality");

axesHandles{6}(3) = nexttile(layoutHandle);
minimumClearance_deg = [summary.MinimumClearance_deg].';
bar(axesHandles{6}(3), seedIndex, 1e6 * minimumClearance_deg, ...
    "FaceColor", palette.Green);
yline(axesHandles{6}(3), 0, "-", "Color", palette.Red);
grid(axesHandles{6}(3), "on");
box(axesHandles{6}(3), "on");
xlabel(axesHandles{6}(3), "Seed index");
ylabel(axesHandles{6}(3), "Minimum clearance (10^{-6} deg)");
title(axesHandles{6}(3), "3. Signed candidate clearance");

axesHandles{6}(4) = nexttile(layoutHandle);
selectionGate = [ ...
    [summary.OptimizerFeasible].', ...
    [summary.ValidationPassed].', ...
    [summary.CollisionResolved].', ...
    seedIndex == selectedSeedIndex];
imagesc(axesHandles{6}(4), selectionGate, [0 1]);
colormap(axesHandles{6}(4), [0.93 0.77 0.77; 0.75 0.90 0.77]);
xticks(axesHandles{6}(4), 1:4);
xticklabels(axesHandles{6}(4), ...
    ["Feasible", "Validated", "Resolved", "Selected"]);
xtickangle(axesHandles{6}(4), 25);
yticks(axesHandles{6}(4), seedIndex);
yticklabels(axesHandles{6}(4), "Seed " + seedIndex);
title(axesHandles{6}(4), "4. Selection gates");

axesHandles{6}(5) = nexttile(layoutHandle);
signedClearance_deg = signedClearanceHistory( ...
    preparedObstacles, result.time_s, result.position_deg);
plot(axesHandles{6}(5), result.time_s, signedClearance_deg, ...
    "Color", palette.Cyan, "LineWidth", 1.6);
yline(axesHandles{6}(5), 0, "-", "Color", palette.Red);
grid(axesHandles{6}(5), "on");
box(axesHandles{6}(5), "on");
xlabel(axesHandles{6}(5), "Time (s)");
ylabel(axesHandles{6}(5), "Sampled signed clearance (deg)");
title(axesHandles{6}(5), "5. Spatial clearance history");

axesHandles{6}(6) = nexttile(layoutHandle);
continuityResidual = polynomialContinuityResidual(result.Polynomial);
bar(axesHandles{6}(6), continuityResidual, ...
    "FaceColor", palette.Navy);
set(axesHandles{6}(6), "YScale", "log");
grid(axesHandles{6}(6), "on");
box(axesHandles{6}(6), "on");
xticks(axesHandles{6}(6), 1:4);
xticklabels(axesHandles{6}(6), ...
    ["Position", "Velocity", "Acceleration", "Jerk"]);
xtickangle(axesHandles{6}(6), 25);
ylabel(axesHandles{6}(6), "Maximum knot residual");
title(axesHandles{6}(6), "6. C3 continuity at every knot");

axesHandles{6}(7) = nexttile(layoutHandle);
peakRatio = [ ...
    result.Validation.PeakVelocity_deg_s ./ ...
    result.Inputs.limits.maxVelocity_deg_s; ...
    result.Validation.PeakAcceleration_deg_s2 ./ ...
    result.Inputs.limits.maxAcceleration_deg_s2; ...
    result.Validation.PeakJerk_deg_s3 ./ ...
    result.Inputs.limits.maxJerk_deg_s3];
bar(axesHandles{6}(7), peakRatio);
yline(axesHandles{6}(7), 1, "--", "Color", palette.Red, ...
    "LineWidth", 1.2, "DisplayName", "Limit");
grid(axesHandles{6}(7), "on");
box(axesHandles{6}(7), "on");
xticks(axesHandles{6}(7), 1:3);
xticklabels(axesHandles{6}(7), ["Velocity", "Acceleration", "Jerk"]);
ylabel(axesHandles{6}(7), "Peak / limit");
legend(axesHandles{6}(7), ["Azimuth", "Elevation", "Limit"], ...
    "Location", "northwest", "FontSize", 7);
title(axesHandles{6}(7), "7. Continuous kinematic margins");

axesHandles{6}(8) = nexttile(layoutHandle);
axis(axesHandles{6}(8), "off");
validation = result.Validation;
summaryLines = [ ...
    "FINAL SUCCESS GATE"; ...
    ""; ...
    "Planner success: " + string(result.Success); ...
    "Independent validation: " + string(independentValidationPassed); ...
    "Polynomial schema: " + string(validation.PolynomialSchemaValid); ...
    "C3 continuity: " + string(validation.PolynomialSegmentContinuity); ...
    "Dynamics consistent: " + string(validation.DynamicsConsistent); ...
    "Collision free / resolved: " + string(validation.CollisionFree) + ...
    " / " + string(validation.CollisionResolved); ...
    "Kinematic limits V/A/J: " + string(validation.VelocityWithinLimits) + ...
    " / " + string(validation.AccelerationWithinLimits) + " / " + ...
    string(validation.JerkWithinLimits); ...
    "Static corridor applicable: " + string(geometryIsStatic); ...
    "Selected seed: " + selectedSeedIndex + " (" + ...
    string(selectedSeed.Source) + ")"; ...
    "Hold recovery / multiplier: " + string(holdRecoveryUsed) + ...
    " / " + compose("%.5g", holdMultiplier); ...
    "Compact C3 attempted / accepted: " + string(compactAttempted) + ...
    " / " + string(compactAccepted); ...
    "Seed spans -> final spans: " + numel(selectedSeedSpanWeight) + ...
    " -> " + segmentCount; ...
    "Duration: " + compose("%.4f", result.TrajectoryDuration_s) + " s"; ...
    "Smoothed length: " + compose("%.4f", ...
    smoothedPathLength_deg) + " deg"; ...
    ""; ...
    "A seed proposes topology and timing."; ...
    "Only the complete timed polynomial can pass."];
summaryText = strjoin(summaryLines, newline);
text(axesHandles{6}(8), 0.02, 0.98, summaryText, ...
    "Units", "normalized", "VerticalAlignment", "top", ...
    "FontName", "Consolas", "FontSize", 8, ...
    "Color", palette.Navy);
figureFiles = exportWalkthroughFigure( ...
    layoutHandle, "06_selection_and_validation", ...
    options, figureFiles);

%% Section 9: Return Walkthrough Evidence

tabGroupHandle.SelectedTab = tabHandles(1);
fprintf("8. Complete: %d tabs, %d panels, selected seed %d, " + ...
    "duration %.4f s.\n\n", numel(tabHandles), ...
    sum(cellfun(@numel, axesHandles)), selectedSeedIndex, ...
    result.TrajectoryDuration_s);
walkthrough = struct( ...
    "Result", result, ...
    "Options", options, ...
    "FigureHandle", figureHandle, ...
    "TabGroupHandle", tabGroupHandle, ...
    "TabHandles", tabHandles, ...
    "AxesHandles", {axesHandles}, ...
    "EnvelopeBoundary_deg", envelopeBoundary_deg, ...
    "ConvexRegions", convexRegions, ...
    "CorridorRecords", corridor, ...
    "SegmentMidpoint_deg", segmentMidpoint_deg, ...
    "SelectedSeedSpanWeight", selectedSeedSpanWeight, ...
    "InitialControlPoint_deg", initialControlPoint_deg, ...
    "RepresentativeSegmentIndex", representativeSegmentIndex, ...
    "RepresentativePositionPower_deg", ...
    representativePositionPower_deg, ...
    "ConstructionDiagnostics", struct( ...
    "HoldRecoveryUsed", holdRecoveryUsed, ...
    "HoldMultiplier", holdMultiplier, ...
    "HoldTrialCount", holdTrialCount, ...
    "CompactAttempted", compactAttempted, ...
    "CompactAccepted", compactAccepted, ...
    "CompactTrialCount", compactTrialCount, ...
    "CompactQpCount", compactQpCount), ...
    "FigureFiles", figureFiles);
end


function result = runScenarioSource(options)
%% Section 0: Header & Readme
% SYNTAX
%   result = runScenarioSource(options)
%**************************************************************************
% PURPOSE
%   - Resolve a supplied result or invoke one compatible scenario function.
%**************************************************************************
% INPUTS
%   - options (scalar struct): source, overrides, and deterministic seed.
%**************************************************************************
% OUTPUTS
%   - result (scalar struct): returned planner result.
%**************************************************************************
% UNITS
%   - Units remain those of the scenario result.
%**************************************************************************

source = options.ScenarioSource;
if isstruct(source)
    result = source;
    return;
end
scenarioOverrides = options.ScenarioOverrides;
scenarioOverrides.PlotOutputs = false;
scenarioOverrides.FigureVisible = "off";
scenarioOverrides.RandomSeed = options.RandomSeed;
if isa(source, "function_handle")
    scenarioFunction = source;
else
    scenarioFunction = str2func(char(string(source)));
end
result = scenarioFunction(scenarioOverrides);
end


function label = scenarioSourceLabel(source)
%% Section 0: Header & Readme
% SYNTAX
%   label = scenarioSourceLabel(source)
%**************************************************************************
% PURPOSE
%   - Produce concise command-window text for one scenario source.
%**************************************************************************
% INPUTS
%   - source: function handle, function name, or result struct.
%**************************************************************************
% OUTPUTS
%   - label (scalar string): readable source description.
%**************************************************************************
% UNITS
%   - None.
%**************************************************************************

if isa(source, "function_handle")
    label = string(func2str(source));
elseif isstruct(source)
    label = "precomputed result";
else
    label = string(source);
end
end


function compatible = isCompatiblePlannerResult(result)
%% Section 0: Header & Readme
% SYNTAX
%   compatible = isCompatiblePlannerResult(result)
%**************************************************************************
% PURPOSE
%   - Check the minimum data contract required by the tabbed walkthrough.
%**************************************************************************
% INPUTS
%   - result: candidate scalar planner result.
%**************************************************************************
% OUTPUTS
%   - compatible (logical scalar): true when required fields are present.
%**************************************************************************
% UNITS
%   - None.
%**************************************************************************

requiredFields = { ...
    'Success', 'Message', 'Inputs', 'Seeds', 'SeedSummaries', ...
    'SearchDiagnostics', ...
    'SelectedSeedIndex', 'time_s', 'position_deg', ...
    'velocity_deg_s', 'acceleration_deg_s2', 'jerk_deg_s3', ...
    'Polynomial', 'Validation', 'ArrivalTime_s', 'TrajectoryDuration_s'};
compatible = isstruct(result) && isscalar(result) && ...
    all(isfield(result, requiredFields));
if compatible
    compatible = ~isempty(result.Seeds) && ...
        result.SelectedSeedIndex >= 1 && ...
        result.SelectedSeedIndex <= numel(result.Seeds);
end
end


function [originalShape, protectedShape] = combinedObstacleShapesAtTime( ...
        obstacles, time_s)
%% Section 0: Header & Readme
% SYNTAX
%   [originalShape, protectedShape] = combinedObstacleShapesAtTime( ...
%       obstacles, time_s)
%**************************************************************************
% PURPOSE
%   - Union all original and protected obstacle shapes at one physical time.
%**************************************************************************
% INPUTS
%   - obstacles (prepared obstacle array), time_s (finite scalar).
%**************************************************************************
% OUTPUTS
%   - originalShape, protectedShape (scalar polyshape): unioned geometry.
%**************************************************************************
% UNITS
%   - Time is seconds and geometry is degrees.
%**************************************************************************

originalShape = polyshape();
protectedShape = polyshape();
for obstacleIndex = 1:numel(obstacles)
    obstacle = obstacles(obstacleIndex);
    protectedAtTime = azElInternal.obstacles.shapeAtTime(obstacle, time_s);
    protectedShape = union(protectedShape, protectedAtTime);
    originalObstacle = obstacle;
    if isfield(obstacle, "originalAz_deg") && ...
            isfield(obstacle, "originalEl_deg")
        originalObstacle.az_deg = obstacle.originalAz_deg;
        originalObstacle.el_deg = obstacle.originalEl_deg;
    end
    if isfield(originalObstacle, "InternalPreparation")
        originalObstacle = rmfield(originalObstacle, "InternalPreparation");
    end
    originalAtTime = azElInternal.obstacles.shapeAtTime( ...
        originalObstacle, time_s);
    originalShape = union(originalShape, originalAtTime);
end
end


function isStatic = obstacleHistoryIsStatic(obstacles)
%% Section 0: Header & Readme
% SYNTAX
%   isStatic = obstacleHistoryIsStatic(obstacles)
%**************************************************************************
% PURPOSE
%   - Prove whether every stored protected obstacle slice is stationary.
%**************************************************************************
% INPUTS
%   - obstacles (prepared obstacle array).
%**************************************************************************
% OUTPUTS
%   - isStatic (logical scalar).
%**************************************************************************
% UNITS
%   - None.
%**************************************************************************

isStatic = true;
for obstacleIndex = 1:numel(obstacles)
    obstacle = obstacles(obstacleIndex);
    for sliceIndex = 2:numel(obstacle.az_deg)
        isStatic = isStatic && ...
            isequal(obstacle.az_deg{sliceIndex}, obstacle.az_deg{1}) && ...
            isequal(obstacle.el_deg{sliceIndex}, obstacle.el_deg{1});
        if ~isStatic
            return;
        end
    end
end
end


function plotBounds_deg = walkthroughPlotBounds( ...
        originalShape, protectedShape, result, initial_deg, goal_deg)
%% Section 0: Header & Readme
% SYNTAX
%   plotBounds_deg = walkthroughPlotBounds( ...
%       originalShape, protectedShape, result, initial_deg, goal_deg)
%**************************************************************************
% PURPOSE
%   - Derive reusable plot bounds from geometry, seeds, motion, and endpoints.
%**************************************************************************
% INPUTS
%   - originalShape, protectedShape (polyshapes), result (planner result).
%   - initial_deg, goal_deg (1-by-2 endpoint positions).
%**************************************************************************
% OUTPUTS
%   - plotBounds_deg (1-by-4): [xmin xmax ymin ymax].
%**************************************************************************
% UNITS
%   - Bounds and all source coordinates are degrees.
%**************************************************************************

point_deg = [initial_deg; goal_deg; result.position_deg];
if ~isempty(originalShape.Vertices)
    point_deg = [point_deg; originalShape.Vertices];
end
if ~isempty(protectedShape.Vertices)
    point_deg = [point_deg; protectedShape.Vertices];
end
for seedIndex = 1:numel(result.Seeds)
    point_deg = [point_deg; result.Seeds(seedIndex).position_deg]; %#ok<AGROW>
end
point_deg = point_deg(all(isfinite(point_deg), 2), :);
minimum_deg = min(point_deg, [], 1);
maximum_deg = max(point_deg, [], 1);
span_deg = max(maximum_deg - minimum_deg, [1 1]);
padding_deg = max(1.5, 0.12 * max(span_deg));
plotBounds_deg = [ ...
    minimum_deg(1) - padding_deg, maximum_deg(1) + padding_deg, ...
    minimum_deg(2) - padding_deg, maximum_deg(2) + padding_deg];
end


function options = resolveWalkthroughOptions(optionOverrides, projectRoot)
%% Section 0: Header & Readme
% SYNTAX
%   options = resolveWalkthroughOptions(optionOverrides, projectRoot)
%**************************************************************************
% PURPOSE
%   - Validate and resolve the sandbox walkthrough controls.
%**************************************************************************
% INPUTS
%   - optionOverrides (scalar struct): caller overrides.
%   - projectRoot (scalar text): project path used for the default output.
%**************************************************************************
% OUTPUTS
%   - options (scalar struct): fully resolved controls.
%**************************************************************************
% UNITS
%   - ExportResolution_dpi is dots per inch; all other controls are unitless.
%**************************************************************************

if ~isstruct(optionOverrides) || ~isscalar(optionOverrides)
    error("explainAzElPlannerWalkthrough:InvalidOptions", ...
        "optionOverrides must be a scalar struct.");
end
defaults = struct( ...
    "FigureVisible", "on", ...
    "SaveFigures", false, ...
    "OutputDirectory", fullfile(projectRoot, "tmp", ...
    "az_el_planner_walkthrough"), ...
    "RandomSeed", 0, ...
    "ExportResolution_dpi", 180, ...
    "ScenarioSource", "exampleUShapedAzElTimeSpace", ...
    "ScenarioOverrides", struct());
options = defaults;
knownNames = string(fieldnames(defaults));
overrideNames = string(fieldnames(optionOverrides));
unknownNames = setdiff(overrideNames, knownNames, "stable");
if ~isempty(unknownNames)
    error("explainAzElPlannerWalkthrough:UnknownOption", ...
        "Unknown option fields: %s.", strjoin(unknownNames, ", "));
end
for name = overrideNames.'
    if ~isempty(optionOverrides.(name))
        options.(name) = optionOverrides.(name);
    end
end
options.FigureVisible = lower(string(options.FigureVisible));
if ~isscalar(options.FigureVisible) || ...
        ~any(options.FigureVisible == ["on", "off"])
    error("explainAzElPlannerWalkthrough:InvalidVisibility", ...
        "FigureVisible must be 'on' or 'off'.");
end
options.SaveFigures = azElInternal.normalizeLogicalScalar( ...
    options.SaveFigures, "SaveFigures", ...
    "explainAzElPlannerWalkthrough:InvalidSaveFigures");
options.OutputDirectory = string(options.OutputDirectory);
if ~isscalar(options.OutputDirectory) || strlength(options.OutputDirectory) == 0
    error("explainAzElPlannerWalkthrough:InvalidOutputDirectory", ...
        "OutputDirectory must be nonempty scalar text.");
end
validateattributes(options.RandomSeed, {'numeric'}, ...
    {'real', 'finite', 'scalar', 'integer'});
validateattributes(options.ExportResolution_dpi, {'numeric'}, ...
    {'real', 'finite', 'scalar', 'integer', 'positive'});
validScenarioSource = isa(options.ScenarioSource, "function_handle") || ...
    (isstruct(options.ScenarioSource) && isscalar(options.ScenarioSource)) || ...
    (isstring(options.ScenarioSource) && ...
    isscalar(options.ScenarioSource)) || ischar(options.ScenarioSource);
if ~validScenarioSource
    error("explainAzElPlannerWalkthrough:InvalidScenarioSource", ...
        "ScenarioSource must be a function, function name, or result struct.");
end
if ~isstruct(options.ScenarioOverrides) || ...
        ~isscalar(options.ScenarioOverrides)
    error("explainAzElPlannerWalkthrough:InvalidScenarioOverrides", ...
        "ScenarioOverrides must be a scalar struct.");
end
end


function [tabHandle, layoutHandle] = createWalkthroughTab( ...
        tabGroupHandle, tabTitle, layoutSize)
%% Section 0: Header & Readme
% SYNTAX
%   [tabHandle, layoutHandle] = createWalkthroughTab( ...
%       tabGroupHandle, tabTitle, layoutSize)
%**************************************************************************
% PURPOSE
%   - Create one explanatory tab and its tiled layout.
%**************************************************************************
% INPUTS
%   - tabGroupHandle (scalar tab group): owning interactive container.
%   - tabTitle (scalar text): displayed tab label.
%   - layoutSize (1-by-2 numeric): tile row and column counts.
%**************************************************************************
% OUTPUTS
%   - tabHandle, layoutHandle: MATLAB graphics handles.
%**************************************************************************
% UNITS
%   - None.
%**************************************************************************

tabHandle = uitab(tabGroupHandle, "Title", tabTitle);
layoutHandle = tiledlayout(tabHandle, layoutSize(1), layoutSize(2), ...
    "TileSpacing", "compact", "Padding", "loose");
end


function configureSpatialAxes(axesHandle, plotBounds_deg)
%% Section 0: Header & Readme
% SYNTAX
%   configureSpatialAxes(axesHandle, plotBounds_deg)
%**************************************************************************
% PURPOSE
%   - Apply the shared spatial-axis style and degree bounds.
%**************************************************************************
% INPUTS
%   - axesHandle (scalar axes): explicit drawing target.
%   - plotBounds_deg (1-by-4): [xmin xmax ymin ymax].
%**************************************************************************
% OUTPUTS
%   - None.
%**************************************************************************
% UNITS
%   - Bounds and labels are degrees.
%**************************************************************************

hold(axesHandle, "on");
grid(axesHandle, "on");
box(axesHandle, "on");
axis(axesHandle, "equal");
xlim(axesHandle, plotBounds_deg(1:2));
ylim(axesHandle, plotBounds_deg(3:4));
xlabel(axesHandle, "Azimuth (deg)");
ylabel(axesHandle, "Elevation (deg)");
end


function drawProblemGeometry(axesHandle, originalShape, protectedShape, ...
        initial_deg, goal_deg, plotBounds_deg, palette)
%% Section 0: Header & Readme
% SYNTAX
%   drawProblemGeometry(axesHandle, originalShape, protectedShape, ...
%       initial_deg, goal_deg, plotBounds_deg, palette)
%**************************************************************************
% PURPOSE
%   - Draw original/protected obstacle geometry, start, and goal.
%**************************************************************************
% INPUTS
%   - axesHandle, originalShape, protectedShape: graphics and polyshapes.
%   - initial_deg, goal_deg: 1-by-2 endpoint positions.
%   - plotBounds_deg: 1-by-4 spatial bounds.
%   - palette: scalar color struct.
%**************************************************************************
% OUTPUTS
%   - None.
%**************************************************************************
% UNITS
%   - Geometry and endpoints are degrees.
%**************************************************************************

configureSpatialAxes(axesHandle, plotBounds_deg);
if ~isempty(originalShape.Vertices)
    plot(axesHandle, originalShape, ...
        "FaceColor", palette.LightBlue, "FaceAlpha", 0.62, ...
        "EdgeColor", palette.Navy, "LineWidth", 1.4, ...
        "DisplayName", "Original obstacle");
end
if ~isempty(protectedShape.Vertices)
    plot(axesHandle, protectedShape, ...
        "FaceColor", "none", "EdgeColor", palette.Blue, ...
        "LineStyle", "--", "LineWidth", 1.4, ...
        "DisplayName", "Protected obstacle");
end
scatter(axesHandle, initial_deg(1), initial_deg(2), ...
    44, palette.Green, "filled", "DisplayName", "Start");
scatter(axesHandle, goal_deg(1), goal_deg(2), ...
    44, palette.Red, "filled", "DisplayName", "Goal");
end


function drawSeed(axesHandle, seed, selected, palette)
%% Section 0: Header & Readme
% SYNTAX
%   drawSeed(axesHandle, seed, selected, palette)
%**************************************************************************
% PURPOSE
%   - Draw one candidate route with selected-state emphasis.
%**************************************************************************
% INPUTS
%   - axesHandle (scalar axes), seed (scalar seed struct).
%   - selected (logical scalar), palette (color struct).
%**************************************************************************
% OUTPUTS
%   - None.
%**************************************************************************
% UNITS
%   - Seed route coordinates are degrees.
%**************************************************************************

if selected
    routeColor = palette.Cyan;
    lineWidth = 2.8;
else
    routeColor = palette.Gray;
    lineWidth = 1.8;
end
plot(axesHandle, seed.position_deg(:, 1), seed.position_deg(:, 2), ...
    "-o", "Color", routeColor, "LineWidth", lineWidth, ...
    "MarkerFaceColor", routeColor, "DisplayName", "Candidate seed");
if selected
    text(axesHandle, 0.03, 0.96, "SELECTED", ...
        "Units", "normalized", "VerticalAlignment", "top", ...
        "FontWeight", "bold", "Color", palette.Green);
end
end


function drawSearchEdges(axesHandle, gridRecord, palette)
%% Section 0: Header & Readme
% SYNTAX
%   drawSearchEdges(axesHandle, gridRecord, palette)
%**************************************************************************
% PURPOSE
%   - Draw accepted and rejected visibility tests from returned diagnostics.
%**************************************************************************
% INPUTS
%   - axesHandle (scalar axes), gridRecord (search diagnostic struct).
%   - palette (scalar color struct).
%**************************************************************************
% OUTPUTS
%   - None.
%**************************************************************************
% UNITS
%   - Edge endpoint coordinates are degrees.
%**************************************************************************

if ~isempty(gridRecord.AcceptedEdges_deg)
    [azimuth_deg, elevation_deg] = edgeLineData( ...
        gridRecord.AcceptedEdges_deg);
    plot(axesHandle, azimuth_deg, elevation_deg, "-", ...
        "Color", palette.Cyan, "LineWidth", 0.6, ...
        "DisplayName", "Accepted edge");
end
if ~isempty(gridRecord.RejectedEdges_deg)
    [azimuth_deg, elevation_deg] = edgeLineData( ...
        gridRecord.RejectedEdges_deg);
    plot(axesHandle, azimuth_deg, elevation_deg, ":", ...
        "Color", palette.Red, "LineWidth", 0.6, ...
        "DisplayName", "Rejected edge");
end
if ~isempty(gridRecord.ExploredNodes_deg)
    scatter(axesHandle, gridRecord.ExploredNodes_deg(:, 1), ...
        gridRecord.ExploredNodes_deg(:, 2), 12, palette.Gray, "filled", ...
        "DisplayName", "Expanded node");
end
end


function [azimuth_deg, elevation_deg] = edgeLineData(edges_deg)
%% Section 0: Header & Readme
% SYNTAX
%   [azimuth_deg, elevation_deg] = edgeLineData(edges_deg)
%**************************************************************************
% PURPOSE
%   - Convert N-by-4 edge endpoints to NaN-separated plot vectors.
%**************************************************************************
% INPUTS
%   - edges_deg (N-by-4 double): [az1 el1 az2 el2].
%**************************************************************************
% OUTPUTS
%   - azimuth_deg, elevation_deg: NaN-separated column vectors.
%**************************************************************************
% UNITS
%   - All values are degrees.
%**************************************************************************

edgeCount = size(edges_deg, 1);
azimuth_deg = reshape( ...
    [edges_deg(:, 1), edges_deg(:, 3), nan(edgeCount, 1)].', [], 1);
elevation_deg = reshape( ...
    [edges_deg(:, 2), edges_deg(:, 4), nan(edgeCount, 1)].', [], 1);
end


function drawConvexRegions(axesHandle, convexRegions)
%% Section 0: Header & Readme
% SYNTAX
%   drawConvexRegions(axesHandle, convexRegions)
%**************************************************************************
% PURPOSE
%   - Draw each exact convex occupied region with a distinct color.
%**************************************************************************
% INPUTS
%   - axesHandle (scalar axes), convexRegions (polyshape array).
%**************************************************************************
% OUTPUTS
%   - None.
%**************************************************************************
% UNITS
%   - Polyshape coordinates are degrees.
%**************************************************************************

regionColors = lines(max(1, numel(convexRegions)));
for regionIndex = 1:numel(convexRegions)
    plot(axesHandle, convexRegions(regionIndex), ...
        "FaceColor", regionColors(regionIndex, :), ...
        "FaceAlpha", 0.38, "EdgeColor", regionColors(regionIndex, :), ...
        "LineWidth", 0.8, "HandleVisibility", "off");
end
end


function drawSupportRecords(axesHandle, corridor, segmentIndex, ...
        plotBounds_deg, palette)
%% Section 0: Header & Readme
% SYNTAX
%   drawSupportRecords(axesHandle, corridor, segmentIndex, ...
%       plotBounds_deg, palette)
%**************************************************************************
% PURPOSE
%   - Draw clearance-offset constraint lines and outward normals for one span.
%**************************************************************************
% INPUTS
%   - axesHandle (scalar axes), corridor (support record array).
%   - segmentIndex (positive integer), plotBounds_deg (1-by-4).
%   - palette (scalar color struct).
%**************************************************************************
% OUTPUTS
%   - None.
%**************************************************************************
% UNITS
%   - Boundary offsets and plotting coordinates are degrees.
%**************************************************************************

recordIndex = find([corridor.SegmentIndex] == segmentIndex);
lineHalfLength_deg = hypot( ...
    diff(plotBounds_deg(1:2)), diff(plotBounds_deg(3:4)));
for index = reshape(recordIndex, 1, [])
    record = corridor(index);
    normal = record.Normal;
    tangent = [-normal(2), normal(1)];
    threshold_deg = record.BoundaryOffset_deg + record.Clearance_deg;
    anchor_deg = threshold_deg * normal;
    linePoint_deg = [ ...
        anchor_deg - lineHalfLength_deg * tangent; ...
        anchor_deg + lineHalfLength_deg * tangent];
    plot(axesHandle, linePoint_deg(:, 1), linePoint_deg(:, 2), ...
        "-", "Color", palette.Green, "LineWidth", 0.8, ...
        "HandleVisibility", "off");
    quiver(axesHandle, anchor_deg(1), anchor_deg(2), ...
        normal(1), normal(2), 0.9, "Color", palette.Red, ...
        "LineWidth", 0.8, "MaxHeadSize", 0.8, ...
        "HandleVisibility", "off");
end
end


function drawSingleHalfspaceExplanation(axesHandle, region, record, ...
        supportPoint_deg, seedPoint_deg, plotBounds_deg, palette)
%% Section 0: Header & Readme
% SYNTAX
%   drawSingleHalfspaceExplanation(axesHandle, region, record, ...
%       supportPoint_deg, seedPoint_deg, plotBounds_deg, palette)
%**************************************************************************
% PURPOSE
%   - Explain one convex support, its normal, the supporting line, and the
%     clearance-offset free half-space retained for a trajectory span.
%**************************************************************************
% INPUTS
%   - axesHandle (scalar axes), region (scalar convex polyshape).
%   - record (scalar corridor support record).
%   - supportPoint_deg, seedPoint_deg (1-by-2 positions).
%   - plotBounds_deg (1-by-4 bounds), palette (color struct).
%**************************************************************************
% OUTPUTS
%   - None.
%**************************************************************************
% UNITS
%   - Positions, offsets, clearance, and plot bounds are degrees.
%**************************************************************************

normal = record.Normal;
boundaryOffset_deg = record.BoundaryOffset_deg;
clearance_deg = record.Clearance_deg;
constraintOffset_deg = boundaryOffset_deg + clearance_deg;
tangent = [-normal(2), normal(1)];
lineHalfLength_deg = hypot( ...
    diff(plotBounds_deg(1:2)), diff(plotBounds_deg(3:4)));
displayRectangle_deg = [ ...
    plotBounds_deg(1), plotBounds_deg(3); ...
    plotBounds_deg(2), plotBounds_deg(3); ...
    plotBounds_deg(2), plotBounds_deg(4); ...
    plotBounds_deg(1), plotBounds_deg(4)];
freeHalfspace_deg = clipPolygonHalfspace( ...
    displayRectangle_deg, normal, constraintOffset_deg);
if ~isempty(freeHalfspace_deg)
    patch(axesHandle, freeHalfspace_deg(:, 1), freeHalfspace_deg(:, 2), ...
        palette.Green, "FaceAlpha", 0.10, "EdgeColor", "none", ...
        "DisplayName", "Retained free half-space");
end
plot(axesHandle, region, "FaceColor", palette.Red, ...
    "FaceAlpha", 0.16, "EdgeColor", palette.Red, ...
    "LineWidth", 1.0, "DisplayName", "Convex occupied region");
supportAnchor_deg = boundaryOffset_deg * normal;
supportLine_deg = [ ...
    supportAnchor_deg - lineHalfLength_deg * tangent; ...
    supportAnchor_deg + lineHalfLength_deg * tangent];
plot(axesHandle, supportLine_deg(:, 1), supportLine_deg(:, 2), "--", ...
    "Color", palette.Red, "LineWidth", 1.5, ...
    "DisplayName", "Supporting line");
constraintAnchor_deg = constraintOffset_deg * normal;
constraintLine_deg = [ ...
    constraintAnchor_deg - lineHalfLength_deg * tangent; ...
    constraintAnchor_deg + lineHalfLength_deg * tangent];
plot(axesHandle, constraintLine_deg(:, 1), constraintLine_deg(:, 2), ...
    "-", "Color", palette.Green, "LineWidth", 1.8, ...
    "DisplayName", "Clearance-offset constraint");
plot(axesHandle, [supportPoint_deg(1), seedPoint_deg(1)], ...
    [supportPoint_deg(2), seedPoint_deg(2)], "-", ...
    "Color", palette.Navy, "LineWidth", 1.2, ...
    "HandleVisibility", "off");
quiver(axesHandle, supportPoint_deg(1), supportPoint_deg(2), ...
    normal(1), normal(2), 1.5, "Color", palette.Navy, ...
    "LineWidth", 1.4, "MaxHeadSize", 0.8, ...
    "DisplayName", "Outward normal n");
scatter(axesHandle, supportPoint_deg(1), supportPoint_deg(2), ...
    42, palette.Red, "filled", "DisplayName", "Support point p");
scatter(axesHandle, seedPoint_deg(1), seedPoint_deg(2), ...
    52, palette.Gold, "filled", "DisplayName", "Seed midpoint s");
equationText = sprintf([ ...
    '$\\mathbf{n}=(\\mathbf{s}-\\mathbf{p})/' ...
    '\\|\\mathbf{s}-\\mathbf{p}\\|$\n' ...
    'support: $\\mathbf{n}^{T}\\mathbf{y}=b$\n' ...
    'keep: $\\mathbf{n}^{T}\\mathbf{y}\\geq b+\\rho$']);
text(axesHandle, 0.03, 0.97, equationText, "Units", "normalized", ...
    "VerticalAlignment", "top", "Interpreter", "latex", ...
    "FontSize", 8.5, "Color", palette.Navy, ...
    "BackgroundColor", "w", "EdgeColor", palette.Gray, "Margin", 4);
end


function safeCell_deg = safeCellForSegment( ...
        corridor, segmentIndex, plotBounds_deg)
%% Section 0: Header & Readme
% SYNTAX
%   safeCell_deg = safeCellForSegment(corridor, segmentIndex, plotBounds_deg)
%**************************************************************************
% PURPOSE
%   - Clip the display rectangle by every free-side inequality for one span.
%**************************************************************************
% INPUTS
%   - corridor (support record array), segmentIndex (positive integer).
%   - plotBounds_deg (1-by-4): clipping rectangle.
%**************************************************************************
% OUTPUTS
%   - safeCell_deg (N-by-2 double): clipped convex display polygon.
%**************************************************************************
% UNITS
%   - Coordinates and offsets are degrees.
%**************************************************************************

safeCell_deg = [ ...
    plotBounds_deg(1), plotBounds_deg(3); ...
    plotBounds_deg(2), plotBounds_deg(3); ...
    plotBounds_deg(2), plotBounds_deg(4); ...
    plotBounds_deg(1), plotBounds_deg(4)];
recordIndex = find([corridor.SegmentIndex] == segmentIndex);
for index = reshape(recordIndex, 1, [])
    record = corridor(index);
    threshold_deg = record.BoundaryOffset_deg + record.Clearance_deg;
    safeCell_deg = clipPolygonHalfspace( ...
        safeCell_deg, record.Normal, threshold_deg);
    if isempty(safeCell_deg)
        return;
    end
end
end


function outputPolygon_deg = clipPolygonHalfspace( ...
        inputPolygon_deg, normal, threshold_deg)
%% Section 0: Header & Readme
% SYNTAX
%   outputPolygon_deg = clipPolygonHalfspace( ...
%       inputPolygon_deg, normal, threshold_deg)
%**************************************************************************
% PURPOSE
%   - Apply one Sutherland-Hodgman clip for normal*x >= threshold.
%**************************************************************************
% INPUTS
%   - inputPolygon_deg (N-by-2), normal (1-by-2), threshold_deg (scalar).
%**************************************************************************
% OUTPUTS
%   - outputPolygon_deg (M-by-2): retained polygon vertices.
%**************************************************************************
% UNITS
%   - Polygon coordinates and threshold are degrees; normal is unitless.
%**************************************************************************

if isempty(inputPolygon_deg)
    outputPolygon_deg = zeros(0, 2);
    return;
end
vertexCount = size(inputPolygon_deg, 1);
outputPolygon_deg = zeros(0, 2);
tolerance_deg = 1e-10;
for vertexIndex = 1:vertexCount
    nextIndex = mod(vertexIndex, vertexCount) + 1;
    current_deg = inputPolygon_deg(vertexIndex, :);
    next_deg = inputPolygon_deg(nextIndex, :);
    currentDistance_deg = current_deg * normal.' - threshold_deg;
    nextDistance_deg = next_deg * normal.' - threshold_deg;
    currentInside = currentDistance_deg >= -tolerance_deg;
    nextInside = nextDistance_deg >= -tolerance_deg;
    if currentInside && nextInside
        outputPolygon_deg(end + 1, :) = next_deg; %#ok<AGROW>
    elseif currentInside && ~nextInside
        interpolation = currentDistance_deg / ...
            (currentDistance_deg - nextDistance_deg);
        outputPolygon_deg(end + 1, :) = current_deg + ...
            interpolation * (next_deg - current_deg); %#ok<AGROW>
    elseif ~currentInside && nextInside
        interpolation = currentDistance_deg / ...
            (currentDistance_deg - nextDistance_deg);
        outputPolygon_deg(end + 1, :) = current_deg + ...
            interpolation * (next_deg - current_deg); %#ok<AGROW>
        outputPolygon_deg(end + 1, :) = next_deg; %#ok<AGROW>
    end
end
end


function signedClearance_deg = signedClearanceHistory( ...
        obstacles, time_s, position_deg)
%% Section 0: Header & Readme
% SYNTAX
%   signedClearance_deg = signedClearanceHistory( ...
%       obstacles, time_s, position_deg)
%**************************************************************************
% PURPOSE
%   - Measure sampled signed distance to protected geometry at physical time.
%**************************************************************************
% INPUTS
%   - obstacles (prepared obstacle array), time_s (N-by-1 double).
%   - position_deg (N-by-2 double).
%**************************************************************************
% OUTPUTS
%   - signedClearance_deg (N-by-1): negative inside, positive outside.
%**************************************************************************
% UNITS
%   - Positions and clearance are degrees.
%**************************************************************************

sampleCount = size(position_deg, 1);
signedClearance_deg = zeros(sampleCount, 1);
for sampleIndex = 1:sampleCount
    [~, protectedShape] = combinedObstacleShapesAtTime( ...
        obstacles, time_s(sampleIndex));
    if isempty(protectedShape.Vertices)
        signedClearance_deg(sampleIndex) = inf;
        continue;
    end
    clearance_deg = azElInternal.geometry.pointPolygonClearance( ...
        protectedShape, position_deg(sampleIndex, :));
    if isinterior(protectedShape, position_deg(sampleIndex, 1), ...
            position_deg(sampleIndex, 2))
        clearance_deg = -clearance_deg;
    end
    signedClearance_deg(sampleIndex) = clearance_deg;
end
end


function maximumResidual = polynomialContinuityResidual(polynomial)
%% Section 0: Header & Readme
% SYNTAX
%   maximumResidual = polynomialContinuityResidual(polynomial)
%**************************************************************************
% PURPOSE
%   - Measure position-through-jerk jumps at every interior polynomial knot.
%**************************************************************************
% INPUTS
%   - polynomial (scalar planner polynomial struct).
%**************************************************************************
% OUTPUTS
%   - maximumResidual (1-by-4): maxima for position, velocity,
%     acceleration, and jerk.
%**************************************************************************
% UNITS
%   - Entries use deg, deg/s, deg/s^2, and deg/s^3, respectively.
%**************************************************************************

arrays = { ...
    polynomial.positionPower_deg, ...
    polynomial.velocityPower_deg_s, ...
    polynomial.accelerationPower_deg_s2, ...
    polynomial.jerkPower_deg_s3};
maximumResidual = zeros(1, 4);
for segmentIndex = 1:polynomial.SegmentCount - 1
    for derivativeIndex = 1:numel(arrays)
        leftValue = sum(reshape( ...
            arrays{derivativeIndex}(segmentIndex, :, :), 2, []), 2).';
        rightValue = reshape( ...
            arrays{derivativeIndex}(segmentIndex + 1, :, 1), 1, 2);
        maximumResidual(derivativeIndex) = max( ...
            maximumResidual(derivativeIndex), ...
            max(abs(leftValue - rightValue)));
    end
end
maximumResidual = max(maximumResidual, realmin("double"));
end


function figureFiles = exportWalkthroughFigure( ...
        figureHandle, fileStem, options, figureFiles)
%% Section 0: Header & Readme
% SYNTAX
%   figureFiles = exportWalkthroughFigure( ...
%       figureHandle, fileStem, options, figureFiles)
%**************************************************************************
% PURPOSE
%   - Optionally export one complete walkthrough figure as a PNG.
%**************************************************************************
% INPUTS
%   - figureHandle (scalar figure), fileStem (scalar text).
%   - options (resolved controls), figureFiles (string column).
%**************************************************************************
% OUTPUTS
%   - figureFiles (string column): updated list of exported files.
%**************************************************************************
% UNITS
%   - Export resolution is dots per inch.
%**************************************************************************

if ~options.SaveFigures
    return;
end
filePath = fullfile(options.OutputDirectory, string(fileStem) + ".png");
exportgraphics(figureHandle, filePath, ...
    "Resolution", options.ExportResolution_dpi);
figureFiles(end + 1, 1) = filePath;
end
