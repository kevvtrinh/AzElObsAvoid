function tests = testCorePlots
% Check folded paths, timed markers, graph nodes, and obstacle copies.
% Run with runtests('tests/testCorePlots.m').
tests = functiontests(localfunctions);
end

function setupOnce(~)
    rootFolder = fileparts(fileparts(mfilename('fullpath')));
    addpath(rootFolder, fullfile(rootFolder, 'trajectory'), fullfile(rootFolder, 'examples'));
end

function testWrappedPathFoldingContracts(testCase)
    runCases(testCase, { ...
        @wrappedYDisplayFoldsBackOverThePole; ...
        @poleDisplayKeepsBoundarySamplesWithTheirSegments});
end

function testTrajectoryMarkerAndGraphPlotContracts(testCase)
    limits = sphereLimits();
    plotOptions = struct('FigureVisible', "off", 'ShowKinematics', false, ...
        'ShowAnimation', false, 'ShowVisibilityGraphs', false, ...
        'ShowSpaceTime', false, 'ShowExpandedWorkspace', false);
    runCases(testCase, { ...
        @(testCase) poleDisplayMarkersFollowTimedPath(testCase, limits, plotOptions); ...
        @(testCase) poleMovingTargetMarkerFollowsItsTrack(testCase, limits, plotOptions); ...
        @(testCase) visibilityNodesOnASeamStayOnTheirEdges(testCase, limits); ...
        @(testCase) expandedPlotDrawsTargetOverThePlannedWindow(testCase, limits)});
end

function testObstacleAndExpandedPlotContracts(testCase)
    auditLimits = struct('xInterval_units', [-10, 10], 'yInterval_units', [-10, 10], ...
        'maxVelocity_units_s', [2, 2], 'maxAcceleration_units_s2', [2, 2], ...
        'maxJerk_units_s3', [4, 4]);
    limits = sphereLimits();
    rowLimits = struct('xInterval_units', [0, 360], 'yInterval_units', [-90, 90], ...
        'maxVelocity_units_s', [5, 1], 'maxAcceleration_units_s2', [5, 2], ...
        'maxJerk_units_s3', [10, 4]);
    runCases(testCase, { ...
        @(testCase) galleryPlotsPartiallyPreparedFailureResult(testCase, auditLimits); ...
        @wrappedPlotUsesTiedObstacleSnapshots; ...
        @(testCase) expandedWorkspacePlotShowsWrappedCopies(testCase, limits); ...
        @(testCase) expandedPlotAcceptsUnequalRowHistories(testCase, rowLimits); ...
        @(testCase) expandedPlotStylesCopiesAfterDroppingOutlyingRing(testCase, rowLimits)});
end

function wrappedYDisplayFoldsBackOverThePole(testCase)
    % A path from (10, 85) over the top pole to (10, 95) is shown as
    % (10, 85) -> (10, 90), then from (190, 90) down to (190, 85): the pole
    % copy of (10, 95) is (190, 85). A NaN row separates the two runs.
    displayPath_units = obstacleAvoidance.plotting.createWrappedSpatialPath( ...
        [10, 85; 10, 95], [0, 360; -90, 90], ["false", "both"]);
    verifyEqual(testCase, displayPath_units, [10, 85; 10, 90; NaN, NaN; 190, 90; 190, 85], 'AbsTol', 1e-12);
    % Modes are read like the planner's: any case, and only the four names.
    verifyEqual(testCase, obstacleAvoidance.plotting.createWrappedSpatialPath( ...
        [10, 85; 10, 95], [0, 360; -90, 90], ["False", "BOTH"]), displayPath_units, 'AbsTol', 1e-12);
    verifyError(testCase, @() obstacleAvoidance.plotting.createWrappedSpatialPath( ...
        [10, 85; 10, 95], [0, 360; -90, 90], ["false", "sideways"]), 'createWrappedSpatialPath:InvalidWrapMode');
end

function poleDisplayKeepsBoundarySamplesWithTheirSegments(testCase)
    % Touching the upper pole stays in the same display run. Crossing at a
    % recorded sample changes azimuth by half a turn and breaks exactly once.
    intervals_units = [0, 360; -90, 90];
    wrapModes = ["false", "both"];
    touching = obstacleAvoidance.plotting.createWrappedSpatialPath( ...
        [10, 89; 11, 90; 12, 89], intervals_units, wrapModes);
    verifyEqual(testCase, touching, [10, 89; 11, 90; 12, 89], 'AbsTol', 1e-12);
    starting = obstacleAvoidance.plotting.createWrappedSpatialPath( ...
        [10, 90; 10, 89], intervals_units, wrapModes);
    verifyEqual(testCase, starting, [10, 90; 10, 89], 'AbsTol', 1e-12);
    ending = obstacleAvoidance.plotting.createWrappedSpatialPath( ...
        [10, 89; 10, 90], intervals_units, wrapModes);
    verifyEqual(testCase, ending, [10, 89; 10, 90], 'AbsTol', 1e-12);
    xEnding = obstacleAvoidance.plotting.createWrappedSpatialPath( ...
        [360, 0; 359, 0], intervals_units, wrapModes);
    verifyEqual(testCase, xEnding, [360, 0; 359, 0], 'AbsTol', 1e-12);
    multiplePoles = obstacleAvoidance.plotting.createWrappedSpatialPath( ...
        [10, 90; 10, 450], intervals_units, wrapModes);
    verifyEqual(testCase, multiplePoles, [190, 90; 190, -90; NaN, NaN; 10, -90; 10, 90], 'AbsTol', 1e-12);
    [crossing, sampleIndices] = obstacleAvoidance.plotting.createWrappedSpatialPath( ...
        [10, 89; 11, 90; 12, 91], intervals_units, wrapModes);
    verifyEqual(testCase, crossing, [10, 89; 11, 90; NaN, NaN; 191, 90; 192, 89], 'AbsTol', 1e-12);
    verifyEqual(testCase, sampleIndices, [1; 2; 3; 3; 3]);
    verifyEqual(testCase, nnz(isnan(crossing(:, 1))), 1);

    % A wait exactly on a pole, or a run along it, keeps its neighbor's
    % copy: no jump to x = 190 and no break. The same holds at the lower
    % pole and for a wait exactly on the x seam.
    waiting = obstacleAvoidance.plotting.createWrappedSpatialPath( ...
        [10, 89; 10, 90; 10, 90; 10, 89], intervals_units, wrapModes);
    verifyEqual(testCase, waiting, [10, 89; 10, 90; 10, 90; 10, 89], 'AbsTol', 1e-12);
    alongPole = obstacleAvoidance.plotting.createWrappedSpatialPath( ...
        [10, 90; 12, 90; 12, 89], intervals_units, wrapModes);
    verifyEqual(testCase, alongPole, [10, 90; 12, 90; 12, 89], 'AbsTol', 1e-12);
    lowerWait = obstacleAvoidance.plotting.createWrappedSpatialPath( ...
        [10, -89; 10, -90; 10, -90; 10, -89], intervals_units, wrapModes);
    verifyEqual(testCase, lowerWait, [10, -89; 10, -90; 10, -90; 10, -89], 'AbsTol', 1e-12);
    seamWait = obstacleAvoidance.plotting.createWrappedSpatialPath( ...
        [360, 0; 360, 0; 359, 0], intervals_units, ["both", "false"]);
    verifyEqual(testCase, seamWait, [360, 0; 360, 0; 359, 0], 'AbsTol', 1e-12);

    % A wait on the x seam keeps its own y band, so a pole crossing right
    % after it still breaks once, at either pole.
    seamThenPole = obstacleAvoidance.plotting.createWrappedSpatialPath( ...
        [359, 89; 360, 89; 360, 91], intervals_units, wrapModes);
    verifyEqual(testCase, seamThenPole, [359, 89; 360, 89; 360, 90; NaN, NaN; 180, 90; 180, 89], 'AbsTol', 1e-12);
    seamThenLowerPole = obstacleAvoidance.plotting.createWrappedSpatialPath( ...
        [359, -89; 360, -89; 360, -91], intervals_units, wrapModes);
    verifyEqual(testCase, seamThenLowerPole, [359, -89; 360, -89; 360, -90; NaN, NaN; 180, -90; 180, -89], 'AbsTol', 1e-12);

    % A long wait on the pole between two ordinary samples borrows from the
    % piece before it, sample by sample, with no jump and no break.
    waitCount = 5000;
    longWait = [10, 89; repmat([10, 90], waitCount, 1); 10, 89];
    verifyEqual(testCase, obstacleAvoidance.plotting.createWrappedSpatialPath( ...
        longWait, intervals_units, wrapModes), longWait, 'AbsTol', 1e-12);
end

function poleDisplayMarkersFollowTimedPath(testCase, limits, plotOptions)
    existingFigures = findall(groot, 'Type', 'figure');
    figureCleanup = onCleanup(@() closeCreatedFigures(existingFigures));
    % Start and terminal markers use the first and last folded path points.
    % A standalone fold of y = 90 would put either marker half a turn away.
    endpointPairs_units = [10, 90, 12, 89; 10, 89, 12, 90];
    for pairIndex = 1:size(endpointPairs_units, 1)
        endpointPair_units = endpointPairs_units(pairIndex, :);
        result = planner([], state(0, endpointPair_units(1:2)), ...
            state(10, endpointPair_units(3:4)), limits, ...
            struct('GoalTimeMode', 'fixedArrival', 'WrapY', "both"));
        verifyTrue(testCase, result.Success, result.Message);
        verifyTrue(testCase, obstacleAvoidance.validateTrajectory(result).Passed);
        displayPath_units = obstacleAvoidance.plotting.createWrappedSpatialPath( ...
            result.position_units, [0, 360; -90, 90], ["false", "both"]);
        handles = obstacleAvoidance.plotting.plotTrajectory(result, plotOptions);
        startHandle = findobj(handles.WorkspaceAxes, 'DisplayName', 'Start');
        goalHandle = findobj(handles.WorkspaceAxes, 'DisplayName', 'Goal');
        verifyEqual(testCase, [startHandle.XData, startHandle.YData], displayPath_units(1, :), 'AbsTol', 1e-9);
        verifyEqual(testCase, [goalHandle.XData, goalHandle.YData], displayPath_units(end, :), 'AbsTol', 1e-9);
        if pairIndex == 2
            plotOptions.ShowWorkspace = false;
            plotOptions.ShowAnimation = true;
            plotOptions.FrameStride = 1000;
            plotOptions.Pause_s = 0;
            animationHandles = obstacleAvoidance.plotting.plotTrajectory(result, plotOptions);
            currentHandle = findobj(animationHandles.AnimationAxes, 'DisplayName', 'Current state');
            verifyEqual(testCase, [currentHandle.XData, currentHandle.YData], ...
                displayPath_units(end, :), 'AbsTol', 1e-9);
        end
    end
end

function poleMovingTargetMarkerFollowsItsTrack(testCase, limits, plotOptions)
    existingFigures = findall(groot, 'Type', 'figure');
    figureCleanup = onCleanup(@() closeCreatedFigures(existingFigures));
    % A target starts at the upper pole and moves below it. The workspace
    % marker uses the track's first piece, which shows (20, 90), not (200, 90).
    targetMotion = struct('time_s', [0; 10], ...
        'position_units', [20, 90; 20, 89], 'InterpolationMethod', 'linear');
    goal = struct('time_s', 10, 'targetMotion', targetMotion);
    result = planner([], state(0, [10, 89]), goal, limits, ...
        struct('GoalTimeMode', 'fixedArrival', 'WrapY', "both"));
    verifyTrue(testCase, result.Success, result.Message);
    verifyTrue(testCase, obstacleAvoidance.validateTrajectory(result).Passed);
    handles = obstacleAvoidance.plotting.plotTrajectory(result, plotOptions);
    targetHandle = findobj(handles.WorkspaceAxes, 'DisplayName', 'Moving target');
    verifyEqual(testCase, [targetHandle.XData, targetHandle.YData], [20, 90], 'AbsTol', 1e-9);
end

function visibilityNodesOnASeamStayOnTheirEdges(testCase, limits)
    existingFigures = findall(groot, 'Type', 'figure');
    figureCleanup = onCleanup(@() closeCreatedFigures(existingFigures));
    % A graph node exactly at x = 360 (a triangle's apex whose other
    % vertices, the start and the goal all lie left of it) is folded to the
    % side its edge is drawn on: x = 360, not x = 0 as a lone fold gives.
    triangle = struct('Vertices_units', [350, -2; 360, 3; 350, 8]);
    result = planner(triangle, state(0, [340, 0]), state(20, [330, 12]), limits, ...
        struct('GoalTimeMode', 'fixedArrival', 'WrapX', "both"));
    verifyTrue(testCase, result.Success, result.Message);
    verifyTrue(testCase, any(abs(result.Diagnostics.VisibilityGraph.NodePosition_units(:, 1) - 360) < 1e-9));
    handles = obstacleAvoidance.plotting.plotTrajectory(result, struct('FigureVisible', 'off', ...
        'ShowVisibilityGraphs', true, 'ShowSearchEdges', true));
    nodeMarkers = findobj(handles.VisibilityAxes, 'DisplayName', 'Visibility nodes (sampled)');
    nodePoints_units = [nodeMarkers.XData(:), nodeMarkers.YData(:)];
    % Compare all folded positions, including repeated positions, without
    % relying on scatter order; these graphs fit below the 80-node cap.
    % Each node follows its first incident edge (accepted rows first),
    % folded as the edges are drawn, to the edge's endpoint.
    graph = result.Diagnostics.VisibilityGraph;
    nodes_units = graph.NodePosition_units;
    edges = [graph.AcceptedNodeIndex; graph.RejectedNodeIndex];
    expectedNodePoints_units = zeros(size(nodes_units));
    for nodeIndex = 1:size(nodes_units, 1)
        edgeRow = find(any(edges == nodeIndex, 2), 1);
        verifyNotEmpty(testCase, edgeRow);
        otherNode = edges(edgeRow, edges(edgeRow, :) ~= nodeIndex);
        if isempty(otherNode)
            otherNode = nodeIndex;
        end
        foldedEdge_units = obstacleAvoidance.plotting.createWrappedSpatialPath( ...
            nodes_units([otherNode(1), nodeIndex], :), [0, 360; -90, 90], ["both", "false"]);
        expectedNodePoints_units(nodeIndex, :) = foldedEdge_units(end, :);
    end
    verifyEqual(testCase, sortrows(nodePoints_units), sortrows(expectedNodePoints_units), 'AbsTol', 1e-9);
    verifyTrue(testCase, any(abs(nodePoints_units(:, 1) - 360) < 1e-9));

    % Explicit coordinates: nodes at 350, 360 and 370 with an accepted edge
    % 1-2 and a rejected edge 2-3. The seam node follows its accepted edge
    % to x = 360; node 3 follows the rejected edge over the seam to x = 10.
    synthetic = result;
    synthetic.Diagnostics.VisibilityGraph.NodePosition_units = [350, 0; 360, 0; 370, 0];
    synthetic.Diagnostics.VisibilityGraph.AcceptedNodeIndex = [1, 2];
    synthetic.Diagnostics.VisibilityGraph.RejectedNodeIndex = [2, 3];
    syntheticHandles = obstacleAvoidance.plotting.plotTrajectory(synthetic, struct('FigureVisible', 'off', ...
        'ShowVisibilityGraphs', true, 'ShowSearchEdges', true));
    syntheticMarkers = findobj(syntheticHandles.VisibilityAxes, 'DisplayName', 'Visibility nodes (sampled)');
    verifyEqual(testCase, sortrows([syntheticMarkers.XData(:), syntheticMarkers.YData(:)]), ...
        [10, 0; 350, 0; 360, 0], 'AbsTol', 1e-9);
end

function expandedPlotDrawsTargetOverThePlannedWindow(testCase, limits)
    existingFigures = findall(groot, 'Type', 'figure');
    figureCleanup = onCleanup(@() closeCreatedFigures(existingFigures));
    % A target recorded past the deadline is drawn only up to it, the same
    % window its copies are listed for: from 350 at 0 s to 370 at 10 s,
    % not on to 390 at 20 s.
    targetMotion = struct('time_s', [0; 10; 20], 'position_units', [350, 0; 10, 0; 30, 0], ...
        'InterpolationMethod', 'linear');
    result = planner([], state(0, [340, 0]), struct('time_s', 10, 'targetMotion', targetMotion), limits, ...
        struct('GoalTimeMode', 'fixedArrival', 'WrapX', "both"));
    verifyTrue(testCase, result.Success, result.Message);
    handles = obstacleAvoidance.plotting.plotTrajectory(result, struct('FigureVisible', 'off'));
    targetLine = findobj(handles.ExpandedWorkspaceAxes, 'DisplayName', 'Moving target (unwrapped)');
    verifyNotEmpty(testCase, targetLine);
    verifyEqual(testCase, [min(targetLine.XData), max(targetLine.XData)], [350, 370], 'AbsTol', 1e-9);
end

function galleryPlotsPartiallyPreparedFailureResult(testCase, limits)
    existingFigures = findall(groot, 'Type', 'figure');
    figureCleanup = onCleanup(@() closeCreatedFigures(existingFigures));
    collinear_units = [0 0; 0.5 0; 1 0];
    smallSquare = [0 0; 1 0; 1 1; 0 1];
    largeSquare = [0 0; 2 0; 2 2; 0 2];
    obstacle = obstacleAvoidance.obstacles.createObstacle('deforming', [0; 1; 2], ...
        {collinear_units(:, 1); smallSquare(:, 1); largeSquare(:, 1)}, ...
        {collinear_units(:, 2); smallSquare(:, 2); largeSquare(:, 2)}, 0);
    initial = struct('time_s', 0, 'position_units', [-4 0]);
    goal = struct('time_s', 2, 'position_units', [4 0]);
    result = planner(obstacle, initial, goal, limits, struct('GoalTimeMode', 'fixedArrival'));
    verifyFalse(testCase, result.Success);
    verifyEqual(testCase, result.TerminationReason, "unsupportedObstacleInterpolation");
    verifyFalse(testCase, all(result.Diagnostics.PreparedObstacles.InternalPreparation.SamplePrepared));

    figureHandles = obstacleAvoidance.plotting.plotTrajectoryGallery({result}, "unsupported", 'off');
    verifyEqual(testCase, numel(figureHandles), 1);
    axesHandle = findobj(figureHandles(1), 'Type', 'axes');
    verifyTrue(testCase, contains(string(axesHandle(1).Title.String), "unprepared snapshot"));

    % The full trajectory plot, with its default space-time view, shows the
    % interval without a continuous model as a gap instead of throwing.
    handles = obstacleAvoidance.plotting.plotTrajectory(result, struct('FigureVisible', "off"));
    verifyNotEmpty(testCase, handles.SpaceTimeAxes);
end

function wrappedPlotUsesTiedObstacleSnapshots(testCase)
    existingFigures = findall(groot, 'Type', 'figure');
    figureCleanup = onCleanup(@() closeCreatedFigures(existingFigures));
    % The obstacle can be prepared and the motion validated over [0 5] s.
    % It sits near y = 90, outside this request's y reach of [-5 5], but
    % the folded 2D display shows its original and protected snapshots.
    square_units = [-1, -1; 1, -1; 1, 1; -1, 1] + [100, 90];
    diamond_units = [-2, 0; 0, -2; 2, 0; 0, 2] + [100, 90];
    tied = obstacleAvoidance.obstacles.createObstacle('display tie', [0; 5], ...
        {square_units(:, 1); diamond_units(:, 1)}, ...
        {square_units(:, 2); diamond_units(:, 2)}, 0.5);
    limits = struct('xInterval_units', [0, 360], 'yInterval_units', [-90, 90], ...
        'maxVelocity_units_s', [10, 1], 'maxAcceleration_units_s2', [10, 2], ...
        'maxJerk_units_s3', [20, 4]);
    result = planner(tied, state(0, [10, 0]), state(5, [20, 0]), limits, ...
        struct('GoalTimeMode', 'fixedArrival', 'WrapY', "both"));
    verifyTrue(testCase, result.Success, result.Message);
    verifyTrue(testCase, obstacleAvoidance.validateTrajectory(result).Passed);
    verifyEqual(testCase, result.Diagnostics.Limits.yInterval_units, [-5, 5]);
    handles = obstacleAvoidance.plotting.plotTrajectory(result, struct('FigureVisible', 'off'));
    verifyNotEmpty(testCase, handles.WorkspaceAxes);
    verifyNotEmpty(testCase, handles.SpaceTimeAxes);
    originalPolygons = findobj(handles.WorkspaceAxes, 'Type', 'polygon', 'DisplayName', 'Original obstacle');
    protectedPolygons = findobj(handles.WorkspaceAxes, 'Type', 'polygon', 'DisplayName', 'Protected obstacle');
    verifyNotEmpty(testCase, originalPolygons);
    verifyNotEmpty(testCase, protectedPolygons);
    verifyGreaterThan(testCase, area(protectedPolygons(1).Shape), area(originalPolygons(1).Shape));
end

function expandedWorkspacePlotShowsWrappedCopies(testCase, limits)
    existingFigures = findall(groot, 'Type', 'figure');
    figureCleanup = onCleanup(@() closeCreatedFigures(existingFigures));
    % With wrapping on, the plot adds an expanded workspace in the planner's
    % unwrapped coordinates showing the obstacle and its pole copy.
    blocker = struct('Vertices_units', [185, 89.2; 195, 89.2; 195, 89.8; 185, 89.8]);
    plotOptions = struct('FigureVisible', "off", 'ShowAnimation', false, ...
        'ShowKinematics', false, 'ShowSpaceTime', true);
    wrapped = planner(blocker, state(0, [10, 89]), state(60, [190, 89]), limits, ...
        struct('GoalTimeMode', 'earliestArrival', 'WrapY', "both"));
    handles = obstacleAvoidance.plotting.plotTrajectory(wrapped, plotOptions);
    verifyNotEmpty(testCase, handles.ExpandedWorkspaceAxes);
    obstaclePolygons = findobj(handles.ExpandedWorkspaceAxes, 'Type', 'polygon');
    verifyGreaterThanOrEqual(testCase, numel(obstaclePolygons), 2);
    unwrapped = planner(blocker, state(0, [10, 89]), state(60, [190, 89]), limits, ...
        struct('GoalTimeMode', 'earliestArrival'));
    plainHandles = obstacleAvoidance.plotting.plotTrajectory(unwrapped, plotOptions);
    verifyEmpty(testCase, plainHandles.ExpandedWorkspaceAxes);
    % A saved record with a wrap value the planner would refuse is not drawn.
    badWrap = wrapped;
    badWrap.Options.WrapX = 2;
    verifyError(testCase, @() obstacleAvoidance.plotting.plotTrajectory(badWrap, plotOptions), ...
        'plotTrajectory:InvalidWrapOption');

    % In the 2D workspace an obstacle across the x seam shows on both sides:
    % [355 365] also appears at [-5 5].
    seamObstacle = struct('Vertices_units', [355, -5; 365, -5; 365, 5; 355, 5]);
    seamResult = planner(seamObstacle, state(0, [340, 0]), state(60, [20, 30]), limits, ...
        struct('GoalTimeMode', 'fixedArrival', 'WrapX', "both"));
    seamHandles = obstacleAvoidance.plotting.plotTrajectory(seamResult, plotOptions);
    polygonHandles = findobj(seamHandles.WorkspaceAxes, 'Type', 'polygon');
    minimumX_units = arrayfun(@(polygonHandle) min(polygonHandle.Shape.Vertices(:, 1)), polygonHandles);
    verifyTrue(testCase, any(minimumX_units < 0) && any(minimumX_units > 300));
end

function expandedPlotAcceptsUnequalRowHistories(testCase, limits)
    existingFigures = findall(groot, 'Type', 'figure');
    figureCleanup = onCleanup(@() closeCreatedFigures(existingFigures));
    % The second moving sample repeats its closing vertex. Its five-value
    % row cannot be stacked with the first sample's four-value row, but
    % obstacle normalization removes the repeat before making copies.
    obstacle = struct('targetName', "row history", 'time_s', [0; 5], ...
        'safetyMargin_units', 0, 'status', ["visible"; "visible"]);
    obstacle.x_units = {[-2, 2, 2, -2]; [-1, 3, 3, -1, -1]};
    obstacle.y_units = {[-1, -1, 1, 1]; [-1, -1, 1, 1, -1]};
    result = planner(obstacle, state(0, [10, 0]), state(5, [20, 0]), limits, ...
        struct('GoalTimeMode', 'fixedArrival', 'WrapY', "both"));
    verifyTrue(testCase, result.Success, result.Message);
    verifyTrue(testCase, obstacleAvoidance.validateTrajectory(result).Passed);

    handles = obstacleAvoidance.plotting.plotTrajectory(result, struct('FigureVisible', "off"));
    polygons = findall(handles.ExpandedWorkspaceAxes, 'Type', 'polygon');
    minimumX_units = arrayfun(@(polygonHandle) min(polygonHandle.Shape.Vertices(:, 1)), polygons);
    original = polygons(minimumX_units < 10);
    shifted = polygons(minimumX_units > 300);
    verifyEqual(testCase, numel(polygons), 2);
    verifyEqual(testCase, string(original.LineStyle), "-");
    verifyEqual(testCase, string(original.DisplayName), "Obstacle");
    verifyEqual(testCase, string(shifted.LineStyle), "--");
    verifyEqual(testCase, string(shifted.DisplayName), "Shifted obstacle copy");
end

function expandedPlotStylesCopiesAfterDroppingOutlyingRing(testCase, limits)
    existingFigures = findall(groot, 'Type', 'figure');
    figureCleanup = onCleanup(@() closeCreatedFigures(existingFigures));
    % A two-point region at x = 1000 is removed from the source shape.
    % Using its raw bounds would add false offsets and style the identity
    % copy as shifted. The real shape has identity, shifted, and pole copies.
    obstacle = struct('targetName', "outlying ring", 'time_s', 0, ...
        'safetyMargin_units', 0, 'status', "visible");
    obstacle.x_units = {[-3; 1; 1; -3; NaN; 1000; 1001]};
    obstacle.y_units = {[88; 88; 89; 89; NaN; 88; 89]};
    result = planner(obstacle, state(0, [10, 89]), state(5, [20, 89]), limits, ...
        struct('GoalTimeMode', 'fixedArrival', 'WrapY', "both"));
    verifyTrue(testCase, result.Success, result.Message);
    verifyTrue(testCase, obstacleAvoidance.validateTrajectory(result).Passed);
    verifyEqual(testCase, result.Diagnostics.WrappedGoalCopies.ObstacleCopyCount, 3);

    handles = obstacleAvoidance.plotting.plotTrajectory(result, struct('FigureVisible', "off"));
    polygons = findall(handles.ExpandedWorkspaceAxes, 'Type', 'polygon');
    minimumX_units = arrayfun(@(polygonHandle) min(polygonHandle.Shape.Vertices(:, 1)), polygons);
    original = polygons(minimumX_units < 10);
    pole = polygons(minimumX_units > 150 & minimumX_units < 200);
    shifted = polygons(minimumX_units > 300);
    verifyEqual(testCase, numel(polygons), 3);
    verifyEqual(testCase, string(original.LineStyle), "-");
    verifyEqual(testCase, string(original.DisplayName), "Obstacle");
    verifyEqual(testCase, string(shifted.LineStyle), "--");
    verifyEqual(testCase, string(shifted.DisplayName), "Shifted obstacle copy");
    verifyEqual(testCase, string(pole.LineStyle), ":");
    verifyEqual(testCase, string(pole.DisplayName), "Pole obstacle copy");
end

function limits = sphereLimits()
    limits = struct('xInterval_units', [0, 360], 'yInterval_units', [-90, 90], ...
        'maxVelocity_units_s', [10, 10], 'maxAcceleration_units_s2', [5, 5], ...
        'maxJerk_units_s3', [10, 10]);
end

function value = state(time_s, position_units)
    value = struct('time_s', time_s, 'position_units', position_units, ...
        'velocity_units_s', [0, 0], 'acceleration_units_s2', [0, 0]);
end

function closeCreatedFigures(existingFigures)
    % Preserve every figure that was open before this case began.
    currentFigures = findall(groot, 'Type', 'figure');
    createdFigures = currentFigures(~ismember(currentFigures, existingFigures));
    close(createdFigures(isgraphics(createdFigures)));
end
