function tests = testCoordinateWrappingPlotting
%% Section 0: Header & Readme
% SYNTAX
%   tests = testCoordinateWrappingPlotting
%**************************************************************************
% PURPOSE
%   - Verify independent periodic axes and plots that split at either seam.
%   - Verify wrapped plans also retain a continuous-coordinate figure.
%**************************************************************************
% INPUTS
%   - None.
%**************************************************************************
% OUTPUTS
%   - tests (matlab.unittest function test array)
%**************************************************************************
% UNITS
%   - Position is coordinate units and time is seconds.
%**************************************************************************
tests = functiontests(localfunctions);
end

function setupOnce(testCase)
    % Add the trajectory entry points used by the public planner.
    repositoryRoot = fileparts(fileparts(mfilename("fullpath")));
    addpath(repositoryRoot);
    addpath(fullfile(repositoryRoot, "trajectory"));
    testCase.TestData.RepositoryRoot = repositoryRoot;
end

function testDisplayPathSplitsBothSeamDirections(testCase)
    % Preserve exact seam endpoints for increasing and decreasing x paths.
    increasing_units        = [179 0; 181 2];
    decreasing_units        = [-179 0; -181 2];
    increasingDisplay_units = obstacleAvoidance.plotting.createWrappedSpatialPath(increasing_units, [-180 180; -90 90], [true false]);
    decreasingDisplay_units = obstacleAvoidance.plotting.createWrappedSpatialPath(decreasing_units, [-180 180; -90 90], [true false]);
    verifyEqual(testCase, increasingDisplay_units, [179 0; 180 1; NaN NaN; -180 1; -179 2]);
    verifyEqual(testCase, decreasingDisplay_units, [-179 0; -180 1; NaN NaN; 180 1; 179 2]);
end

function testWrappedResultCreatesPeriodicAndContinuousViews(testCase)
    % Exercise the shared workspace and animation views on a positive seam cross.
    initialState = stateAt(0, [179 0]);
    goalState    = stateAt(8, [-179 0]);
    limits       = struct();
    limits.maxVelocity_units_s      = [1 1];
    limits.maxAcceleration_units_s2 = [1 1];
    limits.maxJerk_units_s3         = [2 2];
    limits.xInterval_units    = [-180 180];
    limits.yInterval_units  = [-90 90];
    options = planner();
    options.GoalTimeMode         = "fixedArrival";
    options.WrapX = true;
    result = planner([], initialState, goalState, limits, options);
    verifyTrue(testCase, result.Success, result.Message);
    plotOptions = struct("FigureVisible", "off", ...
        "ShowWorkspace", true, ...
        "ShowVisibilityGraphs", false, ...
        "ShowKinematics", false, ...
        "ShowAnimation", true, ...
        "FrameStride", numel(result.time_s), ...
        "Pause_s", 0);

    handles = obstacleAvoidance.plotting.plotTrajectory(result, plotOptions);
    testCase.addTeardown(@() closePlotFigures(handles));

    wrappedMotionHandle    = findobj(handles.WorkspaceAxes, "DisplayName", "Timed motion");
    continuousMotionHandle = findobj(handles.ContinuousWorkspaceAxes, "DisplayName", "Timed motion");
    currentStateHandle     = findobj(handles.AnimationAxes, "DisplayName", "Current state");
    verifyTrue(testCase, isgraphics(handles.ContinuousWorkspaceFigure));
    verifyTrue(testCase, any(isnan(wrappedMotionHandle.XData)));
    verifyEqual(testCase, xlim(handles.WorkspaceAxes), [-180 180]);
    verifyEqual(testCase, continuousMotionHandle.XData, result.position_units(:, 1).');
    verifyEqual(testCase, currentStateHandle.XData, -179, "AbsTol", 1e-9);
    verifyEqual(testCase, result.position_units(end, 1), 181, "AbsTol", 1e-9);
end

function testHalfPeriodTiesRemainStableDuringValidation(testCase)
    % Normalizing an already resolved half-period goal must not flip its sign.
    initial = stateAt(0, [0 110]);
    goal = stateAt(30, [-5 100]);
    limits = struct('maxVelocity_units_s', [2 2], 'maxAcceleration_units_s2', [1 1], 'maxJerk_units_s3', [2 2], 'xInterval_units', [-5 5], 'yInterval_units', [100 120]);
    options = struct('WrapX', true, 'WrapY', true);
    result = planner([], initial, goal, limits, options);
    verifyTrue(testCase, result.Success, result.Message);
    verifyEqual(testCase, result.Inputs.goalState.position_units, [5 120]);
    validation = obstacleAvoidance.validateTrajectory(result);
    verifyTrue(testCase, validation.Passed, validation.Message);
    validation = obstacleAvoidance.validateTrajectory(result, [], initial, goal, limits, result.Options);
    verifyTrue(testCase, validation.Passed, validation.Message);
end

function testIndependentWrappingUsesShiftedUnequalPeriods(testCase)
    % Every mask changes only its enabled axis, including the former bounded y.
    initial = stateAt(0, [4 119]);
    goal = stateAt(30, [-4 101]);
    limits = struct();
    limits.maxVelocity_units_s = [2 2];
    limits.maxAcceleration_units_s2 = [1 1];
    limits.maxJerk_units_s3 = [2 2];
    limits.xInterval_units = [-5 5];
    limits.yInterval_units = [100 120];
    for mask = 0:3
        options = planner();
        options.WrapX = logical(bitget(mask, 1));
        options.WrapY = logical(bitget(mask, 2));
        expected = goal.position_units + [10 * options.WrapX, 20 * options.WrapY];
        result = planner([], initial, goal, limits, options);
        assertTrue(testCase, result.Success, result.Message);
        verifyEqual(testCase, result.position_units(end, :), expected, 'AbsTol', 1e-9);
        verifyEqual(testCase, result.Inputs.goalState.position_units, expected);
        validation = obstacleAvoidance.validateTrajectory(result, [], result.Inputs.initialState, goal, limits, result.Options);
        verifyTrue(testCase, validation.Passed, validation.Message);
        verifyTrue(testCase, validation.WrapPolicySatisfied);
    end
end

function testDisabledAxisBoundsRemainEnforced(testCase)
    limits = struct('maxVelocity_units_s', [2 2], 'maxAcceleration_units_s2', [1 1], 'maxJerk_units_s3', [2 2], 'xInterval_units', [-5 5], 'yInterval_units', [100 120]);
    initial = stateAt(0, [4 119]);
    goal = stateAt(30, [-4 121]);
    result = planner([], initial, goal, limits, struct('WrapX', true, 'WrapY', false));
    verifyFalse(testCase, result.Success);
    verifyEqual(testCase, result.TerminationReason, "endpointOutsideWorkspace");
    initial.position_units = [14 139];
    result = planner([], initial, goal, limits, struct('WrapX', true, 'WrapY', true));
    verifyTrue(testCase, result.Success, result.Message);
    validation = obstacleAvoidance.validateTrajectory(result);
    verifyTrue(testCase, validation.Passed, validation.Message);
end

function testYWrappingRejectsUnsupportedPeriodicGeometry(testCase)
    initial = stateAt(0, [0 119]);
    goal = stateAt(30, [0 101]);
    limits = struct('maxVelocity_units_s', [2 2], 'maxAcceleration_units_s2', [1 1], 'maxJerk_units_s3', [2 2], 'xInterval_units', [-5 5], 'yInterval_units', [100 120]);
    options = struct('WrapY', true);
    obstacle = obstacleAvoidance.obstacles.createObstacle('periodic obstacle', [0; 30], [-1; 1; 1; -1], [109; 109; 111; 111]);
    verifyError(testCase, @() planner(obstacle, initial, goal, limits, options), "planner:UnsupportedWrappedGeometry");
    goal.targetTime_s = [0; 30];
    goal.targetPosition_units = [0 102; 0 101];
    verifyError(testCase, @() planner([], initial, goal, limits, options), "planner:UnsupportedWrappedGeometry");
end

function testMultipleAndSimultaneousDisplaySeams(testCase)
    intervals = [0 10; 100 120];
    display = obstacleAvoidance.plotting.createWrappedSpatialPath([9 119; 11 121], intervals, [true true]);
    verifyEqual(testCase, display, [9 119; 10 120; NaN NaN; 0 100; 1 101]);
    display = obstacleAvoidance.plotting.createWrappedSpatialPath([9 119; -11 79], intervals, [true true]);
    verifyEqual(testCase, nnz(isnan(display(:, 1))), 4);
    finiteRows = display(all(isfinite(display), 2), :);
    verifyTrue(testCase, all(finiteRows(:, 1) >= 0 & finiteRows(:, 1) <= 10));
    verifyTrue(testCase, all(finiteRows(:, 2) >= 100 & finiteRows(:, 2) <= 120));
    display = obstacleAvoidance.plotting.createWrappedSpatialPath([2 119; 4 121], intervals, [false true]);
    verifyEqual(testCase, display, [2 119; 3 120; NaN NaN; 3 100; 4 101]);
end

function testYWrappedPlotPreservesContinuousView(testCase)
    initial = stateAt(0, [0 119]);
    goal = stateAt(30, [0 101]);
    limits = struct('maxVelocity_units_s', [2 2], 'maxAcceleration_units_s2', [1 1], 'maxJerk_units_s3', [2 2], 'xInterval_units', [-5 5], 'yInterval_units', [100 120]);
    result = planner([], initial, goal, limits, struct('WrapY', true));
    assertTrue(testCase, result.Success, result.Message);
    plots = struct('FigureVisible', 'off', 'ShowWorkspace', true, 'ShowVisibilityGraphs', false, 'ShowKinematics', false, 'ShowAnimation', false);
    handles = obstacleAvoidance.plotting.plotTrajectory(result, plots);
    testCase.addTeardown(@() closePlotFigures(handles));
    motion = findobj(handles.WorkspaceAxes, 'DisplayName', 'Timed motion');
    continuous = findobj(handles.ContinuousWorkspaceAxes, 'DisplayName', 'Timed motion');
    verifyTrue(testCase, any(isnan(motion.YData)));
    verifyEqual(testCase, ylim(handles.WorkspaceAxes), [100 120]);
    verifyEqual(testCase, continuous.YData, result.position_units(:, 2).');
end

function state = stateAt(time_s, position_units)
    % Create one rest-to-rest planner endpoint.
    state = struct("time_s", time_s, ...
        "position_units", position_units, ...
        "velocity_units_s", [0 0], ...
        "acceleration_units_s2", [0 0]);
end

function closePlotFigures(handles)
    % Close only figures returned by the plotter under test.
    figureNames = [ ...
        "WorkspaceFigure", "ContinuousWorkspaceFigure", ...
        "VisibilityFigure", "KinematicFigure", "AnimationFigure"];
    % Exercise each name covered by this regression.
    for name = figureNames
        if isfield(handles, name)
            figureHandles = handles.(name);
            figureHandles = figureHandles(isgraphics(figureHandles, "figure"));
            if ~isempty(figureHandles)
                close(figureHandles);
            end
        end
    end
end
