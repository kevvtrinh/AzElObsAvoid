function tests = testAzimuthWrappingPlotting
%% Section 0: Header & Readme
% SYNTAX
%   tests = testAzimuthWrappingPlotting
%**************************************************************************
% PURPOSE
%   - Verify periodic spatial plots split at the azimuth seam.
%   - Verify wrapped plans also retain a continuous-azimuth figure.
%**************************************************************************
% INPUTS
%   - None.
%**************************************************************************
% OUTPUTS
%   - tests (matlab.unittest function test array)
%**************************************************************************
% UNITS
%   - Position is degrees and time is seconds.
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
% Preserve exact seam endpoints for increasing and decreasing azimuth paths.
increasing_deg = [179 0; 181 2];
decreasing_deg = [-179 0; -181 2];
increasingDisplay_deg = ...
    obstacleAvoidance.plotting.createWrappedSpatialPath( ...
    increasing_deg, [-180 180], true);
decreasingDisplay_deg = ...
    obstacleAvoidance.plotting.createWrappedSpatialPath( ...
    decreasing_deg, [-180 180], true);
verifyEqual(testCase, increasingDisplay_deg, ...
    [179 0; 180 1; NaN NaN; -180 1; -179 2]);
verifyEqual(testCase, decreasingDisplay_deg, ...
    [-179 0; -180 1; NaN NaN; 180 1; 179 2]);
end

function testWrappedResultCreatesPeriodicAndContinuousViews(testCase)
% Exercise the shared workspace and animation views on a positive seam cross.
initialState = stateAt(0, [179 0]);
goalState = stateAt(8, [-179 0]);
limits = struct( ...
    "maxVelocity_deg_s", [1 1], ...
    "maxAcceleration_deg_s2", [1 1], ...
    "maxJerk_deg_s3", [2 2], ...
    "azimuthInterval_deg", [-180 180], ...
    "elevationInterval_deg", [-90 90]);
options = obstacleAvoidance.planTrajectory();
options.GoalTimeMode = "fixedArrival";
options.AllowAzimuthWrapping = true;
result = obstacleAvoidance.planTrajectory( ...
    [], initialState, goalState, limits, options);
verifyTrue(testCase, result.Success, result.Message);
plotOptions = struct( ...
    "FigureVisible", "off", ...
    "ShowWorkspace", true, ...
    "ShowVisibilityGraphs", false, ...
    "ShowKinematics", false, ...
    "ShowAnimation", true, ...
    "FrameStride", numel(result.time_s), ...
    "Pause_s", 0);

handles = obstacleAvoidance.plotting.plotTrajectory(result, plotOptions);
testCase.addTeardown(@() closePlotFigures(handles));

wrappedMotionHandle = findobj( ...
    handles.WorkspaceAxes, "DisplayName", "Timed motion");
continuousMotionHandle = findobj( ...
    handles.ContinuousWorkspaceAxes, "DisplayName", "Timed motion");
currentStateHandle = findobj( ...
    handles.AnimationAxes, "DisplayName", "Current state");
verifyTrue(testCase, isgraphics(handles.ContinuousWorkspaceFigure));
verifyTrue(testCase, any(isnan(wrappedMotionHandle.XData)));
verifyEqual(testCase, xlim(handles.WorkspaceAxes), [-180 180]);
verifyEqual(testCase, continuousMotionHandle.XData, ...
    result.position_deg(:, 1).');
verifyEqual(testCase, currentStateHandle.XData, -179, "AbsTol", 1e-9);
verifyEqual(testCase, result.position_deg(end, 1), 181, "AbsTol", 1e-9);
end

function state = stateAt(time_s, position_deg)
% Create one rest-to-rest planner endpoint.
state = struct( ...
    "time_s", time_s, ...
    "position_deg", position_deg, ...
    "velocity_deg_s", [0 0], ...
    "acceleration_deg_s2", [0 0]);
end

function closePlotFigures(handles)
% Close only figures returned by the plotter under test.
figureNames = [ ...
    "WorkspaceFigure", "ContinuousWorkspaceFigure", ...
    "VisibilityFigure", "KinematicFigure", "AnimationFigure"];
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
