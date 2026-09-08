function [result, diagnosis] = exampleOpeningUShapedObstacle(exampleOverrides)
%% Section 0: Header & Readme
% SYNTAX
%   result = exampleOpeningUShapedObstacle()
%   result = exampleOpeningUShapedObstacle(exampleOverrides)
%
% PURPOSE
%   - Demonstrate waiting for a timed opening in one U-shaped obstacle.
%
% INPUTS
%   - exampleOverrides (scalar struct, optional; default struct())
%       Public planner and uniform display controls.
%
% OUTPUTS
%   - result (scalar planTrajectory result)
%       Unmodified public planner result.
%   - diagnosis (optional second output): search attempts and solver details.
%
% UNITS
%   - Position is coordinate units; time is seconds; derivatives use units/s,
%     units/s^2, and units/s^3.
%

%% Section 1: Resolve Example Controls

% Use earliest arrival. The planner can wait for the opening and then finish as
% soon as the motion limits permit.

if nargin < 1 || isempty(exampleOverrides)
    exampleOverrides = struct();
end
[options, displayOptions] = resolveExampleOptions(exampleOverrides, struct("GoalTimeMode", "earliestArrival", "FigureVisible", "on", "Title", "U-shaped obstacle opening after 7 seconds"), [2.5 2.5]);

%% Section 2: Create Obstacles

% Early samples contain one closed U boundary. Later samples contain two rings
% with a gap between them. The narrow time transition gives a clear opening
% event without a scenario-specific planner rule.

missionEndTime_s      = 120;
openingTime_s         = 7;
transitionHalfWidth_s = 1e-3;
safetyMargin_units      = 0.20;
gapHalfWidth_units      = 1.5;
closedBoundary_units    = [ -8, 7; -5, 7; -5, -4; 5, -4; 5, 7; 8, 7; 8, -7; -8, -7];
leftOpenBoundary_units  = [ -8, 7; -5, 7; -5, -4; -gapHalfWidth_units, -4; -gapHalfWidth_units, -7; -8, -7];
rightOpenBoundary_units = [ 5, 7; 8, 7; 8, -7; gapHalfWidth_units, -7; gapHalfWidth_units, -4; 5, -4];
openBoundary_units      = [ leftOpenBoundary_units; NaN NaN; rightOpenBoundary_units];
obstacleTime_s        = [ 0; openingTime_s - transitionHalfWidth_s; openingTime_s + transitionHalfWidth_s; missionEndTime_s];
xByTime_units     = { ...
    closedBoundary_units(:, 1); closedBoundary_units(:, 1); openBoundary_units(:, 1); openBoundary_units(:, 1)};
yByTime_units = { ...
    closedBoundary_units(:, 2); closedBoundary_units(:, 2); openBoundary_units(:, 2); openBoundary_units(:, 2)};
obstacles = obstacleAvoidance.obstacles.createObstacle("U-shaped obstacle with timed gap", obstacleTime_s, xByTime_units, yByTime_units, safetyMargin_units);

%% Section 3: Create Planner Inputs

% Start inside the cavity and place the goal below it. The closed bottom blocks
% every early exit. The final time supplies a planning horizon, not a required
% arrival time.

initialState = struct();
initialState.time_s              = 0;
initialState.position_units        = [0 0];
initialState.velocity_units_s      = [0 0];
initialState.acceleration_units_s2 = [0 0];
goalState = struct("time_s", missionEndTime_s, "position_units", [0 -10], "velocity_units_s", [0 0], "acceleration_units_s2", [0 0]);
limits = struct("maxVelocity_units_s", [2 2], "maxAcceleration_units_s2", [0.75 0.75], "maxJerk_units_s3", displayOptions.MaxJerk_units_s3);

%% Section 4: Run Planner

% The wait seed can produce repeated solver conditioning warnings. Hide only
% these expected warnings. Validation still rejects invalid motion.
warningState = warning;
warning("off", "MATLAB:nearlySingularMatrix");
warning("off", "MATLAB:singularMatrix");
warningCleanup = onCleanup(@() warning(warningState));
[result, diagnosis] = obstacleAvoidance.planTrajectory(obstacles, initialState, goalState, limits, options);
clear warningCleanup;

%% Section 5: Validate Result

% Run common trajectory checks. Then confirm that the selected seed waits and
% crosses the gap only after it opens.

exampleValidation = validateExampleResult(result, "opening U-shaped obstacle", struct(), diagnosis);
openingValidation = validateOpeningUse(result, diagnosis, openingTime_s, gapHalfWidth_units, safetyMargin_units);
exampleValidation.Passed = exampleValidation.Passed && openingValidation.Passed;
if ~openingValidation.Passed
    exampleValidation.Message = exampleValidation.Message + " " + openingValidation.Message;
end
if ~exampleValidation.Passed
    warning("exampleOpeningUShapedObstacle:ValidationFailed", "%s", exampleValidation.Message);
end

%% Section 6: Plot Diagnostics And Motion

% Animation shows the opening event and the later crossing on one time axis.

if displayOptions.PlotOutputs
    obstacleAvoidance.plotting.plotTrajectory(result, displayOptions.PlotOptions, diagnosis);
end

end

function validation = validateOpeningUse(result, diagnosis, openingTime_s, gapHalfWidth_units, safetyMargin_units)
    % Verify that the selected seed waits and then crosses the protected gap.
    waitSeedSelected          = false;
    stayedBeforeClosedBarrier = false;
    crossedOpenGap            = false;
    selectedArrivalTime_s     = NaN;
    comparisonArrivalTime_s   = NaN;
    if result.Success
        selectedSeed              = diagnosis.Routes(diagnosis.SelectedAttemptIndex);
        repeatedPosition          = vecnorm(diff(selectedSeed.position_units, 1, 1), 2, 2) <= 1e-10;
        waitSeedSelected          = any(repeatedPosition) || selectedSeed.Source == "directWait";
        beforeOpening             = result.time_s <= openingTime_s;
        stayedBeforeClosedBarrier = any(beforeOpening) && all(result.position_units(beforeOpening, 2) >= -4 + safetyMargin_units - 1e-6);
        crossesBottomBar          = result.position_units(:, 2) <= -4 + safetyMargin_units & result.position_units(:, 2) >= -7 - safetyMargin_units;
        protectedGapHalfWidth_units = gapHalfWidth_units - safetyMargin_units;
        crossedOpenGap            = any(crossesBottomBar & abs(result.position_units(:, 1)) < protectedGapHalfWidth_units & result.time_s > openingTime_s);
        selectedArrivalTime_s     = result.time_s(end);
        otherValidated            = find([diagnosis.Attempts.ValidationPassed]);
        otherValidated(otherValidated == diagnosis.SelectedAttemptIndex) = [];
        if ~isempty(otherValidated)
            comparisonArrivalTime_s = min([diagnosis.Attempts(otherValidated).ArrivalTime_s]);
        end
    end
    passed = result.Success && waitSeedSelected && stayedBeforeClosedBarrier && crossedOpenGap;
    if passed
        message = "The selected seed waited for and crossed the timed gap.";
    else
        message = sprintf("Opening use failed: success=%s, wait=%s, stayed=%s, crossed=%s.", string(logical(result.Success)), string(logical(waitSeedSelected)), string(logical(stayedBeforeClosedBarrier)), string(logical(crossedOpenGap)));
    end
    validation = struct("Passed", passed, ...
        "Message", string(message), ...
        "WaitSeedSelected", waitSeedSelected, ...
        "StayedBeforeClosedBarrier", stayedBeforeClosedBarrier, ...
        "CrossedOpenGap", crossedOpenGap, ...
        "OpeningTime_s", openingTime_s, ...
        "SelectedArrivalTime_s", selectedArrivalTime_s, "ComparisonArrivalTime_s", comparisonArrivalTime_s);
end
