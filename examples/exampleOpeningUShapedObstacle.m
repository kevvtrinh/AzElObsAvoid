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

% Keep the two U arms stationary and slide the center gate into the left arm.
% Every obstacle retains one corresponding ring, so the opening is an exact
% continuous motion rather than a topology change hidden in one polygon.

missionEndTime_s      = 120;
openingTime_s         = 7;
transitionHalfWidth_s = 1e-3;
safetyMargin_units      = 0.20;
gapHalfWidth_units      = 1.5;
leftOpenBoundary_units  = [ -8, 7; -5, 7; -5, -4; -gapHalfWidth_units, -4; -gapHalfWidth_units, -7; -8, -7];
rightOpenBoundary_units = [ 5, 7; 8, 7; 8, -7; gapHalfWidth_units, -7; gapHalfWidth_units, -4; 5, -4];
closedGate_units        = [ -gapHalfWidth_units, -7; gapHalfWidth_units, -7; gapHalfWidth_units, -4; -gapHalfWidth_units, -4];
openGate_units          = closedGate_units + [-2*gapHalfWidth_units,0];
obstacleTime_s        = [ 0; openingTime_s - transitionHalfWidth_s; openingTime_s + transitionHalfWidth_s; missionEndTime_s];
leftArm = obstacleAvoidance.obstacles.createObstacle("left U arm",0, ...
    {leftOpenBoundary_units(:,1)},{leftOpenBoundary_units(:,2)},safetyMargin_units);
rightArm = obstacleAvoidance.obstacles.createObstacle("right U arm",0, ...
    {rightOpenBoundary_units(:,1)},{rightOpenBoundary_units(:,2)},safetyMargin_units);
gateXByTime_units = {closedGate_units(:,1);closedGate_units(:,1); ...
    openGate_units(:,1);openGate_units(:,1)};
gateYByTime_units = {closedGate_units(:,2);closedGate_units(:,2); ...
    openGate_units(:,2);openGate_units(:,2)};
gate = obstacleAvoidance.obstacles.createObstacle("sliding center gate", ...
    obstacleTime_s,gateXByTime_units,gateYByTime_units,safetyMargin_units);
obstacles = obstacleAvoidance.obstacles.combineObstacles({leftArm;rightArm;gate});

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
[result, diagnosis] = planner(obstacles, initialState, goalState, limits, options);
clear warningCleanup;

%% Section 5: Validate Result

% Run common trajectory checks. Then confirm that the returned motion waits and
% crosses the gap only after it opens.

exampleValidation = validateExampleResult(result, "opening U-shaped obstacle", struct(), diagnosis);
openingValidation = validateOpeningUse(result, openingTime_s, gapHalfWidth_units, safetyMargin_units);
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

function validation = validateOpeningUse(result, openingTime_s, gapHalfWidth_units, safetyMargin_units)
    % Verify the actual stationary interval and crossing of the protected gap.
    hasStationarySpan         = false;
    stayedBeforeClosedBarrier = false;
    crossedOpenGap            = false;
    selectedArrivalTime_s     = NaN;
    if result.Success
        hasStationarySpan         = any(all(result.Polynomial.positionPower_units(:,:,2:end)==0,[2,3]));
        beforeOpening             = result.time_s <= openingTime_s;
        stayedBeforeClosedBarrier = any(beforeOpening) && all(result.position_units(beforeOpening, 2) >= -4 + safetyMargin_units - 1e-6);
        crossesBottomBar          = result.position_units(:, 2) <= -4 + safetyMargin_units & result.position_units(:, 2) >= -7 - safetyMargin_units;
        protectedGapHalfWidth_units = gapHalfWidth_units - safetyMargin_units;
        crossedOpenGap            = any(crossesBottomBar & abs(result.position_units(:, 1)) < protectedGapHalfWidth_units & result.time_s > openingTime_s);
        selectedArrivalTime_s     = result.time_s(end);
    end
    passed = result.Success && hasStationarySpan && stayedBeforeClosedBarrier && crossedOpenGap;
    if passed
        message = "The returned motion waited for and crossed the timed gap.";
    else
        message = sprintf("Opening use failed: success=%s, wait=%s, stayed=%s, crossed=%s.", string(logical(result.Success)), string(logical(hasStationarySpan)), string(logical(stayedBeforeClosedBarrier)), string(logical(crossedOpenGap)));
    end
    validation = struct("Passed", passed, ...
        "Message", string(message), ...
        "HasStationarySpan", hasStationarySpan, ...
        "StayedBeforeClosedBarrier", stayedBeforeClosedBarrier, ...
        "CrossedOpenGap", crossedOpenGap, ...
        "OpeningTime_s", openingTime_s, ...
        "SelectedArrivalTime_s", selectedArrivalTime_s);
end
