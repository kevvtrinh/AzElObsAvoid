function result = exampleOpeningUShapedObstacle(exampleOverrides)
%% Section 0: Header & Readme
% SYNTAX
%   result = exampleOpeningUShapedObstacle()
%   result = exampleOpeningUShapedObstacle(exampleOverrides)
%**************************************************************************
% PURPOSE
%   - Demonstrate waiting for a timed opening in one U-shaped obstacle.
%**************************************************************************
% INPUTS
%   - exampleOverrides (scalar struct, optional; default struct())
%       Public planner and uniform display controls.
%**************************************************************************
% OUTPUTS
%   - result (scalar planTrajectory result)
%       Unmodified public planner result.
%**************************************************************************
% UNITS
%   - Position is degrees; time is seconds; derivatives use deg/s,
%     deg/s^2, and deg/s^3.
%**************************************************************************

%% Section 1: Resolve Example Controls

% Use earliest arrival. The planner can wait for the opening and then finish as
% soon as the motion limits permit.

if nargin < 1 || isempty(exampleOverrides)
    exampleOverrides = struct();
end
[options, displayOptions] = resolveExampleOptions( ...
    exampleOverrides, struct( ...
    "GoalTimeMode", "earliestArrival", ...
    "FigureVisible", "on", ...
    "Title", "U-shaped obstacle opening after 7 seconds"), [2.5 2.5]);

%% Section 2: Create Obstacles

% Early samples contain one closed U boundary. Later samples contain two rings
% with a gap between them. The narrow time transition gives a clear opening
% event without a scenario-specific planner rule.

missionEndTime_s = 120;
openingTime_s = 7;
transitionHalfWidth_s = 1e-3;
safetyMargin_deg = 0.20;
gapHalfWidth_deg = 1.5;
closedBoundary_deg = [ -8, 7; -5, 7; -5, -4; 5, -4; 5, 7; 8, 7; 8, -7; -8, -7];
leftOpenBoundary_deg = [ -8, 7; -5, 7; -5, -4; -gapHalfWidth_deg, -4; -gapHalfWidth_deg, -7; -8, -7];
rightOpenBoundary_deg = [ 5, 7; 8, 7; 8, -7; gapHalfWidth_deg, -7; gapHalfWidth_deg, -4; 5, -4];
openBoundary_deg = [ leftOpenBoundary_deg; NaN NaN; rightOpenBoundary_deg];
obstacleTime_s = [ 0; openingTime_s - transitionHalfWidth_s; openingTime_s + transitionHalfWidth_s; missionEndTime_s];
azimuthByTime_deg = { ...
    closedBoundary_deg(:, 1); closedBoundary_deg(:, 1); openBoundary_deg(:, 1); openBoundary_deg(:, 1)};
elevationByTime_deg = { ...
    closedBoundary_deg(:, 2); closedBoundary_deg(:, 2); openBoundary_deg(:, 2); openBoundary_deg(:, 2)};
obstacles = obstacleAvoidance.obstacles.createObstacle( ...
    "U-shaped obstacle with timed gap", obstacleTime_s, azimuthByTime_deg, elevationByTime_deg, safetyMargin_deg);

%% Section 3: Create Planner Inputs

% Start inside the cavity and place the goal below it. The closed bottom blocks
% every early exit. The final time supplies a planning horizon, not a required
% arrival time.

initialState = struct( "time_s", 0, "position_deg", [0 0], "velocity_deg_s", [0 0], "acceleration_deg_s2", [0 0]);
goalState = struct( ...
    "time_s", missionEndTime_s, "position_deg", [0 -10], "velocity_deg_s", [0 0], "acceleration_deg_s2", [0 0]);
limits = struct( ...
    "maxVelocity_deg_s", [2 2], "maxAcceleration_deg_s2", [0.75 0.75], "maxJerk_deg_s3", displayOptions.MaxJerk_deg_s3);

%% Section 4: Run Planner

% The wait seed can produce repeated solver conditioning warnings. Hide only
% these expected warnings. Validation still rejects invalid motion.
warningState = warning;
warning("off", "MATLAB:nearlySingularMatrix");
warning("off", "MATLAB:singularMatrix");
warningCleanup = onCleanup(@() warning(warningState));
result = obstacleAvoidance.planTrajectory( obstacles, initialState, goalState, limits, options);
clear warningCleanup;

%% Section 5: Validate Result

% Run common trajectory checks. Then confirm that the selected seed waits and
% crosses the gap only after it opens.

exampleValidation = validateExampleResult( result, "opening U-shaped obstacle");
openingValidation = validateOpeningUse( result, openingTime_s, gapHalfWidth_deg, safetyMargin_deg);
exampleValidation.Passed = exampleValidation.Passed && openingValidation.Passed;
if ~openingValidation.Passed
    exampleValidation.Message = exampleValidation.Message + " " + openingValidation.Message;
end
if ~exampleValidation.Passed
    warning("exampleOpeningUShapedObstacle:ValidationFailed", ...
        "%s", exampleValidation.Message);
end

%% Section 6: Plot Diagnostics And Motion

% Animation shows the opening event and the later crossing on one time axis.

if displayOptions.PlotOutputs
    obstacleAvoidance.plotting.plotTrajectory( ...
        result, displayOptions.PlotOptions);
end

end

function validation = validateOpeningUse( result, openingTime_s, gapHalfWidth_deg, safetyMargin_deg)
% Verify that the selected seed waits and then crosses the protected gap.
waitSeedSelected = false;
stayedBeforeClosedBarrier = false;
crossedOpenGap = false;
selectedArrivalTime_s = NaN;
comparisonArrivalTime_s = NaN;
if result.Success
    selectedSeed = result.Seeds(result.SelectedSeedIndex);
    repeatedPosition = vecnorm( diff(selectedSeed.position_deg, 1, 1), 2, 2) <= 1e-10;
    waitSeedSelected = any(repeatedPosition) || selectedSeed.Source == "directWait";
    beforeOpening = result.time_s <= openingTime_s;
    stayedBeforeClosedBarrier = any(beforeOpening) && all( ...
        result.position_deg(beforeOpening, 2) >= -4 + safetyMargin_deg - 1e-6);
    crossesBottomBar = result.position_deg(:, 2) <= ...
        -4 + safetyMargin_deg & result.position_deg(:, 2) >= -7 - safetyMargin_deg;
    protectedGapHalfWidth_deg = gapHalfWidth_deg - safetyMargin_deg;
    crossedOpenGap = any(crossesBottomBar & ...
        abs(result.position_deg(:, 1)) < protectedGapHalfWidth_deg & result.time_s > openingTime_s);
    selectedArrivalTime_s = result.time_s(end);
    otherValidated = find([result.SeedSummaries.ValidationPassed]);
    otherValidated(otherValidated == result.SelectedSeedIndex) = [];
    if ~isempty(otherValidated)
        comparisonArrivalTime_s = min( [result.SeedSummaries(otherValidated).ArrivalTime_s]);
    end
end
passed = result.Success && waitSeedSelected && stayedBeforeClosedBarrier && crossedOpenGap;
if passed
    message = "The selected seed waited for and crossed the timed gap.";
else
    message = sprintf( ...
        "Opening use failed: success=%s, wait=%s, stayed=%s, crossed=%s.", ...
        logicalText(result.Success), logicalText(waitSeedSelected), ...
        logicalText(stayedBeforeClosedBarrier), logicalText(crossedOpenGap));
end

function text = logicalText(value)
% Render scalar logical validation status as true or false.
if logical(value)
    text = "true";
else
    text = "false";
end
end
validation = struct( ...
    "Passed", passed, ...
    "Message", string(message), ...
    "WaitSeedSelected", waitSeedSelected, ...
    "StayedBeforeClosedBarrier", stayedBeforeClosedBarrier, ...
    "CrossedOpenGap", crossedOpenGap, ...
    "OpeningTime_s", openingTime_s, ...
    "SelectedArrivalTime_s", selectedArrivalTime_s, "ComparisonArrivalTime_s", comparisonArrivalTime_s);
end
