function summary = exportPlannerManualData()
%% Section 0: Header & Readme
% SYNTAX
%   summary = exportPlannerManualData()
%**************************************************************************
% PURPOSE
%   - Run the maintained obstacle-avoidance example and export the real data
%     used by the non-technical planner manual and process-walkthrough figures.
%**************************************************************************
% INPUTS
%   - None.
%       This function finds the repository relative to its own location and
%       runs the maintained public example without displayed figures.
%**************************************************************************
% OUTPUTS
%   - summary (scalar struct)
%       Reproduction record for the exported motion and solver attempts.
%**************************************************************************
% UNITS
%   - Position is degrees; time is seconds; derivatives are deg/s, deg/s^2,
%     and deg/s^3. Exported files are tab-delimited numeric matrices.
%**************************************************************************

%% Section 1: Locate The Maintained Public Example

dataDirectory = fileparts(mfilename("fullpath"));
pdfDirectory = fileparts(dataDirectory);
repositoryDirectory = fileparts(fileparts(pdfDirectory));
addpath(repositoryDirectory);
addpath(fullfile(repositoryDirectory, "examples"));
addpath(fullfile(repositoryDirectory, "trajectory"));

%% Section 2: Run And Independently Check The Public Example

result = exampleObstacleAvoidance(struct("PlotOutputs", false));
if ~result.Success || ~result.Validation.Passed
    error("exportPlannerManualData:ExampleFailed", ...
        "The maintained example did not return an independently checked motion: %s", ...
        result.TerminationReason);
end
selectedSummary = result.SeedSummaries( ...
    [result.SeedSummaries.SeedIndex] == result.SelectedSeedIndex);
if numel(selectedSummary) ~= 1
    error("exportPlannerManualData:SelectedSeedSummary", ...
        "Expected one selected seed summary, found %d.", numel(selectedSummary));
end
%% Section 3: Export The Map, Motion, And Candidate-Line Lesson

obstacle = result.Inputs.obstacles(1);
protectedPolygon_deg = [obstacle.az_deg{1}(:), obstacle.el_deg{1}(:)];
motionData = [result.time_s, result.position_deg, result.velocity_deg_s, ...
    result.acceleration_deg_s2, result.jerk_deg_s3];
limits = result.Inputs.limits;
speedData = [result.time_s, max(abs(result.velocity_deg_s), [], 2)];
halfSecondTimes_s = unique([0:0.5:result.time_s(end), result.time_s(end)]).';
halfSecondPosition_deg = interp1(result.time_s, result.position_deg, ...
    halfSecondTimes_s, "pchip");
halfSecondData = [halfSecondTimes_s, halfSecondPosition_deg];
grid = result.SearchDiagnostics.Grid;
directLine_deg = [result.Inputs.initialState.position_deg; ...
    result.Inputs.goalState.position_deg];
allowedLine_deg = grid.AcceptedEdges_deg(1, [1 2; 3 4]);

writematrix(protectedPolygon_deg, fullfile(dataDirectory, "protected_obstacle.dat"), ...
    "Delimiter", "tab");
writematrix(result.SelectedSeed_deg, fullfile(dataDirectory, "selected_seed.dat"), ...
    "Delimiter", "tab");
writematrix(motionData, fullfile(dataDirectory, "selected_motion.dat"), ...
    "Delimiter", "tab");
writematrix(halfSecondData, fullfile(dataDirectory, "half_second_motion.dat"), ...
    "Delimiter", "tab");
writematrix(speedData, fullfile(dataDirectory, "turning_speed.dat"), ...
    "Delimiter", "tab");
writematrix([limits.maxVelocity_deg_s; limits.maxAcceleration_deg_s2; ...
    limits.maxJerk_deg_s3], fullfile(dataDirectory, "motion_limits.dat"), ...
    "Delimiter", "tab");
writematrix(directLine_deg, fullfile(dataDirectory, "rejected_direct_line.dat"), ...
    "Delimiter", "tab");
writematrix(allowedLine_deg, fullfile(dataDirectory, "allowed_candidate_line.dat"), ...
    "Delimiter", "tab");
writematrix(grid.NodePosition_deg, fullfile(dataDirectory, "visibility_nodes.dat"), ...
    "Delimiter", "tab");
writematrix(edgeSegments(grid.AcceptedEdges_deg), ...
    fullfile(dataDirectory, "visibility_accepted_segments.dat"), "Delimiter", "tab");
writematrix(edgeSegments(grid.RejectedEdges_deg), ...
    fullfile(dataDirectory, "visibility_rejected_segments.dat"), "Delimiter", "tab");
writeWalkthroughValues(dataDirectory, result, selectedSummary, obstacle);

%% Section 4: Export The Recorded Repeat History

solver = selectedSummary.SolverDiagnostics;
validTrial = isfinite(solver.TrialDuration_s);
iterationIndex = find(validTrial);
repeatData = [iterationIndex, solver.TrialDuration_s(validTrial), ...
    solver.CollisionPairCountHistory(validTrial), ...
    double(solver.TrialWasCollisionFree(validTrial))];
writematrix(repeatData, fullfile(dataDirectory, "repeat_history.dat"), ...
    "Delimiter", "tab");

%% Section 5: Return The Reproduction Record

summary = struct( ...
    "SelectedSeedIndex", result.SelectedSeedIndex, ...
    "ArrivalTime_s", result.ArrivalTime_s, ...
    "MotionLength_deg", selectedSummary.MotionLength_deg, ...
    "MaximumCoordinateSpeed_deg_s", max(speedData(:, 2)), ...
    "IterationCount", solver.IterationCount, ...
    "ValidationPassed", result.Validation.Passed);
end

function points_deg = edgeSegments(edges_deg)
% Convert edge records to NaN-separated polylines for the LaTeX plots.
edgeCount = size(edges_deg, 1);
points_deg = NaN(3 * edgeCount, 2);
for edgeIndex = 1:edgeCount
    rows = 3 * (edgeIndex - 1) + (1:3);
    points_deg(rows(1:2), :) = [edges_deg(edgeIndex, 1:2); ...
        edges_deg(edgeIndex, 3:4)];
end
end

function writeWalkthroughValues(dataDirectory, result, selectedSummary, obstacle)
% Write the exact request and selected-answer values consumed by LaTeX.
fileName = fullfile(dataDirectory, "walkthrough_values.tex");
fileIdentifier = fopen(fileName, "w");
if fileIdentifier < 0
    error("exportPlannerManualData:WalkthroughValuesWriteFailed", ...
        "Could not open %s for writing.", fileName);
end
cleaner = onCleanup(@() fclose(fileIdentifier)); %#ok<NASGU>
initialState = result.Inputs.initialState;
goalState = result.Inputs.goalState;
limits = result.Inputs.limits;
options = result.Options;
protectedVertexCount = numel(obstacle.az_deg{1});
fprintf(fileIdentifier, "%% Generated by exportPlannerManualData.m. Do not edit.\n");
fprintf(fileIdentifier, "\\newcommand{\\WalkInitialState}{time 0 s; position [-5, 0] deg}\n");
fprintf(fileIdentifier, "\\newcommand{\\WalkGoalState}{time %.0f s; position [%.0f, %.0f] deg}\n", ...
    goalState.time_s, goalState.position_deg);
fprintf(fileIdentifier, "\\newcommand{\\WalkVelocityLimit}{[%.0f, %.0f] deg/s}\n", ...
    limits.maxVelocity_deg_s);
fprintf(fileIdentifier, "\\newcommand{\\WalkAccelerationLimit}{[%.0f, %.0f] deg/s2}\n", ...
    limits.maxAcceleration_deg_s2);
fprintf(fileIdentifier, "\\newcommand{\\WalkJerkLimit}{[%.0f, %.0f] deg/s3}\n", ...
    limits.maxJerk_deg_s3);
fprintf(fileIdentifier, "\\newcommand{\\WalkAzimuthInterval}{[%.0f, %.0f] deg}\n", ...
    limits.azimuthInterval_deg);
fprintf(fileIdentifier, "\\newcommand{\\WalkElevationInterval}{[%.0f, %.0f] deg}\n", ...
    limits.elevationInterval_deg);
fprintf(fileIdentifier, "\\newcommand{\\WalkObstacleTimes}{[%.0f; %.0f] s}\n", ...
    obstacle.time_s);
fprintf(fileIdentifier, "\\newcommand{\\WalkSafetyMargin}{%.1f deg}\n", ...
    obstacle.safetyMargin_deg);
fprintf(fileIdentifier, "\\newcommand{\\WalkProtectedVertexCount}{%d}\n", ...
    protectedVertexCount);
fprintf(fileIdentifier, "\\newcommand{\\WalkSampleTime}{%.2f s}\n", ...
    options.SampleTime_s);
fprintf(fileIdentifier, "\\newcommand{\\WalkMaximumSeedCount}{%d}\n", ...
    options.MaximumSeedCount);
fprintf(fileIdentifier, "\\newcommand{\\WalkMaximumTimeLayerCount}{%d}\n", ...
    options.MaximumTimeLayerCount);
fprintf(fileIdentifier, "\\newcommand{\\WalkCollocationSegmentCount}{%d}\n", ...
    options.CollocationSegmentCount);
fprintf(fileIdentifier, "\\newcommand{\\WalkMaximumNlpIterations}{%d}\n", ...
    options.MaximumNlpIterations);
fprintf(fileIdentifier, "\\newcommand{\\WalkArrivalTolerance}{%.3g s}\n", ...
    options.ArrivalTimeTolerance_s);
fprintf(fileIdentifier, "\\newcommand{\\WalkPlaneReuse}{%s}\n", ...
    string(options.EnablePlaneReuse));
fprintf(fileIdentifier, "\\newcommand{\\WalkArrivalTime}{%.12f s}\n", ...
    result.ArrivalTime_s);
fprintf(fileIdentifier, "\\newcommand{\\WalkMotionLength}{%.12f deg}\n", ...
    selectedSummary.MotionLength_deg);
fprintf(fileIdentifier, "\\newcommand{\\WalkSelectedSeedIndex}{%d}\n", ...
    result.SelectedSeedIndex);
fprintf(fileIdentifier, "\\newcommand{\\WalkGoalHorizon}{%.0f s}\n", ...
    result.GoalHorizon_s);
fprintf(fileIdentifier, "\\newcommand{\\WalkInitialVelocity}{[%.0f, %.0f] deg/s}\n", ...
    initialState.velocity_deg_s);
fprintf(fileIdentifier, "\\newcommand{\\WalkInitialAcceleration}{[%.0f, %.0f] deg/s2}\n", ...
    initialState.acceleration_deg_s2);
end
