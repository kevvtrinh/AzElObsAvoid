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
%   - Position is coordinate units; time is seconds; derivatives are units/s, units/s^2,
%     and units/s^3. Exported files are tab-delimited numeric matrices.
%**************************************************************************

%% Section 1: Locate The Maintained Public Example

dataDirectory       = fileparts(mfilename("fullpath"));
pdfDirectory        = fileparts(dataDirectory);
repositoryDirectory = fileparts(fileparts(pdfDirectory));
addpath(repositoryDirectory);
addpath(fullfile(repositoryDirectory, "examples"));
addpath(fullfile(repositoryDirectory, "trajectory"));

%% Section 2: Run And Independently Check The Public Example

[result, diagnosis] = exampleVietnamKeepoutSlew(struct("PlotOutputs", false));
if ~result.Success || ~result.Validation.Passed
    error("exportPlannerManualData:ExampleFailed", "The maintained example did not return an independently checked motion: %s", result.TerminationReason);
end
selectedSummary = diagnosis.Attempts([diagnosis.Attempts.SeedIndex] == diagnosis.SelectedAttemptIndex);
if numel(selectedSummary) ~= 1
    error("exportPlannerManualData:SelectedSeedSummary", "Expected one selected seed summary, found %d.", numel(selectedSummary));
end
%% Section 3: Export The Map, Motion, And Candidate-Line Lesson

obstacle             = result.Inputs.obstacles(1);
protectedPolygon_units = [obstacle.x_units{1}(:), obstacle.y_units{1}(:)];
motionData           = [result.time_s, result.position_units, result.velocity_units_s, ...
    result.acceleration_units_s2, result.jerk_units_s3];
limits                 = result.Inputs.limits;
speedData              = [result.time_s, max(abs(result.velocity_units_s), [], 2)];
halfSecondTimes_s      = unique([0:0.5:result.time_s(end), result.time_s(end)]).';
halfSecondPosition_units = interp1(result.time_s, result.position_units, halfSecondTimes_s, "pchip");
halfSecondData         = [halfSecondTimes_s, halfSecondPosition_units];
grid                   = diagnosis.Search;
directLine_units         = [result.Inputs.initialState.position_units; ...
    result.Inputs.goalState.position_units];
allowedLine_units = grid.AcceptedEdges_units(1, [1 2; 3 4]);

writematrix(protectedPolygon_units, fullfile(dataDirectory, "protected_obstacle.dat"), "Delimiter", "tab");
writematrix(result.Route_units, fullfile(dataDirectory, "selected_seed.dat"), "Delimiter", "tab");
writematrix(motionData, fullfile(dataDirectory, "selected_motion.dat"), "Delimiter", "tab");
writematrix(halfSecondData, fullfile(dataDirectory, "half_second_motion.dat"), "Delimiter", "tab");
writematrix(speedData, fullfile(dataDirectory, "turning_speed.dat"), "Delimiter", "tab");
writematrix([limits.maxVelocity_units_s; limits.maxAcceleration_units_s2; limits.maxJerk_units_s3], fullfile(dataDirectory, "motion_limits.dat"), "Delimiter", "tab");
writematrix(directLine_units, fullfile(dataDirectory, "rejected_direct_line.dat"), "Delimiter", "tab");
writematrix(allowedLine_units, fullfile(dataDirectory, "allowed_candidate_line.dat"), "Delimiter", "tab");
writematrix(grid.NodePosition_units, fullfile(dataDirectory, "visibility_nodes.dat"), "Delimiter", "tab");
writematrix(edgeSegments(grid.AcceptedEdges_units), fullfile(dataDirectory, "visibility_accepted_segments.dat"), "Delimiter", "tab");
writematrix(edgeSegments(grid.RejectedEdges_units), fullfile(dataDirectory, "visibility_rejected_segments.dat"), "Delimiter", "tab");
writeWalkthroughValues(dataDirectory, result, selectedSummary, obstacle, diagnosis);

%% Section 4: Export The Recorded Repeat History

solver                    = diagnosis.SolverDetails(diagnosis.SolverDetails.Attempt == diagnosis.SelectedAttemptIndex, {'Field','Value'});
trialDuration_s           = detailValue(solver, "TrialDuration_s");
collisionPairCountHistory = detailValue(solver, "CollisionPairCountHistory");
trialWasCollisionFree     = detailValue(solver, "TrialWasCollisionFree");
iterationCount            = detailValue(solver, "IterationCount");
validTrial                = isfinite(trialDuration_s);
iterationIndex            = find(validTrial);
repeatData                = [iterationIndex, trialDuration_s(validTrial), ...
    collisionPairCountHistory(validTrial), double(trialWasCollisionFree(validTrial))];
writematrix(repeatData, fullfile(dataDirectory, "repeat_history.dat"), "Delimiter", "tab");

%% Section 5: Return The Reproduction Record

summary = struct("SelectedSeedIndex", diagnosis.SelectedAttemptIndex, ...
    "ArrivalTime_s", result.ArrivalTime_s, ...
    "MotionLength_units", selectedSummary.MotionLength_units, ...
    "MaximumCoordinateSpeed_units_s", max(speedData(:, 2)), ...
    "IterationCount", iterationCount, ...
    "ValidationPassed", result.Validation.Passed);
end

function points_units = edgeSegments(edges_units)
    % Convert edge records to NaN-separated polylines for the LaTeX plots.
    edgeCount  = size(edges_units, 1);
    points_units = NaN(3 * edgeCount, 2);
    % Process each geometric edge while constructing or checking the region topology.
    for edgeIndex = 1:edgeCount
        rows = 3 * (edgeIndex - 1) + (1:3);
        points_units(rows(1:2), :) = [edges_units(edgeIndex, 1:2); ...
            edges_units(edgeIndex, 3:4)];
    end
end

function writeWalkthroughValues(dataDirectory, result, selectedSummary, obstacle, diagnosis)
    % Write the exact request and selected-answer values consumed by LaTeX.
    fileName       = fullfile(dataDirectory, "walkthrough_values.tex");
    fileIdentifier = fopen(fileName, "w");
    if fileIdentifier < 0
        error("exportPlannerManualData:WalkthroughValuesWriteFailed", "Could not open %s for writing.", fileName);
    end
    cleaner              = onCleanup(@() fclose(fileIdentifier));
    initialState         = result.Inputs.initialState;
    goalState            = result.Inputs.goalState;
    limits               = result.Inputs.limits;
    options              = result.Options;
    protectedVertexCount = numel(obstacle.x_units{1});
    preparedObstacles    = obstacleAvoidance.obstacles.prepareObstacles(result.Inputs.obstacles);
    [endpointFeasible, ~, endpointReason] = obstacleAvoidance.input.validatePlannerEndpoints(preparedObstacles, result.Inputs.initialState, result.Inputs.goalState, result.Inputs.limits, result.Options);
    directAttempt = diagnosis.DirectMotion;
    fixedClock    = diagnosis.PathRefinement;
    grid          = diagnosis.Search;
    fprintf(fileIdentifier, "%% Generated by exportPlannerManualData.m. Do not edit.\n");
    fprintf(fileIdentifier, "\\newcommand{\\WalkInitialState}{time 0 s; position [-5, 0] deg}\n");
    fprintf(fileIdentifier, "\\newcommand{\\WalkGoalState}{time %.0f s; position [%.0f, %.0f] deg}\n", goalState.time_s, goalState.position_units);
    fprintf(fileIdentifier, "\\newcommand{\\WalkVelocityLimit}{[%.0f, %.0f] units/s}\n", limits.maxVelocity_units_s);
    fprintf(fileIdentifier, "\\newcommand{\\WalkAccelerationLimit}{[%.0f, %.0f] units/s2}\n", limits.maxAcceleration_units_s2);
    fprintf(fileIdentifier, "\\newcommand{\\WalkJerkLimit}{[%.0f, %.0f] units/s3}\n", limits.maxJerk_units_s3);
    fprintf(fileIdentifier, "\\newcommand{\\WalkXInterval}{[%.0f, %.0f] deg}\n", limits.xInterval_units);
    fprintf(fileIdentifier, "\\newcommand{\\WalkYInterval}{[%.0f, %.0f] deg}\n", limits.yInterval_units);
    fprintf(fileIdentifier, "\\newcommand{\\WalkObstacleTimes}{[%.0f; %.0f] s}\n", obstacle.time_s);
    fprintf(fileIdentifier, "\\newcommand{\\WalkSafetyMargin}{%.1f deg}\n", obstacle.safetyMargin_units);
    fprintf(fileIdentifier, "\\newcommand{\\WalkProtectedVertexCount}{%d}\n", protectedVertexCount);
    fprintf(fileIdentifier, "\\newcommand{\\WalkSampleTime}{%.2f s}\n", options.SampleTime_s);
    fprintf(fileIdentifier, "\\newcommand{\\WalkMaximumSeedCount}{%d}\n", options.MaximumSeedCount);
    fprintf(fileIdentifier, "\\newcommand{\\WalkMaximumTimeLayerCount}{%d}\n", options.MaximumTimeLayerCount);
    fprintf(fileIdentifier, "\\newcommand{\\WalkArrivalTolerance}{%.3g s}\n", options.ArrivalTimeTolerance_s);
    fprintf(fileIdentifier, "\\newcommand{\\WalkPlaneReuse}{automatic}\n");
    solver                    = diagnosis.SolverDetails(diagnosis.SolverDetails.Attempt == diagnosis.SelectedAttemptIndex, {'Field','Value'});
    trialDuration_s           = detailValue(solver, "TrialDuration_s");
    collisionPairCountHistory = detailValue(solver, "CollisionPairCountHistory");
    trialWasCollisionFree     = detailValue(solver, "TrialWasCollisionFree");
    trajectorySocpCount       = detailValue(solver, "TrajectorySocpCount");
    planeSocpCount            = detailValue(solver, "PlaneSocpCount");
    planeReuseApplied         = detailValue(solver, "PlaneReuseApplied");
    validTrial                = isfinite(trialDuration_s);
    trialIndex                = find(validTrial);
    fprintf(fileIdentifier, "\\newcommand{\\WalkSelectedTrialCount}{%d}\n", numel(trialIndex));
    fprintf(fileIdentifier, "\\newcommand{\\WalkSelectedCollisionFreeTrialCount}{%d}\n", nnz(trialWasCollisionFree(validTrial)));
    fprintf(fileIdentifier, "\\newcommand{\\WalkSelectedTrajectorySocpCount}{%d}\n", trajectorySocpCount);
    fprintf(fileIdentifier, "\\newcommand{\\WalkSelectedPlaneSocpCount}{%d}\n", planeSocpCount);
    fprintf(fileIdentifier, "\\newcommand{\\WalkSelectedPlaneReuseApplied}{%s}\n", string(planeReuseApplied));
    fprintf(fileIdentifier, "\\newcommand{\\WalkSelectedTrialOneDuration}{%.12f s}\n", trialDuration_s(trialIndex(1)));
    fprintf(fileIdentifier, "\\newcommand{\\WalkSelectedTrialOnePairs}{%d}\n", collisionPairCountHistory(trialIndex(1)));
    fprintf(fileIdentifier, "\\newcommand{\\WalkSelectedTrialTwoDuration}{%.12f s}\n", trialDuration_s(trialIndex(2)));
    fprintf(fileIdentifier, "\\newcommand{\\WalkSelectedTrialThreeDuration}{%.12f s}\n", trialDuration_s(trialIndex(3)));
    fprintf(fileIdentifier, "\\newcommand{\\WalkArrivalTime}{%.12f s}\n", result.ArrivalTime_s);
    fprintf(fileIdentifier, "\\newcommand{\\WalkMotionLength}{%.12f deg}\n", selectedSummary.MotionLength_units);
    fprintf(fileIdentifier, "\\newcommand{\\WalkSelectedSeedIndex}{%d}\n", diagnosis.SelectedAttemptIndex);
    fprintf(fileIdentifier, "\\newcommand{\\WalkGoalHorizon}{%.0f s}\n", result.Inputs.goalState.time_s - result.Inputs.initialState.time_s);
    fprintf(fileIdentifier, "\\newcommand{\\WalkInitialVelocity}{[%.0f, %.0f] units/s}\n", initialState.velocity_units_s);
    fprintf(fileIdentifier, "\\newcommand{\\WalkInitialAcceleration}{[%.0f, %.0f] units/s2}\n", initialState.acceleration_units_s2);
    fprintf(fileIdentifier, "\\newcommand{\\WalkEndpointFeasible}{%s}\n", string(endpointFeasible));
    fprintf(fileIdentifier, "\\newcommand{\\WalkEndpointReason}{%s}\n", endpointReason);
    fprintf(fileIdentifier, "\\newcommand{\\WalkDirectDuration}{%.12f s}\n", detailValue(directAttempt, "TrajectoryDuration_s"));
    fprintf(fileIdentifier, "\\newcommand{\\WalkDirectLength}{%.12f deg}\n", detailValue(directAttempt, "MotionLength_units"));
    fprintf(fileIdentifier, "\\newcommand{\\WalkDirectReason}{%s}\n", detailValue(directAttempt, "TerminationReason"));
    fprintf(fileIdentifier, "\\newcommand{\\WalkFixedClockScreeningCount}{%d}\n", detailValue(fixedClock, "ScreeningCount"));
    fprintf(fileIdentifier, "\\newcommand{\\WalkFixedClockValidationCount}{%d}\n", detailValue(fixedClock, "ValidationCount"));
    fprintf(fileIdentifier, "\\newcommand{\\WalkFixedClockReason}{%s}\n", detailValue(fixedClock, "TerminationReason"));
    fprintf(fileIdentifier, "\\newcommand{\\WalkVisibilityNodeCount}{%d}\n", grid.NodeCount);
    fprintf(fileIdentifier, "\\newcommand{\\WalkVisibilityPairCount}{%d}\n", grid.VisibilityCandidatePairCount);
    fprintf(fileIdentifier, "\\newcommand{\\WalkVisibilityAcceptedCount}{%d}\n", grid.VisibilityEdgeCount);
    fprintf(fileIdentifier, "\\newcommand{\\WalkVisibilityRejectedCount}{%d}\n", size(grid.RejectedEdges_units, 1));
    fprintf(fileIdentifier, "\\newcommand{\\WalkGeneratedSeedCount}{%d}\n", grid.GeneratedSeedCount);
    fprintf(fileIdentifier, "\\newcommand{\\WalkRouteClassCount}{%d}\n", grid.RouteClassCount);
    fprintf(fileIdentifier, "\\newcommand{\\WalkTemporalLayerCount}{%d}\n", grid.TemporalLayerCount);
    fprintf(fileIdentifier, "\\newcommand{\\WalkTimedSuppressionReason}{%s}\n", grid.Coverage.TimedSearchSuppressionReason);
    seedNames = ["One", "Two", "Three"];
    % Evaluate each seed before retaining the best admissible candidate.
    for seedIndex = 1:numel(diagnosis.Routes)
        seed     = diagnosis.Routes(seedIndex);
        seedName = seedNames(seedIndex);
        fprintf(fileIdentifier, "\\newcommand{\\WalkSeed%sSource}{%s}\n", seedName, seed.Source);
        fprintf(fileIdentifier, "\\newcommand{\\WalkSeed%sLength}{%.12f deg}\n", seedName, seed.Length_units);
        fprintf(fileIdentifier, "\\newcommand{\\WalkSeed%sDuration}{%.12f s}\n", seedName, seed.EstimatedDuration_s);
    end
end

function value = detailValue(details, name)
    % Read a recorded solver value for the manual.
    value = details.Value{find(details.Field == name, 1)};
end
