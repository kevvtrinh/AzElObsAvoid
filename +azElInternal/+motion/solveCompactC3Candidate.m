function [candidate, diagnostics, elapsedTime_s] = solveCompactC3Candidate( ...
        seed, obstacles, initialState, goalState, limits, options)
%% Section 0: Header & Readme
% Build one normalized, independently validated compact candidate from an
% input-derived topology seed. Dynamic routes receive time-local clearance
% expansion, and bounded duration retries never bypass public validation.

candidateTimer = tic;
seedHasExplicitHold = seed.Source == "directWait" && any( ...
    vecnorm(diff(seed.position_deg), 2, 2) <= 1e-12);
geometryIsStatic = isempty(obstacles) || all(arrayfun(@(obstacle) all( ...
    obstacle.InternalPreparation.IntervalSpeedBound_deg_s == 0), obstacles));
solverSeed = seed;
routeExpansionTimer = tic;
dynamicRouteExpansion_deg = 0;
if ~geometryIsStatic && ~seedHasExplicitHold
    [solverSeed.position_deg, dynamicRouteExpansion_deg] = ...
        azElInternal.search.expandDynamicRoute( ...
        seed, obstacles, initialState.time_s, goalState.time_s);
end
routeExpansionElapsedTime_s = toc(routeExpansionTimer);

motionTimer = tic;
motionOptions = struct( ...
    "SampleTime_s", options.SampleTime_s, ...
    "GoalTimeMode", options.GoalTimeMode, ...
    "AllowAzimuthWrapping", options.AllowAzimuthWrapping, ...
    "ConstraintTolerance", options.ConstraintTolerance, ...
    "CollisionClearanceTolerance_deg", ...
    options.CollisionClearanceTolerance_deg, ...
    "CollisionMinimumTimeStep_s", ...
    options.CollisionMinimumTimeStep_s);
routeMotion = azElInternal.motion.buildQuinticSpline( ...
    solverSeed.position_deg, initialState, goalState, limits, motionOptions);
straightMotion = azElInternal.motion.buildQuinticSpline( ...
    [initialState.position_deg; goalState.position_deg], ...
    initialState, goalState, limits, motionOptions);
preparationElapsedTime_s = toc(motionTimer);
if options.GoalTimeMode == "fixedArrival"
    upperDurations_s = goalState.time_s - initialState.time_s;
    bracketFractions = NaN;
elseif seedHasExplicitHold
    upperDurations_s = seed.EstimatedDuration_s;
    bracketFractions = NaN;
else
    bracketFractions = [0.5 0.8];
    upperDurations_s = straightMotion.MotionDuration_s + ...
        bracketFractions * ...
        (routeMotion.MotionDuration_s - straightMotion.MotionDuration_s);
end
if ~geometryIsStatic && options.GoalTimeMode ~= "fixedArrival"
    upperDurations_s(end + 1) = ...
        goalState.time_s - initialState.time_s;
    bracketFractions(end + 1) = NaN;
end

stageTiming = azElInternal.stageTiming();
stageTiming.CorridorConstructionElapsedTime_s = ...
    routeExpansionElapsedTime_s;
attempts = cell(1, numel(upperDurations_s));
compactMotion = [];
compactValidation = validateAzElTrajectory();
compactPreparation = [];
for bracketIndex = 1:numel(upperDurations_s)
    [compactMotion, compactValidation, attempt, compactPreparation] = ...
        azElInternal.motion.solveCompactC3( ...
        solverSeed, upperDurations_s(bracketIndex), obstacles, ...
        initialState, goalState, limits, options, compactPreparation);
    attempts{bracketIndex} = attempt;
    stageTiming = addStageTiming(stageTiming, attempt.StageTiming);
    if attempt.Accepted
        break;
    end
end
attempts = [attempts{1:bracketIndex}];
diagnostics = attempts(end);
diagnostics.BracketFraction = bracketFractions(bracketIndex);
diagnostics.RetryUsed = bracketIndex > 1;
diagnostics.BracketAttempts = attempts;
diagnostics.TrialCount = sum([attempts.TrialCount]);
diagnostics.QpCount = sum([attempts.QpCount]);
diagnostics.AffineBasisBuildCount = sum([attempts.AffineBasisBuildCount]);
diagnostics.AffineBasisReuseCount = sum([attempts.AffineBasisReuseCount]);
diagnostics.DynamicRouteExpansion_deg = dynamicRouteExpansion_deg;
if ~diagnostics.Accepted
    failureValidation = validateAzElTrajectory( ...
        routeMotion, obstacles, initialState, goalState, limits, options);
    stageTiming.CollisionCheckingElapsedTime_s = ...
        stageTiming.CollisionCheckingElapsedTime_s + ...
        failureValidation.CollisionCheckingElapsedTime_s;
    stageTiming.FinalValidationElapsedTime_s = ...
        stageTiming.FinalValidationElapsedTime_s + max(0, ...
        failureValidation.ElapsedTime_s - ...
        failureValidation.CollisionCheckingElapsedTime_s);
end
stageTiming.MotionSolvingElapsedTime_s = ...
    stageTiming.MotionSolvingElapsedTime_s + preparationElapsedTime_s;
elapsedTime_s = toc(candidateTimer);
diagnostics.StageTiming = azElInternal.stageTiming( ...
    stageTiming, elapsedTime_s);

candidate = routeMotion;
if diagnostics.Accepted
    motionFields = fieldnames(compactMotion);
    for fieldIndex = 1:numel(motionFields)
        candidate.(motionFields{fieldIndex}) = ...
            compactMotion.(motionFields{fieldIndex});
    end
    candidate.Validation = compactValidation;
    candidate.Success = true;
    candidate.Message = "The compact C3 controller passed validation.";
    candidate.TerminationReason = "compactC3Validated";
else
    candidate.Validation = failureValidation;
    candidate.Success = false;
    candidate.Message = "The compact C3 controller found no validated duration.";
    candidate.TerminationReason = "compactC3NotValidated";
end
candidate.OriginalRoute_deg = seed.position_deg;
candidate.ExpandedRoute_deg = solverSeed.position_deg;
candidate.ReducedRoute_deg = solverSeed.position_deg;
candidate.OptimizerDiagnostics = diagnostics;
candidate.OptimizerDiagnostics.CompactC3 = diagnostics;
if diagnostics.Accepted
    candidate.OptimizerDiagnostics.CandidateMaximumInequality_deg = 0;
else
    candidate.OptimizerDiagnostics.CandidateMaximumInequality_deg = Inf;
end
candidate.OptimizerDiagnostics.ValidationPassed = diagnostics.Accepted;
candidate.OptimizerDiagnostics.ContinuousClearance_deg = ...
    candidate.Validation.MinimumClearance_deg;
candidate.OptimizerDiagnostics.MotionDuration_s = candidate.MotionDuration_s;
end

function stageTiming = addStageTiming(stageTiming, contribution)
% Accumulate the five exclusive planner stages.
stageNames = [ ...
    "TopologyElapsedTime_s", "CorridorConstructionElapsedTime_s", ...
    "MotionSolvingElapsedTime_s", "CollisionCheckingElapsedTime_s", ...
    "FinalValidationElapsedTime_s"];
for name = stageNames
    stageTiming.(name) = stageTiming.(name) + contribution.(name);
end
end
