function nextMethodMayRun = nextMethodAllowed(methodResult)
%% Section 0: Header & Readme
% SYNTAX
%   nextMethodMayRun = obstacleAvoidance.planning.nextMethodAllowed(methodResult)
%**************************************************************************
% PURPOSE
%   - The planner tries its methods in a fixed order. When one method fails,
%     this decides whether the next method may run. Only a failure that is
%     specific to the method can allow another attempt. A returned
%     motion that fails independent validation stops the sequence.
%**************************************************************************
% INPUTS
%   - methodResult (scalar struct)
%       Result from the method just tried, including its reason for stopping.
%**************************************************************************
% OUTPUTS
%   - nextMethodMayRun (logical scalar)
%       True only when the next method in the sequence may run.
%**************************************************************************
% UNITS
%   - Unitless policy decision.
%**************************************************************************

%% Section 1: Decide Whether The Next Method May Run

% A returned motion that fails independent validation must stop planning.
% Continuing could hide a defect in motion that the engine reported as valid.
nextMethodMayRun = false;
if string(methodResult.TerminationReason) == "invalidMotion"
    return
end

% Some planning paths store failure details only in their last attempt.
% Use those details when the overall result does not contain a failure stage.
failureStage = readFailureField(methodResult, "FailureStage");
failureKind  = readFailureField(methodResult, "FailureKind");
if failureStage == "" && isfield(methodResult, 'Attempts') && ...
        ~isempty(methodResult.Attempts)
    failureStage = string(methodResult.Attempts(end).FailureStage);
    failureKind  = string(methodResult.Attempts(end).FailureKind);
end

if any(failureStage == ["search", "timing", "proposal"])
    % This attempt found no route, usable timing, or motion for its proposed route.
    % Another method may provide a different route or arrival time.
    nextMethodMayRun = true;
elseif failureStage == "optimization"
    % Continue only for these known limits on solver progress. Other solver
    % failures need to remain visible instead of starting another method.
    nextMethodMayRun = any(failureKind == ["iterationLimit", ...
        "trajectorySolverIterationLimit", "timedPairSetStalled"]);
elseif failureStage == ""
    % Early failures may have only a termination reason. Allow another
    % method for the known reasons below; an unknown reason keeps false.
    nextMethodMayRun = any(string(methodResult.TerminationReason) == ...
        ["noSpatialRoute", "noVisibilityRoute", "noTimedRoute", "noDepartureWindow", ...
        "timedMotionInfeasible", "noOptimizedFeasibleIterate"]);
end
end

%% Section 2: Local Functions

function failureText = readFailureField(methodResult, fieldName)
    % An empty string means this result did not supply the failure detail.
    failureText = "";
    if isstruct(methodResult) && isscalar(methodResult) && isfield(methodResult, fieldName)
        failureText = string(methodResult.(fieldName));
    end
end
