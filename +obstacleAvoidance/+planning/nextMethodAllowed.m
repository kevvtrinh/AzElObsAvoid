function eligible = nextMethodAllowed(candidateResult)
%% Section 0: Header & Readme
% SYNTAX
%   eligible = obstacleAvoidance.planning.nextMethodAllowed(candidateResult)
%**************************************************************************
% PURPOSE
%   - The planner tries its methods in a fixed order. When one method fails,
%     this decides whether the next method may run. Only a failure that is
%     local to the method (no route, no feasible motion) hands on; a defect
%     in acceptance stops the sequence, so a later method cannot hide it.
%**************************************************************************
% INPUTS
%   - candidateResult (scalar struct)
%       Planner candidate outcome with termination and failure evidence.
%**************************************************************************
% OUTPUTS
%   - eligible (logical scalar)
%       True only when the next method in the sequence may run.
%**************************************************************************
% UNITS
%   - Unitless policy decision.
%**************************************************************************

%% Section 1: Decide Whether The Next Method May Run

% Admit another method only for a typed method-local miss or bounded
% optimization exhaustion. Unknown and acceptance-defect outcomes stop.
eligible = false;
if string(candidateResult.TerminationReason) == "invalidMotion"
    return
end
failureStage = readStringField(candidateResult, "FailureStage");
failureKind  = readStringField(candidateResult, "FailureKind");
if failureStage == "" && isfield(candidateResult, 'Attempts') && ...
        ~isempty(candidateResult.Attempts)
    failureStage = string(candidateResult.Attempts(end).FailureStage);
    failureKind  = string(candidateResult.Attempts(end).FailureKind);
end
eligible = any(failureStage == ["search", "timing", "proposal", "optimization"]);
if failureStage == "optimization"
    eligible = any(failureKind == ["iterationLimit", ...
        "trajectorySolverIterationLimit", "timedPairSetStalled"]);
end
if failureStage == ""
    eligible = any(string(candidateResult.TerminationReason) == ...
        ["noSpatialRoute", "noVisibilityRoute", "noTimedRoute", "noDepartureWindow", ...
        "timedMotionInfeasible", "noOptimizedFeasibleIterate"]);
end
end

%% Section 2: Local Functions

function value = readStringField(record, name)
    % Read an optional diagnostic string.
    value = "";
    if isstruct(record) && isscalar(record) && isfield(record, name)
        value = string(record.(name));
    end
end
