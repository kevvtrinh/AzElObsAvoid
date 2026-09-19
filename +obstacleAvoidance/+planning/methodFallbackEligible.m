function eligible = methodFallbackEligible(candidateResult)
%% Section 0: Header & Readme
% SYNTAX
%   eligible = obstacleAvoidance.planning.methodFallbackEligible(candidateResult)
%**************************************************************************
% PURPOSE
%   - Decide whether a typed method-local miss admits another planning
%     method without treating an acceptance defect as recoverable.
%**************************************************************************
% INPUTS
%   - candidateResult (scalar struct)
%       Planner candidate outcome with termination and failure evidence.
%**************************************************************************
% OUTPUTS
%   - eligible (logical scalar)
%       True only when the planner's fallback policy admits another method.
%**************************************************************************
% UNITS
%   - Unitless policy decision.
%**************************************************************************

%% Section 1: Apply The Fallback Policy

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
