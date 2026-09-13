function result = finalizeCandidate(result, candidate, route_units, diagnostics)
%% Section 0: Header & Readme
% SYNTAX: result = obstacleAvoidance.input.finalizeCandidate(result,candidate,route,diagnostics)
% PURPOSE: Assemble one BMTP candidate and apply the public independent validator.
% INPUTS: Stable planner result, BMTP candidate, selected route, and solver diagnostics.
% OUTPUTS: Stable planner result with one uniform candidate-acceptance decision.
% UNITS: Position is coordinate units and time is seconds.

%% Section 1: Assemble The Complete Candidate

for fieldName=reshape(string(fieldnames(candidate)),1,[])
    result.(fieldName)=candidate.(fieldName);
end
result.Route_units=route_units;
result.SolverDiagnostics=diagnostics;
goalState=result.Inputs.goalState;
if candidate.Success && ~isempty(goalState.targetMotion)
    [targetPosition_units,targetVelocity_units_s,targetAcceleration_units_s2]= ...
        obstacleAvoidance.input.targetPositionAtTime(goalState.targetMotion,candidate.ArrivalTime_s);
    result.Intercept=struct('Time_s',candidate.ArrivalTime_s, ...
        'TargetPosition_units',targetPosition_units, ...
        'TerminalVelocityPolicy',"explicit", ...
        'TerminalAccelerationPolicy',"explicit");
    if all(candidate.velocity_units_s(end,:)==0)
        result.Intercept.TerminalVelocityPolicy="zero";
    end
    if all(candidate.acceleration_units_s2(end,:)==0)
        result.Intercept.TerminalAccelerationPolicy="zero";
    end
    if result.Options.MatchTargetVelocity
        result.Intercept.TerminalVelocityPolicy="matched";
        result.Intercept.TargetVelocity_units_s=targetVelocity_units_s;
    end
    if result.Options.MatchTargetAcceleration
        result.Intercept.TerminalAccelerationPolicy="matched";
        result.Intercept.TargetAcceleration_units_s2=targetAcceleration_units_s2;
    end
end

%% Section 2: Apply The One Public Acceptance Gate

result.Validation=obstacleAvoidance.validateTrajectory(result);
result.Success=candidate.Success && result.Validation.Passed;
if candidate.Success && ~result.Validation.Passed
    result.Message="BMTP returned motion that failed independent validation: "+ ...
        result.Validation.Message;
    result.TerminationReason="invalidMotion";
end
end
