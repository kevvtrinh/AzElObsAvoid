function stageTiming = addHs3CandidateTiming(stageTiming, candidate)
%% Section 0: Header & Readme
% SYNTAX
%   stageTiming = azElPlannerMethods.hs3.internal.addHs3CandidateTiming( ...
%       stageTiming, candidate)
%**************************************************************************
% PURPOSE
%   - Add one HS3-family candidate's solver and validation work to the
%     exclusive planner-stage totals.
%**************************************************************************
% INPUTS
%   - stageTiming (scalar struct)
%       Shared aggregate planner-stage timing record.
%   - candidate (scalar struct)
%       Analytic or HS3 candidate with solver and validation diagnostics.
%**************************************************************************
% OUTPUTS
%   - stageTiming (scalar struct)
%       Value-updated aggregate planner-stage timing record.
%**************************************************************************
% UNITS
%   - All timing fields are seconds.
%**************************************************************************

solverDiagnostics = candidate.SolverDiagnostics;
stageTiming.CorridorConstructionElapsedTime_s = ...
    stageTiming.CorridorConstructionElapsedTime_s + ...
    solverDiagnostics.CorridorConstructionElapsedTime_s;
stageTiming.MotionSolvingElapsedTime_s = ...
    stageTiming.MotionSolvingElapsedTime_s + ...
    solverDiagnostics.MotionSolvingElapsedTime_s;

validation = candidate.Validation;
collisionElapsedTime_s = validation.CollisionCheckingElapsedTime_s;
stageTiming.CollisionCheckingElapsedTime_s = ...
    stageTiming.CollisionCheckingElapsedTime_s + collisionElapsedTime_s;
stageTiming.FinalValidationElapsedTime_s = ...
    stageTiming.FinalValidationElapsedTime_s + max( ...
    0, validation.ElapsedTime_s - collisionElapsedTime_s);
end
