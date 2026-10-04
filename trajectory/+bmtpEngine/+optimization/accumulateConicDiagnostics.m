function solverTotals = accumulateConicDiagnostics(solverTotals, solveOutput)
%% Section 0: Header & Readme
% SYNTAX
%   solverTotals = bmtpEngine.optimization.accumulateConicDiagnostics()
%   solverTotals = bmtpEngine.optimization.accumulateConicDiagnostics(solverTotals, solveOutput)
%**************************************************************************
% PURPOSE
%   - Add coneprog call counts and elapsed solver time to the running totals.
%**************************************************************************
% INPUTS
%   - solverTotals (scalar struct)
%       Counts and solver time accumulated by previous calls.
%   - solveOutput (scalar struct)
%       Output for the latest numerical solver stage.
%**************************************************************************
% OUTPUTS
%   - solverTotals (scalar struct)
%       Updated CallCount and TotalTime_s. With no inputs, both start at 0.
%**************************************************************************
% UNITS
%   - TotalTime_s is seconds; CallCount is a completed-call count.
%**************************************************************************

%% Section 1: Initialize Totals When Called Without Inputs

if nargin == 0
    solverTotals = struct();
    solverTotals.CallCount   = 0;
    solverTotals.TotalTime_s = 0;
    return
end

%% Section 2: Add The Latest Numerical-Solver Work

% Some stages report several solver calls together. Use their supplied
% count; an output without SolveCount represents one call.
newCallCount = 1;
if isfield(solveOutput, 'SolveCount')
    newCallCount = solveOutput.SolveCount;
end

solverTotals.CallCount   = solverTotals.CallCount + newCallCount;
solverTotals.TotalTime_s = solverTotals.TotalTime_s + solveOutput.TotalTime_s;
end
