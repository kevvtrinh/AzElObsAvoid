function options = defaultOptions()
%% Section 0: Header & Readme
% SYNTAX
%   options = hs3Engine.defaultOptions()
%**************************************************************************
% PURPOSE
%   - Return the single source of argument-independent HS3 engine defaults.
%**************************************************************************
% INPUTS
%   - None.
%**************************************************************************
% OUTPUTS
%   - options (scalar struct)
%       Fixed/free-time mode, mesh, sampling, solver, and tolerance controls.
%**************************************************************************
% UNITS
%   - Time values use the caller's time unit.
%   - Coordinate tolerances use the caller's coordinate unit.
%**************************************************************************
% ALGORITHM NOTES
%   - SegmentCount controls approximation freedom: each segment contributes
%     one midpoint jerk value and shares boundary jerk with its neighbors.
%   - SampleTime affects returned history density only. Continuous feasibility
%     is checked from polynomial Bernstein bounds, not from sampled points.
%   - ArrivalTimeTolerance decides when a free-time result is effectively at
%     the caller's time limit. ConstraintTolerance controls solver feasibility.
%     Validation applies a small guard factor for numerical errors.
%   - The three solver tolerances have different roles: constraint residual,
%     first-order optimality, and decision-step size.
options = struct( ...
    "TimeMode", "earliestArrival", ...
    "FinalTime", [], ...
    "SegmentCount", 10, ...
    "SampleTime", 0.05, ...
    "MaximumIterations", 300, ...
    "MaximumFunctionEvaluations", 30000, ...
    "MaximumSolveTime", 115, ...
    "ArrivalTimeTolerance", 1e-3, ...
    "ConstraintTolerance", 1e-7, ...
    "OptimalityTolerance", 1e-7, ...
    "StepTolerance", 1e-10, ...
    "Verbose", false);
end
