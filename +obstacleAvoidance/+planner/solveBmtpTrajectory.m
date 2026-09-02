function [candidate, diagnostics] = solveBmtpTrajectory( ...
        seed, staticRepresentation, initialState, goalState, limits, options)
%% Section 0: Header & Readme
% SYNTAX
%   [candidate, diagnostics] = ...
%       obstacleAvoidance.planner.solveBmtpTrajectory( ...
%       seed, obstacles, initialState, goalState, limits, options)
%**************************************************************************
% PURPOSE
%   - Adapt one static obstacle-planner seed to the independent BMTP engine.
%   - Own protected-geometry coverage while the engine owns trajectory math.
%**************************************************************************
% INPUTS
%   - seed (scalar struct)
%       position_deg is N-by-2 and tau increases from zero through one.
%   - staticRepresentation (scalar struct)
%       Source-derived output from createBmtpStaticRepresentation for the
%       same request horizon. Public validation remains authoritative.
%   - initialState, goalState, limits, options (resolved scalar structs)
%       Normalized planner request and fully resolved planner options.
%**************************************************************************
% OUTPUTS
%   - candidate (scalar struct)
%       BMTP motion or stable expected-failure record for public validation.
%   - diagnostics (scalar struct)
%       Engine timing, convergence, motion, and plane-certificate evidence.
%**************************************************************************
% UNITS
%   - Position is degrees and time is seconds. Derivatives use deg/s,
%     deg/s^2, and deg/s^3. Histories are N-by-2.
%**************************************************************************

%% Section 1: Validate The Reusable Static Representation

if ~isstruct(staticRepresentation) || ~isscalar(staticRepresentation) || ...
        ~all(isfield(staticRepresentation, ["Regions_deg", "Coverage"]))
    error("solveBmtpTrajectory:InvalidStaticRepresentation", ...
        ["staticRepresentation must be the scalar struct returned by " ...
        "createBmtpStaticRepresentation."]);
end
regions_deg = staticRepresentation.Regions_deg;
coverage = staticRepresentation.Coverage;

%% Section 2: Generate The Motion In The Independent Engine

[candidate, diagnostics] = bmtpEngine.solve( ...
    seed, regions_deg, coverage, initialState, goalState, limits, options);
end
