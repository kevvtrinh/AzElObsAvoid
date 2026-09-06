function seed = createEmptyPathGuess()
%% Section 0: Header & Readme
% SYNTAX
%   seed = obstacleAvoidance.search.createEmptyPathGuess()
% PURPOSE
%   Define the path-guess record shared by search, solving, and diagnostics.
% INPUTS
%   None.
% OUTPUTS
%   seed contains empty geometry, source, parameter basis, and estimates.
% UNITS
%   Degrees and seconds; tau is dimensionless.

%% Section 1: Assemble The Stable Seed

seed = struct( ...
    "Index", 0, "Source", "", ...
    "position_deg", zeros(0, 2), "tau", zeros(0, 1), ...
    "ParameterBasis", "normalizedDistance", ...
    "ObstacleEnvelope_deg", zeros(0, 2), ...
    "UsesConservativeEnvelope", false, ...
    "EstimatedDuration_s", NaN, "Length_deg", NaN);
end
