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
%   Coordinate units and seconds; tau is dimensionless.

%% Section 1: Assemble The Stable Seed

seed = struct();
seed.Index                    = 0;
seed.Source                   = "";
seed.position_units             = zeros(0, 2);
seed.tau                      = zeros(0, 1);
seed.ParameterBasis           = "normalizedDistance";
seed.ObstacleEnvelope_units     = zeros(0, 2);
seed.UsesConservativeEnvelope = false;
seed.EstimatedDuration_s      = NaN;
seed.Length_units               = NaN;
end
