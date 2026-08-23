function seed = candidateSeed(candidate, originalSeed)
%% Section 0: Header & Readme
% SYNTAX
%   seed = azElPlannerMethods.hs3.internal.candidateSeed( ...
%       candidate, originalSeed)
%**************************************************************************
% PURPOSE
%   - Reassociate a failed collision corridor without adding a topology.
%**************************************************************************
% INPUTS
%   - candidate (scalar struct), sampled nonempty HS3 motion.
%   - originalSeed (scalar struct), topology and normalized control times.
%**************************************************************************
% OUTPUTS
%   - seed (scalar struct), original topology with the candidate path/time law.
%**************************************************************************
% UNITS
%   - Positions are degrees, durations are seconds, and tau is dimensionless.
%**************************************************************************

seed = originalSeed;
sampleTau = (candidate.time_s - candidate.time_s(1)) / ...
    (candidate.time_s(end) - candidate.time_s(1));
[sampleTau, retainedIndex] = unique(sampleTau, "stable");
seed.tau = candidate.ControlTau;
seed.position_deg = interp1( ...
    sampleTau, candidate.position_deg(retainedIndex, :), seed.tau, "linear");
seed.EstimatedDuration_s = candidate.MotionDuration_s;
end
