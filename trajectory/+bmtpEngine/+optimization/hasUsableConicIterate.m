function usable = hasUsableConicIterate(values, exitFlag)
%% Section 0: Header & Readme
% SYNTAX
%   usable = bmtpEngine.optimization.hasUsableConicIterate(values, exitFlag)
%**************************************************************************
% PURPOSE
%   - Apply one proposal-retention policy to every BMTP conic solve.
%**************************************************************************
% INPUTS
%   - values (numeric array)
%       Solver iterate returned by the conic solve.
%   - exitFlag (numeric scalar)
%       The coneprog status. A finite -7 iterate remains only a proposal.
%**************************************************************************
% OUTPUTS
%   - usable (logical scalar)
%       True when independent certification may run on this iterate.
%**************************************************************************
% UNITS
%   - Inherited from values.
%**************************************************************************

%% Section 1: Classify The Numerical Proposal

iterateIsFinite = ~isempty(values) && all(isfinite(values), 'all');
usable          = iterateIsFinite && (exitFlag > 0 || exitFlag == -7);
end
