function usable=hasUsableConicIterate(values,exitFlag)
%% Section 0: Header & Readme
% SYNTAX: usable=bmtpEngine.hasUsableConicIterate(values,exitFlag)
% PURPOSE: Apply one proposal-retention policy to every BMTP conic solve.
% INPUTS: values (numeric array) Solver iterate; exitFlag (numeric scalar)
%   is the coneprog status. A finite -7 iterate remains only a proposal.
% OUTPUTS: usable (logical scalar) True when independent certification may run.
% UNITS: Inherited from values.

%% Section 1: Classify The Numerical Proposal
usable=~isempty(values) && all(isfinite(values),'all') && ...
    (exitFlag>0 || exitFlag==-7);
end
