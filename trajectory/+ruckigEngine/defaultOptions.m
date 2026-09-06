function options = defaultOptions()
%% Section 0: Header & Readme
% SYNTAX
%   options = ruckigEngine.defaultOptions()
%
% PURPOSE
%   - Return the direct Ruckig-derived engine defaults.
%
% INPUTS
%   - None.
%
% OUTPUTS
%   - options (scalar struct)
%       Fixed/free-time policy, sampling, and certification tolerance.
%
% UNITS
%   - Time values use the caller's consistent time unit.
%

%% Section 1: Create Defaults

options = struct();
options.TimeMode             = "earliestArrival";
options.FinalTime            = [];
options.SampleTime           = 0.05;
options.ConstraintTolerance  = 1e-7;
options.ArrivalTimeTolerance = 1e-3;
options.Verbose              = false;
end
