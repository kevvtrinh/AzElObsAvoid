function stats = accumulateConicDiagnostics(stats, output)
%% Section 0: Header & Readme
% SYNTAX: stats = bmtpEngine.accumulateConicDiagnostics();
%   stats = bmtpEngine.accumulateConicDiagnostics(stats, output)
% PURPOSE: Count production coneprog calls and preserve elapsed solver time.
% INPUTS: Previous statistics and the original solver output with TotalTime_s.
% OUTPUTS: Solver name, call count, and total elapsed solver time.
% UNITS: Elapsed time is seconds; counts are completed conic calls.

%% Section 1: Initialize Or Accumulate One Coneprog Call
if nargin == 0
    stats = struct();
    stats.Solver      = 'coneprog';
    stats.CallCount   = 0;
    stats.TotalTime_s = 0;
    return;
end
stats.CallCount   = stats.CallCount + 1;
stats.TotalTime_s = stats.TotalTime_s + output.TotalTime_s;
end
