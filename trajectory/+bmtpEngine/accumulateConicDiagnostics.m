function stats = accumulateConicDiagnostics(stats, output)
%% Section 0: Header & Readme
% SYNTAX: stats = bmtpEngine.accumulateConicDiagnostics();
%   stats = bmtpEngine.accumulateConicDiagnostics(stats, output)
% PURPOSE: Count production coneprog calls and preserve elapsed solver time.
% INPUTS: Previous statistics and the original solver output with TotalTime_s.
% OUTPUTS: Solver counts and timings; retired backend counters remain zero.
% UNITS: Elapsed time is seconds; counts are completed conic calls.

%% Section 1: Initialize Or Accumulate One Coneprog Call
if nargin == 0
    stats = struct('Solver','coneprog','CallCount',0,'NativeAcceptedCount',0, ...
        'MatlabAcceptedCount',0,'AnalyticalPlaneCount',0,'CertifiedInfeasibleCount',0, ...
        'RecoveryCount',0,'TotalTime_s',0,'LastMethod','','LastRecoveryReason','');
    return;
end
stats.CallCount = stats.CallCount + 1;
stats.TotalTime_s = stats.TotalTime_s + output.TotalTime_s;
stats.LastMethod = 'coneprog';
end
