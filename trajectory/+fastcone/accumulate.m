function stats=accumulate(stats,output)
%% Section 0: Header & Readme
% SYNTAX: stats=fastcone.accumulate(); stats=fastcone.accumulate(stats,output)
% PURPOSE: Preserve executed solver methods, recovery counts, and elapsed time.
% INPUTS: Previous scalar statistics and one fastcone.solve output structure.
% OUTPUTS: Updated statistics; no persistent state or planner decisions.
% UNITS: Elapsed time is seconds; counts are completed conic calls.

%% Section 1: Initialize Or Accumulate One Solve
if nargin==0
    stats=struct('Solver','fastcone','CallCount',0,'NativeAcceptedCount',0, ...
        'AnalyticalPlaneCount',0,'RecoveryCount',0,'TotalTime_s',0, ...
        'LastMethod','','LastRecoveryReason','');
    return
end
stats.CallCount=stats.CallCount+1;
stats.NativeAcceptedCount=stats.NativeAcceptedCount+double(output.NativeAccepted);
stats.AnalyticalPlaneCount=stats.AnalyticalPlaneCount+ ...
    double(~output.FallbackUsed && ~output.NativeAccepted);
stats.RecoveryCount=stats.RecoveryCount+double(output.FallbackUsed);
stats.TotalTime_s=stats.TotalTime_s+output.TotalTime_s;
stats.LastMethod=output.Method;
if output.FallbackUsed && isfield(output.Prototype,'message')
    stats.LastRecoveryReason=output.Prototype.message;
end
end
