function stats = accumulateConicDiagnostics(stats, output)
%% Section 0: Header & Readme
% SYNTAX
%   stats = bmtpEngine.accumulateConicDiagnostics()
%   stats = bmtpEngine.accumulateConicDiagnostics(stats, output)
%**************************************************************************
% PURPOSE
%   - Count production coneprog calls and preserve elapsed solver time.
%**************************************************************************
% INPUTS
%   - stats (scalar struct)
%       Statistics accumulated by the previous calls.
%   - output (scalar struct)
%       Original solver output; analytic outputs contribute no call.
%**************************************************************************
% OUTPUTS
%   - stats (scalar struct)
%       Solver name, call count, and total elapsed solver time.
%   - stats (scalar struct, zero-input call)
%       Zeroed accumulator ready for the first call.
%**************************************************************************
% UNITS
%   - TotalTime_s is seconds; CallCount is a completed-call count.
%**************************************************************************

%% Section 1: Return The Zeroed Accumulator
if nargin == 0
    stats             = struct();
    stats.Solver      = 'coneprog';
    stats.CallCount   = 0;
    stats.TotalTime_s = 0;
    return
end

%% Section 2: Accumulate One Production Coneprog Call
outputIsAnalytic = isfield(output, 'IsAnalytic') && output.IsAnalytic;
if outputIsAnalytic
    return
end

callCount = 1;
if isfield(output, 'SolveCount')
    callCount = output.SolveCount;
end

stats.CallCount   = stats.CallCount + callCount;
stats.TotalTime_s = stats.TotalTime_s + output.TotalTime_s;
end
