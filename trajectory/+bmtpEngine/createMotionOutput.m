function candidate = createMotionOutput( ...
        candidate, request, preparedMotion, operations)
%% Section 0: Header & Readme
% SYNTAX
%   candidate = bmtpEngine.createMotionOutput( ...
%       candidate, request, preparedMotion, operations)
%**************************************************************************
% PURPOSE
%   - Convert the prepared BMTP curve into the stable polynomial and sampled
%     motion fields consumed outside the engine.
%**************************************************************************
% INPUTS
%   - candidate (scalar struct)
%       Stable empty candidate record to populate.
%   - request, preparedMotion (scalar structs)
%       Checked request and final prepared control net.
%   - operations (scalar struct of function handles)
%       Motion export kernel owned by BMTP solve.
%**************************************************************************
% OUTPUTS
%   - candidate (scalar struct)
%       Candidate with polynomial, sampled histories, and motion measures.
%**************************************************************************
% UNITS
%   - Position is degrees and time is seconds; derivatives use deg/s,
%     deg/s^2, and deg/s^3.
%**************************************************************************

%% Section 1: Create The Stable Motion Record

candidate = operations.createMotionOutput( ...
    candidate, preparedMotion.ControlPoint_deg, ...
    preparedMotion.SegmentTime_s, request.InitialState.time_s, ...
    request.Options.SampleTime_s, preparedMotion.MotionCertificate);
end
