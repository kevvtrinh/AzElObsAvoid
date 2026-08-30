function serve(protocolRoot, optionOverrides)
%% Section 0: Header & Readme
% SYNTAX
%   webSandbox.fileProtocol.serve(protocolRoot)
%   webSandbox.fileProtocol.serve(protocolRoot, optionOverrides)
%**************************************************************************
% PURPOSE
%   - Keep a MATLAB process available to serve queued browser plan requests.
%**************************************************************************
% INPUTS
%   - protocolRoot (scalar text)
%       Shared local protocol folder initialized by the browser bridge.
%   - optionOverrides (scalar struct, optional; default struct())
%       PollPeriod_s defaults to 0.1 and StopFileName defaults to "stop".
%**************************************************************************
% OUTPUTS
%   - None. Progress is printed as requests complete.
%**************************************************************************
% UNITS
%   - PollPeriod_s is seconds.
%**************************************************************************

%% Section 1: Resolve The Worker Controls

if nargin < 2 || isempty(optionOverrides)
    optionOverrides = struct();
end
if ~isstruct(optionOverrides) || ~isscalar(optionOverrides)
    error("webSandbox:serve:InvalidOptions", ...
        "optionOverrides must be a scalar struct.");
end
options = struct("PollPeriod_s", 0.1, "StopFileName", "stop");
knownNames = string(fieldnames(options));
unknownNames = setdiff(string(fieldnames(optionOverrides)), knownNames);
if ~isempty(unknownNames)
    warning("webSandbox:serve:UnknownOptions", ...
        "Ignoring unknown option fields: %s. No behavior changed.", ...
        strjoin(unknownNames, ", "));
end
for fieldName = reshape(knownNames, 1, [])
    if isfield(optionOverrides, fieldName) && ...
            ~isempty(optionOverrides.(fieldName))
        options.(fieldName) = optionOverrides.(fieldName);
    end
end
validateattributes(options.PollPeriod_s, {'numeric'}, ...
    {'real', 'finite', 'scalar', 'positive'});
options.StopFileName = string(options.StopFileName);
if ~isscalar(options.StopFileName) || strlength(options.StopFileName) == 0
    error("webSandbox:serve:InvalidStopFile", ...
        "StopFileName must be nonempty scalar text.");
end
repositoryRoot = fileparts(fileparts(fileparts(fileparts(fileparts( ...
    mfilename("fullpath"))))));
addpath(repositoryRoot, fullfile(repositoryRoot, "trajectory"));
protocolInfo = webSandbox.fileProtocol.initialize(protocolRoot);
stopPath = fullfile(protocolInfo.ProtocolRoot, options.StopFileName);
fprintf("Web Sandbox file worker ready at %s\n", protocolInfo.ProtocolRoot);

%% Section 2: Process Requests Until The Explicit Stop File Exists

while ~isfile(stopPath)
    processed = webSandbox.fileProtocol.processOnce(protocolInfo.ProtocolRoot);
    if processed.Processed
        fprintf("[Web Sandbox] %s success=%s elapsed=%.3f s: %s\n", ...
            processed.JobId, string(processed.Success), ...
            processed.ElapsedWallTime_s, processed.Message);
    else
        pause(options.PollPeriod_s);
    end
end
delete(stopPath);
fprintf("Web Sandbox file worker stopped.\n");
end
