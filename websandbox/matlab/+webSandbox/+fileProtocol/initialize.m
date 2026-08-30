function protocolInfo = initialize(protocolRoot)
%% Section 0: Header & Readme
% SYNTAX
%   protocolInfo = webSandbox.fileProtocol.initialize(protocolRoot)
%**************************************************************************
% PURPOSE
%   - Create the local request, response, and native-result folders.
%**************************************************************************
% INPUTS
%   - protocolRoot (scalar text)
%       Shared local folder used only by the browser bridge and MATLAB worker.
%**************************************************************************
% OUTPUTS
%   - protocolInfo (scalar struct)
%       Absolute protocol, request, response, and native-result paths.
%**************************************************************************
% UNITS
%   - Not applicable.
%**************************************************************************

%% Section 1: Create The Explicit Local Protocol Layout

protocolRoot = string(protocolRoot);
if ~isscalar(protocolRoot) || ismissing(protocolRoot) || ...
        strlength(strtrim(protocolRoot)) == 0
    error("webSandbox:initialize:InvalidProtocolRoot", ...
        "protocolRoot must be nonempty scalar text.");
end
protocolRoot = string(char(java.io.File(char(protocolRoot)).getCanonicalPath()));
requestDirectory = fullfile(protocolRoot, "requests");
responseDirectory = fullfile(protocolRoot, "responses");
nativeDirectory = fullfile(protocolRoot, "native");
directories = [protocolRoot, requestDirectory, responseDirectory, ...
    nativeDirectory];
for directoryName = reshape(directories, 1, [])
    if ~isfolder(directoryName)
        mkdir(directoryName);
    end
end
protocolInfo = struct( ...
    "ProtocolRoot", protocolRoot, ...
    "RequestDirectory", string(requestDirectory), ...
    "ResponseDirectory", string(responseDirectory), ...
    "NativeDirectory", string(nativeDirectory));
end
