function processed = processOnce(protocolRoot)
%% Section 0: Header & Readme
% SYNTAX
%   processed = webSandbox.fileProtocol.processOnce(protocolRoot)
%**************************************************************************
% PURPOSE
%   - Process at most one queued web request through the public planner.
%**************************************************************************
% INPUTS
%   - protocolRoot (scalar text)
%       Shared local protocol folder initialized by the browser bridge.
%**************************************************************************
% OUTPUTS
%   - processed (scalar struct)
%       Whether a request was handled, its job ID, outcome, and elapsed time.
%**************************************************************************
% UNITS
%   - ElapsedWallTime_s is seconds; planner coordinates remain degrees.
%**************************************************************************

%% Section 1: Discover The Oldest Complete Request

repositoryRoot = fileparts(fileparts(fileparts(fileparts(fileparts( ...
    mfilename("fullpath"))))));
addpath(repositoryRoot, fullfile(repositoryRoot, "trajectory"));
protocolInfo = webSandbox.fileProtocol.initialize(protocolRoot);
requestRecords = dir(fullfile(protocolInfo.RequestDirectory, "*.request.json"));
processed = struct( ...
    "Processed", false, "JobId", "", "Success", false, ...
    "ElapsedWallTime_s", NaN, "Message", "No queued request.");
if isempty(requestRecords)
    return;
end
[~, order] = sort([requestRecords.datenum]);
requestRecord = requestRecords(order(1));
requestPath = fullfile(requestRecord.folder, requestRecord.name);
jobId = erase(string(requestRecord.name), ".request.json");

%% Section 2: Run The Maintained Public Planner With File Cancellation

startedTimer = tic;
try
    request = jsondecode(fileread(requestPath));
    inputs = webSandbox.fileProtocol.createRequestInputs(request);
    cancelFile = fullfile(protocolInfo.RequestDirectory, jobId + ".cancel");
    if inputs.Mode == "trajectory"
        options = inputs.plannerOptions;
        options.CancellationCheckFcn = @() isfile(cancelFile);
        result = obstacleAvoidance.planTrajectory( ...
            inputs.obstacles, inputs.initialState, inputs.goalState, ...
            inputs.limits, options);
    else
        options = inputs.interceptOptions;
        if ~isfield(options, "PlannerOptions") || isempty(options.PlannerOptions)
            options.PlannerOptions = struct();
        end
        options.PlannerOptions.CancellationCheckFcn = @() isfile(cancelFile);
        result = obstacleAvoidance.planMovingTargetIntercept( ...
            inputs.obstacles, inputs.initialState, inputs.targetMotion, ...
            inputs.limits, options);
    end
    response = webSandbox.fileProtocol.createResponse(result, toc(startedTimer));
    nativeResult = result;
catch exception
    response = struct( ...
        "Success", false, ...
        "Error", struct( ...
            "Identifier", string(exception.identifier), ...
            "Message", string(exception.message)), ...
        "ElapsedWallTime_s", toc(startedTimer), ...
        "Result", struct(), "Diagnostics", struct());
    nativeResult = struct();
end

%% Section 3: Atomically Publish The Browser Response And Native Result

response.JobId = jobId;
response.Request = request;
writeJsonAtomically(fullfile(protocolInfo.ResponseDirectory, ...
    jobId + ".response.json"), response);
if ~isempty(fieldnames(nativeResult))
    save(fullfile(protocolInfo.NativeDirectory, jobId + ".mat"), ...
        "nativeResult", "-v7.3");
end
delete(requestPath);
if isfile(cancelFile)
    delete(cancelFile);
end
processed = struct( ...
    "Processed", true, "JobId", jobId, ...
    "Success", logical(response.Success), ...
    "ElapsedWallTime_s", response.ElapsedWallTime_s, ...
    "Message", responseMessage(response));
end

function writeJsonAtomically(filePath, payload)
% A rename prevents the bridge from reading an incomplete JSON response.
temporaryPath = filePath + ".tmp";
fileHandle = fopen(temporaryPath, "w", "n", "UTF-8");
if fileHandle < 0
    error("webSandbox:processOnce:ResponseWriteFailed", ...
        "Could not open response file %s.", temporaryPath);
end
cleanup = onCleanup(@() fclose(fileHandle));
fwrite(fileHandle, jsonencode(payload), "char");
clear cleanup;
movefile(temporaryPath, filePath, "f");
end

function message = responseMessage(response)
% Keep worker-console reporting useful even when a request was invalid.
if response.Success
    message = response.Result.Message;
elseif isfield(response, "Error") && isfield(response.Error, "Message")
    message = response.Error.Message;
else
    message = "Planner returned an unsuccessful response.";
end
end
