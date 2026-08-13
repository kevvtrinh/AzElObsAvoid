function [useParallel, diagnostics] = resolveAzElParallelExecution( ...
        requestedMode, verbose, stageName, taskCount)
%% Section 0: Header & Readme
% SYNTAX
%   [useParallel, diagnostics] = resolveAzElParallelExecution( ...
%       requestedMode, verbose, stageName, taskCount)
%**************************************************************************
% PURPOSE
%   - Resolve auto/on/off parallel execution without making Parallel
%     Computing Toolbox a required dependency.
%   - Start or reuse a local pool when parallel work was requested and the
%     toolbox is available; otherwise return a documented serial fallback.
%**************************************************************************
% INPUTS
%   - requestedMode ("auto", "on", "off", or logical scalar)
%   - verbose (logical scalar)
%   - stageName (scalar text used only in progress output)
%   - taskCount (nonnegative integer; a single task remains serial)
%**************************************************************************
% OUTPUTS
%   - useParallel (logical scalar)
%   - diagnostics (scalar struct)
%       RequestedMode, ToolboxAvailable, Enabled, WorkerCount, Message.
%**************************************************************************
% UNITS
%   - taskCount and WorkerCount are dimensionless.
%**************************************************************************

%% Section 1: Validate Inputs & Resolve Execution Mode
if nargin < 1 || isempty(requestedMode)
    requestedMode = "auto";
end
if nargin < 2 || isempty(verbose)
    verbose = false;
end
if nargin < 3 || isempty(stageName)
    stageName = "az/el work";
end
if nargin < 4 || isempty(taskCount)
    taskCount = Inf;
end
validateattributes(verbose, {'logical','numeric'}, {'scalar'});
verbose = logical(verbose);
stageName = string(stageName);
if ~isscalar(stageName)
    error("resolveAzElParallelExecution:InvalidStageName", ...
        "stageName must be scalar text.");
end
validateattributes(taskCount, {'numeric'}, ...
    {'real','scalar','nonnegative'});

if (islogical(requestedMode) || isnumeric(requestedMode)) && ...
        isscalar(requestedMode)
    validateattributes(requestedMode, ...
        {'logical','numeric'}, {'real','finite','scalar'});
    if isnumeric(requestedMode) && ~any(requestedMode == [0 1])
        error("resolveAzElParallelExecution:InvalidMode", ...
            "Numeric UseParallel values must be zero or one.");
    end
    if logical(requestedMode)
        requestedMode = "on";
    else
        requestedMode = "off";
    end
else
    requestedMode = lower(string(requestedMode));
end
if ~isscalar(requestedMode) || ...
        ~any(requestedMode == ["auto" "on" "off"])
    error("resolveAzElParallelExecution:InvalidMode", ...
        "UseParallel must be auto, on, off, or a logical scalar.");
end

toolboxAvailable = license("test", "Distrib_Computing_Toolbox") && ...
    exist("parpool", "file") == 2 && exist("gcp", "file") == 2 && ...
    exist("parallel.pool.DataQueue", "class") == 8;
useParallel = requestedMode ~= "off" && toolboxAvailable && taskCount > 1;
workerCount = 0;
message = "Serial execution selected.";
if taskCount <= 1
    message = "One or fewer independent tasks; using serial execution.";
elseif requestedMode ~= "off" && ~toolboxAvailable
    message = "Parallel Computing Toolbox is unavailable; using serial execution.";
elseif useParallel
    try
        pool = gcp("nocreate");
        if isempty(pool)
            pool = parpool;
        end
        workerCount = pool.NumWorkers;
        message = sprintf( ...
            "Parallel execution enabled with %d workers.", workerCount);
    catch parallelError
        useParallel = false;
        message = "Parallel pool was unavailable; using serial execution. " + ...
            string(parallelError.message);
    end
end
diagnostics = struct( ...
    "RequestedMode", requestedMode, ...
    "ToolboxAvailable", toolboxAvailable, ...
    "Enabled", useParallel, ...
    "WorkerCount", workerCount, ...
    "Message", string(message));
if verbose
    fprintf("[parallel] %s: %s\n", stageName, diagnostics.Message);
end
end
