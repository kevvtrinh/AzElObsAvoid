function response = createResponse(result, elapsedWallTime_s)
%% Section 0: Header & Readme
% SYNTAX
%   response = webSandbox.fileProtocol.createResponse(result, elapsedWallTime_s)
%**************************************************************************
% PURPOSE
%   - Select the public motion and diagnostics needed by the browser client.
%**************************************************************************
% INPUTS
%   - result (scalar struct)
%       Result from an existing public planner entry point.
%   - elapsedWallTime_s (nonnegative scalar)
%       Wall time measured around the public planner call.
%**************************************************************************
% OUTPUTS
%   - response (scalar struct)
%       JSON-safe status, motion histories, validation, and search summaries.
%**************************************************************************
% UNITS
%   - Positions are degrees and time quantities are seconds.
%**************************************************************************

%% Section 1: Copy Stable Planner Payloads

motion = struct( ...
    "Success", logical(result.Success), "Message", string(result.Message), ...
    "TerminationReason", string(result.TerminationReason), ...
    "ArrivalTime_s", fieldOr(result, "ArrivalTime_s", NaN), ...
    "TrajectoryDuration_s", fieldOr(result, "TrajectoryDuration_s", NaN), ...
    "time_s", fieldOr(result, "time_s", zeros(0, 1)), ...
    "position_deg", fieldOr(result, "position_deg", zeros(0, 2)), ...
    "velocity_deg_s", fieldOr(result, "velocity_deg_s", zeros(0, 2)), ...
    "acceleration_deg_s2", fieldOr(result, "acceleration_deg_s2", ...
        zeros(0, 2)), ...
    "jerk_deg_s3", fieldOr(result, "jerk_deg_s3", zeros(0, 2)));
searchDiagnostics = fieldOr(result, "SearchDiagnostics", struct());
response = struct( ...
    "Success", motion.Success, ...
    "Error", struct(), ...
    "ElapsedWallTime_s", double(elapsedWallTime_s), ...
    "Result", motion, ...
    "Diagnostics", struct( ...
        "TerminationReason", motion.TerminationReason, ...
        "Validation", scalarSummary(fieldOr(result, "Validation", struct())), ...
        "StageTiming", stageTimingSummary(searchDiagnostics), ...
        "SeedSummaries", seedSummary(searchDiagnostics), ...
        "SearchCounts", scalarSummary(searchDiagnostics)));
end

function value = fieldOr(source, fieldName, fallback)
% Read an optional stable field without coupling to planner-private content.
value = fallback;
if isstruct(source) && isscalar(source) && isfield(source, fieldName)
    value = source.(fieldName);
end
end

function summary = scalarSummary(source)
% Keep small scalar and vector diagnostic values readable in the browser.
summary = struct();
if ~isstruct(source) || ~isscalar(source)
    return;
end
fieldNames = string(fieldnames(source));
for fieldName = reshape(fieldNames, 1, [])
    value = source.(fieldName);
    if (isnumeric(value) || islogical(value) || isstring(value) || ...
            ischar(value)) && numel(value) <= 64
        summary.(fieldName) = value;
    end
end
end

function summary = stageTimingSummary(searchDiagnostics)
% Convert tables when present while retaining the planner's timing vocabulary.
summary = struct();
if ~isstruct(searchDiagnostics) || ~isscalar(searchDiagnostics) || ...
        ~isfield(searchDiagnostics, "StageTiming")
    return;
end
stageTiming = searchDiagnostics.StageTiming;
if istable(stageTiming)
    stageTiming = table2struct(stageTiming);
end
if isstruct(stageTiming) && isscalar(stageTiming)
    summary = scalarSummary(stageTiming);
elseif isstruct(stageTiming)
    summary = stageTiming;
end
end

function summary = seedSummary(searchDiagnostics)
% Preserve per-seed scalar evidence without sending geometrically large traces.
summary = cell(0, 1);
if ~isstruct(searchDiagnostics) || ~isscalar(searchDiagnostics) || ...
        ~isfield(searchDiagnostics, "SeedSummaries") || ...
        ~isstruct(searchDiagnostics.SeedSummaries)
    return;
end
source = searchDiagnostics.SeedSummaries;
summary = cell(numel(source), 1);
for seedIndex = 1:numel(source)
    % Seed records legitimately expose different fields for different methods;
    % a cell keeps those truthful differences instead of forcing empty aliases.
    summary{seedIndex} = scalarSummary(source(seedIndex));
end
end
