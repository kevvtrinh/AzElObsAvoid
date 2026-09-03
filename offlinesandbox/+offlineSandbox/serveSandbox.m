function serverInfo = serveSandbox(port)
%% Section 0: Header & Readme
% SYNTAX
%   serverInfo = offlineSandbox.serveSandbox()
%   serverInfo = offlineSandbox.serveSandbox(port)
%**************************************************************************
% PURPOSE
%   - Serve the dependency-free planner page and its existing JSON adapter
%     from MATLAB on the IPv4 loopback interface only.
%**************************************************************************
% INPUTS
%   - port (numeric scalar integer, optional; default 52731)
%       TCP port in [1024, 65535]. Empty selects the default.
%**************************************************************************
% OUTPUTS
%   - serverInfo (scalar struct)
%       URL, port, stop-file path, plan and bundle request counts, elapsed
%       time, and the termination reason after the blocking server loop stops.
%       Bundle replays are counted as plan requests.
%**************************************************************************
% UNITS
%   - Port and request counts are dimensionless. Elapsed time is seconds.
%**************************************************************************

%% Section 1: Validate Inputs & Resolve Local Resources

defaultPort = 52731;
if nargin < 1 || isempty(port)
    port = defaultPort;
end
validateattributes(port, {'numeric'}, ...
    {'real', 'finite', 'scalar', 'integer', '>=', 1024, '<=', 65535}, ...
    "serveSandbox", "port");
port = double(port);

sandboxFolder = fileparts(fileparts(mfilename("fullpath")));
pagePath = fullfile(sandboxFolder, "az_el_planner_sandbox.html");
if ~isfile(pagePath)
    error("serveSandbox:PageNotFound", ...
        "The sandbox page does not exist: %s", pagePath);
end
pageBytes = readFileBytes(pagePath);
stopFilePath = fullfile(tempdir, ...
    "offlineSandbox-stop-" + string(port) + ".txt");
bundleFilePath = fullfile(tempdir, ...
    "offlineSandbox-bundle-" + string(port) + ".mat");
bundleRequestIdPath = fullfile(tempdir, ...
    "offlineSandbox-bundle-" + string(port) + ".txt");

%% Section 2: Bind The Loopback Server

loopbackAddress = java.net.InetAddress.getByName('127.0.0.1');
try
    serverSocket = java.net.ServerSocket( ...
        int32(port), int32(8), loopbackAddress);
catch exception
    error("serveSandbox:BindFailed", ...
        "Could not bind 127.0.0.1:%d: %s", port, exception.message);
end
% Delete a stale marker only after binding. A second server therefore cannot
% consume the running server's stop request before its own bind fails.
deleteFileIfPresent(stopFilePath);
deleteFileIfPresent(bundleFilePath);
deleteFileIfPresent(bundleRequestIdPath);
acceptTimeout_ms = 250;
serverSocket.setSoTimeout(int32(acceptTimeout_ms));
serverCleanup = onCleanup( ...
    @() closeServer(serverSocket, stopFilePath, bundleFilePath, ...
        bundleRequestIdPath));

url = "http://127.0.0.1:" + string(port) + "/";
fprintf("Az/El planner sandbox: %s\n", url);
fprintf("Stop file: %s\n", stopFilePath);
fprintf("Press Ctrl-C or create the stop file to stop the server.\n");

%% Section 3: Serve Requests Until Stopped

serverTimer = tic;
requestCount = 0;
planRequestCount = 0;
bundleRequestCount = 0;
terminationReason = "stopFile";
while ~isfile(stopFilePath)
    try
        clientSocket = serverSocket.accept();
    catch exception
        if isSocketTimeout(exception)
            continue;
        end
        rethrow(exception);
    end

    requestCount = requestCount + 1;
    try
        [wasPlanRequest, wasBundleRequest] = serveClient( ...
            clientSocket, serverSocket, pageBytes, sandboxFolder, ...
            port, stopFilePath, bundleFilePath, bundleRequestIdPath);
        planRequestCount = planRequestCount + double(wasPlanRequest);
        bundleRequestCount = ...
            bundleRequestCount + double(wasBundleRequest);
    catch exception
        if isUserInterruption(exception)
            rethrow(exception);
        end
        fprintf(2, "serveSandbox request failed: %s\n", exception.message);
    end
end

%% Section 4: Close The Server & Return A Summary

elapsedTime_s = toc(serverTimer);
serverInfo = struct( ...
    "URL", url, ...
    "Port", port, ...
    "StopFilePath", stopFilePath, ...
    "RequestCount", requestCount, ...
    "PlanRequestCount", planRequestCount, ...
    "BundleRequestCount", bundleRequestCount, ...
    "ElapsedTime_s", elapsedTime_s, ...
    "TerminationReason", terminationReason);
clear serverCleanup;
fprintf("Sandbox server stopped after %.3f s (%d requests).\n", ...
    elapsedTime_s, requestCount);

end

%% Section 5: Local Functions

function [wasPlanRequest, wasBundleRequest] = serveClient( ...
        clientSocket, serverSocket, pageBytes, sandboxFolder, port, ...
        stopFilePath, bundleFilePath, bundleRequestIdPath)
% Read, route, and close one HTTP/1.1 connection.
clientCleanup = onCleanup(@() closeSocket(clientSocket));
wasPlanRequest = false;
wasBundleRequest = false;
try
    request = readHttpRequest(clientSocket, 2000);
catch exception
    if isUserInterruption(exception)
        rethrow(exception);
    end
    writeErrorResponse(clientSocket, requestErrorStatus(exception), ...
        exception.identifier, exception.message, "");
    return;
end

method = request.Method;
path = request.Path;
corsOrigin = allowedCorsOrigin(request.Origin, request.HasOrigin, port);
if request.HasOrigin && strlength(corsOrigin) == 0
    writeErrorResponse(clientSocket, 403, ...
        "serveSandbox:OriginNotAllowed", ...
        "Browser requests are accepted only from this loopback page or " + ...
        "from the local file page.", "");
    return;
end
knownPath = any(path == ["/", "/health", "/plan", "/cancel", ...
    "/bundle", "/run-bundle"]);
if method == "OPTIONS" && knownPath
    writeHttpResponse(clientSocket, 204, "No Content", ...
        "text/plain; charset=utf-8", zeros(1, 0, 'uint8'), ...
        strings(0, 1), corsOrigin);
elseif method == "GET" && path == "/"
    writeHttpResponse(clientSocket, 200, "OK", ...
        "text/html; charset=utf-8", pageBytes, strings(0, 1), corsOrigin);
elseif method == "GET" && path == "/health"
    body = createHealthBody("ready", port, sandboxFolder);
    writeJsonResponse(clientSocket, 200, "OK", body, ...
        strings(0, 1), corsOrigin);
elseif method == "POST" && path == "/plan"
    wasPlanRequest = true;
    servePlanningRequest( ...
        clientSocket, serverSocket, request.BodyBytes, stopFilePath, ...
        port, corsOrigin, bundleFilePath, bundleRequestIdPath);
elseif method == "POST" && path == "/run-bundle"
    wasPlanRequest = true;
    serveBundleReplayRequest( ...
        clientSocket, serverSocket, request.BodyBytes, stopFilePath, ...
        port, corsOrigin, bundleFilePath, bundleRequestIdPath);
elseif method == "POST" && path == "/bundle"
    wasBundleRequest = true;
    serveBundleRequest(clientSocket, request.BodyBytes, corsOrigin, ...
        bundleFilePath, bundleRequestIdPath);
elseif method == "POST" && path == "/cancel"
    writeErrorResponse(clientSocket, 409, ...
        "serveSandbox:NoActivePlan", ...
        "No planning request is active.", corsOrigin);
elseif knownPath
    writeErrorResponse(clientSocket, 405, ...
        "serveSandbox:MethodNotAllowed", ...
        "The requested HTTP method is not supported for this path.", ...
        corsOrigin);
else
    writeErrorResponse(clientSocket, 404, ...
        "serveSandbox:NotFound", "The requested path was not found.", ...
        corsOrigin);
end
clear clientCleanup;
end

function servePlanningRequest(clientSocket, serverSocket, ...
        requestBytes, stopFilePath, port, corsOrigin, bundleFilePath, ...
        bundleRequestIdPath)
% Run the unchanged file adapter and return its exact result JSON bytes.
if isempty(requestBytes)
    writeErrorResponse(clientSocket, 400, ...
        "serveSandbox:EmptyPlanRequest", ...
        "POST /plan requires a JSON request body.", corsOrigin);
    return;
end

requestFilePath = string(tempname) + ".json";
resultFilePath = string(tempname) + ".json";
temporaryCleanup = onCleanup(@() deleteTemporaryFiles( ...
    requestFilePath, resultFilePath));
planningTimer = tic;
try
    writeFileBytes(requestFilePath, requestBytes);
    activeRequestId = previewRequestId(requestBytes);
    [response, diagnosisBundle] = offlineSandbox.runPlanningRequest( ...
        requestFilePath, resultFilePath, ...
        @() cancellationRequested( ...
            serverSocket, stopFilePath, activeRequestId, port));
    resultBytes = readFileBytes(resultFilePath);
    cacheDiagnosisBundle(bundleFilePath, bundleRequestIdPath, ...
        activeRequestId, diagnosisBundle);
catch exception
    if isUserInterruption(exception)
        rethrow(exception);
    end
    if exception.identifier == "planTrajectory:UserCancelled"
        writeErrorResponse(clientSocket, 409, exception.identifier, ...
            "Planning was canceled at a safe planner checkpoint.", ...
            corsOrigin);
    elseif isRequestFailure(exception)
        writeErrorResponse(clientSocket, 400, exception.identifier, ...
            exception.message, corsOrigin);
    else
        writeErrorResponse(clientSocket, 500, ...
            "serveSandbox:PlanningFailed", exception.message, corsOrigin);
    end
    return;
end

serverTime_s = toc(planningTimer);
plannerTime_s = response.result.ElapsedPlanningTime_s;
timingHeaders = [ ...
    "Server-Timing: planner;dur=" + ...
        compose("%.6f", 1000 * plannerTime_s) + ...
        ", server;dur=" + compose("%.6f", 1000 * serverTime_s); ...
    "X-Offline-Sandbox-Planner-Time-s: " + ...
        compose("%.9f", plannerTime_s); ...
    "X-Offline-Sandbox-Server-Time-s: " + ...
        compose("%.9f", serverTime_s)];
writeHttpResponse(clientSocket, 200, "OK", ...
    "application/json; charset=utf-8", resultBytes, timingHeaders, ...
    corsOrigin);
fprintf("Plan %s: planner %.6f s; server before transport %.6f s.\n", ...
    activeRequestId, plannerTime_s, serverTime_s);
clear temporaryCleanup;
end

function serveBundleReplayRequest(clientSocket, serverSocket, ...
        bundleBytes, stopFilePath, port, corsOrigin, bundleFilePath, ...
        bundleRequestIdPath)
% Run one uploaded diagnosis bundle and return the fresh browser result.
if isempty(bundleBytes)
    writeErrorResponse(clientSocket, 400, ...
        "serveSandbox:EmptyBundleReplayRequest", ...
        "POST /run-bundle requires a diagnosis MAT-file body.", ...
        corsOrigin);
    return;
end

uploadedBundlePath = string(tempname) + ".mat";
resultFilePath = string(tempname) + ".json";
temporaryCleanup = onCleanup(@() deleteTemporaryFiles( ...
    uploadedBundlePath, resultFilePath));
planningTimer = tic;
activeRequestId = "bundle-replay";
try
    writeFileBytes(uploadedBundlePath, bundleBytes);
    [response, reproducedBundle] = ...
        offlineSandbox.replayDiagnosisBundle( ...
        uploadedBundlePath, resultFilePath, ...
        @() cancellationRequested( ...
            serverSocket, stopFilePath, activeRequestId, port));
    resultBytes = readFileBytes(resultFilePath);
    cacheDiagnosisBundle(bundleFilePath, bundleRequestIdPath, ...
        string(response.requestId), reproducedBundle);
catch exception
    if isUserInterruption(exception)
        rethrow(exception);
    end
    if exception.identifier == "planTrajectory:UserCancelled"
        writeErrorResponse(clientSocket, 409, exception.identifier, ...
            "Bundle replay was canceled at a safe planner checkpoint.", ...
            corsOrigin);
    elseif isRequestFailure(exception) || ...
            startsWith(string(exception.identifier), ...
            "replayDiagnosisBundle:")
        writeErrorResponse(clientSocket, 400, exception.identifier, ...
            exception.message, corsOrigin);
    else
        writeErrorResponse(clientSocket, 500, ...
            "serveSandbox:BundleReplayFailed", exception.message, ...
            corsOrigin);
    end
    return;
end

serverTime_s = toc(planningTimer);
plannerTime_s = response.result.ElapsedPlanningTime_s;
timingHeaders = [ ...
    "Server-Timing: planner;dur=" + ...
        compose("%.6f", 1000 * plannerTime_s) + ...
        ", server;dur=" + compose("%.6f", 1000 * serverTime_s); ...
    "X-Offline-Sandbox-Planner-Time-s: " + ...
        compose("%.9f", plannerTime_s); ...
    "X-Offline-Sandbox-Server-Time-s: " + ...
        compose("%.9f", serverTime_s)];
writeHttpResponse(clientSocket, 200, "OK", ...
    "application/json; charset=utf-8", resultBytes, timingHeaders, ...
    corsOrigin);
fprintf("Bundle replay %s: planner %.6f s; server %.6f s.\n", ...
    string(response.requestId), plannerTime_s, serverTime_s);
clear temporaryCleanup;
end

function serveBundleRequest(clientSocket, requestBytes, corsOrigin, ...
        bundleFilePath, bundleRequestIdPath)
% Return the exact cached MAT diagnosis bundle for the matching live result.
requestId = previewRequestId(requestBytes);
if strlength(requestId) == 0
    writeErrorResponse(clientSocket, 400, ...
        "serveSandbox:InvalidBundleRequest", ...
        "POST /bundle requires a nonempty JSON requestId.", corsOrigin);
    return;
end
if ~isfile(bundleFilePath) || ~isfile(bundleRequestIdPath)
    writeErrorResponse(clientSocket, 404, ...
        "serveSandbox:BundleNotAvailable", ...
        "No completed live plan is available for bundle export.", ...
        corsOrigin);
    return;
end
cachedRequestId = strtrim(string(native2unicode( ...
    readFileBytes(bundleRequestIdPath), "UTF-8")));
if cachedRequestId ~= requestId
    writeErrorResponse(clientSocket, 409, ...
        "serveSandbox:BundleRequestMismatch", ...
        "The requested result is not the latest live plan on this server.", ...
        corsOrigin);
    return;
end
bundleBytes = readFileBytes(bundleFilePath);
downloadHeaders = ...
    "Content-Disposition: attachment; filename=az-el-sandbox-bundle.mat";
writeHttpResponse(clientSocket, 200, "OK", ...
    "application/vnd.matlab.mat-file", bundleBytes, downloadHeaders, ...
    corsOrigin);
end

function cacheDiagnosisBundle(bundleFilePath, bundleRequestIdPath, ...
        requestId, diagnosisBundle)
% Atomically replace the server-owned bundle cache after a completed plan.
temporaryBundlePath = string(tempname) + ".mat";
temporaryRequestIdPath = string(tempname) + ".txt";
cacheCleanup = onCleanup(@() deleteTemporaryFiles( ...
    temporaryBundlePath, temporaryRequestIdPath));
save(char(temporaryBundlePath), 'diagnosisBundle', '-v7.3');
writeFileBytes(temporaryRequestIdPath, ...
    unicode2native(char(requestId), "UTF-8"));
[bundleMoved, bundleMessage] = movefile( ...
    temporaryBundlePath, bundleFilePath, "f");
if ~bundleMoved || ~isfile(bundleFilePath)
    error("serveSandbox:BundleCacheFailed", ...
        "Could not cache the diagnosis bundle: %s", bundleMessage);
end
[requestIdMoved, requestIdMessage] = movefile( ...
    temporaryRequestIdPath, bundleRequestIdPath, "f");
if ~requestIdMoved || ~isfile(bundleRequestIdPath)
    deleteFileIfPresent(bundleFilePath);
    error("serveSandbox:BundleCacheFailed", ...
        "Could not cache the bundle request identifier: %s", ...
        requestIdMessage);
end
clear cacheCleanup;
end

function stopRequested = cancellationRequested( ...
        serverSocket, stopFilePath, activeRequestId, port)
% Poll the stop file and queued HTTP cancellation without blocking planning.
stopRequested = isfile(stopFilePath);
if stopRequested
    return;
end

previousTimeout_ms = serverSocket.getSoTimeout();
serverSocket.setSoTimeout(int32(1));
timeoutCleanup = onCleanup( ...
    @() serverSocket.setSoTimeout(previousTimeout_ms));
try
    queuedSocket = serverSocket.accept();
catch exception
    if isSocketTimeout(exception)
        return;
    end
    rethrow(exception);
end
queuedCleanup = onCleanup(@() closeSocket(queuedSocket));

try
    request = readHttpRequest(queuedSocket, 100);
catch exception
    if isUserInterruption(exception)
        rethrow(exception);
    end
    writeErrorResponse(queuedSocket, requestErrorStatus(exception), ...
        exception.identifier, exception.message, "");
    return;
end
corsOrigin = allowedCorsOrigin(request.Origin, request.HasOrigin, port);
if request.HasOrigin && strlength(corsOrigin) == 0
    writeErrorResponse(queuedSocket, 403, ...
        "serveSandbox:OriginNotAllowed", ...
        "The browser origin is not allowed.", "");
elseif request.Method == "OPTIONS" && ...
        any(request.Path == ["/health", "/cancel"])
    writeHttpResponse(queuedSocket, 204, "No Content", ...
        "text/plain; charset=utf-8", zeros(1, 0, 'uint8'), ...
        strings(0, 1), corsOrigin);
elseif request.Method == "GET" && request.Path == "/health"
    body = struct( ...
        "schemaVersion", "offlineSandboxTransport/v1", ...
        "status", "planning");
    writeJsonResponse(queuedSocket, 200, "OK", body, ...
        strings(0, 1), corsOrigin);
elseif request.Method == "POST" && request.Path == "/cancel"
    cancelRequestId = previewRequestId(request.BodyBytes);
    stopRequested = strlength(activeRequestId) > 0 && ...
        cancelRequestId == activeRequestId;
    if stopRequested
        body = struct("accepted", true, "requestId", activeRequestId);
        writeJsonResponse(queuedSocket, 202, "Accepted", body, ...
            strings(0, 1), corsOrigin);
    else
        writeErrorResponse(queuedSocket, 409, ...
            "serveSandbox:CancellationRequestMismatch", ...
            "The cancellation requestId does not match the active plan.", ...
            corsOrigin);
    end
else
    writeErrorResponse(queuedSocket, 503, ...
        "serveSandbox:PlanningInProgress", ...
        "MATLAB is already processing a planning request.", corsOrigin);
end
clear queuedCleanup timeoutCleanup;
end

function request = readHttpRequest(clientSocket, readTimeout_ms)
% Read one bounded Content-Length request and reject unsupported framing.
maximumHeaderBytes = 32768;
clientSocket.setSoTimeout(int32(readTimeout_ms));
inputStream = clientSocket.getInputStream();
headerBytes = zeros(1, maximumHeaderBytes, 'uint8');
headerByteCount = 0;
headerComplete = false;
while headerByteCount < maximumHeaderBytes
    value = inputStream.read();
    if value < 0
        error("serveSandbox:TruncatedHeaders", ...
            "The HTTP connection ended before the headers were complete.");
    end
    headerByteCount = headerByteCount + 1;
    headerBytes(headerByteCount) = uint8(value);
    if headerByteCount >= 4 && isequal( ...
            headerBytes(headerByteCount - 3:headerByteCount), ...
            uint8([13 10 13 10]))
        headerComplete = true;
        break;
    end
end
if ~headerComplete
    error("serveSandbox:HeadersTooLarge", ...
        "HTTP headers exceed the 32768-byte limit.");
end

headerText = native2unicode( ...
    headerBytes(1:headerByteCount), "UTF-8");
headerText = strrep(headerText, char([13 10]), newline);
lines = regexp(headerText, '\n', 'split');
requestParts = regexp(strtrim(lines{1}), '\s+', 'split');
if numel(requestParts) ~= 3 || ~startsWith(requestParts{3}, 'HTTP/')
    error("serveSandbox:InvalidRequestLine", ...
        "The HTTP request line is invalid.");
end
method = upper(string(requestParts{1}));
target = string(requestParts{2});
if ~startsWith(target, "/")
    error("serveSandbox:InvalidRequestTarget", ...
        "The HTTP request target must be an absolute path.");
end
targetParts = split(target, "?");
path = targetParts(1);
if path == "/run-bundle"
    maximumBodyBytes = 128 * 1024 * 1024;
else
    maximumBodyBytes = 16 * 1024 * 1024;
end

contentLength = 0;
hasContentLength = false;
origin = "";
hasOrigin = false;
for lineIndex = 2:numel(lines)
    line = lines{lineIndex};
    if isempty(line)
        continue;
    end
    colonIndex = find(line == ':', 1, 'first');
    if isempty(colonIndex)
        error("serveSandbox:InvalidHeader", ...
            "Each HTTP header must contain a colon.");
    end
    name = lower(strtrim(string(line(1:colonIndex - 1))));
    value = strtrim(string(line(colonIndex + 1:end)));
    if name == "transfer-encoding" && lower(value) ~= "identity"
        error("serveSandbox:UnsupportedTransferEncoding", ...
            "Chunked or encoded HTTP request bodies are not supported.");
    elseif name == "content-length"
        if hasContentLength || isempty(regexp(char(value), '^\d+$', 'once'))
            error("serveSandbox:InvalidContentLength", ...
                "Content-Length must be one nonnegative decimal integer.");
        end
        contentLength = str2double(value);
        hasContentLength = true;
    elseif name == "origin"
        if hasOrigin
            error("serveSandbox:DuplicateOrigin", ...
                "An HTTP request may contain only one Origin header.");
        end
        origin = value;
        hasOrigin = true;
    end
end
if ~isfinite(contentLength) || contentLength > maximumBodyBytes
    error("serveSandbox:PayloadTooLarge", ...
        "HTTP request bodies may not exceed %d bytes.", maximumBodyBytes);
end

if contentLength == 0
    bodyBytes = zeros(1, 0, 'uint8');
else
    bodyBuffer = java.nio.ByteBuffer.allocate(int32(contentLength));
    bodyChannel = java.nio.channels.Channels.newChannel(inputStream);
    while bodyBuffer.hasRemaining()
        readByteCount = bodyChannel.read(bodyBuffer);
        if readByteCount < 0
            error("serveSandbox:TruncatedBody", ...
                "The HTTP body ended before Content-Length bytes arrived.");
        end
    end
    signedBody = int8(bodyBuffer.array());
    bodyBytes = typecast(reshape(signedBody, 1, []), 'uint8');
end
request = struct( ...
    "Method", method, ...
    "Path", path, ...
    "BodyBytes", bodyBytes, ...
    "Origin", origin, ...
    "HasOrigin", hasOrigin);
end

function writeJsonResponse( ...
        clientSocket, statusCode, reason, value, headers, corsOrigin)
% Encode one JSON response without changing planning-result projection.
jsonText = jsonencode(value);
bodyBytes = unicode2native(char(jsonText), "UTF-8");
writeHttpResponse(clientSocket, statusCode, reason, ...
    "application/json; charset=utf-8", bodyBytes, headers, corsOrigin);
end

function writeErrorResponse( ...
        clientSocket, statusCode, identifier, message, corsOrigin)
% Return a stable transport error without representing it as a plan result.
body = struct( ...
    "schemaVersion", "offlineSandboxError/v1", ...
    "error", struct( ...
        "identifier", string(identifier), ...
        "message", string(message)));
writeJsonResponse(clientSocket, statusCode, httpReason(statusCode), ...
    body, strings(0, 1), corsOrigin);
end

function writeHttpResponse(clientSocket, statusCode, reason, ...
        contentType, bodyBytes, additionalHeaders, corsOrigin)
% Write one byte-counted response and always close the HTTP connection.
bodyBytes = reshape(uint8(bodyBytes), 1, []);
headerLines = [ ...
    "HTTP/1.1 " + string(statusCode) + " " + string(reason); ...
    "Content-Type: " + string(contentType); ...
    "Content-Length: " + string(numel(bodyBytes)); ...
    "Connection: close"; ...
    "Cache-Control: no-store"; ...
    "X-Offline-Sandbox-Transport: offlineSandboxHttp/v1"; ...
    reshape(string(additionalHeaders), [], 1)];
if strlength(corsOrigin) > 0
    headerLines = [headerLines; ...
        "Access-Control-Allow-Origin: " + corsOrigin; ...
        "Vary: Origin"; ...
        "Access-Control-Allow-Methods: GET, POST, OPTIONS"; ...
        "Access-Control-Allow-Headers: Content-Type"; ...
        "Access-Control-Expose-Headers: Server-Timing, " + ...
            "X-Offline-Sandbox-Planner-Time-s, " + ...
            "X-Offline-Sandbox-Server-Time-s"];
end
headerLines = [headerLines; ""; ""];
headerText = strjoin(headerLines, sprintf('\r\n'));
headerBytes = unicode2native(char(headerText), "UTF-8");
outputStream = clientSocket.getOutputStream();
writeJavaBytes(outputStream, headerBytes);
writeJavaBytes(outputStream, bodyBytes);
outputStream.flush();
end

function writeJavaBytes(outputStream, bytes)
% Convert unsigned MATLAB bytes to Java's signed byte representation.
if isempty(bytes)
    return;
end
signedBytes = typecast(reshape(uint8(bytes), 1, []), 'int8');
outputStream.write(signedBytes, int32(0), int32(numel(signedBytes)));
end

function body = createHealthBody(status, port, sandboxFolder)
% Identify this exact loopback service and expose the fallback MATLAB path.
body = struct( ...
    "schemaVersion", "offlineSandboxTransport/v1", ...
    "status", string(status), ...
    "mode", "live", ...
    "port", port, ...
    "sandboxFolder", string(sandboxFolder));
end

function corsOrigin = allowedCorsOrigin(origin, hasOrigin, port)
% Permit only the served loopback page and the deliberate local file page.
corsOrigin = "";
if ~hasOrigin
    return;
end
loopbackOrigin = "http://127.0.0.1:" + string(port);
if origin == loopbackOrigin || origin == "null"
    corsOrigin = origin;
end
end

function requestId = previewRequestId(requestBytes)
% Read only the request identifier needed to match out-of-band cancellation.
requestId = "";
try
    value = jsondecode(native2unicode(requestBytes, "UTF-8"));
catch exception
    if isUserInterruption(exception)
        rethrow(exception);
    end
    return;
end
if ~isstruct(value) || ~isscalar(value) || ~isfield(value, "requestId")
    return;
end
candidate = value.requestId;
if (isstring(candidate) && isscalar(candidate)) || ...
        (ischar(candidate) && isrow(candidate))
    requestId = string(candidate);
end
end

function statusCode = requestErrorStatus(exception)
% Map bounded parser errors to client-visible HTTP status codes.
if exception.identifier == "serveSandbox:PayloadTooLarge"
    statusCode = 413;
elseif isSocketTimeout(exception)
    statusCode = 408;
else
    statusCode = 400;
end
end

function reason = httpReason(statusCode)
% Return the small status vocabulary used by this server.
switch statusCode
    case 400
        reason = "Bad Request";
    case 403
        reason = "Forbidden";
    case 404
        reason = "Not Found";
    case 405
        reason = "Method Not Allowed";
    case 408
        reason = "Request Timeout";
    case 409
        reason = "Conflict";
    case 413
        reason = "Content Too Large";
    case 500
        reason = "Internal Server Error";
    case 503
        reason = "Service Unavailable";
    otherwise
        reason = "Error";
end
end

function isFailure = isRequestFailure(exception)
% Distinguish invalid request/planner requirements from server defects.
identifier = string(exception.identifier);
isFailure = contains(identifier, "runPlanningRequest:") || ...
    startsWith(identifier, "planTrajectory:") || ...
    startsWith(identifier, "createObstacle:") || ...
    startsWith(identifier, "combineObstacles:") || ...
    startsWith(identifier, "bmtpEngine:") || ...
    startsWith(identifier, "ruckigEngine:");
end

function isTimeout = isSocketTimeout(exception)
% Recognize Java timeouts without depending on MATLAB's wrapper identifier.
isTimeout = contains(string(exception.message), ...
    "java.net.SocketTimeoutException");
end

function isInterruption = isUserInterruption(exception)
% Preserve Ctrl-C semantics instead of converting interruption to HTTP errors.
identifier = string(exception.identifier);
message = string(exception.message);
isInterruption = contains(identifier, "OperationTerminated") || ...
    contains(message, "Operation terminated by user");
end

function bytes = readFileBytes(filePath)
% Read one local file as exact bytes.
[fileIdentifier, openMessage] = fopen(filePath, "r", "n");
if fileIdentifier < 0
    error("serveSandbox:FileOpenFailed", ...
        "Could not open '%s': %s", filePath, openMessage);
end
try
    bytes = fread(fileIdentifier, Inf, '*uint8').';
    closeStatus = fclose(fileIdentifier);
catch exception
    fclose(fileIdentifier);
    rethrow(exception);
end
if closeStatus ~= 0
    error("serveSandbox:FileCloseFailed", ...
        "Could not close '%s' after reading.", filePath);
end
end

function writeFileBytes(filePath, bytes)
% Write one temporary request as exact bytes.
[fileIdentifier, openMessage] = fopen(filePath, "w", "n");
if fileIdentifier < 0
    error("serveSandbox:FileOpenFailed", ...
        "Could not create '%s': %s", filePath, openMessage);
end
try
    writtenByteCount = fwrite(fileIdentifier, bytes, 'uint8');
    closeStatus = fclose(fileIdentifier);
catch exception
    fclose(fileIdentifier);
    rethrow(exception);
end
if writtenByteCount ~= numel(bytes) || closeStatus ~= 0
    error("serveSandbox:FileWriteFailed", ...
        "Could not write all request bytes to '%s'.", filePath);
end
end

function deleteTemporaryFiles(requestFilePath, resultFilePath)
% Remove only per-request files created by the HTTP adapter.
deleteFileIfPresent(requestFilePath);
deleteFileIfPresent(resultFilePath);
end

function deleteFileIfPresent(filePath)
% Remove one adapter-owned temporary or stop marker file if present.
if isfile(filePath)
    delete(filePath);
end
end

function closeSocket(socket)
% Close a client socket without masking an earlier request error.
try
    if ~isempty(socket) && ~socket.isClosed()
        socket.close();
    end
catch
end
end

function closeServer(serverSocket, stopFilePath, bundleFilePath, ...
        bundleRequestIdPath)
% Release the listening port and consume this server's stop marker.
try
    if ~isempty(serverSocket) && ~serverSocket.isClosed()
        serverSocket.close();
    end
catch
end
deleteFileIfPresent(stopFilePath);
deleteFileIfPresent(bundleFilePath);
deleteFileIfPresent(bundleRequestIdPath);
end
