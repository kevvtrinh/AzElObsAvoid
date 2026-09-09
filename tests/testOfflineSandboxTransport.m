function tests = testOfflineSandboxTransport
%% Section 0: Header & Readme
% SYNTAX tests = testOfflineSandboxTransport
% PURPOSE Verify disconnected HTTP clients do not abort the sandbox server.
% INPUTS None.
% OUTPUTS MATLAB function-based unit tests.
% UNITS Ports and counts are dimensionless; timeouts are milliseconds.
tests = functiontests(localfunctions);
end

function setupOnce(testCase)
    % Exercise the production local functions through a temporary dispatch wrapper.
    root   = fileparts(fileparts(mfilename('fullpath')));
    source = fileread(fullfile(root, 'offlinesandbox', '+offlineSandbox', 'serveSandbox.m'));
    start  = strfind(source, 'function [wasPlanRequest, wasBundleRequest] = serveClient(');
    folder = tempname;
    mkdir(folder);
    header  = sprintf(['function varargout = sandboxTransportHook(action, varargin)\n' 'switch action\ncase "write"\nwriteHttpResponse(varargin{:});\n' 'case "serve"\n[varargout{1},varargout{2}] = serveClient(varargin{:});\n' 'case "save"\nvarargout{1} = saveBundleWithDialog(varargin{:});\nend\nend\n']);
    fid     = fopen(fullfile(folder, 'sandboxTransportHook.m'), 'w');
    cleanup = onCleanup(@() fclose(fid));
    fprintf(fid, '%s', [header, source(start(1):end)]);
    clear cleanup;
    addpath(folder);
    testCase.TestData.Folder = folder;
end

function teardownOnce(testCase)
    rmpath(testCase.TestData.Folder);
    clear sandboxTransportHook;
    rmdir(testCase.TestData.Folder, 's');
end

function testClosedSocketResponseIsContained(testCase)
    socket = java.net.Socket();
    socket.close();
    text = evalc('writeResponse(socket);');
    verifySubstring(testCase, text, 'could not be delivered');
end

function testSaveDialogWritesChosenBundleAndReportsPath(testCase)
    folder = testCase.TestData.Folder;
    sourcePath = fullfile(folder, 'cached.mat');
    diagnosisBundle = struct('Sentinel', pi); %#ok<NASGU>
    save(sourcePath, 'diagnosisBundle');
    chooseFile = @(varargin) deal('chosen', folder);
    outcome = sandboxTransportHook('save', sourcePath, 'test-request', chooseFile);
    verifyTrue(testCase, outcome.Saved);
    verifyFalse(testCase, outcome.Cancelled);
    expected = string(java.io.File(fullfile(folder, 'chosen.mat')).getCanonicalPath());
    verifyEqual(testCase, outcome.FilePath, expected);
    verifyGreaterThan(testCase, outcome.Bytes, 0);
    loaded = load(outcome.FilePath, 'diagnosisBundle');
    verifyEqual(testCase, loaded.diagnosisBundle, diagnosisBundle);
end

function testSaveDialogCancellationDoesNotReportSaved(testCase)
    chooseFile = @(varargin) deal(0, 0);
    outcome = sandboxTransportHook('save', 'unused.mat', 'test-request', chooseFile);
    verifyFalse(testCase, outcome.Saved);
    verifyTrue(testCase, outcome.Cancelled);
    verifyEqual(testCase, outcome.FilePath, "");
end

function testSaveDialogRejectsWrongExtension(testCase)
    chooseFile = @(varargin) deal('chosen.txt', testCase.TestData.Folder);
    verifyError(testCase, @() sandboxTransportHook('save', 'unused.mat', 'test-request', chooseFile), ...
        'serveSandbox:InvalidBundleExtension');
end

function testResetPeerResponseIsContained(testCase)
    server        = java.net.ServerSocket(int32(0), int32(8), java.net.InetAddress.getByName('127.0.0.1'));
    serverCleanup = onCleanup(@() server.close());
    client        = java.net.Socket('127.0.0.1', server.getLocalPort());
    peer          = server.accept();
    peerCleanup   = onCleanup(@() peer.close());
    client.setSoLinger(true, int32(0));
    client.close();
    % Consume the reset before attempting the response, avoiding timing sleeps.
    try
        peer.getInputStream().read();
    catch exception
        verifySubstring(testCase, exception.message, 'java.net.SocketException');
    end
    text = evalc('writeResponse(peer);');
    verifySubstring(testCase, text, 'could not be delivered');
end

function testCancelEndpointIsRemoved(testCase)
    server        = java.net.ServerSocket(int32(0), int32(8), java.net.InetAddress.getByName('127.0.0.1'));
    cleanup       = onCleanup(@() server.close());
    port          = double(server.getLocalPort());
    client        = queueRequest(port, sprintf('POST /cancel HTTP/1.1\r\nContent-Length: 0\r\n\r\n'));
    clientCleanup = onCleanup(@() client.close());
    peer          = server.accept();
    [wasPlan, wasBundle] = sandboxTransportHook("serve", peer, uint8('page'), "sandbox", port, string(tempname), string(tempname));
    verifyFalse(testCase, wasPlan);
    verifyFalse(testCase, wasBundle);
    client.setSoTimeout(int32(1000));
    stream = client.getInputStream(); bytes = uint8([]);
    % Continue the search until complete d reaches an explicit termination condition.
    while true
        value = stream.read(); if value < 0, break; end
        bytes(end+1) = uint8(value);
    end
    verifySubstring(testCase, char(bytes), '404 Not Found');
end

function testNonSocketFailuresAreNotSuppressed(testCase)
    threw = false;
    try
        writeResponse(struct());
    catch exception
        threw = true;
        verifyFalse(testCase, contains(exception.message, 'java.net.SocketException'));
    end
    verifyTrue(testCase, threw);
end

function writeResponse(socket)
    sandboxTransportHook("write", socket, 200, "OK", "text/plain", uint8('test response'), strings(0, 1), "");
end

function client = queueRequest(port, request)
    client = java.net.Socket('127.0.0.1', int32(port));
    bytes  = typecast(uint8(request), 'int8');
    client.getOutputStream().write(bytes, int32(0), int32(numel(bytes)));
    client.getOutputStream().flush();
end
