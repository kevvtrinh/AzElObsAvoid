function tests = testOfflineSandboxTransport
%% Section 0: Header & Readme
% SYNTAX tests = testOfflineSandboxTransport
% PURPOSE Verify disconnected HTTP clients cannot abort the planner callback.
% INPUTS None.
% OUTPUTS MATLAB function-based unit tests.
% UNITS Ports and counts are dimensionless; timeouts are milliseconds.
tests = functiontests(localfunctions);
end

function setupOnce(testCase)
% Exercise the production local functions through a temporary dispatch wrapper.
root = fileparts(fileparts(mfilename('fullpath')));
source = fileread(fullfile(root, 'offlinesandbox', '+offlineSandbox', 'serveSandbox.m'));
start = strfind(source, 'function [wasPlanRequest, wasBundleRequest] = serveClient(');
folder = tempname;
mkdir(folder);
header = sprintf(['function varargout = sandboxTransportHook(action, varargin)\n' ...
    'switch action\ncase "write"\nwriteHttpResponse(varargin{:});\n' ...
    'case "poll"\nvarargout{1} = cancellationRequested(varargin{:});\nend\nend\n']);
fid = fopen(fullfile(folder, 'sandboxTransportHook.m'), 'w');
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

function testResetPeerResponseIsContained(testCase)
server = java.net.ServerSocket(int32(0), int32(8), ...
    java.net.InetAddress.getByName('127.0.0.1'));
serverCleanup = onCleanup(@() server.close());
client = java.net.Socket('127.0.0.1', server.getLocalPort());
peer = server.accept();
peerCleanup = onCleanup(@() peer.close());
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

function testAbandonedPollDoesNotCancelAndServerStillHandlesRequests(testCase)
server = java.net.ServerSocket(int32(0), int32(8), ...
    java.net.InetAddress.getByName('127.0.0.1'));
cleanup = onCleanup(@() server.close());
port = double(server.getLocalPort());
stopFile = string(tempname);
client = java.net.Socket('127.0.0.1', int32(port));
client.setSoLinger(true, int32(0));
client.close();
verifyFalse(testCase, sandboxTransportHook("poll", server, stopFile, "active", port));
client = queueRequest(port, sprintf('GET /health HTTP/1.1\r\nHost: localhost\r\n\r\n'));
clientCleanup = onCleanup(@() client.close());
verifyFalse(testCase, sandboxTransportHook("poll", server, stopFile, "active", port));
clear clientCleanup;
body = '{"requestId":"active"}';
request = sprintf('POST /cancel HTTP/1.1\r\nContent-Length: %d\r\n\r\n%s', numel(body), body);
client = queueRequest(port, request);
clientCleanup = onCleanup(@() client.close());
verifyTrue(testCase, sandboxTransportHook("poll", server, stopFile, "active", port));
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
sandboxTransportHook("write", socket, 200, "OK", "text/plain", ...
    uint8('test response'), strings(0, 1), "");
end

function client = queueRequest(port, request)
client = java.net.Socket('127.0.0.1', int32(port));
bytes = typecast(uint8(request), 'int8');
client.getOutputStream().write(bytes, int32(0), int32(numel(bytes)));
client.getOutputStream().flush();
end
