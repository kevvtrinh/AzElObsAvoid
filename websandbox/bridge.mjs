import { createServer } from 'node:http';
import { mkdir, readFile, rename, stat, unlink, writeFile } from 'node:fs/promises';
import { join, resolve } from 'node:path';
import { randomUUID } from 'node:crypto';

const protocolRoot = resolve(process.env.WEBSANDBOX_PROTOCOL_ROOT || 'websandbox/runtime');
const port = Number(process.env.WEBSANDBOX_PORT || 42831);
const directories = {
  requests: join(protocolRoot, 'requests'),
  responses: join(protocolRoot, 'responses'),
  native: join(protocolRoot, 'native'),
};

await Promise.all([mkdir(protocolRoot, { recursive: true }), ...Object.values(directories).map((directory) => mkdir(directory, { recursive: true }))]);

const sendJson = (response, status, payload) => {
  const body = JSON.stringify(payload);
  response.writeHead(status, {
    'Access-Control-Allow-Origin': 'http://localhost:5173',
    'Access-Control-Allow-Methods': 'GET, POST, OPTIONS',
    'Content-Type': 'application/json; charset=utf-8',
    'Content-Length': Buffer.byteLength(body),
  });
  response.end(body);
};

const readJsonBody = async (request) => {
  const chunks = [];
  for await (const chunk of request) chunks.push(chunk);
  const body = Buffer.concat(chunks).toString('utf8');
  if (!body) throw new Error('A JSON request body is required.');
  return JSON.parse(body);
};

const responsePath = (jobId) => join(directories.responses, `${jobId}.response.json`);
const requestPath = (jobId) => join(directories.requests, `${jobId}.request.json`);

const bridge = createServer(async (request, response) => {
  try {
    if (request.method === 'OPTIONS') return sendJson(response, 204, {});
    if (request.method === 'GET' && request.url === '/api/health') {
      return sendJson(response, 200, { Success: true, ProtocolRoot: protocolRoot, Transport: 'file-watch' });
    }
    if (request.method === 'POST' && request.url === '/api/plan') {
      const sceneRequest = await readJsonBody(request);
      const jobId = `job_${randomUUID().replaceAll('-', '')}`;
      const temporaryPath = `${requestPath(jobId)}.tmp`;
      await writeFile(temporaryPath, JSON.stringify(sceneRequest), 'utf8');
      await rename(temporaryPath, requestPath(jobId));
      return sendJson(response, 202, { Success: true, JobId: jobId, Status: 'queued' });
    }
    const jobMatch = request.url?.match(/^\/api\/jobs\/(job_[A-Za-z0-9_]+)$/);
    if (request.method === 'GET' && jobMatch) {
      const jobId = jobMatch[1];
      try {
        const completed = JSON.parse(await readFile(responsePath(jobId), 'utf8'));
        return sendJson(response, 200, { Success: completed.Success, JobId: jobId, Status: 'completed', Response: completed, Request: completed.Request });
      } catch (error) {
        if (error.code !== 'ENOENT') throw error;
        try {
          await stat(requestPath(jobId));
          return sendJson(response, 200, { Success: true, JobId: jobId, Status: 'queued' });
        } catch {
          return sendJson(response, 404, { Success: false, Error: { Identifier: 'webSandbox:UnknownJob', Message: 'Unknown job ID.' } });
        }
      }
    }
    const cancelMatch = request.url?.match(/^\/api\/jobs\/(job_[A-Za-z0-9_]+)\/cancel$/);
    if (request.method === 'POST' && cancelMatch) {
      const jobId = cancelMatch[1];
      await writeFile(join(directories.requests, `${jobId}.cancel`), '', 'utf8');
      return sendJson(response, 202, { Success: true, JobId: jobId, Status: 'cancelling' });
    }
    sendJson(response, 404, { Success: false, Error: { Identifier: 'webSandbox:RouteNotFound', Message: 'No matching local API route.' } });
  } catch (error) {
    sendJson(response, 400, { Success: false, Error: { Identifier: 'webSandbox:BridgeRequestError', Message: error.message } });
  }
});

bridge.listen(port, '127.0.0.1', () => {
  console.log(`Web Sandbox bridge ready at http://127.0.0.1:${port}`);
});
