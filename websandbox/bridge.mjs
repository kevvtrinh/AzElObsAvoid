import { createServer } from 'node:http';
import { mkdir, readFile, rename, stat, writeFile } from 'node:fs/promises';
import { basename, join, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';
import { randomUUID } from 'node:crypto';

const sourceDirectory = resolve(fileURLToPath(new URL('.', import.meta.url)));
const protocolRoot = resolve(process.env.WEBSANDBOX_PROTOCOL_ROOT || join(sourceDirectory, 'runtime'));
const port = Number(process.env.WEBSANDBOX_PORT || 42831);
const directories = {
  requests: join(protocolRoot, 'requests'), responses: join(protocolRoot, 'responses'),
  native: join(protocolRoot, 'native'), bundleImports: join(protocolRoot, 'bundles', 'imports'),
  bundleExports: join(protocolRoot, 'bundles', 'exports'),
};
await Promise.all([mkdir(protocolRoot, { recursive: true }), ...Object.values(directories).map((directory) => mkdir(directory, { recursive: true }))]);

const sendJson = (response, status, payload) => {
  const body = JSON.stringify(payload);
  response.writeHead(status, { 'Access-Control-Allow-Origin': 'http://localhost:5173', 'Access-Control-Allow-Headers': 'Content-Type, X-Bundle-Name', 'Access-Control-Allow-Methods': 'GET, POST, OPTIONS', 'Content-Type': 'application/json; charset=utf-8', 'Content-Length': Buffer.byteLength(body) });
  response.end(body);
};
const readBody = async (request) => { const chunks = []; for await (const chunk of request) chunks.push(chunk); return Buffer.concat(chunks); };
const readJsonBody = async (request) => { const body = (await readBody(request)).toString('utf8'); if (!body) throw new Error('A JSON request body is required.'); return JSON.parse(body); };
const sanitizeBundleName = (value, fallback) => { const name = basename(String(value || fallback)).replace(/[^A-Za-z0-9._-]/g, '_'); return name.toLowerCase().endsWith('.mat') ? name : `${name}.mat`; };
const responsePath = (jobId) => join(directories.responses, `${jobId}.response.json`);
const requestPath = (jobId) => join(directories.requests, `${jobId}.request.json`);
const newJobId = () => `job_${randomUUID().replaceAll('-', '')}`;
const queueJsonJob = async (payload) => {
  const jobId = newJobId(); const temporaryPath = `${requestPath(jobId)}.tmp`;
  await writeFile(temporaryPath, JSON.stringify(payload), 'utf8'); await rename(temporaryPath, requestPath(jobId)); return jobId;
};

const bridge = createServer(async (request, response) => {
  try {
    if (request.method === 'OPTIONS') return sendJson(response, 204, {});
    if (request.method === 'GET' && request.url === '/api/health') return sendJson(response, 200, { Success: true, ProtocolRoot: protocolRoot, Transport: 'file-watch' });
    if (request.method === 'POST' && request.url === '/api/plan') return sendJson(response, 202, { Success: true, JobId: await queueJsonJob(await readJsonBody(request)), Status: 'queued' });
    if (request.method === 'POST' && request.url === '/api/bundles/import') {
      const bundleBytes = await readBody(request); if (!bundleBytes.length) throw new Error('A nonempty MATLAB bundle is required.');
      const jobId = newJobId(); const name = `${jobId}-${sanitizeBundleName(request.headers['x-bundle-name'], 'sandbox-import.mat')}`;
      await writeFile(join(directories.bundleImports, name), bundleBytes);
      const temporaryPath = `${requestPath(jobId)}.tmp`;
      await writeFile(temporaryPath, JSON.stringify({ protocolAction: 'bundleImport', bundleFileName: name }), 'utf8'); await rename(temporaryPath, requestPath(jobId));
      return sendJson(response, 202, { Success: true, JobId: jobId, Status: 'queued' });
    }
    if (request.method === 'POST' && request.url === '/api/bundles/export') {
      const scene = await readJsonBody(request); const jobId = await queueJsonJob({ protocolAction: 'bundleExport', bundleFileName: sanitizeBundleName(scene.bundleFileName, 'az_el_web_sandbox.mat'), scene });
      return sendJson(response, 202, { Success: true, JobId: jobId, Status: 'queued' });
    }
    const bundleMatch = request.url?.match(/^\/api\/bundles\/(job_[A-Za-z0-9_]+)$/);
    if (request.method === 'GET' && bundleMatch) {
      const bytes = await readFile(join(directories.bundleExports, `${bundleMatch[1]}.mat`));
      response.writeHead(200, { 'Access-Control-Allow-Origin': 'http://localhost:5173', 'Content-Disposition': `attachment; filename="${bundleMatch[1]}.mat"`, 'Content-Length': bytes.length, 'Content-Type': 'application/octet-stream' }); return response.end(bytes);
    }
    const jobMatch = request.url?.match(/^\/api\/jobs\/(job_[A-Za-z0-9_]+)$/);
    if (request.method === 'GET' && jobMatch) {
      const jobId = jobMatch[1];
      try { const completed = JSON.parse(await readFile(responsePath(jobId), 'utf8')); return sendJson(response, 200, { Success: completed.Success, JobId: jobId, Status: 'completed', Response: completed, Request: completed.Request }); }
      catch (error) { if (error.code !== 'ENOENT') throw error; try { await stat(requestPath(jobId)); return sendJson(response, 200, { Success: true, JobId: jobId, Status: 'queued' }); } catch { return sendJson(response, 404, { Success: false, Error: { Identifier: 'webSandbox:UnknownJob', Message: 'Unknown job ID.' } }); } }
    }
    const cancelMatch = request.url?.match(/^\/api\/jobs\/(job_[A-Za-z0-9_]+)\/cancel$/);
    if (request.method === 'POST' && cancelMatch) { const jobId = cancelMatch[1]; await writeFile(join(directories.requests, `${jobId}.cancel`), '', 'utf8'); return sendJson(response, 202, { Success: true, JobId: jobId, Status: 'cancelling' }); }
    sendJson(response, 404, { Success: false, Error: { Identifier: 'webSandbox:RouteNotFound', Message: 'No matching local API route.' } });
  } catch (error) { sendJson(response, 400, { Success: false, Error: { Identifier: 'webSandbox:BridgeRequestError', Message: error.message } }); }
});
bridge.listen(port, '127.0.0.1', () => console.log(`Web Sandbox bridge ready at http://127.0.0.1:${port}`));
