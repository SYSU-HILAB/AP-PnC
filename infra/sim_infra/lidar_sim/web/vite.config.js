import { defineConfig } from 'vite';
import { randomUUID } from 'node:crypto';
import { readFile, writeFile, rename } from 'node:fs/promises';
import path from 'node:path';

const root = process.env.AP_PNC_DIR;
if (!root || !path.isAbsolute(root)) throw new Error('absolute AP_PNC_DIR required');
const assets = path.join(root, '.artifacts/dev-sim/city-loop/assets');
export default defineConfig({
  server: { watch: { ignored: ['**/public/**'] } },
  plugins: [{
    name: 'local-simulation-control',
    configureServer(server) {
      for (const name of ['live.json', 'poses.json', 'verification.json']) {
        server.middlewares.use(`/assets/${name}`, async (_request, response) => {
          try {
            const data = await readFile(path.join(assets, name));
            response.writeHead(200, { 'Content-Type': 'application/json', 'Cache-Control': 'no-store' });
            response.end(data);
          } catch {
            if (name === 'verification.json') {
              response.writeHead(200, { 'Content-Type': 'application/json', 'Cache-Control': 'no-store' });
              response.end(JSON.stringify({ status: 'unverified' }));
            } else response.writeHead(503).end('waiting for simulation');
          }
        });
      }
      server.middlewares.use('/api/control', async (request, response) => {
        // Loopback-bound viewer only; same-origin POST, never aircraft dispatch.
        if (request.method !== 'POST' || request.headers.origin !== `http://${request.headers.host}`) {
          response.writeHead(403).end(); return;
        }
        try {
          let body = '';
          for await (const chunk of request) {
            body += chunk;
            if (body.length > 128) throw new Error('oversized command');
          }
          const { operation } = JSON.parse(body);
          if (!['resume', 'pause', 'restart'].includes(operation)) throw new Error('invalid operation');
          const live = JSON.parse(await readFile(path.join(assets, 'live.json'), 'utf8'));
          const stream = live.stream;
          const relative = path.relative(path.join(root, '.artifacts/dev-sim/city-loop/jobs'), stream);
          if (relative.startsWith('..') || path.isAbsolute(relative) || path.basename(stream) !== 'stream') throw new Error('invalid stream');
          const id = randomUUID();
          const temporary = path.join(stream, `control.${id}.tmp`);
          await writeFile(temporary, JSON.stringify({ operation, id }));
          await rename(temporary, path.join(stream, 'control.json'));
          response.writeHead(202, { 'Content-Type': 'application/json' });
          response.end(JSON.stringify({ id, accepted: 'queued, await simulation acknowledgement' }));
        } catch {
          response.writeHead(400).end('simulation command unavailable');
        }
      });
    },
  }],
});
