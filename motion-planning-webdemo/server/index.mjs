import { createServer } from 'node:http';
import { fileURLToPath } from 'node:url';
import { resolve } from 'node:path';
import { WebSocketServer, WebSocket } from 'ws';
import { SolverSession } from './session.mjs';

const root = fileURLToPath(new URL('../../', import.meta.url));
const executable = process.env.MOTION_PLANNING_BACKEND || resolve(root, 'build/motion-planning-webdemo/backend/motion_planning_optimizer');
const port = Number(process.env.MOTION_PLANNING_PORT || 8766);
const server = createServer((request, response) => {
  response.writeHead(request.url === '/health' ? 200 : 404, { 'Content-Type': 'application/json' });
  response.end(JSON.stringify(request.url === '/health' ? { service: 'motion-planning-native-bridge' } : { error: 'Not found' }));
});
const wss = new WebSocketServer({ noServer: true, maxPayload: 16 * 1024 });
server.on('upgrade', (request, socket, head) => {
  let allowed = request.url === '/socket';
  if (request.headers.origin) {
    try { allowed &&= ['localhost', '127.0.0.1', '[::1]'].includes(new URL(request.headers.origin).hostname); }
    catch { allowed = false; }
  }
  if (!allowed) { socket.end('HTTP/1.1 403 Forbidden\r\n\r\n'); return; }
  wss.handleUpgrade(request, socket, head, ws => wss.emit('connection', ws));
});
wss.on('connection', ws => {
  const session = new SolverSession({
    executable, cwd: root,
    send: message => { if (ws.readyState === WebSocket.OPEN) ws.send(JSON.stringify(message)); },
    onClose: () => ws.close(1011, 'Native optimizer stopped'),
  });
  ws.on('message', data => {
    try { session.accept(JSON.parse(data.toString())); }
    catch { ws.send(JSON.stringify({ type: 'error', message: 'Invalid JSON command.' })); }
  });
  ws.on('close', () => session.close());
  ws.on('error', () => session.close());
});
server.listen(port, '127.0.0.1', () => console.log(`Motion planning C++ bridge: http://127.0.0.1:${port}`));
server.on('error', error => { console.error(error.message); process.exitCode = 1; });
for (const signal of ['SIGINT', 'SIGTERM']) process.once(signal, () => {
  for (const ws of wss.clients) ws.close();
  wss.close();
  server.close();
});
