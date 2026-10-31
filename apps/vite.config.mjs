import { defineConfig } from 'vite';
import apps from './registry.json' with { type: 'json' };

export default defineConfig({
  base: './',
  // Static preview must serve the built apps, without development proxies.
  preview: { proxy: {} },
  server: {
    host: process.env.GTDYNAMICS_APPS_HOST || '127.0.0.1',
    allowedHosts: (process.env.GTDYNAMICS_APPS_ALLOWED_HOSTS || '')
      .split(',').map(host => host.trim()).filter(Boolean),
    port: 3000,
    strictPort: true,
    proxy: {
      '/socket': { target: 'ws://127.0.0.1:8766', ws: true },
      ...Object.fromEntries(apps.map((app, index) => [
        `/${app.id}/`, { target: `http://127.0.0.1:${3001 + index}`, changeOrigin: true, ws: true },
      ])),
    },
  },
});
