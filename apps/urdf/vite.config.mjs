import Vue from '@vitejs/plugin-vue';
import { defineConfig } from 'vite';

// The GTDynamics Python backend (scripts/server.py). Keep the default in sync
// with DEFAULT_PORT there.
const backend = `http://127.0.0.1:${process.env.URDF_BACKEND_PORT || 8767}`;

export default defineConfig({
  base: './',
  plugins: [Vue()],
  server: {
    proxy: {
      // The page requests api/, files/ and pkg/ relative to its own URL, so
      // under the apps site they arrive as /urdf/api/...; the backend serves /api/....
      '^(/[^/?#]+)?/(api|files|pkg)/': {
        target: backend,
        rewrite: path => path.replace(/^.*?\/(api|files|pkg)\//, '/$1/'),
      },
    },
  },
});
