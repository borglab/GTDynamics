import Vue from '@vitejs/plugin-vue'
import { defineConfig } from 'vite'

export default defineConfig({
  plugins: [Vue()],
  define: { 'process.env': {} },
  server: {
    host: '127.0.0.1',
    port: 3001,
    strictPort: true,
    proxy: {
      '/socket': {
        target: 'ws://127.0.0.1:8766',
        ws: true,
      },
    },
  },
})
