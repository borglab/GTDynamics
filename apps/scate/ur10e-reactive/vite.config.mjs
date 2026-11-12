import { defineConfig } from 'vite';
import Vue from '@vitejs/plugin-vue';
import paperMath from '../../shared/paper/mathPlugin.mjs';
import scateLiveServer from '../shared/liveServer.mjs';
export default defineConfig(({ mode }) => ({
  base: './', plugins: [paperMath(), Vue(), scateLiveServer()],
  build: mode === 'standalone' ? {
    outDir: 'dist-standalone', cssCodeSplit: false,
    rollupOptions: { output: { inlineDynamicImports: true } },
  } : {},
}));
