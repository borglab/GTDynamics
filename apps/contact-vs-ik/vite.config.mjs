import paperMath from '../shared/paper/mathPlugin.mjs';
import Vue from '@vitejs/plugin-vue';
import { defineConfig } from 'vite';
export default defineConfig({ base: './', plugins: [Vue(), paperMath()] });
