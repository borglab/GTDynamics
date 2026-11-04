import Vue from '@vitejs/plugin-vue';
import { defineConfig } from 'vite';
import paperMath from '../shared/paper/mathPlugin.mjs';
export default defineConfig({ base: './', plugins: [paperMath(), Vue()] });
