import { defineConfig } from 'vite';
import react from '@vitejs/plugin-react';

// `npm run dev` serves the page with hot reload and forwards API calls to a running coven-server
const api = 'http://127.0.0.1:8000';

export default defineConfig({
  plugins: [react()],
  base: './',
  build: { outDir: '../coven/web', emptyOutDir: true, chunkSizeWarningLimit: 2000 },
  server: { proxy: { '/scenes': api, '/stats': api, '/panda': api, '/problems': api } },
});
