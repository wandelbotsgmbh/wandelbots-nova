import tailwind from '@tailwindcss/vite'
import react from '@vitejs/plugin-react-swc'
import { defineConfig } from 'vite'

// https://vite.dev/config/
export default defineConfig({
  base: './',
  plugins: [react(), tailwind()],
  build: {
    outDir: 'dist',
    assetsDir: 'src/assets', // default; fine either way
    sourcemap: false,
  },
})
