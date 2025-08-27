import { resolve } from 'path'
import { defaultExclude, defineConfig } from 'vitest/config'

const r = (p: string) => resolve(__dirname, p)

const alias = {
  '~': r('app'),
  '~/': r('app/'),
  '~~': r('.'),
  '~~/': r('./'),
  '@@': r('.'),
  '@@/': r('./'),
}

export default defineConfig({
  // root: '.',
  test: {
    // options: https://vitest.dev/config/
    setupFiles: 'dotenv/config',
    testTimeout: 10000,
    projects: [
      {
        test: {
          name: 'unit',
          benchmark: { include: [] },
          environment: 'node',
          root: resolve(__dirname, '.'),
          include: ['./src/**/*.unit.test.ts'],
          exclude: [...defaultExclude, 'src/test/**'],
          alias,
        },
      },
    ],
  },
})
