'use client'

import {
  type Theme,
  type ThemeOptions,
  createTheme,
} from '@mui/material/styles'

import { createDarkTheme } from './createDarkTheme'
import { createLightTheme } from './createLightTheme'

/**
 * Create the default Wandelbots Nova Material UI theme, overriding
 * any defaults with the provided theme options.
 */
export function createNovaMuiTheme(opts: ThemeOptions): Theme {
  let isDark = true
  if (opts.palette?.mode === 'light') {
    isDark = false
  } else if (opts.palette?.mode !== 'dark') {
    const browserPrefersLight =
      typeof window !== 'undefined' &&
      window.matchMedia?.('(prefers-color-scheme: light)')?.matches
    isDark = !browserPrefersLight
  }

  const baseTheme = isDark ? createDarkTheme() : createLightTheme()

  return createTheme(baseTheme, opts)
}
