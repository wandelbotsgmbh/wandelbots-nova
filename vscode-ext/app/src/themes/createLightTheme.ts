import { createDarkTheme } from './createDarkTheme'

export function createLightTheme() {
  // Keep the dark theme until we support a light theme
  const theme = createDarkTheme()
  theme.palette.mode = 'light'
  return theme
}
