import { type Theme, createTheme, rgbToHex } from '@mui/material'

export function createDarkTheme(): Theme {
  const baseTheme = createTheme({
    palette: {
      mode: 'dark',
      text: {
        primary: 'rgba(255, 255, 255, 1)',
        secondary: 'rgba(255, 255, 255, 0.7)',
        disabled: 'rgba(255, 255, 255, 0.38)',
      },
      primary: {
        main: 'rgba(142, 86, 252, 1)',
        dark: 'rgba(136, 58, 255, 1)',
        light: 'rgba(157, 131, 246, 1)',
        contrastText: 'rgba(255, 255, 255, 0.87)',
      },
      secondary: {
        main: 'rgba(255, 255, 255, 1)',
        dark: 'rgba(255, 255, 255, 0.06)',
        contrastText: 'rgba(255, 255, 255, 0.87)',
      },
      tertiary: {
        main: 'rgba(100, 255, 218, 1)',
        dark: 'rgba(38, 166, 154, 1)',
        light: 'rgba(167, 255, 235, 1)',
        contrastText: 'rgba(0, 0, 0, 1)',
      },
      error: {
        main: 'rgba(239, 83, 80, 1)',
        dark: 'rgba(229, 57, 53, 1)',
        light: 'rgba(239, 154, 154, 1)',
        contrastText: 'rgba(255, 255, 255, 1)',
      },
      warning: {
        main: 'rgba(255, 171, 64, 1)',
        dark: 'rgba(255, 145, 0, 1)',
        light: 'rgba(255, 209, 128, 1)',
        contrastText: 'rgba(0, 0, 0, 0.87)',
      },
      success: {
        main: 'rgba(38, 166, 154, 1)',
        dark: 'rgba(0, 121, 107, 1)',
        light: 'rgba(128, 203, 196, 1)',
        contrastText: 'rgba(255, 255, 255, 0.87)',
      },
      background: {
        default: 'rgba(2, 6, 23, 1)',
        paper: 'rgba(17, 19, 31, 1)',
      },
      action: {
        active: 'rgba(255, 255, 255, 0.56)',
        hover: 'rgba(255, 255, 255, 0.08)',
        selected: 'rgba(255, 255, 255, 0.16)',
        disabledBackground: 'rgba(255, 255, 255, 0.12)',
        focus: 'rgba(255, 255, 255, 0.12)',
        disabled: 'rgba(255, 255, 255, 0.38)',
      },
      common: {
        white: 'rgba(255, 255, 255, 1)',
        black: 'rgba(0, 0, 0, 1)',
      },
      divider: 'rgba(255, 255, 255, 0.12)',
      backgroundPaperElevation: {
        0: 'rgba(17,19,31,1.0)',
        1: 'rgba(20,22,35,1.0)',
        2: 'rgba(23,25,39,1.0)',
        3: 'rgba(26,28,43,1.0)',
        4: 'rgba(29,31,47,1.0)',
        5: 'rgba(32,34,51,1.0)',
        6: 'rgba(35,37,55,1.0)',
        7: 'rgba(38,40,59,1.0)',
        8: 'rgba(41,43,63,1.0)',
        9: 'rgba(44,46,67,1.0)',
        10: 'rgba(47,49,71,1.0)',
        11: 'rgba(50,52,75,1.0)',
        12: 'rgba(53,55,79,1.0)',
        13: 'rgba(56,58,83,1.0)',
        14: 'rgba(59,61,87,1.0)',
        15: 'rgba(62,64,91,1.0)',
        16: 'rgba(57,63,87,1.0)',
      },
    },
    paletteExt: {
      primary: {
        hover: 'rgba(142, 86, 252, 0.08)',
        selected: 'rgba(142, 86, 252, 0.16)',
        focus: 'rgba(142, 86, 252, 0.12)',
        focusVisible: 'rgba(142, 86, 252, 0.3)',
        outlineBorder: 'rgba(142, 86, 252, 0.5)',
      },
      secondary: {
        tonal: 'rgba(255,255,255,0.1)',
      },
    },
  })

  return {
    ...baseTheme,
    components: {
      MuiCssBaseline: {
        styleOverrides: {
          'html, body, *': {
            scrollbarWidth: 'none',
            msOverflowStyle: 'none',
          },

          'html:hover, body:hover, *:hover': {
            scrollbarWidth: 'thin',
            scrollbarColor: `${baseTheme.palette.divider} transparent`,
          },

          // Global styles for DataGrid filter popup
          '.MuiDataGrid-panelContent': {
            borderRadius: '16px !important',
            '& .MuiPaper-root': {
              borderRadius: '16px !important',
            },
          },
          '.MuiDataGrid-filterForm': {
            borderRadius: '16px !important',
            '& .MuiInputBase-root': {
              borderRadius: '16px !important',
              '& fieldset': {
                borderRadius: '16px !important',
              },
              '& .MuiOutlinedInput-notchedOutline': {
                borderRadius: '16px !important',
              },
            },
            '& .MuiTextField-root': {
              '& .MuiInputBase-root': {
                borderRadius: '16px !important',
                '& fieldset': {
                  borderRadius: '16px !important',
                },
                '& .MuiOutlinedInput-notchedOutline': {
                  borderRadius: '16px !important',
                },
              },
            },
          },
          '.MuiDataGrid-filterFormValueInput': {
            '& .MuiInputBase-root': {
              borderRadius: '16px !important',
              '& fieldset': {
                borderRadius: '16px !important',
              },
              '& .MuiOutlinedInput-notchedOutline': {
                borderRadius: '16px !important',
              },
            },
          },
        },
      },
      MuiDataGrid: {
        styleOverrides: {
          root: {
            // Main DataGrid styling can go here if needed
          },
          panelContent: {
            borderRadius: '16px !important',
          },
          filterForm: {
            '& .MuiInputBase-root': {
              borderRadius: '10px !important',
              '& fieldset': {
                borderRadius: '10px !important',
              },
              '& .MuiOutlinedInput-notchedOutline': {
                borderRadius: '10px !important',
              },
            },
          },
          filterFormValueInput: {
            '& .MuiInputBase-root': {
              borderRadius: '10px !important',
              '& fieldset': {
                borderRadius: '10px !important',
              },
              '& .MuiOutlinedInput-notchedOutline': {
                borderRadius: '10px !important',
              },
            },
          },
        },
      },
      MuiButton: {
        styleOverrides: {
          root: {
            textTransform: 'none',
            borderRadius: '10px',
          },
        },
        variants: [
          {
            props: { color: 'secondary', variant: 'contained' },
            style: {
              border: `1px solid ${baseTheme.palette.divider}`,
              background: baseTheme.paletteExt?.secondary?.tonal?.toString(),
            },
          },
          {
            props: { color: 'secondary', variant: 'outlined' },
            style: {
              border: `1px solid ${baseTheme.palette.divider}`,
            },
          },
        ],
      },
      MuiTab: {
        styleOverrides: {
          root: {
            textTransform: 'none',
            color: baseTheme.palette.text.secondary,
            '&.Mui-selected': {
              color: baseTheme.palette.text.primary,
            },
          },
        },
      },
      MuiTabs: {
        styleOverrides: {
          root: {
            background: baseTheme.palette.backgroundPaperElevation?.[1],
          },
        },
      },
      MuiToggleButtonGroup: {
        styleOverrides: {
          root: {
            '.MuiToggleButtonGroup-firstButton': {
              borderRadius: '10px 0 0 10px',
            },
            '.MuiToggleButtonGroup-lastButton': {
              borderRadius: '0 10px 10px 0',
            },
          },
        },
      },
      MuiToggleButton: {
        styleOverrides: {
          root: {
            textTransform: 'none',
            padding: '6px 16px',
          },
        },
      },
      MuiSelect: {
        variants: [
          {
            props: { variant: 'filled' },
            style: {
              borderRadius: '10px',
              '::before, ::after': {
                visibility: 'collapse',
              },
              '.MuiInputBase-input': {
                padding: '6px 16px',
                borderRadius: '10px',
              },
            },
          },
        ],
      },
      MuiDrawer: {
        styleOverrides: {
          paper: {
            border: 'none',
            boxShadow: '4px 0 8px -4px rgba(0, 0, 0, 0.3)',
          },
        },
      },
      MuiFab: {
        styleOverrides: {
          root: {
            background: baseTheme.paletteExt?.secondary?.tonal,
          },
        },
        variants: [
          {
            props: { color: 'primary' },
            style: {
              background: baseTheme.palette.primary.main,
            },
          },
        ],
      },
    },

    componentsExt: {
      JoggingPanel: {
        JoggingCartesian: {
          Axis: {
            X: {
              backgroundColor: 'rgba(215, 66, 56, 1)',
              borderColor: 'rgba(215, 66, 56, 1)',
              buttonBackgroundColor: {
                default: 'rgba(241, 77, 66, 1)',
                pressed: 'rgba(138, 41, 35, 1)',
                hovered: 'rgba(241, 77, 66, 1)',
                disabled: 'rgba(241, 77, 66, 1)',
              },
              color: 'rgba(255, 198, 198, 1)',
              labelColor: baseTheme.palette.text.primary,
            },
            Y: {
              backgroundColor: 'rgba(20, 151, 108, 1)',
              borderColor: 'rgba(20, 151, 108, 1)',
              buttonBackgroundColor: {
                default: 'rgba(28, 188, 135, 1)',
                pressed: 'rgba(11, 89, 63, 1)',
                disabled: 'rgba(28, 188, 135, 1)',
                hovered: 'rgba(28, 188, 135, 1)',
              },
              color: 'rgba(215, 255, 242, 1)',
              labelColor: baseTheme.palette.text.primary,
            },
            Z: {
              backgroundColor: 'rgba(1, 87, 155, 1)',
              borderColor: 'rgba(1, 87, 155, 1)',
              buttonBackgroundColor: {
                default: 'rgba(2, 136, 209, 1)',
                pressed: 'rgba(2, 64, 114, 1)',
                disabled: 'rgba(2, 136, 209, 1)',
                hovered: 'rgba(2, 136, 209, 1)',
              },
              color: 'rgba(210, 239, 255, 1)',
              labelColor: baseTheme.palette.text.primary,
            },
          },
        },
        JoggingJoint: {
          Joint: {
            arrowColor: baseTheme.palette.text.primary,
          },
        },
        VelocitySlider: {
          sliderLegendColor: baseTheme.palette.text.primary,
        },
      },
      WandelscriptEditor: {
        monacoTheme: {
          base: 'vs-dark',
          inherit: true,
          rules: [
            {
              token: 'keyword',
              foreground: '#b349a9ff',
            },
          ],
          colors: {
            'editor.background': rgbToHex(
              baseTheme.palette.backgroundPaperElevation?.[5] ?? '',
            ),
            'editor.foreground': '#d4d4d4',
            'editorLineNumber.foreground': '#797979',
            'editorLineNumber.activeForeground': '#e9e9e9',
            'editor.lineHighlightBorder': '#e9e9e911',
          },
        },
      },
    },
  } as Theme
}
