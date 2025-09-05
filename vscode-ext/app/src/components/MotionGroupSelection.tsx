import {
  Box,
  FormControl,
  MenuItem,
  Select,
  SelectChangeEvent,
  Typography,
} from '@mui/material'
import { NovaClient } from '@wandelbots/nova-js/v1'
import React from 'react'

export type MotionGroupOption = {
  value: string
  label: string
  icon?: React.ReactNode
}

// TODO: pass nova client
// /** Either an existing NovaClient or the base url of a deployed Nova instance */
//   nova: NovaClient | string
export type MotionGroupSelectionProps = {
  value: string
  options: MotionGroupOption[]
  onChange: (value: string) => void
  disabled?: boolean
}

export default function MotionGroupSelection({
  value,
  options,
  onChange,
  disabled,
}: MotionGroupSelectionProps) {
  const handleChange = (event: SelectChangeEvent<string>) => {
    onChange(event.target.value as string)
  }

  // Show "no motion group found" message when there are no options
  if (options.length === 0) {
    return (
      <Box
        sx={{
          width: '100%',
          height: 40,
          display: 'flex',
          alignItems: 'center',
          justifyContent: 'center',
          borderRadius: 9999,
          backgroundColor: 'rgba(255,255,255,0.06)',
          border: '1px solid rgba(255,255,255,0.1)',
          px: 1.25,
        }}
      >
        <Typography
          variant="body2"
          sx={{ color: 'text.secondary', fontStyle: 'italic' }}
        >
          No motion group found
        </Typography>
      </Box>
    )
  }

  return (
    <FormControl size="small" sx={{ width: '100%' }} disabled={disabled}>
      <Select
        value={value}
        onChange={handleChange}
        displayEmpty
        renderValue={(selected) => {
          const option = options.find((o) => o.value === selected)
          if (!option) return ''
          return (
            <Box sx={{ display: 'flex', alignItems: 'center', gap: 1 }}>
              {option.icon}
              <Typography variant="body2" sx={{ fontWeight: 600 }}>
                {option.label}
              </Typography>
            </Box>
          )
        }}
        sx={{
          borderRadius: 9999,
          backgroundColor: 'rgba(255,255,255,0.06)',
          color: 'inherit',
          '& .MuiSelect-select': {
            display: 'flex',
            alignItems: 'center',
            gap: 8,
            py: 0.75,
            pl: 1.25,
          },
          '& fieldset': { border: 'none' },
          '& .MuiOutlinedInput-notchedOutline': { border: 'none' },
        }}
        MenuProps={{
          PaperProps: {
            sx: {
              mt: 1,
              borderRadius: 2,
              minWidth: '100%',
            },
          },
        }}
      >
        {options.map((opt) => (
          <MenuItem key={opt.value} value={opt.value} sx={{ gap: 1 }}>
            {opt.icon}
            <Typography variant="body2">{opt.label}</Typography>
          </MenuItem>
        ))}
      </Select>
    </FormControl>
  )
}
