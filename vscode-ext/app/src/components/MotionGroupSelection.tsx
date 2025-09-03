import {
  Box,
  FormControl,
  MenuItem,
  Select,
  SelectChangeEvent,
  Typography,
} from '@mui/material'
import React from 'react'

export type MotionGroupOption = {
  value: string
  label: string
  icon?: React.ReactNode
}

export type MotionGroupSelectionProps = {
  value: string
  options: MotionGroupOption[]
  onChange: (value: string) => void
  disabled?: boolean
  width?: number | string
}

export default function MotionGroupSelection({
  value,
  options,
  onChange,
  disabled,
  width,
}: MotionGroupSelectionProps) {
  const handleChange = (event: SelectChangeEvent<string>) => {
    onChange(event.target.value as string)
  }

  return (
    <FormControl size="small" sx={{ width: width ?? 260 }} disabled={disabled}>
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
              minWidth: width ?? 260,
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
