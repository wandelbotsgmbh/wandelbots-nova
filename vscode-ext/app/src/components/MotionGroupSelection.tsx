import {
  Box,
  FormControl,
  MenuItem,
  Select,
  SelectChangeEvent,
  Typography,
} from '@mui/material'
import React, { useEffect, useState } from 'react'

import { accessToken, cellId, novaApi } from '../config'
import { NovaApi } from '../utils/novaAPI'

export type MotionGroupOption = {
  label: string
  value: string
  icon?: React.ReactNode
}

// TODO: pass nova client
export type MotionGroupSelectionProps = {
  /** Either an existing NovaClient or the base url of a deployed Nova instance */
  // nova: NovaClient | String
  onChange: (value: string) => void
  disabled?: boolean
}

export default function MotionGroupSelection(props: MotionGroupSelectionProps) {
  const [motionGroupOptions, setMotionGroupOptions] = useState<
    MotionGroupOption[]
  >([])
  const [selectedMotionGroupId, setSelectedMotionGroupId] = useState<
    string | null
  >(null)

  const handleChange = (event: SelectChangeEvent<string | null>) => {
    const value = event.target.value as string
    setSelectedMotionGroupId(value)
    props.onChange(value)
  }

  async function fetchMotionGroups() {
    console.log('fetchMotionGroups', novaApi, accessToken, cellId)
    const nova = new NovaApi()
    await nova.connect({
      apiUrl: novaApi,
      accessToken,
      cellId,
    })

    try {
      if (!nova) return

      const controllerNames = await nova.getControllersNames()
      if (controllerNames.length === 0) {
        console.warn('No controllers found')
        return
      }

      const motionGroupIds = await Promise.all(
        controllerNames.map(
          async (controllerName) => await nova.getMotionGroups(controllerName),
        ),
      ).then((groups) => groups.flat())

      const options = motionGroupIds.map((mgId) => ({
        value: mgId,
        label: `${mgId}`,
      }))

      setMotionGroupOptions(options)

      // Select the first motion group
      if (motionGroupIds.length > 0) {
        setSelectedMotionGroupId(motionGroupIds[0])
        // Notify parent so dependent controls become enabled immediately
        props.onChange(motionGroupIds[0])
      }
    } catch (error) {
      console.error('Failed to fetch motion groups:', error)
      // Fallback to default options
      setMotionGroupOptions([])
      setSelectedMotionGroupId(null)
    }
  }

  // Fetch motion groups when component mounts
  useEffect(() => {
    fetchMotionGroups()
  }, [])

  if (motionGroupOptions.length === 0) {
    return (
      <Box
        sx={{
          width: '100%',
          display: 'flex',
          alignItems: 'center',
          justifyContent: 'center',
          borderRadius: 9999,
          backgroundColor: 'rgba(255,255,255,0.06)',
          border: '1px solid rgba(255,255,255,0.1)',
          px: 1.25,
          py: 0.75,
        }}
      >
        <Typography
          variant="body2"
          sx={{ color: 'text.secondary', fontStyle: 'italic' }}
        >
          No motion group
        </Typography>
      </Box>
    )
  }

  return (
    <FormControl size="small" sx={{ width: '100%' }} disabled={props.disabled}>
      <Select
        value={selectedMotionGroupId}
        onChange={handleChange}
        displayEmpty
        renderValue={(selected) => {
          const option = motionGroupOptions.find((o) => o.value === selected)
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
        {motionGroupOptions.map((opt) => (
          <MenuItem key={opt.value} value={opt.value} sx={{ gap: 1 }}>
            {opt.icon}
            <Typography variant="body2">{opt.label}</Typography>
          </MenuItem>
        ))}
      </Select>
    </FormControl>
  )
}
