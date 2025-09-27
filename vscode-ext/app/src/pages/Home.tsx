import Button from '@mui/material/Button'
import React from 'react'

import logo from '../assets/logo-wandelbots-nova.png'

type NovaHomeProps = {
  onOpenFineTuning?: () => void
}

export default function NovaHome({ onOpenFineTuning }: NovaHomeProps) {
  return (
    <>
      <div className="flex flex-col items-center gap-12 max-w-md py-12 mx-auto">
        <div className="space-y-3 flex flex-col items-center">
          <img src={logo} alt="logo" className="h-6" />
          <div className="text-xs font-bold text-gray-500">Extension</div>
        </div>
        {/*<div className="flex flex-col items-center gap-3">
          <Button
            onClick={() => {}}
            variant="contained"
            size="large"
            className="w-full"
            color="secondary"
          >
            3D Viz: rerun
          </Button>
          <div className="text-sm text-gray-500">
            See planned paths and robot movement in 3D for visual debugging and
            analysis
          </div>
        </div>*/}
        <div className="flex flex-col items-center gap-3">
          <Button
            onClick={() => onOpenFineTuning?.()}
            variant="contained"
            size="large"
            className="w-full"
            color="secondary"
          >
            Program Fine-Tuning
          </Button>
          <div className="text-sm text-gray-500">
            Adjust robot programs step by step with extended program executoder
            ion controls while transfering robot programs to physical setup
          </div>
        </div>
      </div>
    </>
  )
}
