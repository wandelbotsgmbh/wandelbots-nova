import { ThemeProvider } from '@mui/material'
import { TabBar } from '@wandelbots/wandelbots-js-react-components'
import {
  JoggingPanel,
  createNovaMuiTheme,
} from '@wandelbots/wandelbots-js-react-components'
import React, { useState } from 'react'

import './App.css'
import FineTuning from './FineTune'
import NovaHome from './pages/Home'

const theme = createNovaMuiTheme({
  palette: {
    mode: 'dark',
  },
})

function Button({ label, onClick }: { label: string; onClick: () => void }) {
  return (
    <button className="bg-blue-500 text-white p-2 rounded-md" onClick={onClick}>
      {label}
    </button>
  )
}

function Page({ children }: { children: React.ReactNode }) {
  return (
    <div className="px-3 py-3 h-full w-full overflow-y-auto">{children}</div>
  )
}

function App() {
  return (
    <ThemeProvider theme={theme}>
      <div className="flex flex-col h-full w-full from-slate-950 to-slate-900 text-slate-100 bg-gradient-to-b">
        <TabBar
          defaultActiveTab={0}
          items={[
            {
              content: (
                <Page>
                  <NovaHome />
                </Page>
              ),
              id: 'tab1',
              label: 'WB Nova Home',
            },
            {
              content: (
                <Page>
                  <div>Content for second tab</div>
                </Page>
              ),
              id: 'tab2',
              label: '3D Viz: rerun',
            },
            {
              content: (
                <Page>
                  <FineTuning />
                </Page>
              ),
              id: 'tab3',
              label: 'Fine-Tuning',
            },
          ]}
          onTabChange={function WCe() {}}
        />
        <div className="flex-1 overflow-auto">
          {/* Tab content will be rendered here by TabBar component */}
        </div>
      </div>
    </ThemeProvider>
  )
}

export default App
