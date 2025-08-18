import { ThemeProvider } from '@mui/material'
import { TabBar } from '@wandelbots/wandelbots-js-react-components'
import { createNovaMuiTheme } from '@wandelbots/wandelbots-js-react-components'
import React, { useEffect, useState } from 'react'

import './App.css'
import FineTuning from './pages/FineTune'
import NovaHome from './pages/Home'

const theme = createNovaMuiTheme({
  palette: {
    mode: 'dark',
  },
})

function Page({ children }: { children: React.ReactNode }) {
  return <div className="p-3 h-full w-full overflow-y-auto">{children}</div>
}

function App() {
  const [activeTab, setActiveTab] = useState<number | undefined>(() => {
    const injected: any =
      typeof window !== 'undefined'
        ? (window as any).__NOVA_CONFIG__
        : undefined
    const initial = injected?.initialTab
    return typeof initial === 'number' ? initial : 0
  })

  useEffect(() => {
    function handleMessage(event: MessageEvent) {
      const msg = event.data as { command?: string; index?: number }
      if (msg?.command === 'selectTab' && typeof msg.index === 'number') {
        setActiveTab(msg.index)
      }
    }
    window.addEventListener('message', handleMessage)
    return () => window.removeEventListener('message', handleMessage)
  }, [])

  const items = [
    {
      content: (
        <Page>
          <NovaHome onOpenFineTuning={() => setActiveTab(2)} />
        </Page>
      ),
      id: 'tab1',
      label: 'WB Nova Home',
    },
    /*{
      content: (
        <Page>
          <div>Content for second tab</div>
        </Page>
      ),
      id: 'tab2',
      label: '3D Viz: rerun',
    },*/
    {
      content: (
        <Page>
          <FineTuning />
        </Page>
      ),
      id: 'tab3',
      label: 'Fine-Tuning',
    },
  ]

  return (
    <ThemeProvider theme={theme}>
      <div className="flex flex-col h-full w-full from-slate-950 to-slate-900 text-slate-100 bg-gradient-to-b">
        <TabBar
          defaultActiveTab={0}
          activeTab={activeTab}
          items={items}
          onTabChange={setActiveTab}
          sx={{ padding: 2, flexShrink: 0 }}
        />
        <div className="flex-1 min-h-0">{items[activeTab || 0]?.content}</div>
      </div>
    </ThemeProvider>
  )
}

export default App
