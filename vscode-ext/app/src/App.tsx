import React from 'react'
import { TabBar } from '@wandelbots/wandelbots-js-react-components'

import './App.css'
import logo from './logo.svg'

function App() {
  return (
    <div className="App">
      <header className="App-header">
        <TabBar
          defaultActiveTab={0}
          items={[
            {
              content: <div>Content for first tab</div>,
              id: 'tab1',
              label: 'First Tab'
            },
            {
              content: <div>Content for second tab</div>,
              id: 'tab2',
              label: 'Second Tab'
            },
            {
              content: <div>Content for third tab</div>,
              id: 'tab3',
              label: 'Third Tab'
            }
          ]}
          onTabChange={function WCe(){}}
        />
        <img src={logo} className="App-logo" alt="logo" />
        <p>
          Edit <code>src/App.tsx</code> and save to reload.
        </p>
        <a
          className="App-link"
          href="https://reactjs.org"
          target="_blank"
          rel="noopener noreferrer"
        >
          Learn React
        </a>
      </header>
    </div>
  )
}

export default App
