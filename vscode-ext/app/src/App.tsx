import { useState } from 'react'
import './App.css'
import { TabBar } from '@wandelbots/wandelbots-js-react-components'
import FineTuning from './FineTune'

function Button({ label, onClick }: { label: string, onClick: () => void }) {
  return <button className='bg-blue-500 text-white p-2 rounded-md' onClick={onClick}>{label}</button>
}

function NovaHome() {
  return <>
    <div className='space-y-3 flex flex-col items-center'>
      <img src="/logo-wandelbots-nova.png" alt="logo" className='max-w-48' />
      <div className="text-xs font-bold text-gray-500">Extension</div>
    </div>
    <div>
      <Button label='3D Viz: rerun' onClick={() => {}} />
      <div>
        See planned paths and robot movement in 3D for visual debugging and analysis
      </div>
    </div>
    <div>
      <Button label='Program Fine-Tuning' onClick={() => {}} />
      <div>
      Adjust robot programs step by step with extended program execution controls while transfering robot programs to physical setup
      </div>
    </div>
  </>
}

function App() {
  const [count, setCount] = useState(0)

  return (
    <>
      <TabBar
        defaultActiveTab={0}
        items={[
          {
            content: <NovaHome />,
            id: 'tab1',
            label: 'WB Nova Home'
          },
          {
            content: <div>Content for second tab</div>,
            id: 'tab2',
            label: '3D Viz: rerun'
          },
          {
            content: <FineTuning />,
            id: 'tab3',
            label: 'Fine-Tuning'
          }
        ]}
        onTabChange={function WCe(){}}
      />
    </>
  )
}

export default App
