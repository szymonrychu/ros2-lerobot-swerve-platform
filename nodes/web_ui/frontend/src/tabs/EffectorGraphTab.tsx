import SensorGraphTab from './SensorGraphTab'
import { TabConfig } from '../types'

interface Props {
  tab: TabConfig
  topicData: Record<string, unknown>
  publish: (topic: string, msgType: string, data: unknown) => void
}

export default function EffectorGraphTab(props: Props) {
  return <SensorGraphTab {...props} />
}
