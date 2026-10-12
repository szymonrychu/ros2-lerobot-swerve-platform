import { CameraStream } from '../components/CameraStream'
import { TabConfig } from '../types'

interface Props {
  tab: TabConfig
  topicData: Record<string, unknown>
  publish: (topic: string, msgType: string, data: unknown) => void
}

export default function CameraTab({ tab, topicData }: Props) {
  return <CameraStream topic={tab.topic} data={tab.topic ? topicData[tab.topic] : undefined} />
}
