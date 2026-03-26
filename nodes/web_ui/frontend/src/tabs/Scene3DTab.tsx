import { RobotScene } from '../components3d/RobotScene'
import { LaserScanLayer } from '../components3d/LaserScanLayer'
import { CostmapLayer } from '../components3d/CostmapLayer'
import { TabConfig } from '../types'

interface Props {
  tab: TabConfig
  topicData: Record<string, unknown>
  publish: (topic: string, msgType: string, data: unknown) => void
}

export default function Scene3DTab({ tab, topicData }: Props) {
  const jointStates = topicData['/controller/swerve_drive/joint_states'] as
    | { name?: string[]; position?: number[] }
    | undefined

  const scanData = tab.scan_topic ? (topicData[tab.scan_topic] as Parameters<typeof LaserScanLayer>[0]['data']) : undefined
  const costmapData = tab.costmap_topic ? (topicData[tab.costmap_topic] as Parameters<typeof CostmapLayer>[0]['data']) : undefined

  return (
    <div style={{ width: '100%', height: '100%' }}>
      <RobotScene urdfFile={tab.urdf_file ?? 'robot.urdf'} jointStates={jointStates}>
        <LaserScanLayer data={scanData} />
        <CostmapLayer data={costmapData} />
      </RobotScene>
    </div>
  )
}
