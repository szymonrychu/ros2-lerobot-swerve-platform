import { RobotScene } from '../components3d/RobotScene'
import { RobotModel } from '../components3d/RobotModel'
import { LaserScanLayer } from '../components3d/LaserScanLayer'
import { CostmapLayer } from '../components3d/CostmapLayer'
import { TabConfig } from '../types'
import { Suspense } from 'react'

interface Props {
  tab: TabConfig
  topicData: Record<string, unknown>
  publish: (topic: string, msgType: string, data: unknown) => void
}

export default function Scene3DTab({ tab, topicData }: Props) {
  const jointStates = tab.topic
    ? (topicData[tab.topic] as { name?: string[]; position?: number[] } | undefined)
    : undefined

  const armJointStates = tab.arm_joint_topic
    ? (topicData[tab.arm_joint_topic] as { name?: string[]; position?: number[] } | undefined)
    : undefined

  const scanData = tab.scan_topic ? (topicData[tab.scan_topic] as Parameters<typeof LaserScanLayer>[0]['data']) : undefined
  const costmapData = tab.costmap_topic ? (topicData[tab.costmap_topic] as Parameters<typeof CostmapLayer>[0]['data']) : undefined

  return (
    <div style={{ width: '100%', height: '100%' }}>
      <RobotScene urdfFile={tab.urdf_file ?? 'robot.urdf'} jointStates={jointStates}>
        {tab.arm_urdf_file && (
          <Suspense fallback={null}>
            <RobotModel urdfFile={tab.arm_urdf_file} jointStates={armJointStates} />
          </Suspense>
        )}
        <LaserScanLayer data={scanData} />
        <CostmapLayer data={costmapData} />
      </RobotScene>
    </div>
  )
}
