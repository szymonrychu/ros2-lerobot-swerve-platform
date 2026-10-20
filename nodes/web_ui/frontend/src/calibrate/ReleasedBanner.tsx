import Alert from '@mui/material/Alert'
import { ServoGroupsState, useServoGroups } from './servoApi'

/**
 * The group's label when its fresh torque state says released, else null (not configured, unknown, stale or enabled).
 *
 * @param servo - result of useServoGroups
 * @param groupId - servo group id (head, arm, base)
 * @returns label of the released group, or null
 */
export function releasedLabel(servo: ServoGroupsState, groupId: string): string | null {
  const group = servo.groups.find((g) => g.id === groupId)
  return group && group.state && !group.stale && group.state.released ? group.label : null
}

/**
 * True while the group's fresh torque state is released.
 *
 * @param groupId - servo group id
 * @returns whether commands to the group must be disabled
 */
export function useGroupReleased(groupId: string): boolean {
  return releasedLabel(useServoGroups(), groupId) !== null
}

interface ViewProps {
  groupId: string
  servo: ServoGroupsState
}

/**
 * Warning banner for a released group. Tabs that already poll the groups pass `servo` so they keep one poll.
 *
 * @param groupId - servo group id
 * @param servo - state from the tab's own useServoGroups call
 */
export function ReleasedBannerView({ groupId, servo }: ViewProps) {
  const label = releasedLabel(servo, groupId)
  if (!label) return null
  return (
    <Alert severity="warning" data-testid={`released-banner-${groupId}`}>
      {`${label} torque is off - controls disabled (Calibrate page)`}
    </Alert>
  )
}

/**
 * Standalone banner: polls the groups itself.
 *
 * @param groupId - servo group id
 */
export function ReleasedBanner({ groupId }: { groupId: string }) {
  return <ReleasedBannerView groupId={groupId} servo={useServoGroups()} />
}
