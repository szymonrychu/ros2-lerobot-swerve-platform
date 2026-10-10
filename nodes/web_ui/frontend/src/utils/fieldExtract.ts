/**
 * Extract a nested numeric field from a plain object using dot-notation + array indexing.
 * e.g. "linear_acceleration.x" or "position[2]".
 *
 * An element can also be selected by name: "position[name=fl_steer]" takes the element of `position` at the index
 * where the sibling `name` array of the same object holds "fl_steer" (sensor_msgs/JointState), so the value does not
 * depend on the joint order.
 */
export function extractField(data: unknown, path: string): number | null {
  let current: unknown = data
  let parent: unknown = null
  const parts = path.replace(/\[/g, '.[').split('.')
  for (const part of parts) {
    if (current === null || current === undefined) return null
    const indexMatch = part.match(/^\[(\d+)\]$/)
    const nameMatch = part.match(/^\[name=(.+)\]$/)
    if (indexMatch) {
      current = (current as unknown[])[parseInt(indexMatch[1])]
    } else if (nameMatch) {
      const names = (parent as Record<string, unknown> | null)?.name
      if (!Array.isArray(names)) return null
      const index = names.indexOf(nameMatch[1])
      if (index < 0) return null
      current = (current as unknown[])[index]
    } else {
      parent = current
      current = (current as Record<string, unknown>)[part]
    }
  }
  if (typeof current === 'number') return current
  return null
}
