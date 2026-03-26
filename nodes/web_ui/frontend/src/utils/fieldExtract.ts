/**
 * Extract a nested numeric field from a plain object using dot-notation + array indexing.
 * e.g. "linear_acceleration.x" or "position[2]"
 */
export function extractField(data: unknown, path: string): number | null {
  let current: unknown = data
  const parts = path.replace(/\[/g, '.[').split('.')
  for (const part of parts) {
    if (current === null || current === undefined) return null
    const arrayMatch = part.match(/^\[(\d+)\]$/)
    if (arrayMatch) {
      current = (current as unknown[])[parseInt(arrayMatch[1])]
    } else {
      current = (current as Record<string, unknown>)[part]
    }
  }
  if (typeof current === 'number') return current
  return null
}
