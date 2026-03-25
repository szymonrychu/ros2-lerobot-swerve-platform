/**
 * Extract a nested field value from a ROS2 message object.
 *
 * Supports dot-notation paths with optional array indexing:
 *   "linear_acceleration.x"           -> msg.linear_acceleration.x
 *   "position[0]"                     -> msg.position[0]
 *   "twist.twist.linear.x"            -> msg.twist.twist.linear.x
 *   "covariance[0]"                   -> msg.covariance[0]
 */
export function extractField(obj: Record<string, unknown>, path: string): number | null {
  if (!obj || typeof path !== "string" || path.length === 0) return null;
  const parts = parsePath(path);
  let current: unknown = obj;
  for (const part of parts) {
    if (current === null || current === undefined) return null;
    if (typeof part === "number") {
      if (!Array.isArray(current)) return null;
      current = current[part];
    } else {
      if (typeof current !== "object") return null;
      current = (current as Record<string, unknown>)[part];
    }
  }
  if (typeof current === "number") return current;
  return null;
}

type PathSegment = string | number;

/**
 * Parse a field path string into segments.
 *
 * "twist.twist.linear.x"  -> ["twist", "twist", "linear", "x"]
 * "position[2]"           -> ["position", 2]
 * "covariance[0]"         -> ["covariance", 0]
 */
export function parsePath(path: string): PathSegment[] {
  const segments: PathSegment[] = [];
  const parts = path.split(".");
  for (const part of parts) {
    const bracketIdx = part.indexOf("[");
    if (bracketIdx !== -1) {
      const key = part.slice(0, bracketIdx);
      const idxStr = part.slice(bracketIdx + 1, part.indexOf("]"));
      if (key.length > 0) segments.push(key);
      segments.push(parseInt(idxStr, 10));
    } else {
      if (part.length > 0) segments.push(part);
    }
  }
  return segments;
}

/**
 * Format a number using a printf-style format string (subset).
 *
 * ".2f" -> toFixed(2), ".6f" -> toFixed(6), ".0f" -> toFixed(0)
 */
export function formatValue(value: number, format?: string, unit?: string): string {
  let str: string;
  if (format && /^\.\d+f$/.test(format)) {
    const decimals = parseInt(format.slice(1, -1), 10);
    str = value.toFixed(decimals);
  } else {
    str = String(value);
  }
  return unit ? `${str} ${unit}` : str;
}
