import { useMemo } from 'react'

interface LaserScanMsg {
  angle_min?: number
  angle_increment?: number
  ranges?: number[]
  range_max?: number
}

interface Props {
  data: LaserScanMsg | undefined
}

export function LaserScanLayer({ data }: Props) {
  const points = useMemo(() => {
    if (!data?.ranges) return new Float32Array(0)
    const positions: number[] = []
    for (let i = 0; i < data.ranges.length; i++) {
      const r = data.ranges[i]
      if (!isFinite(r) || r <= 0 || (data.range_max && r >= data.range_max)) continue
      const angle = (data.angle_min ?? 0) + i * (data.angle_increment ?? 0)
      positions.push(Math.cos(angle) * r, 0.05, -Math.sin(angle) * r)
    }
    return new Float32Array(positions)
  }, [data])

  if (points.length === 0) return null

  return (
    <points>
      <bufferGeometry>
        <bufferAttribute
          attach="attributes-position"
          array={points}
          count={points.length / 3}
          itemSize={3}
        />
      </bufferGeometry>
      <pointsMaterial color="#00ff00" size={0.02} />
    </points>
  )
}
