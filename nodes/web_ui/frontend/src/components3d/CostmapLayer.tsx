import { useMemo } from 'react'
import * as THREE from 'three'

interface OccupancyGrid {
  info?: { width: number; height: number; resolution: number }
  data?: number[]
}

interface Props {
  data: OccupancyGrid | undefined
}

export function CostmapLayer({ data }: Props) {
  const { texture, width, height } = useMemo(() => {
    if (!data?.info || !data.data) return { texture: null, width: 1, height: 1 }
    const { width: w, height: h } = data.info
    const pixels = new Uint8Array(w * h * 4)
    for (let i = 0; i < data.data.length; i++) {
      const val = data.data[i]
      const brightness = val < 0 ? 60 : val === 0 ? 20 : Math.min(255, val * 2 + 60)
      pixels[i * 4] = brightness
      pixels[i * 4 + 1] = brightness
      pixels[i * 4 + 2] = brightness
      pixels[i * 4 + 3] = val === 0 ? 80 : 200
    }
    const tex = new THREE.DataTexture(pixels, w, h, THREE.RGBAFormat)
    tex.needsUpdate = true
    return { texture: tex, width: w * data.info.resolution, height: h * data.info.resolution }
  }, [data])

  if (!texture) return null

  return (
    <mesh rotation={[-Math.PI / 2, 0, 0]} position={[0, 0.001, 0]}>
      <planeGeometry args={[width, height]} />
      <meshBasicMaterial map={texture} transparent side={THREE.DoubleSide} />
    </mesh>
  )
}
