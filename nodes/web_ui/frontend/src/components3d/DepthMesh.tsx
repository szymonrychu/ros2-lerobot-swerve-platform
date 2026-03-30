import { useEffect, useRef } from 'react'
import * as THREE from 'three'

interface Intrinsics {
  fx: number
  fy: number
  cx: number
  cy: number
  width: number
  height: number
}

interface Props {
  depthB64: string
  depthWidth: number
  depthHeight: number
  colorSmallB64: string
  intrinsics: Intrinsics
}

const DISCONTINUITY_MM = 200

function base64ToUint8Array(b64: string): Uint8Array {
  const binary = atob(b64)
  const out = new Uint8Array(binary.length)
  for (let i = 0; i < binary.length; i++) out[i] = binary.charCodeAt(i)
  return out
}

function decodeJpegPixels(b64: string, w: number, h: number): Promise<Uint8ClampedArray> {
  return new Promise((resolve) => {
    const img = new Image()
    img.onload = () => {
      const canvas = document.createElement('canvas')
      canvas.width = w
      canvas.height = h
      const ctx = canvas.getContext('2d')!
      ctx.drawImage(img, 0, 0, w, h)
      resolve(ctx.getImageData(0, 0, w, h).data)
    }
    img.onerror = () => resolve(new Uint8ClampedArray(w * h * 4))
    img.src = `data:image/jpeg;base64,${b64}`
  })
}

export default function DepthMesh({
  depthB64, depthWidth, depthHeight, colorSmallB64, intrinsics,
}: Props) {
  const geoRef = useRef<THREE.BufferGeometry>(null!)

  useEffect(() => {
    if (!depthB64 || !colorSmallB64 || !geoRef.current) return
    let cancelled = false

    const W = depthWidth
    const H = depthHeight

    // Scale intrinsics from original resolution to mesh resolution
    const sx = W / intrinsics.width
    const sy = H / intrinsics.height
    const fx = intrinsics.fx * sx
    const fy = intrinsics.fy * sy
    const cx = intrinsics.cx * sx
    const cy = intrinsics.cy * sy

    // Decode raw uint16 depth bytes
    const bytes = base64ToUint8Array(depthB64)
    const depth = new Uint16Array(bytes.buffer, bytes.byteOffset, bytes.byteLength / 2)

    decodeJpegPixels(colorSmallB64, W, H).then((rgba) => {
      if (cancelled || !geoRef.current) return

      const nVerts = W * H
      const positions = new Float32Array(nVerts * 3)
      const colors = new Float32Array(nVerts * 3)

      // Build vertex positions (RealSense Z-forward, Three.js Y-up)
      for (let v = 0; v < H; v++) {
        for (let u = 0; u < W; u++) {
          const i = v * W + u
          const d = depth[i]
          const z = d / 1000.0  // mm → metres
          if (d > 0) {
            positions[i * 3 + 0] = (u - cx) * z / fx      // x: right
            positions[i * 3 + 1] = -((v - cy) * z / fy)   // y: up (flip image Y)
            positions[i * 3 + 2] = z                        // z: depth (forward)
          } else {
            positions[i * 3 + 0] = 0
            positions[i * 3 + 1] = 0
            positions[i * 3 + 2] = 0
          }
          colors[i * 3 + 0] = rgba[i * 4 + 0] / 255
          colors[i * 3 + 1] = rgba[i * 4 + 1] / 255
          colors[i * 3 + 2] = rgba[i * 4 + 2] / 255
        }
      }

      // Triangulate: two triangles per 2x2 quad, skip at depth discontinuities
      const maxIndices = (W - 1) * (H - 1) * 6
      const indexArray = new Uint32Array(maxIndices)
      let indexCount = 0

      for (let v = 0; v < H - 1; v++) {
        for (let u = 0; u < W - 1; u++) {
          const i00 = v * W + u
          const i10 = v * W + (u + 1)
          const i01 = (v + 1) * W + u
          const i11 = (v + 1) * W + (u + 1)
          const d00 = depth[i00], d10 = depth[i10]
          const d01 = depth[i01], d11 = depth[i11]

          // Upper triangle: top-left → top-right → bottom-left
          if (
            d00 > 0 && d10 > 0 && d01 > 0 &&
            Math.max(d00, d10, d01) - Math.min(d00, d10, d01) < DISCONTINUITY_MM
          ) {
            indexArray[indexCount++] = i00
            indexArray[indexCount++] = i10
            indexArray[indexCount++] = i01
          }
          // Lower triangle: top-right → bottom-right → bottom-left
          if (
            d10 > 0 && d11 > 0 && d01 > 0 &&
            Math.max(d10, d11, d01) - Math.min(d10, d11, d01) < DISCONTINUITY_MM
          ) {
            indexArray[indexCount++] = i10
            indexArray[indexCount++] = i11
            indexArray[indexCount++] = i01
          }
        }
      }

      const geo = geoRef.current
      geo.setAttribute('position', new THREE.BufferAttribute(positions, 3))
      geo.setAttribute('color', new THREE.BufferAttribute(colors, 3))
      geo.setIndex(new THREE.BufferAttribute(indexArray.subarray(0, indexCount).slice(), 1))
    })

    return () => { cancelled = true }
  }, [depthB64, colorSmallB64, depthWidth, depthHeight, intrinsics])

  return (
    <mesh>
      <bufferGeometry ref={geoRef} />
      <meshBasicMaterial vertexColors side={THREE.DoubleSide} />
    </mesh>
  )
}
