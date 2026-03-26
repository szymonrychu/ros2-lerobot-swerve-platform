import { useEffect, useRef } from 'react'
import L from 'leaflet'
import 'leaflet/dist/leaflet.css'
import { TabConfig } from '../types'

delete (L.Icon.Default.prototype as unknown as Record<string, unknown>)._getIconUrl
L.Icon.Default.mergeOptions({
  iconUrl: 'https://unpkg.com/leaflet@1.9.4/dist/images/marker-icon.png',
  iconRetinaUrl: 'https://unpkg.com/leaflet@1.9.4/dist/images/marker-icon-2x.png',
  shadowUrl: 'https://unpkg.com/leaflet@1.9.4/dist/images/marker-shadow.png',
})

interface Props {
  tab: TabConfig
  topicData: Record<string, unknown>
  publish: (topic: string, msgType: string, data: unknown) => void
}

interface GpsFix {
  latitude?: number
  longitude?: number
}

export default function NavGpsTab({ tab, topicData, publish }: Props) {
  const containerRef = useRef<HTMLDivElement>(null)
  const mapRef = useRef<L.Map | null>(null)
  const markerRef = useRef<L.Marker | null>(null)

  useEffect(() => {
    if (!containerRef.current || mapRef.current) return
    const map = L.map(containerRef.current).setView([0, 0], tab.default_zoom ?? 18)
    L.tileLayer(
      tab.tile_url ?? 'https://{s}.basemaps.cartocdn.com/light_all/{z}/{x}/{y}{r}.png',
      { attribution: '© OpenStreetMap © CARTO', maxZoom: 21 },
    ).addTo(map)
    mapRef.current = map

    if (tab.goal_topic) {
      map.on('click', (evt) => {
        publish(tab.goal_topic!, 'geometry_msgs/PoseStamped', {
          header: { frame_id: 'map' },
          pose: {
            position: { x: evt.latlng.lng, y: evt.latlng.lat, z: 0.0 },
            orientation: { x: 0, y: 0, z: 0, w: 1 },
          },
        })
      })
    }

    return () => { map.remove(); mapRef.current = null }
  }, [])  // eslint-disable-line react-hooks/exhaustive-deps

  useEffect(() => {
    const map = mapRef.current
    if (!map || !tab.fix_topic) return
    const fix = topicData[tab.fix_topic] as GpsFix | undefined
    if (!fix?.latitude || !fix?.longitude) return

    const latlng: L.LatLngExpression = [fix.latitude, fix.longitude]
    if (!markerRef.current) {
      markerRef.current = L.marker(latlng).addTo(map)
      map.setView(latlng, tab.default_zoom ?? 18)
    } else {
      markerRef.current.setLatLng(latlng)
    }
  }, [topicData, tab.fix_topic])  // eslint-disable-line react-hooks/exhaustive-deps

  return <div ref={containerRef} style={{ width: '100%', height: '100%' }} />
}
