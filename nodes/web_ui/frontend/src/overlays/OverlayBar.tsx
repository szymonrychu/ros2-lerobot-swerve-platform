import { OverlayItem } from '../types'
import { extractField } from '../utils/fieldExtract'
import { WheelYawMini } from '../components3d/WheelYawMini'

interface Props {
  overlays: OverlayItem[]
  topicData: Record<string, unknown>
  connected: boolean
}

export function OverlayBar({ overlays, topicData, connected }: Props) {
  return (
    <div className="overlay-bar">
      {overlays.map((item) => {
        const data = topicData[item.topic] as Record<string, unknown> | undefined
        const value = data ? extractField(data, item.field) : null
        const formatted =
          value !== null && value !== undefined
            ? item.format
              ? value.toFixed(parseInt(item.format.replace('.', '').replace('f', '')))
              : String(value)
            : '—'
        return (
          <div key={`${item.topic}:${item.field}`} className="overlay-item">
            <span className="overlay-label">{item.label}</span>
            <span className="overlay-value">{formatted}</span>
            {item.unit && <span className="overlay-unit">{item.unit}</span>}
          </div>
        )
      })}
      <WheelYawMini topicData={topicData} />
      <span className={`ws-status${connected ? ' connected' : ''}`}>
        {connected ? '● live' : '○ connecting…'}
      </span>
    </div>
  )
}
