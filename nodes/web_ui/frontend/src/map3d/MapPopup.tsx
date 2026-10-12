/**
 * The map's one popup (inside the Canvas): a drei Html card anchored at a POI or photo with an arrow pointing at
 * the anchor. It owns its pointer input (a press on it never reaches the map gestures or the camera) and closes on
 * its X, on Escape and on a tap on the map.
 */
import { useEffect } from 'react'
import type { CSSProperties, SyntheticEvent } from 'react'
import { useThree } from '@react-three/fiber'
import { Html } from '@react-three/drei'
import type { Vec2 } from '../map/mapMath'
import { capturedAt, photoThumbUrl } from '../photos/api'
import { isClick } from '../poi/editor'
import { itemId, type PoiItem } from '../poi/poiItems'
import { fillColor, STATUS_COLORS, ZONE_COLORS } from '../poi/style'
import { PANEL_BG, SANS_FONT, TOUCH_TARGET_PX } from '../theme'
import { rosToThree } from './coords'
import type { FocusTarget } from './mapFocus'

/** Anchor height above the ground (three y, m): above the POI markers. */
const POPUP_LIFT = 0.05
/** Above the POI labels (zIndexRange [5, 0]). */
const POPUP_Z_RANGE: [number, number] = [7, 6]
const ARROW_PX = 9
const CARD_WIDTH_PX = 240
const THUMB_HEIGHT_PX = 120
const BORDER_COLOR = '#30363d'

export interface MapPopupProps {
  target: FocusTarget
  /** X, Escape or a tap on the map. */
  onClose: () => void
  /** The action button: open the POI editor or the photo panel for the item. */
  onOpen: (item: PoiItem) => void
}

function stop(e: SyntheticEvent): void {
  e.stopPropagation()
}

function Tag({ text, color }: { text: string; color: string }) {
  return (
    <span style={{ border: `1px solid ${color}`, color, borderRadius: 4, padding: '0 5px', font: `600 11px ${SANS_FONT}` }}>
      {text}
    </span>
  )
}

/**
 * Popup card for one POI or photo.
 *
 * @param props - target (anchor and item), onClose, onOpen
 * @returns the drei Html element
 */
export function MapPopup({ target, onClose, onOpen }: MapPopupProps) {
  const invalidate = useThree((s) => s.invalidate)
  const canvas = useThree((s) => s.gl.domElement) as HTMLElement | null
  const { anchor, item } = target

  // frameloop is "demand": the Html is placed during a frame, so request one on open and on every content change.
  useEffect(
    () => invalidate(),
    [invalidate, item.key, item.kind, item.name, item.note, item.status, item.zone, item.updated, anchor.x, anchor.y],
  )

  useEffect(() => {
    const onKey = (e: KeyboardEvent) => {
      if (e.key === 'Escape') onClose()
    }
    window.addEventListener('keydown', onKey)
    return () => window.removeEventListener('keydown', onKey)
  }, [onClose])

  // A tap on the map (press and release without moving) closes; a drag (pan/orbit) keeps the popup.
  useEffect(() => {
    if (!canvas) return
    let down: Vec2 | null = null
    const onDown = (e: PointerEvent) => {
      down = e.button === 0 ? { x: e.clientX, y: e.clientY } : null
    }
    const onUp = (e: PointerEvent) => {
      const start = down
      down = null
      if (start && isClick(start, { x: e.clientX, y: e.clientY })) onClose()
    }
    canvas.addEventListener('pointerdown', onDown)
    window.addEventListener('pointerup', onUp)
    return () => {
      canvas.removeEventListener('pointerdown', onDown)
      window.removeEventListener('pointerup', onUp)
    }
  }, [canvas, onClose])

  const id = itemId(item)
  const isPhoto = item.source === 'photo'
  const name = item.name || '(unnamed)'
  return (
    <Html position={rosToThree(anchor, POPUP_LIFT)} zIndexRange={POPUP_Z_RANGE} style={{ pointerEvents: 'auto' }}>
      <div
        role="dialog"
        aria-label={name}
        onPointerDown={stop}
        onPointerUp={stop}
        onClick={stop}
        onDoubleClick={stop}
        onWheel={stop}
        onContextMenu={stop}
        style={{
          position: 'relative',
          width: CARD_WIDTH_PX,
          transform: `translate(-50%, calc(-100% - ${ARROW_PX}px))`,
          background: PANEL_BG,
          border: `1px solid ${BORDER_COLOR}`,
          borderRadius: 6,
          padding: 8,
          color: '#e6edf3',
          font: `13px ${SANS_FONT}`,
          boxShadow: '0 4px 14px rgba(0, 0, 0, 0.5)',
          cursor: 'default',
        }}
      >
        <div style={{ display: 'flex', alignItems: 'flex-start', gap: 6 }}>
          <div style={{ flex: 1, fontWeight: 700, overflowWrap: 'anywhere' }}>{name}</div>
          <button type="button" aria-label="Close popup" onClick={onClose} style={buttonStyle(false)}>
            x
          </button>
        </div>
        <div style={{ display: 'flex', flexWrap: 'wrap', gap: 4, margin: '6px 0' }}>
          <Tag text={item.kind} color={fillColor(item.kind, item.zone, item.status)} />
          {item.zone && item.zone !== 'none' && <Tag text={item.zone} color={ZONE_COLORS[item.zone]} />}
          {item.status && <Tag text={item.status} color={STATUS_COLORS[item.status]} />}
        </div>
        {isPhoto && (
          <>
            <img
              src={photoThumbUrl(id)}
              alt={name}
              style={{ display: 'block', width: '100%', height: THUMB_HEIGHT_PX, objectFit: 'cover', borderRadius: 4 }}
            />
            <div style={{ opacity: 0.7, fontSize: 11, marginTop: 4 }}>{capturedAt(item.updated)}</div>
          </>
        )}
        {item.note && <div style={{ marginTop: 6, maxHeight: 90, overflowY: 'auto', whiteSpace: 'pre-wrap' }}>{item.note}</div>}
        <div style={{ marginTop: 8, textAlign: 'right' }}>
          <button type="button" onClick={() => onOpen(item)} style={buttonStyle(true)}>
            {isPhoto ? 'Open photo' : 'Edit'}
          </button>
        </div>
        <div
          data-testid="map-popup-arrow"
          style={{
            position: 'absolute',
            left: '50%',
            bottom: -ARROW_PX,
            marginLeft: -ARROW_PX,
            width: 0,
            height: 0,
            borderLeft: `${ARROW_PX}px solid transparent`,
            borderRight: `${ARROW_PX}px solid transparent`,
            borderTop: `${ARROW_PX}px solid ${BORDER_COLOR}`,
          }}
        />
      </div>
    </Html>
  )
}

function buttonStyle(primary: boolean): CSSProperties {
  return {
    minWidth: TOUCH_TARGET_PX,
    minHeight: TOUCH_TARGET_PX,
    padding: '0 10px',
    font: `600 12px ${SANS_FONT}`,
    color: primary ? '#0b0d10' : '#e6edf3',
    background: primary ? '#4cc9f0' : 'transparent',
    border: `1px solid ${primary ? '#4cc9f0' : BORDER_COLOR}`,
    borderRadius: 4,
    cursor: 'pointer',
  }
}
