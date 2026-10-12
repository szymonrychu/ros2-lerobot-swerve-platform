import { useState } from 'react'
import type { ReactNode, Ref } from 'react'
import { Virtuoso } from 'react-virtuoso'
import type { VirtuosoHandle } from 'react-virtuoso'
import Box from '@mui/material/Box'
import CircularProgress from '@mui/material/CircularProgress'
import Collapse from '@mui/material/Collapse'
import IconButton from '@mui/material/IconButton'
import Stack from '@mui/material/Stack'
import Typography from '@mui/material/Typography'
import ExpandMoreIcon from '@mui/icons-material/ExpandMore'
import { PhaseProgress } from './PhaseProgress'
import { phaseProgress, statusLabel } from './agentModel'
import type { ChatItem, RobotEventSeverity, ToolContent } from './agentModel'
import { MONO_FONT, TOUCH_TARGET_PX } from '../theme'

type ToolItem = Extract<ChatItem, { kind: 'tool' }>
type PlanItem = Extract<ChatItem, { kind: 'plan' | 'plan_revised' }>
type PhaseStartedItem = Extract<ChatItem, { kind: 'phase_started' }>
type PhaseCompletedItem = Extract<ChatItem, { kind: 'phase_completed' }>
type Role = 'User' | 'Agent' | 'Tool' | 'System'

const ROLE_WIDTH = '5rem'
// Distance from the bottom (px) within which the transcript keeps following new messages.
const STICK_THRESHOLD_PX = 48
const RESULT_MAX_HEIGHT = 'clamp(8rem, 30dvh, 14rem)'
const THUMB_MAX_HEIGHT = 'clamp(5rem, 15dvh, 7.5rem)'
const SEVERITY_COLOR: Record<RobotEventSeverity, string> = { info: 'text.primary', warning: 'warning.main', critical: 'error.main' }

/**
 * One log row: a fixed-width role column and the content beside it.
 * @param role - column label
 * @param children - row content
 * @returns the row element
 */
function LogRow({ role, children }: { role: Role; children: ReactNode }) {
  return (
    <Box data-testid="chat-row" sx={{ display: 'flex', gap: 1, alignItems: 'baseline', minWidth: 0 }}>
      <Typography data-testid="chat-role" variant="caption" color="text.secondary" sx={{ width: ROLE_WIDTH, flexShrink: 0, fontFamily: MONO_FONT }}>
        {role}
      </Typography>
      <Box sx={{ flex: 1, minWidth: 0 }}>{children}</Box>
    </Box>
  )
}

function ImageThumb({ content, onOpen }: { content: Extract<ToolContent, { type: 'image' }>; onOpen: (src: string) => void }) {
  const src = `data:${content.media_type};base64,${content.data_b64}`
  return (
    <Box
      component="img"
      src={src}
      alt="tool result"
      onClick={() => onOpen(src)}
      sx={{ maxWidth: `calc(${THUMB_MAX_HEIGHT} * 1.6)`, maxHeight: THUMB_MAX_HEIGHT, borderRadius: 1, cursor: 'zoom-in', border: 1, borderColor: 'divider' }}
    />
  )
}

function ToolEntry({ item, onOpenImage }: { item: ToolItem; onOpenImage: (src: string) => void }) {
  const [open, setOpen] = useState(false)
  const { result } = item
  const texts = result?.content.filter((c): c is Extract<ToolContent, { type: 'text' }> => c.type === 'text') ?? []
  const images = result?.content.filter((c): c is Extract<ToolContent, { type: 'image' }> => c.type === 'image') ?? []
  const isError = result?.isError === true
  const status = result === null ? 'running' : isError ? 'error' : 'ok'
  return (
    <LogRow role="Tool">
      <Box onClick={() => setOpen((o) => !o)} sx={{ display: 'flex', alignItems: 'center', gap: 1, minHeight: TOUCH_TARGET_PX, cursor: 'pointer', flexWrap: 'wrap' }}>
        <Typography variant="body2" sx={{ fontFamily: MONO_FONT, fontWeight: 500, wordBreak: 'break-all' }}>
          {item.orphan ? `result of call ${item.id}` : item.name}
        </Typography>
        {!item.orphan && (
          <Typography data-testid="tool-kind" variant="caption" color="text.secondary" sx={{ fontFamily: MONO_FONT }}>
            {item.toolKind}
          </Typography>
        )}
        <Typography data-testid="tool-status" data-error={isError} variant="caption" color={isError ? 'error.main' : 'text.secondary'} sx={{ fontFamily: MONO_FONT }}>
          {status}
        </Typography>
        <Box sx={{ flex: 1 }} />
        <IconButton size="small" aria-label={open ? 'Collapse tool call' : 'Expand tool call'} sx={{ transform: open ? 'rotate(180deg)' : 'none' }}>
          <ExpandMoreIcon />
        </IconButton>
      </Box>
      {images.length > 0 && (
        <Stack direction="row" gap={1} flexWrap="wrap" sx={{ pb: 1 }}>
          {images.map((img, i) => (
            <ImageThumb key={i} content={img} onOpen={onOpenImage} />
          ))}
        </Stack>
      )}
      <Collapse in={open} unmountOnExit>
        <Box sx={{ pb: 1 }}>
          {!item.orphan && (
            <>
              <Typography variant="overline" color="text.secondary">Input</Typography>
              <Box component="pre" sx={{ m: 0, p: 1, bgcolor: 'background.default', borderRadius: 1, fontFamily: MONO_FONT, fontSize: 12, overflow: 'auto', maxHeight: RESULT_MAX_HEIGHT }}>
                {JSON.stringify(item.input, null, 2)}
              </Box>
            </>
          )}
          {texts.length > 0 && (
            <>
              <Typography variant="overline" color="text.secondary">Output</Typography>
              <Box
                component="pre"
                sx={{ m: 0, p: 1, bgcolor: 'background.default', borderRadius: 1, fontFamily: MONO_FONT, fontSize: 12, overflow: 'auto', maxHeight: RESULT_MAX_HEIGHT, whiteSpace: 'pre-wrap', wordBreak: 'break-word', color: isError ? 'error.main' : 'text.primary' }}
              >
                {texts.map((t) => t.text).join('\n')}
              </Box>
              {result?.truncated && <Typography variant="caption" color="warning.main">output truncated</Typography>}
            </>
          )}
        </Box>
      </Collapse>
    </LogRow>
  )
}

function PlanEntry({ item }: { item: PlanItem }) {
  const revised = item.kind === 'plan_revised'
  return (
    <LogRow role="System">
      <Box sx={{ border: 1, borderColor: 'divider', borderRadius: 1, px: 2, py: 1 }}>
        <Typography variant="body2" sx={{ fontWeight: 500 }}>
          {revised ? 'Plan revised' : 'Task plan'}: {item.complexity.replace('_', ' ')}, {item.phases.length} phases
        </Typography>
        <Typography variant="body2" color="text.secondary" sx={{ mt: 0.5, whiteSpace: 'pre-wrap', wordBreak: 'break-word', overflowWrap: 'anywhere' }}>
          {revised ? item.revisionRationale : item.rationale}
        </Typography>
        <Stack component="ol" sx={{ m: 0, mt: 0.5, pl: 3 }}>
          {item.phases.map((p) => (
            <Typography component="li" variant="caption" key={p.index} sx={{ overflowWrap: 'anywhere' }}>
              <b>{p.name}</b> ({p.goal}) - rw {p.rwCap}, turns {p.turnCap}
            </Typography>
          ))}
        </Stack>
      </Box>
    </LogRow>
  )
}

function PhaseStartedEntry({ item }: { item: PhaseStartedItem }) {
  return (
    <LogRow role="System">
      <Stack direction="row" gap={1} alignItems="center" flexWrap="wrap">
        <Typography variant="body2" sx={{ fontWeight: 500 }}>Phase {item.index + 1} started: {item.name}</Typography>
        <PhaseProgress {...phaseProgress(item.attempt, item.maxAttempts, item.iteration)} pRetrySuccess={null} />
        <Typography variant="caption" color="text.secondary" sx={{ overflowWrap: 'anywhere' }}>
          {item.goal} (rw {item.rwCap}, turns {item.turnCap})
        </Typography>
      </Stack>
    </LogRow>
  )
}

function PhaseCompletedEntry({ item }: { item: PhaseCompletedItem }) {
  const bad = item.outcome !== 'done'
  return (
    <LogRow role="System">
      <Stack direction="row" gap={1} alignItems="center" flexWrap="wrap">
        <Typography variant="body2" color={bad ? 'error.main' : 'text.primary'} sx={{ fontWeight: 500 }}>
          Phase {item.index + 1} {item.outcome}: {item.name}
        </Typography>
        <PhaseProgress {...phaseProgress(item.attempt, item.maxAttempts, item.iteration)} pRetrySuccess={item.pRetrySuccess} />
        <Typography variant="caption" color="text.secondary">
          sensors {item.usage.roUsed}, rw {item.usage.rwUsed}/{item.caps.rwCap}, turns {item.usage.turnsUsed}/{item.caps.turnCap}
        </Typography>
        {item.summary && <Typography variant="caption" color="text.secondary" sx={{ flexBasis: '100%', overflowWrap: 'anywhere' }}>{item.summary}</Typography>}
      </Stack>
    </LogRow>
  )
}

function TextEntry({ role, text }: { role: Role; text: string }) {
  return (
    <LogRow role={role}>
      <Typography variant="body2" component="div" sx={{ whiteSpace: 'pre-wrap', wordBreak: 'break-word', overflowWrap: 'anywhere' }}>{text}</Typography>
    </LogRow>
  )
}

function SystemText({ color, children }: { color: string; children: ReactNode }) {
  return (
    <LogRow role="System">
      <Typography variant="body2" color={color} sx={{ overflowWrap: 'anywhere' }}>{children}</Typography>
    </LogRow>
  )
}

/**
 * One transcript entry as a flat log row.
 * @param item - the chat item
 * @param onOpenImage - called with a data URL when a tool image is clicked
 * @returns the row element
 */
export function ChatEntry({ item, onOpenImage }: { item: ChatItem; onOpenImage: (src: string) => void }) {
  switch (item.kind) {
    case 'user':
      return <TextEntry role="User" text={item.text} />
    case 'assistant':
      return <TextEntry role="Agent" text={item.text} />
    case 'tool':
      return <ToolEntry item={item} onOpenImage={onOpenImage} />
    case 'denied':
      return <SystemText color="warning.main">Tool denied: <b>{item.name}</b> - {item.reason}</SystemText>
    case 'turn_end': {
      const s = statusLabel(item.status)
      return (
        <SystemText color="text.secondary">
          Turn {s.label}: {item.numTurns} turns, {item.effectorCalls} rw calls, ${item.costUsd.toFixed(4)}
        </SystemText>
      )
    }
    case 'plan':
    case 'plan_revised':
      return <PlanEntry item={item} />
    case 'phase_started':
      return <PhaseStartedEntry item={item} />
    case 'phase_completed':
      return <PhaseCompletedEntry item={item} />
    case 'robot_event':
      return (
        <SystemText color={SEVERITY_COLOR[item.severity]}>
          Robot event ({item.severity}) <b>{item.eventType}</b>
          {item.source ? ` from ${item.source}` : ''}: {item.message}
        </SystemText>
      )
    case 'error':
      return <SystemText color="error.main">{item.message}</SystemText>
  }
}

interface ListContext {
  loadingOlder: boolean
  hasMore: boolean
}

function ListHeader({ context }: { context?: ListContext }) {
  if (!context) return null
  return (
    <Box sx={{ pt: 2, pb: 1, display: 'flex', justifyContent: 'center', alignItems: 'center', gap: 1 }}>
      {context.loadingOlder ? (
        <>
          <CircularProgress size={14} />
          <Typography variant="caption" color="text.secondary">Loading older messages...</Typography>
        </>
      ) : (
        !context.hasMore && <Typography variant="caption" color="text.secondary">Start of session</Typography>
      )}
    </Box>
  )
}

function ListFooter() {
  return <Box sx={{ height: 12 }} />
}

const LIST_COMPONENTS = { Header: ListHeader, Footer: ListFooter }

interface ChatLogProps {
  items: ChatItem[]
  firstItemIndex: number
  loadingOlder: boolean
  hasMore: boolean
  listRef: Ref<VirtuosoHandle>
  onOpenImage: (src: string) => void
  onFollowingChange: (atBottom: boolean) => void
  onStartReached: () => void
}

/**
 * The agent transcript: a virtualised flat log with a role column.
 * @param props - items and scroll callbacks
 * @returns the list element
 */
export function ChatLog({ items, firstItemIndex, loadingOlder, hasMore, listRef, onOpenImage, onFollowingChange, onStartReached }: ChatLogProps) {
  const context: ListContext = { loadingOlder, hasMore }
  return (
    <Virtuoso
      ref={listRef}
      style={{ height: '100%' }}
      data={items}
      firstItemIndex={firstItemIndex}
      initialTopMostItemIndex={Math.max(items.length - 1, 0)}
      computeItemKey={(_, item) => item.key}
      followOutput={(atBottom) => (atBottom ? 'auto' : false)}
      atBottomThreshold={STICK_THRESHOLD_PX}
      atBottomStateChange={onFollowingChange}
      startReached={onStartReached}
      context={context}
      components={LIST_COMPONENTS}
      itemContent={(_, item) => (
        // Padding, not margin: Virtuoso measures the wrapper and margins would not count.
        <Box sx={{ px: 2, pb: 1, minWidth: 0 }}>
          <ChatEntry item={item} onOpenImage={onOpenImage} />
        </Box>
      )}
    />
  )
}
