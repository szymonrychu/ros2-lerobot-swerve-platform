import { useCallback, useRef, useState } from 'react'
import type { KeyboardEvent } from 'react'
import { Virtuoso } from 'react-virtuoso'
import type { VirtuosoHandle } from 'react-virtuoso'
import Alert from '@mui/material/Alert'
import Box from '@mui/material/Box'
import Button from '@mui/material/Button'
import Chip from '@mui/material/Chip'
import Collapse from '@mui/material/Collapse'
import Dialog from '@mui/material/Dialog'
import DialogActions from '@mui/material/DialogActions'
import DialogContent from '@mui/material/DialogContent'
import DialogContentText from '@mui/material/DialogContentText'
import DialogTitle from '@mui/material/DialogTitle'
import IconButton from '@mui/material/IconButton'
import CircularProgress from '@mui/material/CircularProgress'
import LinearProgress from '@mui/material/LinearProgress'
import Paper from '@mui/material/Paper'
import Stack from '@mui/material/Stack'
import TextField from '@mui/material/TextField'
import Typography from '@mui/material/Typography'
import ExpandMoreIcon from '@mui/icons-material/ExpandMore'
import SendIcon from '@mui/icons-material/Send'
import StopIcon from '@mui/icons-material/Stop'
import RestartAltIcon from '@mui/icons-material/RestartAlt'
import { postAgent } from '../agent/agentApi'
import { composerBlockReason, headerStatus, statusLabel } from '../agent/agentModel'
import type { ChatItem, ToolContent, ToolKind } from '../agent/agentModel'
import { useAgentChat } from '../agent/useAgentChat'
import { cutoffBanner } from '../battery/batteryStatus'
import type { BatteryStatus } from '../battery/batteryStatus'
import { MONO_FONT, TOUCH_TARGET_PX } from '../theme'
import { TabConfig } from '../types'

interface Props {
  tab: TabConfig
  topicData: Record<string, unknown>
  publish: (topic: string, msgType: string, data: unknown) => void
  battery?: BatteryStatus
}

type ToolItem = Extract<ChatItem, { kind: 'tool' }>

const KIND_COLORS: Record<ToolKind, 'info' | 'warning' | 'default' | 'secondary'> = {
  sensor: 'info',
  effector: 'warning',
  uncapped: 'default',
  notes: 'secondary',
}
// Distance from the bottom (px) within which the transcript keeps following new messages.
const STICK_THRESHOLD_PX = 48
const RESULT_MAX_HEIGHT_PX = 220
const THUMB_MAX_PX = 120

function ImageThumb({ content, onOpen }: { content: Extract<ToolContent, { type: 'image' }>; onOpen: (src: string) => void }) {
  const src = `data:${content.media_type};base64,${content.data_b64}`
  return (
    <Box
      component="img"
      src={src}
      alt="tool result"
      onClick={() => onOpen(src)}
      sx={{ maxWidth: THUMB_MAX_PX * 1.6, maxHeight: THUMB_MAX_PX, borderRadius: 1, cursor: 'zoom-in', border: 1, borderColor: 'divider' }}
    />
  )
}

function ToolCard({ item, onOpenImage }: { item: ToolItem; onOpenImage: (src: string) => void }) {
  const [open, setOpen] = useState(false)
  const { result } = item
  const texts = result?.content.filter((c): c is Extract<ToolContent, { type: 'text' }> => c.type === 'text') ?? []
  const images = result?.content.filter((c): c is Extract<ToolContent, { type: 'image' }> => c.type === 'image') ?? []
  const status = result === null ? { label: 'running', color: 'default' as const } : result.isError ? { label: 'error', color: 'error' as const } : { label: 'ok', color: 'success' as const }
  return (
    <Paper variant="outlined" sx={{ alignSelf: 'stretch', maxWidth: '100%', overflow: 'hidden' }}>
      <Box
        onClick={() => setOpen((o) => !o)}
        sx={{ display: 'flex', alignItems: 'center', gap: 1, px: 1.5, minHeight: TOUCH_TARGET_PX, cursor: 'pointer', flexWrap: 'wrap', py: 0.5 }}
      >
        <Typography variant="body2" sx={{ fontFamily: MONO_FONT, fontWeight: 600, wordBreak: 'break-all' }}>
          {item.orphan ? `result of call ${item.id}` : item.name}
        </Typography>
        {!item.orphan && <Chip size="small" label={item.toolKind} color={KIND_COLORS[item.toolKind]} variant={item.toolKind === 'uncapped' ? 'outlined' : 'filled'} />}
        <Chip size="small" label={status.label} color={status.color} variant="outlined" />
        <Box sx={{ flex: 1 }} />
        <IconButton size="small" aria-label={open ? 'Collapse tool call' : 'Expand tool call'} sx={{ transform: open ? 'rotate(180deg)' : 'none' }}>
          <ExpandMoreIcon />
        </IconButton>
      </Box>
      {images.length > 0 && (
        <Stack direction="row" gap={1} flexWrap="wrap" sx={{ px: 1.5, pb: 1 }}>
          {images.map((img, i) => (
            <ImageThumb key={i} content={img} onOpen={onOpenImage} />
          ))}
        </Stack>
      )}
      <Collapse in={open} unmountOnExit>
        <Box sx={{ px: 1.5, pb: 1.5 }}>
          {!item.orphan && (
            <>
              <Typography variant="overline" color="text.secondary">Input</Typography>
              <Box component="pre" sx={{ m: 0, p: 1, bgcolor: 'background.default', borderRadius: 1, fontFamily: MONO_FONT, fontSize: 12, overflow: 'auto', maxHeight: RESULT_MAX_HEIGHT_PX }}>
                {JSON.stringify(item.input, null, 2)}
              </Box>
            </>
          )}
          {texts.length > 0 && (
            <>
              <Typography variant="overline" color="text.secondary">Output</Typography>
              <Box
                component="pre"
                sx={{ m: 0, p: 1, bgcolor: 'background.default', borderRadius: 1, fontFamily: MONO_FONT, fontSize: 12, overflow: 'auto', maxHeight: RESULT_MAX_HEIGHT_PX, whiteSpace: 'pre-wrap', wordBreak: 'break-word', color: result?.isError ? 'error.main' : 'text.primary' }}
              >
                {texts.map((t) => t.text).join('\n')}
              </Box>
              {result?.truncated && <Typography variant="caption" color="warning.main">output truncated</Typography>}
            </>
          )}
        </Box>
      </Collapse>
    </Paper>
  )
}

function Bubble({ item }: { item: Extract<ChatItem, { kind: 'user' | 'assistant' }> }) {
  const user = item.kind === 'user'
  return (
    <Paper
      elevation={0}
      sx={{
        alignSelf: user ? 'flex-end' : 'flex-start',
        maxWidth: '85%',
        px: 1.5,
        py: 1,
        bgcolor: user ? 'primary.dark' : 'background.paper',
        border: user ? 0 : 1,
        borderColor: 'divider',
        whiteSpace: 'pre-wrap',
        wordBreak: 'break-word',
        overflowWrap: 'anywhere',
      }}
    >
      <Typography variant="body1" component="div">{item.text}</Typography>
    </Paper>
  )
}

function renderItem(item: ChatItem, onOpenImage: (src: string) => void) {
  switch (item.kind) {
    case 'user':
    case 'assistant':
      return <Bubble item={item} />
    case 'tool':
      return <ToolCard item={item} onOpenImage={onOpenImage} />
    case 'denied':
      return (
        <Alert severity="warning" sx={{ alignSelf: 'stretch' }}>
          Tool denied: <b>{item.name}</b> - {item.reason}
        </Alert>
      )
    case 'turn_end': {
      const s = statusLabel(item.status)
      return (
        <Stack direction="row" gap={1} alignItems="center" flexWrap="wrap" justifyContent="center" sx={{ py: 0.5 }}>
          <Chip size="small" label={s.label} color={s.color} />
          <Typography variant="caption" color="text.secondary">
            {item.numTurns} turns, {item.effectorCalls} effector calls, ${item.costUsd.toFixed(4)}
          </Typography>
        </Stack>
      )
    }
    case 'error':
      return (
        <Alert severity="error" sx={{ alignSelf: 'stretch' }}>
          {item.message}
        </Alert>
      )
  }
}

interface ListContext {
  loadingOlder: boolean
  hasMore: boolean
}

function ListHeader({ context }: { context?: ListContext }) {
  if (!context) return null
  return (
    <Box sx={{ pt: 1.5, pb: 1, display: 'flex', justifyContent: 'center', alignItems: 'center', gap: 1 }}>
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

export default function AgentChatTab({ battery }: Props) {
  const { chat, info, connected, refreshInfo, loadOlder, loadingOlder, setFollowing, clearCache } = useAgentChat()
  const [text, setText] = useState('')
  const [actionError, setActionError] = useState<string | null>(null)
  const [confirmReset, setConfirmReset] = useState(false)
  const [viewImage, setViewImage] = useState<string | null>(null)
  const listRef = useRef<VirtuosoHandle>(null)

  const header = headerStatus(info, chat)
  const batteryMessage = battery ? cutoffBanner(battery) : null
  const blockReason = composerBlockReason({ busy: chat.busy, connected, batteryCutoff: batteryMessage !== null, batteryMessage })
  const canSend = blockReason === null && text.trim().length > 0
  const resetBlocked = chat.busy || batteryMessage !== null

  const scrollToEnd = useCallback(() => listRef.current?.scrollToIndex({ index: 'LAST', align: 'end' }), [])

  const listContext = { loadingOlder, hasMore: chat.hasMore }

  const send = async () => {
    if (!canSend) return
    setActionError(null)
    const res = await postAgent('message', { text: text.trim() })
    if (res.ok) {
      setText('')
      setFollowing(true)
      scrollToEnd()
    } else setActionError(res.message || 'Message rejected')
    refreshInfo()
  }

  const onKeyDown = (e: KeyboardEvent) => {
    if (e.key === 'Enter' && !e.shiftKey && !e.nativeEvent.isComposing) {
      e.preventDefault()
      void send()
    }
  }

  const stop = async () => {
    setActionError(null)
    const res = await postAgent('stop')
    if (!res.ok) setActionError(res.message || 'Stop failed')
  }

  const reset = async () => {
    setConfirmReset(false)
    setActionError(null)
    const res = await postAgent('reset')
    if (res.ok) clearCache()
    else setActionError(res.message || 'Reset failed')
    refreshInfo()
  }

  return (
    // Absolutely fill the (position: relative) main area so the transcript, not the page, is what scrolls.
    <Box sx={{ position: 'absolute', inset: 0, display: 'flex', flexDirection: 'column', minHeight: 0, minWidth: 0 }}>
      <Box sx={{ px: 2, py: 1, display: 'flex', alignItems: 'center', gap: 1, flexWrap: 'wrap', borderBottom: 1, borderColor: 'divider' }}>
        <Chip size="small" label={header.model ?? 'model unknown'} variant="outlined" />
        <Chip size="small" color={header.busy ? 'warning' : connected ? 'success' : 'default'} label={header.busy ? 'working' : connected ? 'idle' : 'disconnected'} />
        <Chip size="small" variant="outlined" label={`effector ${header.effectorLabel}`} />
        {header.maxTurns !== null && <Chip size="small" variant="outlined" label={`turn cap ${header.maxTurns}`} />}
        <Box sx={{ flex: 1 }} />
        <Button size="small" startIcon={<StopIcon />} color="error" variant="outlined" disabled={!chat.busy} onClick={() => void stop()} sx={{ minHeight: 36 }}>
          Stop
        </Button>
        <Button size="small" startIcon={<RestartAltIcon />} variant="outlined" disabled={resetBlocked} onClick={() => setConfirmReset(true)} sx={{ minHeight: 36 }}>
          New session
        </Button>
      </Box>
      {chat.busy && <LinearProgress />}

      <Box sx={{ flex: 1, minHeight: 0, position: 'relative' }}>
        {chat.items.length === 0 && (
          <Typography color="text.secondary" sx={{ position: 'absolute', inset: 0, display: 'flex', alignItems: 'center', justifyContent: 'center', textAlign: 'center', px: 2 }}>
            {connected ? 'Ask the robot agent to do something.' : 'Connecting to the agent...'}
          </Typography>
        )}
        <Virtuoso
          ref={listRef}
          style={{ height: '100%' }}
          data={chat.items}
          firstItemIndex={chat.firstItemIndex}
          initialTopMostItemIndex={Math.max(chat.items.length - 1, 0)}
          computeItemKey={(_, item) => item.key}
          followOutput={(atBottom) => (atBottom ? 'auto' : false)}
          atBottomThreshold={STICK_THRESHOLD_PX}
          atBottomStateChange={setFollowing}
          startReached={loadOlder}
          context={listContext}
          components={LIST_COMPONENTS}
          itemContent={(_, item) => (
            // Padding, not margin: Virtuoso measures the wrapper and margins would not count.
            <Box sx={{ px: 2, pb: 1, display: 'flex', flexDirection: 'column', minWidth: 0 }}>{renderItem(item, setViewImage)}</Box>
          )}
        />
      </Box>

      <Box sx={{ px: 2, py: 1, borderTop: 1, borderColor: 'divider' }}>
        {actionError && (
          <Alert severity="error" onClose={() => setActionError(null)} sx={{ mb: 1 }}>
            {actionError}
          </Alert>
        )}
        {blockReason && !chat.busy && (
          <Alert severity={batteryMessage ? 'error' : 'info'} sx={{ mb: 1 }}>
            {blockReason}
          </Alert>
        )}
        <Stack direction="row" gap={1} alignItems="flex-end">
          <TextField
            value={text}
            onChange={(e) => setText(e.target.value)}
            onKeyDown={onKeyDown}
            placeholder="Message (Enter sends, Shift+Enter for a new line)"
            multiline
            maxRows={6}
            fullWidth
            size="small"
            disabled={blockReason !== null}
            inputProps={{ 'aria-label': 'Message to the agent' }}
          />
          <Button variant="contained" endIcon={<SendIcon />} disabled={!canSend} onClick={() => void send()} sx={{ minHeight: TOUCH_TARGET_PX }}>
            Send
          </Button>
        </Stack>
      </Box>

      <Dialog open={confirmReset} onClose={() => setConfirmReset(false)}>
        <DialogTitle>Start a new session?</DialogTitle>
        <DialogContent>
          <DialogContentText>The conversation and the effector call count are cleared.</DialogContentText>
        </DialogContent>
        <DialogActions>
          <Button onClick={() => setConfirmReset(false)}>Cancel</Button>
          <Button color="error" onClick={() => void reset()}>New session</Button>
        </DialogActions>
      </Dialog>

      <Dialog open={viewImage !== null} onClose={() => setViewImage(null)} maxWidth="lg">
        {viewImage && <Box component="img" src={viewImage} alt="tool result enlarged" sx={{ maxWidth: '100%', display: 'block' }} onClick={() => setViewImage(null)} />}
      </Dialog>
    </Box>
  )
}
