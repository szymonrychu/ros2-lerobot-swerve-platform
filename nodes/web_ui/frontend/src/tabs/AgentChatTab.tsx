import { useCallback, useRef, useState } from 'react'
import type { KeyboardEvent } from 'react'
import type { VirtuosoHandle } from 'react-virtuoso'
import Alert from '@mui/material/Alert'
import Box from '@mui/material/Box'
import Button from '@mui/material/Button'
import Dialog from '@mui/material/Dialog'
import Stack from '@mui/material/Stack'
import TextField from '@mui/material/TextField'
import Typography from '@mui/material/Typography'
import SendIcon from '@mui/icons-material/Send'
import { postAgent } from '../agent/agentApi'
import { AgentHeader } from '../agent/AgentHeader'
import { ChatLog } from '../agent/ChatLog'
import { composerBlockReason } from '../agent/agentModel'
import { useAgentChat } from '../agent/useAgentChat'
import { cutoffBanner } from '../battery/batteryStatus'
import type { BatteryStatus } from '../battery/batteryStatus'
import { TOUCH_TARGET_PX } from '../theme'
import { TabConfig } from '../types'

interface Props {
  tab: TabConfig
  topicData: Record<string, unknown>
  publish: (topic: string, msgType: string, data: unknown) => void
  battery?: BatteryStatus
}

export default function AgentChatTab({ battery }: Props) {
  const { chat, info, connected, refreshInfo, loadOlder, loadingOlder, setFollowing, clearCache } = useAgentChat()
  const [text, setText] = useState('')
  const [actionError, setActionError] = useState<string | null>(null)
  const [viewImage, setViewImage] = useState<string | null>(null)
  const listRef = useRef<VirtuosoHandle>(null)

  const batteryMessage = battery ? cutoffBanner(battery) : null
  const blockReason = composerBlockReason({ busy: chat.busy, connected, batteryCutoff: batteryMessage !== null, batteryMessage })
  const canSend = blockReason === null && text.trim().length > 0
  const resetBlocked = chat.busy || batteryMessage !== null

  const scrollToEnd = useCallback(() => listRef.current?.scrollToIndex({ index: 'LAST', align: 'end' }), [])

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
    setActionError(null)
    const res = await postAgent('reset')
    if (res.ok) clearCache()
    else setActionError(res.message || 'Reset failed')
    refreshInfo()
  }

  return (
    // Absolutely fill the (position: relative) main area so the transcript, not the page, is what scrolls.
    <Box sx={{ position: 'absolute', inset: 0, display: 'flex', flexDirection: 'column', minHeight: 0, minWidth: 0 }}>
      <AgentHeader info={info} chat={chat} connected={connected} resetBlocked={resetBlocked} onStop={() => void stop()} onReset={() => void reset()} />

      <Box sx={{ flex: 1, minHeight: 0, position: 'relative' }}>
        {chat.items.length === 0 && (
          <Typography color="text.secondary" sx={{ position: 'absolute', inset: 0, display: 'flex', alignItems: 'center', justifyContent: 'center', textAlign: 'center', px: 2 }}>
            {connected ? 'No messages yet' : 'Connecting to the agent...'}
          </Typography>
        )}
        <ChatLog
          items={chat.items}
          firstItemIndex={chat.firstItemIndex}
          loadingOlder={loadingOlder}
          hasMore={chat.hasMore}
          listRef={listRef}
          onOpenImage={setViewImage}
          onFollowingChange={setFollowing}
          onStartReached={loadOlder}
        />
      </Box>

      <Box sx={{ px: 2, py: 1, borderTop: 1, borderColor: 'divider' }}>
        {actionError && (
          <Alert severity="error" onClose={() => setActionError(null)} sx={{ mb: 1 }}>
            {actionError}
          </Alert>
        )}
        {blockReason && !chat.busy && (
          <Typography variant="caption" color={batteryMessage ? 'error' : 'text.secondary'} component="p" sx={{ mb: 1 }}>
            {blockReason}
          </Typography>
        )}
        <Stack direction="row" gap={1} alignItems="flex-end">
          <TextField
            value={text}
            onChange={(e) => setText(e.target.value)}
            onKeyDown={onKeyDown}
            placeholder="Message"
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

      <Dialog open={viewImage !== null} onClose={() => setViewImage(null)} maxWidth="lg">
        {viewImage && <Box component="img" src={viewImage} alt="tool result enlarged" sx={{ maxWidth: '100%', display: 'block' }} onClick={() => setViewImage(null)} />}
      </Dialog>
    </Box>
  )
}
