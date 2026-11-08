import { useEffect, useState } from 'react'
import Box from '@mui/material/Box'
import Link from '@mui/material/Link'
import Typography from '@mui/material/Typography'
import { getState } from './rgbdApi'
import { BoardRole, BoardSpec, ModelFile, boardDownloads, boardLabel, listModels } from './boardsApi'

/** Download link text per file extension. */
const FORMAT_LABELS: Record<string, string> = { pdf: 'PDF', '3mf': '3MF' }

const formatLabel = (file: ModelFile) => FORMAT_LABELS[file.name.split('.').pop() ?? ''] ?? file.name

/**
 * The board a calibration step needs, from the node state, with its PDF and 3MF downloads from web_ui.
 * Fetches both once; a failed call shows the API message instead (nothing is made up).
 */
export function BoardNeeded({ role }: { role: BoardRole }) {
  const [board, setBoard] = useState<BoardSpec | null>(null)
  const [files, setFiles] = useState<ModelFile[]>([])
  const [errors, setErrors] = useState<string[]>([])

  useEffect(() => {
    let live = true
    void (async () => {
      const [state, models] = await Promise.all([getState(), listModels()])
      if (!live) return
      if (state.ok) setBoard(state.value.boards[role])
      if (models.ok) setFiles(models.value)
      setErrors([state, models].flatMap((r) => (r.ok ? [] : [r.message])))
    })()
    return () => {
      live = false
    }
  }, [role])

  const downloads = board ? boardDownloads(board, files) : []
  return (
    <Box sx={{ mt: 1 }}>
      {board && <Typography variant="body2">{boardLabel(role, board)}</Typography>}
      {downloads.length > 0 && (
        <Typography variant="body2">
          {downloads.map((f, i) => (
            <span key={f.name}>
              {i > 0 && ' / '}
              <Link href={f.url} download={f.name}>{`${formatLabel(f)} (${f.name})`}</Link>
            </span>
          ))}
        </Typography>
      )}
      {errors.map((m, i) => (
        <Typography key={i} variant="body2" color="error">
          {m}
        </Typography>
      ))}
    </Box>
  )
}
