/** The printed ChArUco boards of the calibration steps and their downloads (GET /api/models of web_ui). */
import { GET, call } from './rgbdApi'
import type { BoardSpec, RgbdApiResult } from './rgbdApi'

export type { BoardSpec }

/** Which board a step needs: FULL (intrinsics) or ARM (held by its handle in the gripper; head-arm and arm fits). */
export type BoardRole = 'full' | 'arm'

/** One printable model file served by web_ui at `url` (GET /api/models lists them). */
export interface ModelFile {
  name: string
  size: number
  url: string
}

const MODELS_URL = '/api/models'
/** Download formats of a board, in display order. */
const BOARD_EXTENSIONS = ['pdf', '3mf']

const isModelFile = (v: unknown) => {
  const f = v as Record<string, unknown> | null
  return typeof f === 'object' && f !== null && typeof f.name === 'string' && typeof f.size === 'number' && typeof f.url === 'string'
}
const isModels = (v: unknown) => {
  const files = (v as Record<string, unknown> | null)?.files
  return Array.isArray(files) && files.every(isModelFile)
}

/** The served model files (`files` of GET /api/models); the API message on a failure. */
export const listModels = async (): Promise<RgbdApiResult<ModelFile[]>> => {
  const r = await call<{ files: ModelFile[] }>(MODELS_URL, GET, isModels)
  return r.ok ? { ok: true, value: r.value.files } : r
}

/** The PDF then the 3MF of a board, matched by its name as the file stem; formats not served are left out. */
export function boardDownloads(board: BoardSpec, files: ModelFile[]): ModelFile[] {
  return BOARD_EXTENSIONS.flatMap((ext) => files.filter((f) => f.name === `${board.name}.${ext}`))
}

/** Operator label of a board, e.g. "ARM board: charuco-arm-6x5-28mm, 6x5 squares of 28 mm". */
export function boardLabel(role: BoardRole, board: BoardSpec): string {
  return `${role.toUpperCase()} board: ${board.name}, ${board.squares_x}x${board.squares_y} squares of ${board.square_mm} mm`
}
