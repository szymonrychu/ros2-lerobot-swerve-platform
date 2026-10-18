/** Toolbar above the POI list: category filter toggles (not MUI Chips: the list rows own the chip look) with live counts, mark-all-visible, clear marks, marked count. Pure view. */
import Button from '@mui/material/Button'
import Stack from '@mui/material/Stack'
import Typography from '@mui/material/Typography'
import { CATEGORIES, CATEGORY_LABELS } from './categories'
import { UNCATEGORISED } from './selection'

/** Filter toggles in display order: every category, then uncategorised. */
export const FILTER_CATEGORIES: readonly string[] = [...CATEGORIES, UNCATEGORISED]

/**
 * Label of a filter chip.
 * @param category - a CATEGORIES entry or UNCATEGORISED.
 * @returns the human label.
 */
export function filterLabel(category: string): string {
  return category in CATEGORY_LABELS ? CATEGORY_LABELS[category as keyof typeof CATEGORY_LABELS] : category
}

export interface PoiListToolbarProps {
  /** Number of POIs per category (missing = 0). */
  counts: Readonly<Record<string, number>>
  /** Selected categories; empty shows all. */
  selected: ReadonlySet<string>
  markedCount: number
  onToggleCategory: (category: string) => void
  onMarkAllVisible: () => void
  onClearMarks: () => void
}

export function PoiListToolbar({ counts, selected, markedCount, onToggleCategory, onMarkAllVisible, onClearMarks }: PoiListToolbarProps) {
  return (
    <Stack spacing={0.5} sx={{ pb: 0.5 }} data-testid="poi-list-toolbar">
      <Stack direction="row" spacing={0.5} useFlexGap sx={{ flexWrap: 'wrap' }}>
        {FILTER_CATEGORIES.map((c) => (
          <Button
            key={c}
            size="small"
            aria-pressed={selected.has(c)}
            variant={selected.has(c) ? 'contained' : 'outlined'}
            onClick={() => onToggleCategory(c)}
            sx={{ borderRadius: 4, textTransform: 'none', minWidth: 0 }}
          >
            {`${filterLabel(c)} (${counts[c] ?? 0})`}
          </Button>
        ))}
      </Stack>
      <Stack direction="row" spacing={1} sx={{ alignItems: 'center', flexWrap: 'wrap' }}>
        <Button size="small" onClick={onMarkAllVisible}>
          Mark all visible
        </Button>
        <Button size="small" onClick={onClearMarks} disabled={markedCount === 0}>
          Clear marks
        </Button>
        <Typography variant="caption" data-testid="poi-marked-count">
          {markedCount} marked
        </Typography>
      </Stack>
    </Stack>
  )
}
