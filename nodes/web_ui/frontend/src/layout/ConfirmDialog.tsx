/** The one confirm pattern for destructive actions: a modal with Cancel and an error-coloured confirm button. */
import { Button, Dialog, DialogActions, DialogContent, DialogContentText, DialogTitle } from '@mui/material'

export interface ConfirmDialogProps {
  /** Closed dialogs render nothing. */
  open: boolean
  /** Question shown as the dialog heading. */
  title: string
  /** Optional explanation below the heading. */
  body?: string
  /** Label of the destructive button, e.g. "Delete". */
  confirmLabel: string
  /** Called once when the confirm button is pressed. */
  onConfirm: () => void
  /** Called on Cancel, Escape and backdrop click; never calls onConfirm. */
  onCancel: () => void
}

/**
 * @param props see ConfirmDialogProps.
 * @returns the dialog element.
 */
export function ConfirmDialog({ open, title, body, confirmLabel, onConfirm, onCancel }: ConfirmDialogProps) {
  return (
    <Dialog open={open} onClose={onCancel} aria-labelledby="confirm-dialog-title">
      <DialogTitle id="confirm-dialog-title">{title}</DialogTitle>
      {body && (
        <DialogContent>
          <DialogContentText>{body}</DialogContentText>
        </DialogContent>
      )}
      <DialogActions>
        <Button onClick={onCancel}>Cancel</Button>
        <Button color="error" variant="contained" onClick={onConfirm}>{confirmLabel}</Button>
      </DialogActions>
    </Dialog>
  )
}
