// @vitest-environment jsdom
import { afterEach, describe, expect, it, vi } from 'vitest'
import { cleanup, fireEvent, render, screen } from '@testing-library/react'
import { ThemeProvider } from '@mui/material/styles'
import { theme } from '../theme'
import { ConfirmDialog } from './ConfirmDialog'

afterEach(() => cleanup())

function setup(open = true) {
  const onConfirm = vi.fn()
  const onCancel = vi.fn()
  render(
    <ThemeProvider theme={theme}>
      <ConfirmDialog open={open} title="Delete photo" body="This cannot be undone." confirmLabel="Delete" onConfirm={onConfirm} onCancel={onCancel} />
    </ThemeProvider>,
  )
  return { onConfirm, onCancel }
}

describe('ConfirmDialog', () => {
  it('renders nothing when closed', () => {
    setup(false)
    expect(screen.queryByRole('dialog')).toBeNull()
  })

  it('shows title, body and both buttons', () => {
    setup()
    expect(screen.getByText('Delete photo')).toBeTruthy()
    expect(screen.getByText('This cannot be undone.')).toBeTruthy()
    expect(screen.getByRole('button', { name: 'Cancel' })).toBeTruthy()
    expect(screen.getByRole('button', { name: 'Delete' })).toBeTruthy()
  })

  it('confirm calls onConfirm once and never onCancel', () => {
    const { onConfirm, onCancel } = setup()
    fireEvent.click(screen.getByRole('button', { name: 'Delete' }))
    expect(onConfirm).toHaveBeenCalledTimes(1)
    expect(onCancel).not.toHaveBeenCalled()
  })

  it('Cancel calls onCancel and never onConfirm', () => {
    const { onConfirm, onCancel } = setup()
    fireEvent.click(screen.getByRole('button', { name: 'Cancel' }))
    expect(onCancel).toHaveBeenCalledTimes(1)
    expect(onConfirm).not.toHaveBeenCalled()
  })

  it('Escape calls onCancel and never onConfirm', () => {
    const { onConfirm, onCancel } = setup()
    fireEvent.keyDown(screen.getByRole('dialog'), { key: 'Escape' })
    expect(onCancel).toHaveBeenCalledTimes(1)
    expect(onConfirm).not.toHaveBeenCalled()
  })

  it('confirm is an outlined error button, not a second filled control', () => {
    setup()
    const confirm = screen.getByRole('button', { name: 'Delete' })
    expect(confirm.className).toContain('MuiButton-outlinedError')
    expect(confirm.className).not.toContain('MuiButton-contained')
  })
})
