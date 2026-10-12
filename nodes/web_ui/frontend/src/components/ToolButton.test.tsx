// @vitest-environment jsdom
import { afterEach, describe, expect, it, vi } from 'vitest'
import { cleanup, fireEvent, render, screen } from '@testing-library/react'
import { ThemeProvider } from '@mui/material/styles'
import PlaceIcon from '@mui/icons-material/Place'
import { ToolButton, ToolGroup } from './ToolButton'
import { designViolations, SOURCES } from '../layout/designRules'
import { theme } from '../theme'

function wrap(node: React.ReactNode) {
  return render(<ThemeProvider theme={theme}>{node}</ThemeProvider>)
}

afterEach(cleanup)

describe('ToolButton', () => {
  it('is an icon button whose aria-label is the tooltip text, with no title attribute', () => {
    wrap(<ToolButton label="Set goal" icon={<PlaceIcon />} onClick={() => undefined} />)
    const button = screen.getByRole('button', { name: 'Set goal' })
    expect(button.getAttribute('aria-label')).toBe('Set goal')
    expect(button.hasAttribute('title')).toBe(false)
    expect(button.textContent).toBe('')
  })

  it('calls onClick', () => {
    const onClick = vi.fn()
    wrap(<ToolButton label="Fit map" icon={<PlaceIcon />} onClick={onClick} />)
    fireEvent.click(screen.getByRole('button', { name: 'Fit map' }))
    expect(onClick).toHaveBeenCalledTimes(1)
  })

  it('sets aria-pressed only for toggles', () => {
    wrap(
      <>
        <ToolButton label="Plain" icon={<PlaceIcon />} onClick={() => undefined} />
        <ToolButton label="On" icon={<PlaceIcon />} onClick={() => undefined} pressed />
        <ToolButton label="Off" icon={<PlaceIcon />} onClick={() => undefined} pressed={false} />
      </>,
    )
    expect(screen.getByRole('button', { name: 'Plain' }).hasAttribute('aria-pressed')).toBe(false)
    expect(screen.getByRole('button', { name: 'On' }).getAttribute('aria-pressed')).toBe('true')
    expect(screen.getByRole('button', { name: 'Off' }).getAttribute('aria-pressed')).toBe('false')
  })

  it('shows the active state as the accent icon colour on bgRaised, and a quiet colour otherwise', () => {
    wrap(
      <>
        <ToolButton label="On" icon={<PlaceIcon />} onClick={() => undefined} pressed />
        <ToolButton label="Off" icon={<PlaceIcon />} onClick={() => undefined} pressed={false} />
      </>,
    )
    const on = getComputedStyle(screen.getByRole('button', { name: 'On' }))
    const off = getComputedStyle(screen.getByRole('button', { name: 'Off' }))
    expect(on.color).toBe('rgb(232, 234, 237)') // TOKENS.accent
    expect(on.backgroundColor).toBe('rgb(29, 32, 38)') // TOKENS.bgRaised
    expect(off.color).not.toBe(on.color)
  })

  it('a disabled button does not call onClick and keeps its label', () => {
    const onClick = vi.fn()
    wrap(<ToolButton label="Center on robot" icon={<PlaceIcon />} onClick={onClick} disabled />)
    const button = screen.getByRole('button', { name: 'Center on robot' }) as HTMLButtonElement
    expect(button.disabled).toBe(true)
    fireEvent.click(button)
    expect(onClick).not.toHaveBeenCalled()
  })
})

describe('ToolGroup', () => {
  it('is a labelled group whose buttons share one 1 px bordered strip', () => {
    wrap(
      <ToolGroup label="Map view">
        <ToolButton label="One" icon={<PlaceIcon />} onClick={() => undefined} />
        <ToolButton label="Two" icon={<PlaceIcon />} onClick={() => undefined} />
      </ToolGroup>,
    )
    const group = screen.getByRole('group', { name: 'Map view' })
    expect(group.querySelectorAll('button')).toHaveLength(2)
    const style = getComputedStyle(group)
    expect(style.borderWidth).toBe('1px')
    expect(style.borderStyle).toBe('solid')
  })
})

describe('design rules', () => {
  it('ToolButton.tsx has no violations', () => {
    expect(designViolations('components/ToolButton.tsx', SOURCES['components/ToolButton.tsx'])).toEqual([])
  })
})
