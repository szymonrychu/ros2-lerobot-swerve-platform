/** Job header chip: the time budget the agent declared in set_task_plan and the time left of it, ticking each second. */
import { useEffect, useState } from 'react'
import { StatusItem, type StatusState } from '../components/StatusItem'
import { timeBudget, timeBudgetLabel } from './agentModel'
import type { ChatState } from './agentModel'

/** Refresh period of the time left while the job runs. */
const TICK_MS = 1000
/** Seconds left below which the reading is marked as a warning. */
const LOW_LEFT_S = 120

/**
 * Current Unix time in milliseconds, refreshed every TICK_MS while running.
 *
 * @param running - tick only while true (the job runs); otherwise the last value stays
 * @returns Date.now() as of the last tick
 */
export function useNowMs(running: boolean): number {
  const [nowMs, setNowMs] = useState(() => Date.now())
  useEffect(() => {
    if (!running) return undefined
    setNowMs(Date.now())
    const id = window.setInterval(() => setNowMs(Date.now()), TICK_MS)
    return () => window.clearInterval(id)
  }, [running])
  return nowMs
}

/**
 * Declared time budget and time left of the running job.
 *
 * @param state - live chat state (plan with time_budget_min and deadline, busy flag)
 * @returns the chip, or nothing before a plan declared a budget
 */
export function TimeBudgetChip({ state }: { state: ChatState }) {
  const nowMs = useNowMs(state.busy)
  const budget = timeBudget(state, nowMs)
  if (budget === null) return null
  const level: StatusState = budget.leftS <= 0 ? 'error' : budget.leftS < LOW_LEFT_S ? 'warn' : 'ok'
  return (
    <span data-testid="time-budget">
      <StatusItem label="" value={timeBudgetLabel(budget)} state={level} />
    </span>
  )
}
