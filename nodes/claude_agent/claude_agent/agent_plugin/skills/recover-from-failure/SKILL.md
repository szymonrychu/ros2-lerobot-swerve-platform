---
name: recover-from-failure
description: Use when a phase failed, a motion was refused or interrupted, or the robot got stuck. Stops safely, re-senses, classifies the failure, retries only with a change, and records what happened.
---

# Recover from failure

Follow these steps in order. Do not skip the stop and re-sense steps even when the cause looks obvious.

## 1. Stop motion safely

- Call `stop()` first. If a motion may still be running, call `cancel_motions()` as well.
- Do not issue any new motion until the robot is confirmed still.

## 2. Re-sense

- Call `get_body_state()` for the arm and base pose and `get_control_state()` for what is moving, leased or refused.
- Look with `get_camera_image()` (and `look_around()` when the view is blocked) so the next decision rests on what is there now, not on what you assumed.
- Read the refusal or error text exactly. A guard refusal states the rule that fired.

## 3. Classify the failure

Report it with `agent.complete_phase`, using its failure types:

- execution_failure: the step did not achieve its effect. A retry with a change may work.
- plan_failure: the steps worked but the plan was wrong. Do not retry; revise the plan.

Set `p_retry_success` honestly. It is your estimate that one more attempt succeeds, not a hope. If you cannot name what would be different, it is low.

## 4. Retry only with a change

- A retry must differ from the failed attempt: another approach angle, a slower speed, a different grasp or a clearer view. Write it in `retry_change`.
- Never repeat the same call with the same arguments.
- If the failure is a plan failure, or a retry is unlikely to work, call `agent.revise_plan`. If no plan is possible, report the failure plainly to the user.

## 5. Mark what hurt

- When the robot got stuck or a place is hazardous, record it with `add_poi` using category `problem` and a `problem_type` (for example window, mirror, glass, tight, overhang, low_obstacle, drop, cable, soft_floor or other).
- When the place must never be entered, mark a keep-out area instead.
- Check `list_pois` first so you do not add a duplicate.

## 6. Never work around a guard

- A guard refusal (speed, zone, limit, lease, budget) is a safety rule. Do not look for a different tool or sequence that reaches the same effect.
- Change the plan or report the refusal; ask the user when the task cannot proceed without the guard being lifted.

## 7. Record for the retrospective

- State in the phase summary what happened, what you saw, what you changed and why. The retrospective is built from these words, so name places, tools and the refusal text.
