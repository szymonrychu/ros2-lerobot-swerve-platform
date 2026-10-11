"""CLI labelling of one trial: is the object in the target, then one key per pipeline stage, then an optional note.

Stage keys: y success, n fail, r not reached, x not applicable. In arm_only mode navigate defaults to x (an empty
answer accepts it). Any other key is refused and the question is asked again.
"""

from collections.abc import Callable

from pydantic import BaseModel

from scripts.eval.stages import (
    STAGE_NAVIGATE,
    STAGES,
    STATUS_FAIL,
    STATUS_NA,
    STATUS_NOT_REACHED,
    STATUS_OK,
    StageStatus,
)
from scripts.eval.testset import MODE_ARM_ONLY

STAGE_KEYS: dict[str, StageStatus] = {
    "y": STATUS_OK,
    "n": STATUS_FAIL,
    "r": STATUS_NOT_REACHED,
    "x": STATUS_NA,
}
SUCCESS_KEYS = {"y": True, "n": False}
STAGE_KEY_HELP = "y success / n fail / r not reached / x n/a"


class LabelResult(BaseModel):
    """Human labels of one trial."""

    success: bool
    labels: dict[str, StageStatus]
    first_failed_stage: str | None
    note: str


def ask_key(prompt: str, allowed: dict[str, object], input_fn: Callable[[str], str], default: str | None = None) -> str:
    """Ask until the answer is one of the allowed keys (case-insensitive); an empty answer takes the default.

    Args:
        prompt (str): Question text.
        allowed (dict[str, object]): Accepted keys.
        input_fn (Callable[[str], str]): Reads one answer.
        default (str | None): Key taken for an empty answer; None refuses empty answers.

    Returns:
        str: The accepted key.
    """
    while True:
        answer = input_fn(prompt).strip().lower()
        if not answer and default is not None:
            return default
        if answer in allowed:
            return answer
        print(f"  invalid key {answer!r}; use one of: {', '.join(allowed)}")


def ask(stages_claimed: dict[str, StageStatus], mode: str, input_fn: Callable[[str], str]) -> LabelResult:
    """Label one trial.

    Asks first whether the object is in the target at the end (the trial success), then each stage in pipeline order,
    then an optional note.

    Args:
        stages_claimed (dict[str, StageStatus]): The agent's claimed outcome per stage (shown as a hint).
        mode (str): ``arm_only`` makes navigate default to n/a.
        input_fn (Callable[[str], str]): Reads one answer.

    Returns:
        LabelResult: success, the label per stage, the first stage labelled fail (None when none) and the note.
    """
    success = SUCCESS_KEYS[ask_key("Is the object in the target now? [y/n]: ", SUCCESS_KEYS, input_fn)]
    labels: dict[str, StageStatus] = {}
    for stage in STAGES:
        default = "x" if stage == STAGE_NAVIGATE and mode == MODE_ARM_ONLY else None
        hint = f" [default {default}]" if default else ""
        prompt = f"{stage} (agent claims {stages_claimed.get(stage, STATUS_NOT_REACHED)}) [{STAGE_KEY_HELP}]{hint}: "
        labels[stage] = STAGE_KEYS[ask_key(prompt, STAGE_KEYS, input_fn, default)]
    note = input_fn("Note (optional, Enter for none): ").strip()
    first_failed = next((s for s in STAGES if labels[s] == STATUS_FAIL), None)
    return LabelResult(success=success, labels=labels, first_failed_stage=first_failed, note=note)
