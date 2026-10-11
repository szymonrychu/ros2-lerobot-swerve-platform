"""Plan tree (seq / if / while / phase), the cursor and advance(), answer_condition, loop budgets, revise of a tree, and
the retry policy of complete_phase (p_retry_success, retry_change, automatic re-entry)."""

from typing import Any

import pytest

from claude_agent.budget import PlanTracker
from claude_agent.config import ClaudeAgentConfig
from claude_agent.plan_tools import ANSWER_CONDITION_TOOL, FULL_TOOL_NAMES, build_plan_tools
from claude_agent.plan_tree import PlanError

END = "END"
FAILURE = {
    "failure_type": "execution_failure",
    "eai_error_type": "goal_unsatisfied",
    "explanation": "the jaws closed on nothing",
}


def ph(node_id: str, rw: int = 2, turns: int = 8, **extra: Any) -> dict:
    return {
        "type": "phase",
        "id": node_id,
        "name": node_id.title(),
        "goal": f"{node_id} goal",
        "rw_cap": rw,
        "turn_cap": turns,
        **extra,
    }


def seq(*children: dict, **extra: Any) -> dict:
    return {"type": "seq", "children": list(children), **extra}


def ask(question: str = "is it red?") -> dict:
    return {"ask": question}


def outcome_of(phase_id: str, *values: str) -> dict:
    return {"outcome_of": phase_id, "in": list(values)}


def if_(node_id: str, condition: dict, then: dict, otherwise: dict | None = None) -> dict:
    node = {"type": "if", "id": node_id, "condition": condition, "then": then}
    if otherwise is not None:
        node["else"] = otherwise
    return node


def while_(node_id: str, condition: dict, body: dict, max_iterations: int = 3) -> dict:
    return {"type": "while", "id": node_id, "condition": condition, "max_iterations": max_iterations, "body": body}


class Recorder:
    """Collects activations (phase ids, '?<node id>' for a pending ask) in order."""

    def __init__(self, **config: Any) -> None:
        self.trace: list[str] = []
        self.events: list[tuple[str, dict]] = []
        self.tracker = PlanTracker(
            ClaudeAgentConfig(**config),
            on_phase_started=lambda p: (self.trace.append(p["id"]), self.events.append(("started", p))),
            on_phase_completed=lambda p: self.events.append(("completed", p)),
            on_condition_pending=lambda c: (self.trace.append("?" + c["node_id"]), self.events.append(("pending", c))),
            on_condition_answered=lambda c: self.events.append(("answered", c)),
        )

    def plan(self, tree: dict) -> "Recorder":
        self.tracker.set_plan("moderate", "tree", plan=tree)
        return self


def apply(t: PlanTracker, action: Any) -> None:
    if action == "done":
        t.complete_phase("done", "ok")
    elif action == "fail":
        t.complete_phase("failed", "missed", p_retry_success=0.0, retry_change="nothing", **FAILURE)
    elif action == "retry":
        t.complete_phase("failed", "missed", p_retry_success=0.9, retry_change="move closer", **FAILURE)
    elif action == "skip":
        t.complete_phase("skipped", "not needed")
    else:
        _, node_id, value = action
        t.answer_condition(node_id, value, f"camera shows {value}")


def run_trace(tree: dict, actions: list, **config: Any) -> list[str]:
    rec = Recorder(**config).plan(tree)
    for action in actions:
        apply(rec.tracker, action)
    if rec.tracker.plan_finished:
        rec.trace.append(END)
    return rec.trace


PICK_LOOP = seq(
    ph("look", rw=0),
    while_(
        "pickloop",
        outcome_of("pick", "not_run", "failed"),
        seq(ph("approach"), ph("pick", max_attempts=1)),
        max_iterations=3,
    ),
)
COLOUR_BRANCH = seq(ph("look", rw=0), if_("check", ask(), ph("red"), ph("blue")), ph("final"))
MORE_ITEMS = while_(
    "more",
    ask("is there another item on the table?"),
    seq(ph("pick", max_attempts=1), if_("recover_if", outcome_of("pick", "failed"), ph("recover"))),
    max_iterations=3,
)


@pytest.mark.parametrize(
    ("tree", "actions", "expected"),
    [
        pytest.param(seq(ph("a"), ph("b")), ["done", "fail"], ["a", "b", END], id="legacy-like seq"),
        pytest.param(
            PICK_LOOP,
            ["done", "done", "fail", "done", "done"],
            ["look", "approach", "pick", "approach", "pick", END],
            id="while outcome_of repeats until done",
        ),
        pytest.param(
            PICK_LOOP,
            ["done", "done", "fail", "done", "fail", "done", "fail"],
            ["look", "approach", "pick", "approach", "pick", "approach", "pick", END],
            id="while exits on max_iterations",
        ),
        pytest.param(
            PICK_LOOP, ["done", "done", "done"], ["look", "approach", "pick", END], id="while body once when done"
        ),
        pytest.param(
            COLOUR_BRANCH,
            ["done", ("answer", "check", True), "done", "done"],
            ["look", "?check", "red", "final", END],
            id="if ask then",
        ),
        pytest.param(
            COLOUR_BRANCH,
            ["done", ("answer", "check", False), "done", "done"],
            ["look", "?check", "blue", "final", END],
            id="if ask else",
        ),
        pytest.param(
            seq(ph("look"), if_("check", ask(), ph("red")), ph("final")),
            ["done", ("answer", "check", False), "done"],
            ["look", "?check", "final", END],
            id="if without else skips",
        ),
        pytest.param(
            seq(ph("look"), if_("again", outcome_of("look", "failed"), ph("relook")), ph("final")),
            ["fail", "done", "done"],
            ["look", "relook", "final", END],
            id="if outcome_of taken",
        ),
        pytest.param(
            seq(ph("look"), if_("again", outcome_of("look", "failed"), ph("relook")), ph("final")),
            ["done", "done"],
            ["look", "final", END],
            id="if outcome_of not taken",
        ),
        pytest.param(
            seq(if_("first", outcome_of("later", "not_run"), ph("early")), ph("later")),
            ["done", "done"],
            ["early", "later", END],
            id="not_run before the phase ran",
        ),
        pytest.param(
            MORE_ITEMS,
            [("answer", "more", True), "done", ("answer", "more", True), "fail", "done", ("answer", "more", False)],
            ["?more", "pick", "?more", "pick", "recover", "?more", END],
            id="nested while ask with if outcome_of",
        ),
        pytest.param(
            MORE_ITEMS,
            [("answer", "more", True), "done", ("answer", "more", True), "done", ("answer", "more", True), "done"],
            ["?more", "pick", "?more", "pick", "?more", "pick", END],
            id="ask loop ends at max_iterations without asking again",
        ),
        pytest.param(
            seq(ph("a", max_attempts=3), ph("b")),
            ["retry", "retry", "retry", "done"],
            ["a", "a", "a", "b", END],
            id="retry re-enters the phase until attempts are used up",
        ),
    ],
)
def test_activation_traces(tree: dict, actions: list, expected: list[str]) -> None:
    assert run_trace(tree, actions) == expected


def test_nested_loops_and_iteration_numbers() -> None:
    tree = while_(
        "outer",
        outcome_of("place", "not_run", "failed"),
        seq(
            while_("inner", outcome_of("pick", "not_run", "failed"), ph("pick", max_attempts=1), max_iterations=2),
            ph("place", max_attempts=1),
        ),
        max_iterations=2,
    )
    rec = Recorder().plan(tree)
    for action in ["fail", "fail", "fail", "done", "done"]:
        apply(rec.tracker, action)
    assert rec.trace == ["pick", "pick", "place", "pick", "place"], "the inner loop starts over in each outer iteration"
    assert rec.tracker.plan_finished
    started = [p for kind, p in rec.events if kind == "started"]
    assert [p["iteration"] for p in started] == [1, 2, 1, 1, 2], "iteration of the innermost loop"


def test_ask_pending_blocks_effectors_but_not_sensors() -> None:
    rec = Recorder().plan(COLOUR_BRANCH)
    t = rec.tracker
    t.complete_phase("done", "seen")
    assert t.pending_condition == {
        "node_id": "check",
        "node_type": "if",
        "question": "is it red?",
        "iteration": 0,
    }
    assert t.active is None and not t.plan_finished
    reason = t.charge("effector")
    assert reason is not None and "answer condition 'check' first" in reason
    assert t.charge("sensor") is None, "sensors stay allowed so the model can look before answering"
    with pytest.raises(PlanError, match="answer condition 'check' first"):
        t.complete_phase("done", "x")
    with pytest.raises(PlanError, match="answer condition 'check' first"):
        t.raise_phase_budget(5, None, "x")
    t.answer_condition("check", True, "the tomato is red")
    assert t.pending_condition is None
    assert t.charge("effector") is None


def test_answer_condition_validation_and_events() -> None:
    rec = Recorder().plan(COLOUR_BRANCH)
    t = rec.tracker
    with pytest.raises(PlanError, match="no condition is pending"):
        t.answer_condition("check", True, "x")
    t.complete_phase("done", "seen")
    with pytest.raises(PlanError, match="'check'"):
        t.answer_condition("other", True, "x")
    with pytest.raises(PlanError, match="evidence"):
        t.answer_condition("check", True, "  ")
    with pytest.raises(PlanError, match="true or false"):
        t.answer_condition("check", "yes", "red")
    answered, following = t.answer_condition("check", False, "it is blue")
    assert answered == {
        "node_id": "check",
        "node_type": "if",
        "question": "is it red?",
        "iteration": 0,
        "value": False,
        "evidence": "it is blue",
    }
    assert following is not None and following["id"] == "blue"
    kinds = [kind for kind, _ in rec.events]
    assert kinds.index("pending") < kinds.index("answered") < len(kinds) - 1
    assert ("answered", answered) in rec.events
    with pytest.raises(PlanError, match="set_task_plan"):
        PlanTracker(ClaudeAgentConfig()).answer_condition("x", True, "y")


def test_plan_starting_with_an_ask_sets_it_pending_at_once() -> None:
    rec = Recorder()
    plan, _ = rec.tracker.set_plan("simple", "r", plan=MORE_ITEMS)
    assert rec.trace == ["?more"]
    assert plan["active_phase"] is None
    assert plan["pending_condition"]["node_id"] == "more"
    assert plan["pending_condition"]["iteration"] == 1


def loop_tree(rw: int = 3, max_iterations: int = 5) -> dict:
    return while_("loop", outcome_of("work", "not_run", "failed"), ph("work", rw=rw, max_attempts=1), max_iterations)


def spend(t: PlanTracker, calls: int) -> None:
    for _ in range(calls):
        assert t.charge("effector") is None


def test_loop_exits_with_reason_budget_when_no_rw_room_is_left() -> None:
    rec = Recorder(max_rw_cap=6).plan(loop_tree(rw=3))
    t = rec.tracker
    spend(t, 3)
    t.complete_phase("failed", "missed", p_retry_success=0.0, retry_change="x", **FAILURE)
    assert rec.trace == ["work", "work"]
    spend(t, 3)
    t.complete_phase("failed", "missed", p_retry_success=0.0, retry_change="x", **FAILURE)
    assert t.plan_finished and rec.trace == ["work", "work"]
    assert any("budget" in note for note in t.transition_notes)
    assert t.tree_payload()["exit_reason"] == "budget"


def test_loop_exit_reason_max_iterations_and_condition() -> None:
    rec = Recorder().plan(loop_tree(rw=1, max_iterations=2))
    rec.tracker.complete_phase("failed", "m", p_retry_success=0.0, retry_change="x", **FAILURE)
    rec.tracker.complete_phase("failed", "m", p_retry_success=0.0, retry_change="x", **FAILURE)
    assert rec.tracker.tree_payload()["exit_reason"] == "max_iterations"
    assert any("max_iterations" in note for note in rec.tracker.transition_notes)
    rec = Recorder().plan(loop_tree(rw=1))
    rec.tracker.complete_phase("done", "ok")
    assert rec.tracker.tree_payload()["exit_reason"] == "condition"


def test_reentry_caps_are_fresh_and_clamped_to_the_room_left() -> None:
    rec = Recorder(max_rw_cap=5).plan(loop_tree(rw=3))
    t = rec.tracker
    t.raise_phase_budget(4, None, "need one more")
    spend(t, 3)
    t.complete_phase("failed", "missed", p_retry_success=0.0, retry_change="x", **FAILURE)
    work = t.phase_dicts()[0]
    assert (work["rw_cap"], work["rw_used"], work["raised"], work["iteration"]) == (2, 0, False, 2)
    phase, _ = t.raise_phase_budget(3, None, "the raise is per entry")
    assert phase["raised"] is True


def test_first_entry_after_a_loop_is_clamped_to_the_room_left() -> None:
    tree = seq(
        while_("loop", outcome_of("a", "not_run", "failed"), ph("a", rw=40, max_attempts=1), max_iterations=5),
        ph("b", rw=40),
    )
    rec = Recorder(max_rw_cap=150).plan(tree)
    t = rec.tracker
    while t.phases[t.active].id == "a":
        while t.charge("effector") is None:
            pass
        t.complete_phase("failed", "missed", p_retry_success=0.0, retry_change="x", **FAILURE)
    assert rec.trace[-1] == "b"
    b = t.phase_dicts()[-1]
    assert b["rw_cap"] == 150 - t.rw_used, "a first entry gets at most the room left, not its planned cap"
    while t.charge("effector") is None:
        pass
    assert t.rw_used <= 150, "the instruction maximum stays the hard backstop"


def test_effector_refused_once_the_instruction_rw_maximum_is_used() -> None:
    rec = Recorder(max_rw_cap=3).plan(seq(ph("a", rw=3)))
    t = rec.tracker
    spend(t, 2)
    t.phases[t.active].rw_cap = 10
    assert t.charge("effector") is None
    reason = t.charge("effector")
    assert reason is not None and "instruction" in reason and "3/3" in reason
    assert t.rw_used == 3


def test_sensing_only_phase_loops_without_rw_room() -> None:
    tree = while_("loop", outcome_of("look", "not_run", "failed"), ph("look", rw=0, max_attempts=1), 3)
    rec = Recorder(max_rw_cap=1).plan(seq(ph("act", rw=1), tree))
    spend(rec.tracker, 1)
    rec.tracker.complete_phase("done", "ok")
    rec.tracker.complete_phase("failed", "m", p_retry_success=0.0, retry_change="x", **FAILURE)
    assert rec.trace == ["act", "look", "look"], "rw room 0 does not stop a phase with rw_cap 0"


def test_payload_keeps_flat_preorder_phases_and_adds_tree_fields() -> None:
    rec = Recorder().plan(PICK_LOOP)
    plan = rec.tracker.plan
    assert plan is not None
    assert [(p["index"], p["id"], p["depth"]) for p in plan["phases"]] == [
        (0, "look", 2),
        (1, "approach", 4),
        (2, "pick", 4),
    ]
    first = plan["phases"][0]
    assert (first["attempt"], first["max_attempts"], first["iteration"], first["p_retry_success"]) == (1, 2, 0, None)
    assert plan["phases"][1]["status"] == "pending" and plan["phases"][1]["attempt"] == 0
    tree = plan["tree"]
    assert tree["type"] == "seq" and tree["children"][0] == {"type": "phase", "id": "look", "index": 0}
    loop = tree["children"][1]
    assert loop["type"] == "while" and loop["id"] == "pickloop" and loop["max_iterations"] == 3
    assert loop["condition"] == {"outcome_of": "pick", "in": ["not_run", "failed"]}
    assert loop["body"]["children"][1] == {"type": "phase", "id": "pick", "index": 2}
    assert plan["pending_condition"] is None


def test_legacy_phases_become_a_seq_with_generated_ids() -> None:
    t = PlanTracker(ClaudeAgentConfig())
    plan, _ = t.set_plan(
        "simple", "r", [ph("x") | {"type": "ignored"}, {"name": "B", "goal": "g", "rw_cap": 1, "turn_cap": 2}]
    )
    assert plan["tree"]["type"] == "seq"
    assert [p["id"] for p in plan["phases"]] == ["p1", "p2"]
    assert [p["depth"] for p in plan["phases"]] == [2, 2]


def test_plan_needs_exactly_one_of_phases_and_plan() -> None:
    t = PlanTracker(ClaudeAgentConfig())
    with pytest.raises(PlanError, match="phases or plan"):
        t.set_plan("simple", "r")
    with pytest.raises(PlanError, match="phases or plan"):
        t.set_plan("simple", "r", [ph("a")], plan=seq(ph("b")))


def deep(levels: int) -> dict:
    node: dict = ph("leaf")
    for level in range(levels - 1):
        node = seq(node, id=f"s{level}")
    return node


@pytest.mark.parametrize(
    ("tree", "match"),
    [
        pytest.param(deep(5), "depth", id="depth 5"),
        pytest.param(seq(ph("a"), ph("a")), "unique", id="duplicate ids"),
        pytest.param(seq(ph("a"), if_("a", ask(), ph("b"))), "unique", id="duplicate id across types"),
        pytest.param(
            seq(*[if_(f"i{n}", ask(), ph(f"p{n}", rw=0, turns=1)) for n in range(12)]), "24 nodes", id="25 nodes"
        ),
        pytest.param(
            seq(
                seq(*[ph(f"a{n}", rw=0, turns=1) for n in range(7)]),
                seq(*[ph(f"b{n}", rw=0, turns=1) for n in range(6)]),
            ),
            "12 phase",
            id="13 phases",
        ),
        pytest.param(while_("w", ask(), ph("a"), max_iterations=6), "max_iterations", id="max_iterations 6"),
        pytest.param(while_("w", ask(), ph("a"), max_iterations=0), "max_iterations", id="max_iterations 0"),
        pytest.param(seq(ph("a", max_attempts=0)), "max_attempts", id="max_attempts 0"),
        pytest.param(seq(ph("a", max_attempts=6)), "max_attempts", id="max_attempts 6"),
        pytest.param({"type": "loop", "id": "x"}, "type", id="unknown type"),
        pytest.param(seq({"name": "n", "goal": "g", "rw_cap": 1, "turn_cap": 1, "type": "phase"}), "id", id="no id"),
        pytest.param({"type": "if", "id": "i", "condition": ask()}, "then", id="if without then"),
        pytest.param({"type": "while", "id": "w", "condition": ask(), "max_iterations": 2}, "body", id="no body"),
        pytest.param(seq(ph("a"), if_("i", outcome_of("ghost", "done"), ph("b"))), "ghost", id="unknown phase"),
        pytest.param(seq(ph("a"), if_("i", outcome_of("a", "maybe"), ph("b"))), "in", id="bad outcome value"),
        pytest.param(seq(ph("a"), if_("i", outcome_of("a"), ph("b"))), "in", id="empty outcome list"),
        pytest.param(seq(ph("a"), if_("i", {"other": 1}, ph("b"))), "condition", id="bad condition"),
        pytest.param(seq(ph("a"), if_("i", {"ask": " "}, ph("b"))), "ask", id="empty question"),
        pytest.param(seq(ph("a"), if_("i", outcome_of("s", "done"), ph("b")), id="s"), "phase", id="outcome of seq"),
        pytest.param(seq(), "1 to 12", id="empty seq"),
        pytest.param(seq(*[ph(f"a{n}", rw=0, turns=1) for n in range(13)]), "1 to 12", id="13 children"),
        pytest.param(seq(ph("a", rw=-1)), "rw_cap", id="bad cap"),
        pytest.param("not a node", "object", id="not an object"),
    ],
)
def test_tree_validation_errors(tree: Any, match: str) -> None:
    t = PlanTracker(ClaudeAgentConfig())
    with pytest.raises(PlanError, match=match):
        t.set_plan("moderate", "r", plan=tree)
    assert t.plan is None


def test_depth_4_and_24_nodes_and_12_phases_are_allowed() -> None:
    PlanTracker(ClaudeAgentConfig()).set_plan("moderate", "r", plan=deep(4))
    many = seq(*[if_(f"i{n}", ask(), ph(f"p{n}", rw=0, turns=1)) for n in range(11)], ph("last", rw=0, turns=1))
    plan, _ = PlanTracker(ClaudeAgentConfig()).set_plan("moderate", "r", plan=many)
    assert len(plan["phases"]) == 12


def test_max_iterations_limit_follows_config() -> None:
    t = PlanTracker(ClaudeAgentConfig(max_loop_iterations=2))
    with pytest.raises(PlanError, match="max_iterations"):
        t.set_plan("moderate", "r", plan=while_("w", ask(), ph("a"), max_iterations=3))


def test_plan_time_sum_counts_every_phase_once_with_both_branches() -> None:
    tree = seq(if_("i", ask(), ph("a", rw=4), ph("b", rw=4)), while_("w", ask(), ph("c", rw=4), max_iterations=5))
    with pytest.raises(PlanError, match=r"rw_cap.*12.*11"):
        PlanTracker(ClaudeAgentConfig(max_rw_cap=11)).set_plan("moderate", "r", plan=tree)
    PlanTracker(ClaudeAgentConfig(max_rw_cap=12)).set_plan("moderate", "r", plan=tree)


def test_revise_replaces_a_tree_mid_loop() -> None:
    rec = Recorder().plan(PICK_LOOP)
    t = rec.tracker
    for action in ["done", "done", "fail", "done"]:
        apply(t, action)
    assert rec.trace == ["look", "approach", "pick", "approach", "pick"]
    plan, _ = t.revise_plan(
        "picking from this side cannot work",
        plan=seq(ph("reposition"), if_("again", outcome_of("pick", "failed"), ph("pick2"))),
    )
    assert plan["revised"] is True
    assert [(p["id"], p["status"]) for p in plan["phases"]] == [
        ("look", "done"),
        ("approach", "done"),
        ("pick", "failed"),
        ("reposition", "active"),
        ("pick2", "pending"),
    ]
    assert [p["index"] for p in plan["phases"]] == [0, 1, 2, 3, 4]
    assert plan["tree"]["children"][0] == {"type": "phase", "id": "reposition", "index": 3}
    t.complete_phase("done", "moved")
    assert rec.trace[-2:] == ["reposition", "pick2"], "outcome_of may name a closed phase of the old tree"
    t.complete_phase("done", "picked")
    assert t.plan_finished
    with pytest.raises(PlanError, match="already revised"):
        t.revise_plan("again", plan=seq(ph("z")))


def test_revise_rejects_ids_of_kept_phases_and_clears_a_pending_ask() -> None:
    rec = Recorder().plan(COLOUR_BRANCH)
    t = rec.tracker
    t.complete_phase("done", "seen")
    assert t.pending_condition is not None
    with pytest.raises(PlanError, match="unique"):
        t.revise_plan("r", plan=seq(ph("look")))
    plan, _ = t.revise_plan("r", phases=[{"name": "N", "goal": "g", "rw_cap": 1, "turn_cap": 2}])
    assert t.pending_condition is None and plan["pending_condition"] is None
    assert [p["id"] for p in plan["phases"]] == ["look", "p2"]
    assert plan["phases"][1]["status"] == "active"


# ---- retry policy (U4) ----


def test_failed_needs_p_retry_success_and_retry_change() -> None:
    t = PlanTracker(ClaudeAgentConfig())
    t.set_plan("simple", "r", plan=seq(ph("a"), ph("b")))
    with pytest.raises(PlanError, match="p_retry_success"):
        t.complete_phase("failed", "missed", **FAILURE)
    with pytest.raises(PlanError, match="retry_change"):
        t.complete_phase("failed", "missed", p_retry_success=0.2, **FAILURE)
    with pytest.raises(PlanError, match="retry_change"):
        t.complete_phase("failed", "missed", p_retry_success=0.2, retry_change=" ", **FAILURE)
    for bad in (-0.1, 1.5, "0.5", True):
        with pytest.raises(PlanError, match="p_retry_success"):
            t.complete_phase("failed", "missed", p_retry_success=bad, retry_change="x", **FAILURE)
    assert t.active == 0
    closed, _ = t.complete_phase("failed", "missed", p_retry_success=0, retry_change="nothing would help", **FAILURE)
    assert closed["p_retry_success"] == 0.0 and closed["retry_change"] == "nothing would help"


def test_done_ignores_retry_fields_but_records_them() -> None:
    t = PlanTracker(ClaudeAgentConfig())
    t.set_plan("simple", "r", plan=seq(ph("a")))
    closed, nxt = t.complete_phase("done", "ok", p_retry_success=0.9, retry_change="n/a")
    assert nxt is None and closed["status"] == "done" and t.auto_retries == 0


@pytest.mark.parametrize(("p", "retried"), [(0.49, False), (0.5, True)])
def test_threshold_boundary(p: float, retried: bool) -> None:
    rec = Recorder().plan(seq(ph("a"), ph("b")))
    rec.tracker.complete_phase("failed", "missed", p_retry_success=p, retry_change="move closer", **FAILURE)
    assert rec.trace == (["a", "a"] if retried else ["a", "b"])
    assert rec.tracker.auto_retries == (1 if retried else 0)


def test_retry_entry_is_announced_with_attempt_and_fresh_caps() -> None:
    rec = Recorder().plan(seq(ph("a", rw=2, max_attempts=3), ph("b")))
    t = rec.tracker
    t.raise_phase_budget(4, None, "more")
    spend(t, 2)
    closed, following = t.complete_phase("failed", "slipped", p_retry_success=0.7, retry_change="grip lower", **FAILURE)
    assert closed["attempt"] == 1 and closed["status"] == "failed"
    assert following is not None and following["id"] == "a" and following["attempt"] == 2
    assert (following["rw_cap"], following["rw_used"], following["raised"], following["status"]) == (
        2,
        0,
        False,
        "active",
    )
    assert following["p_retry_success"] is None
    assert any("Automatic retry 2/3 - apply: grip lower" in note for note in t.transition_notes)
    completed = [p for kind, p in rec.events if kind == "completed"]
    assert completed[-1]["p_retry_success"] == 0.7 and completed[-1]["attempt"] == 1


def test_attempts_exhausted_flows_on() -> None:
    rec = Recorder().plan(seq(ph("a"), ph("b")))
    for _ in range(2):
        rec.tracker.complete_phase("failed", "missed", p_retry_success=0.9, retry_change="x", **FAILURE)
    assert rec.trace == ["a", "a", "b"]
    assert rec.tracker.auto_retries == 1


def test_retry_cap_of_6_across_two_phases() -> None:
    rec = Recorder().plan(seq(ph("a", rw=1, turns=1, max_attempts=5), ph("b", rw=1, turns=1, max_attempts=5)))
    t = rec.tracker
    for _ in range(5):
        t.complete_phase("failed", "missed", p_retry_success=0.9, retry_change="x", **FAILURE)
    assert rec.trace == ["a"] * 5 + ["b"] and t.auto_retries == 4
    for _ in range(3):
        t.complete_phase("failed", "missed", p_retry_success=0.9, retry_change="x", **FAILURE)
    assert rec.trace == ["a"] * 5 + ["b"] * 3 and t.auto_retries == 6
    assert t.plan_finished
    assert any("6" in note and "retr" in note for note in t.transition_notes)


def test_retry_cap_follows_config() -> None:
    rec = Recorder(max_auto_retries=0).plan(seq(ph("a")))
    rec.tracker.complete_phase("failed", "missed", p_retry_success=1.0, retry_change="x", **FAILURE)
    assert rec.trace == ["a"] and rec.tracker.plan_finished


def test_threshold_follows_config() -> None:
    rec = Recorder(retry_success_threshold=0.8).plan(seq(ph("a")))
    rec.tracker.complete_phase("failed", "missed", p_retry_success=0.7, retry_change="x", **FAILURE)
    assert rec.trace == ["a"]


def test_no_retry_when_budget_room_is_0() -> None:
    rec = Recorder(max_rw_cap=3).plan(seq(ph("a", rw=3)))
    t = rec.tracker
    spend(t, 3)
    t.complete_phase("failed", "missed", p_retry_success=0.9, retry_change="x", **FAILURE)
    assert rec.trace == ["a"] and t.plan_finished and t.auto_retries == 0
    assert any("budget" in note for note in t.transition_notes)


def test_no_retry_when_turn_room_is_0() -> None:
    rec = Recorder(max_turn_cap=2).plan(seq(ph("a", rw=0, turns=2)))
    t = rec.tracker
    t.count_turn()
    t.count_turn()
    t.complete_phase("failed", "missed", p_retry_success=0.9, retry_change="x", **FAILURE)
    assert rec.trace == ["a"] and t.plan_finished


def test_failed_phase_inside_while_retries_before_the_loop_retests() -> None:
    tree = while_("loop", outcome_of("pick", "not_run", "failed"), ph("pick", max_attempts=2), max_iterations=2)
    assert run_trace(tree, ["retry", "fail", "done"]) == ["pick", "pick", "pick", END]


def test_revise_close_needs_no_retry_fields() -> None:
    t = PlanTracker(ClaudeAgentConfig())
    t.set_plan("simple", "r", plan=seq(ph("a")))
    plan, _ = t.revise_plan("other way", plan=seq(ph("b")))
    assert plan["phases"][0]["status"] == "failed" and plan["phases"][0]["p_retry_success"] is None


def test_reset_clears_the_tree_state() -> None:
    rec = Recorder().plan(COLOUR_BRANCH)
    rec.tracker.complete_phase("done", "x")
    rec.tracker.reset()
    assert rec.tracker.plan is None and rec.tracker.pending_condition is None and rec.tracker.auto_retries == 0


# ---- tools ----


def handlers(t: PlanTracker) -> dict:
    return {tool.name: tool.handler for tool in build_plan_tools(t)}


def test_tool_names_and_schemas() -> None:
    assert ANSWER_CONDITION_TOOL == "answer_condition"
    assert "mcp__agent__answer_condition" in FULL_TOOL_NAMES
    tools = {tool.name: tool for tool in build_plan_tools(PlanTracker(ClaudeAgentConfig()))}
    assert set(tools) == {"set_task_plan", "complete_phase", "revise_plan", "raise_phase_budget", "answer_condition"}
    plan_schema = tools["set_task_plan"].input_schema
    assert plan_schema["required"] == ["complexity", "rationale"]
    assert {"phases", "plan"} <= set(plan_schema["properties"])
    assert tools["revise_plan"].input_schema["required"] == ["rationale"]
    complete = tools["complete_phase"].input_schema["properties"]
    assert complete["p_retry_success"]["minimum"] == 0 and complete["p_retry_success"]["maximum"] == 1
    assert "retry_change" in complete
    answer = tools["answer_condition"].input_schema
    assert answer["required"] == ["node_id", "value", "evidence"]
    assert answer["properties"]["value"]["type"] == "boolean"


async def test_set_task_plan_handler_accepts_a_tree() -> None:
    t = PlanTracker(ClaudeAgentConfig())
    res = await handlers(t)["set_task_plan"]({"complexity": "moderate", "rationale": "loop", "plan": PICK_LOOP})
    assert not res.get("is_error")
    text = res["content"][0]["text"]
    assert "pickloop" in text and "Look" in text and "while" in text


async def test_complete_phase_handler_reports_automatic_retry() -> None:
    t = PlanTracker(ClaudeAgentConfig())
    t.set_plan("simple", "r", plan=seq(ph("a")))
    h = handlers(t)
    bad = await h["complete_phase"]({"outcome": "failed", "summary": "missed", **FAILURE})
    assert bad["is_error"] is True and "p_retry_success" in bad["content"][0]["text"]
    res = await h["complete_phase"](
        {
            "outcome": "failed",
            "summary": "missed",
            "p_retry_success": 0.6,
            "retry_change": "approach from the left",
            **FAILURE,
        }
    )
    assert "Automatic retry 2/2 - apply: approach from the left" in res["content"][0]["text"]


async def test_complete_phase_handler_names_the_pending_question() -> None:
    t = PlanTracker(ClaudeAgentConfig())
    t.set_plan("simple", "r", plan=COLOUR_BRANCH)
    res = await handlers(t)["complete_phase"]({"outcome": "done", "summary": "seen"})
    text = res["content"][0]["text"]
    assert "answer_condition" in text and "check" in text and "is it red?" in text


async def test_answer_condition_handler() -> None:
    t = PlanTracker(ClaudeAgentConfig())
    t.set_plan("simple", "r", plan=COLOUR_BRANCH)
    h = handlers(t)
    err = await h["answer_condition"]({"node_id": "check", "value": True, "evidence": "x"})
    assert err["is_error"] is True
    t.complete_phase("done", "seen")
    res = await h["answer_condition"]({"node_id": "check", "value": True, "evidence": "red in the image"})
    assert not res.get("is_error")
    text = res["content"][0]["text"]
    assert "'check'" in text and "Red" in text and "active" in text
    t.complete_phase("done", "red")
    res = await handlers(t)["complete_phase"]({"outcome": "done", "summary": "final"})
    assert "plan is finished" in res["content"][0]["text"].lower()


async def test_revise_plan_handler_accepts_a_tree() -> None:
    t = PlanTracker(ClaudeAgentConfig())
    t.set_plan("simple", "r", plan=seq(ph("a")))
    res = await handlers(t)["revise_plan"]({"rationale": "new", "plan": seq(ph("b"))})
    assert not res.get("is_error") and "B" in res["content"][0]["text"]


# --- budget room outside phases (M23), loop re-entry text (M24), raise notes (L14), no-retry text (L15) -------------


def test_room_counts_turns_outside_phases() -> None:
    t = PlanTracker(ClaudeAgentConfig(max_turn_cap=20))
    for _ in range(9):
        t.count_turn()
    t.set_plan("simple", "r", plan=seq(ph("a", turns=5)))
    assert t.room()["turn_cap"] == 11, "9 turns were used before the plan existed"
    with pytest.raises(PlanError, match="only 11 remain"):
        t.revise_plan("more", plan=seq(ph("b", turns=12)))
    t.revise_plan("fits", plan=seq(ph("c", turns=11)))


def test_room_counts_rw_calls_outside_phases() -> None:
    t = PlanTracker(ClaudeAgentConfig(max_rw_cap=10))
    t.set_plan("simple", "r", plan=seq(ph("a", rw=2)))
    t.complete_phase("done", "ok")
    t.charge_release()
    t.charge_release()
    assert t.rw_used == 2 and t.room()["rw_cap"] == 8


async def test_loop_reentry_text_names_the_iteration_not_a_retry() -> None:
    t = PlanTracker(ClaudeAgentConfig())
    t.set_plan(
        "simple", "r", plan=while_("pickloop", outcome_of("pick", "not_run", "failed"), ph("pick", max_attempts=1))
    )
    res = await handlers(t)["complete_phase"](
        {"outcome": "failed", "summary": "missed", "p_retry_success": 0.2, "retry_change": "closer", **FAILURE}
    )
    text = res["content"][0]["text"]
    assert "No automatic retry: p_retry_success 0.2 is below the threshold 0.5" in text
    assert "Loop 'pickloop' iteration 2/3" in text
    assert "is now active" in text and "active again" not in text
    assert "loop iteration 2" in text
    assert t.retried is False


async def test_automatic_retry_text_says_active_again() -> None:
    t = PlanTracker(ClaudeAgentConfig())
    t.set_plan("simple", "r", plan=seq(ph("a")))
    res = await handlers(t)["complete_phase"](
        {"outcome": "failed", "summary": "missed", "p_retry_success": 0.9, "retry_change": "closer", **FAILURE}
    )
    text = res["content"][0]["text"]
    assert "Automatic retry 2/2" in text and "is active again" in text and "Loop" not in text
    assert t.retried is True


def test_raise_note_says_the_cap_stays_when_no_room_is_left() -> None:
    t = PlanTracker(ClaudeAgentConfig(max_turn_cap=20))
    for _ in range(9):
        t.count_turn()
    t.set_plan("simple", "r", plan=seq(ph("a", turns=8), ph("b", turns=3)))
    phase, notes = t.raise_phase_budget(None, 10, "need more")
    assert phase["turn_cap"] == 8
    assert notes == ["turn_cap stays at 8: nothing is left of the instruction maximum 20 beyond the current cap"]


def test_raise_room_ignores_phases_in_branches_not_taken() -> None:
    tree = seq(if_("check", ask(), ph("b", rw=4), ph("c", rw=4)), ph("a", rw=2))
    t = PlanTracker(ClaudeAgentConfig(max_rw_cap=10))
    t.set_plan("simple", "r", plan=tree)
    t.answer_condition("check", True, "red")
    phase, notes = t.raise_phase_budget(8, None, "need more")
    assert phase["rw_cap"] == 8 and notes == [], "c can no longer run, so its cap is not reserved"


async def test_no_retry_for_budget_does_not_say_report_and_continue() -> None:
    t = PlanTracker(ClaudeAgentConfig(max_rw_cap=4))
    t.set_plan("simple", "r", plan=seq(ph("a", rw=4), ph("b", rw=0)))
    spend(t, 4)
    res = await handlers(t)["complete_phase"](
        {"outcome": "failed", "summary": "missed", "p_retry_success": 0.9, "retry_change": "closer", **FAILURE}
    )
    text = res["content"][0]["text"]
    assert "No automatic retry: the instruction budget has no room left (rw 0" in text
    assert "report to the user" not in text
    assert "Phase 2 'B'" in text and "is now active" in text


async def test_a_phase_entered_without_effector_room_says_so() -> None:
    tree = seq(while_("loop", outcome_of("a", "not_run", "failed"), ph("a", rw=3, max_attempts=1), 2), ph("b", rw=3))
    t = PlanTracker(ClaudeAgentConfig(max_rw_cap=6))
    t.set_plan("simple", "r", plan=tree)
    h = handlers(t)
    spend(t, 3)
    await h["complete_phase"](
        {"outcome": "failed", "summary": "m", "p_retry_success": 0.0, "retry_change": "x", **FAILURE}
    )
    spend(t, 3)
    res = await h["complete_phase"](
        {"outcome": "failed", "summary": "m", "p_retry_success": 0.0, "retry_change": "x", **FAILURE}
    )
    text = res["content"][0]["text"]
    assert "Phase 2 'B' gets rw_cap 0" in text and "no effector calls are left" in text


def test_complete_phase_description_states_the_retry_threshold() -> None:
    tools = {tool.name: tool for tool in build_plan_tools(PlanTracker(ClaudeAgentConfig(retry_success_threshold=0.7)))}
    assert "retry threshold (0.7)" in tools["complete_phase"].description
