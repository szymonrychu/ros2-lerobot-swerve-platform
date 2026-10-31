# Development flow: vanilla superpowers, with workbench amendments

Superpowers governs (superpowers:using-superpowers), from the user-scope plugin
`superpowers@claude-plugins-official`, enabled in `.claude/settings.json`. Never
fork or edit it. Non-trivial work starts with superpowers:brainstorming, then
superpowers:writing-plans, then superpowers:subagent-driven-development or
superpowers:executing-plans. Reviews go through superpowers:requesting-code-review
and superpowers:receiving-code-review, failures through
superpowers:systematic-debugging, and a finished branch through
superpowers:finishing-a-development-branch. Workbench plugins are helpers that
complement it (research, worktrees, PR tooling, observability, secrets); they
never replace or fork it.

The amendments below override superpowers defaults. Instructions from the user
override both. `<name>` is the improvement's kebab-case name; `<root>` is the
repository root (the main checkout).

1. **Research first.** Before superpowers:brainstorming proposes a design, run
   `workbench:research-docs` (the docs of every tool, library and system the plan
   will use) and `workbench:research-code-internal` (a codebase sweep for reuse
   and the smallest coherent diff). Both always run as this step.
2. **One human gate.** The gate is the brainstorming plan (spec) plus its
   questions. The plan is detailed: what and why, research findings, design,
   files and components touched, test approach, risks, and scope from MVP to
   complete. Every question offers options and marks one "MVP" and one
   "Recommended" (they may be the same). After the answers,
   superpowers:writing-plans writes the implementation plan with no second human
   review, and superpowers:subagent-driven-development starts immediately.
3. **Paths.** Worktrees `<root>/.worktrees/<name>/` (this overrides the location
   choice of superpowers:using-git-worktrees); scratchpad
   `<root>/.scratchpad/<name>/`; logs of every tool the agents run
   `<root>/.logs/<name>/`; plan `docs/plans/<name>/plan.md`; implementation plan
   `docs/plans/<name>/implementation.md`. `.gitignore` ignores `.worktrees/`,
   `.scratchpad/`, `.logs/` and `docs/plans/`.
4. **Retro.** The final reviewer checks correctness and completeness against the
   agreed scope (under- and over-delivery) and writes
   `docs/plans/<name>/retro.md`: what came up during implementation and what
   discovery and planning overlooked. The orchestrator relays its
   recommendations in the closing message.
5. **The orchestrator never updates code** (source, tests, skills, agents,
   hooks, rules, docs). It dispatches subagents instead of editing inline, even
   where a superpowers skill would edit inline.
6. **Helpers.** `git-worktrees-create`, `git-worktrees-commit` and
   `git-worktrees-push` use `.worktrees/<name>/<repo>`; `workbench.py scratchpad
   [<name>]` and `workbench.py logs [<name>]` print the per-improvement dir (no
   argument prints the root).
