# Housekeeping rules

The user's standing rules for every agent and every task.

1. **No em dash characters** in any code, comment, commit, message or content of
   any kind. Use a plain hyphen.
2. **No backward compatibility** (shims, aliases, legacy fallbacks) unless the
   user explicitly asks for it.
3. **Tests are not negotiable.** Never edit, delete or xfail a test to make it
   pass, unless the test itself is no longer accurate; then say which test and
   why. Tests make the code repeatable and guard against regressions.
4. **One scratchpad dir per improvement.** Every scratchpad file for an
   improvement lands under `.scratchpad/<name>/` in the repository root (the
   main checkout, never a worktree).
5. **One log dir per improvement.** Every log of an improvement lands under the
   gitignored `.logs/<name>/` in the repository root.
6. **The orchestrator never edits.** It dispatches subagents, reads their
   results, runs the repo's test and lint commands to verify, and git merges and
   pushes reviewed work. It is accountable for all work its subagents do.
