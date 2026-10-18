# Tuning sessions

Write one file per tuning session of `affect.yaml`, named `YYYY-MM-DD-<where>.md`, for
example `2026-10-20-sim.md` or `2026-10-21-robot.md`. It records why the shipped values
are what they are. The procedure is in the package README, under "Tuning loop".

Put in each file:

- **Where and what**: sim or robot, the git commit of `r2_personality`, and the scenario
  you drove (for example the acceptance script of plan §8, or which steps of it).
- **Bags**: the name and where it is kept. Bags are not committed. Keep the ones a note
  depends on until the values they justify have been replaced.
- **What looked wrong**: the transition or reaction, at what time in the bag. Paste the
  `replay` output lines if they help.
- **What changed and why**: each `affect.yaml` key, old → new, and the reason. Then what
  `replay <bag> --config new.yaml` showed before you applied it live.
- **Result**: whether the live run after `/mood/reload_config` read right, and what is
  still open.

Commit the note together with the tuned `affect.yaml`.
