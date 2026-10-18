# r2_personality

The personality stack for David's R2-D2 droid: the ROS 2 Jazzy packages that give R2
moods, reactions and expression. The design is `docs/personality-architecture.md` in the
[r2_dev](https://github.com/daveismith/r2_dev) workspace; the build plan for the current
phase is `docs/personality-phase0-plan.md` there.

One repository, several packages, one per layer:

| package | layer | status |
|---|---|---|
| `r2_affect` | 2, affect: the numeric mood model (`/stimuli` → `/mood/state`, `/mood/set`) | phase 0 |
| `r2_stimulus` | 1, perception: the `stimulate` CLI and adapters for existing signals | phase 0 |
| `r2_expression` | 4, per-channel arbiters and renderers | phase 1 |
| `r2_sound` | hardware, clips and synthesis | phase 3 |
| `r2_behavior` | 3, behaviour trees and ambient styling | phase 4 |
| `r2_mocks` | dev, hardware stand-ins for the Mac | later |

The packages are imported into the workspace by `r2_bringup/r2d2.repos` and built with
`make build` from the workspace root. Interfaces live in `astromech_msgs`, never here.
Apache-2.0.
