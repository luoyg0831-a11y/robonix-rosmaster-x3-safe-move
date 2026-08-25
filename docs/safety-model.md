# Safety Model

## Execution rules

1. Project code does not publish `/cmd_vel`.
2. `preview_move_options` does not publish a navigation goal.
3. The executable path accepts only an option stored in the active preview
   session; callers cannot supply execution coordinates.
4. Preparation checks AMCL, target cost, the footprint window, and
   `/move_base/make_plan`.
5. Prepared tokens expire, are claimed atomically, and cannot be reused.
6. The CLI displays the target before asking for the exact confirmation phrase.
7. Execution repeats the AMCL, costmap, and plan checks before publishing one
   goal.
8. An uncertain publish result is returned as unknown and is not retried.
9. Success requires a correlated `SUCCEEDED` result, motion evidence, and a
   final idle state.
10. Cancellation addresses the tracked `move_base` goal.

## Stages

| Stage | Work performed | Robot motion |
|---|---|---|
| Preview | Generate candidates and check costmap/path | No |
| Prepare | Recheck the selected candidate and issue a token | No |
| Confirm | Display the target and read the operator response | No |
| Execute | Claim the token, run final checks, publish and monitor one goal | Possible |

## Distance and direction limits

- Preview radii: 0.80 m through 0.10 m, checked from farthest to nearest.
- Preview direction: forward ±30°.
- Result list: at most five candidates from one passing radius.
- Odometry-path and AMCL-displacement argument cap: 1.00 m.
- Stop margin at the 1.00 m cap: 0.10 m; cancellation starts at 0.90 m.
- DWA XY goal tolerance recorded on the robot: 0.025 m.

The preparation endpoint also has a 0.93 m target-distance ceiling. This is not
the preview radius. In the numbered-candidate path, the selected target already
comes from a preview capped at 0.80 m, and preparation is rejected if the robot
has shifted more than 0.03 m since preview.

The current 0.80 m configuration has not completed a supervised live acceptance
run. Changing these limits requires another no-motion check before any live run.

## Emergency stop

The robot has a software hard-stop command and a physical power stop. The
software command terminates `move_base`; the navigation stack must be restarted
before another run. Host-specific commands are kept outside this repository.
