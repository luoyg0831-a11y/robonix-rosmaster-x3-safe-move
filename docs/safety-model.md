# Safety Model

## Invariants

1. No project component directly publishes `/cmd_vel`.
2. A caller can select only an option from a live server-side preview session.
3. Preview does not create a navigation goal.
4. Preparation revalidates localization, target cost, and planner reachability.
5. Prepared tokens expire, are claimed atomically, and cannot be replayed.
6. The CLI displays the prepared target and requires exact operator confirmation
   before it can enter the independent execution gate.
7. At most one navigation goal publish attempt is made for a claimed token.
8. An unknown publish outcome is returned as unknown and is not retried.
9. Success requires correlated `SUCCEEDED`, motion evidence, and final idle state.
10. Cancellation targets the associated goal rather than broadcasting raw velocity.

## Validation stages

| Stage | Purpose | Motion |
|---|---|---|
| Preview | Generate 1-5 bounded candidates and inspect cost/path | None |
| Prepare | Revalidate selected candidate and issue token | None |
| Confirm | Display target; exact phrase continues, all other input stays closed | None |
| Execute | Claim token, recheck current state, publish once, observe result | Possible |

## Current physical constraints

- Candidate radius in the local candidate: 0.08 m.
- Maximum cumulative odometry path during the charging-cable experiment: 0.10 m.
- Expected DWA goal tolerance from the latest recorded configuration: 0.03 m.
- Direct speed control, long-distance navigation, multiple waypoints, voice, vision, elevator control, and charging automation are out of scope.

These values must not be relaxed without a new safety review and hardware authorization.

## Emergency behavior

The field environment has a software hard-estop script and a physical power
stop. The hard-estop terminates `move_base`, so the navigation stack must be
fully restarted afterward. Host-specific emergency commands remain outside the
public snapshot.
