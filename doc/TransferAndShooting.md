# Transfer loading and shooting sequence

Stage 1 keeps its existing intake/loading behavior. Stage 2 stays at zero power
while collecting, lifting balls into the queue, confirming full, aiming, opening
the gate, and recovering. It runs only forward during gate-open shot feeding.
There is no reverse-loading command or gate-closed stage-2 diagnostic bypass.

The table below describes the shared competition, autonomous, calibration and
transfer-only defaults. **Test: Intake + Index + Shooter** has an explicitly
requested bench-only sequence: stage 2 starts 0.10 s after opening, stage 1 starts
at the midpoint (0.25 s by default), and the gate closes after a 0.40 s stage-2
feed (0.50 s total gate-open command). Its continuous mode immediately opens the
gate and runs both transfers at full power until either trigger is released,
bypassing readiness and timing. See [bench controls](IntakeShooterTest.md).

| Phase | Gate | Stage 1 | Stage 2 | Duration |
| --- | --- | --- | --- | --- |
| Ordinary loading | Closed | Full loading power until full or paused | Off | Until the full sensor is HIGH continuously for 0.5 s |
| Full confirmation / intake reversal | Closed | Continue forward | Off | 0.5 s run-on; pause, feed or shutdown cancels it |
| Confirmed full | Closed | Off | Off | While HIGH remains confirmed; LOW resumes loading |
| Shot readiness | Closed | Paused by the existing shooting controls | Off | Readiness stable for 0.15 s for timed shots |
| Gate command delay | Held open | Off | Off | 0.20 s after the open command |
| Timed shot feed | Held open | Full forward-feed power | Full forward-feed power | 0.50 s |
| Recovery | Closed | Off | Off | 0.35 s before readiness requalifies |

Each fresh HIGH confirmation starts the run-on once while stage 1 is already loading.
LOW immediately clears full and resets the confirmation/run-on timers. Another HIGH
needs a fresh continuous 0.5 seconds. Both bench and competition intake reverse at
confirmation and resume forward on LOW, with RB/LB overriding the sensor. This replaces
the previous full-queue stop/latch policy. Both use the same stage-1 run-on timing.

An uninterrupted timed attempt commands the gate open for about **0.70 seconds**:
0.20 seconds after the open command plus 0.50 seconds of feeding. Both transfer stages
run together for the entire feed window. Continued requests repeat readiness,
gate opening, feed and recovery. Releasing the shot request, feed inhibit, loss of
required readiness, or STOP may end a cycle early. After shooting, ordinary
stage-1 loading resumes according to the OpMode's existing controls; stage 2
remains off.

Competition manual override (GP1 LT or GP2 RT) keeps its hold-to-feed behavior:
the gate first gets the same 0.20-second command-to-feed delay, then both stages feed
continuously while held. GP2 LT closes the gate and stops both transfers with
priority. Manual override bypasses shot readiness. The gate-only test also stays
open while its trigger is held and does not initialize transfer motors.

## Shared settings and affected OpModes

- `GateSubsystem.FEED_DELAY_AFTER_OPEN_COMMAND_SECONDS` owns the command-to-feed delay;
  `ShootingConfiguration.GATE_TO_FEED_DELAY_SECONDS` uses that same value.
- `ShootingConfiguration.FEED_WINDOW_SECONDS` owns the two-stage feed duration.
  Recovery and readiness remain 0.35 s and 0.15 s.
- Both alliance competition TeleOps, both match-autonomous adapters and guided
  shot calibration use the shared settings. The combined intake/index/shooter
  test uses the bench-only overrides described above.
  Calibration requests one attempt; the other timed shooting controls repeat
  while requested. Autonomous scoring remains bounded by the route's scoring
  window and match deadline; check those windows when tuning feed duration.
- `Test: Transfer Only` automatically loads with stage 1 while stage 2 stays off. Hold RB
  to request repeated timed gate-open feeds of both stages; LB pauses loading and
  inhibits feeding with priority. Releasing RB closes the gate, stops stage 2 and
  returns to ordinary stage-1 loading. No shooter/flywheel is initialized or
  powered, and shooter readiness is bypassed for this deliberate mechanism test.
  Start empty when checking directions and gate timing.
- Pollen collection keeps its gate closed and stage 2 off for the whole run.
  Intake-only, turret-only and auto-aim tests do not initialize the transfer.

The team reports immediate gate settling. Software applies the requested fixed
0.20-second command-to-feed delay with no additional settling allowance.
The 0.50-second feed duration remains a starting value for loaded robot testing;
gate arrival has no position feedback.
`FEED_TO_EXIT_SECONDS` remains the separate, unmeasured 0.12-second exit-delay
estimate; it does not stop the motors. `RELEASE_TIMING_MEASURED` remains false.
Confirm ball clearance, exit timing and flywheel recovery on the robot before
declaring release timing measured. Timed attempts do not count launched balls.

## Desktop verification

All 153 unit tests pass with zero failures, errors or skips. Debug packaging and
lint pass; lint reports 0 errors and the existing 14 warnings (12 native-library
alignment warnings and 2 unused resources). No deployment or loaded timing
measurement was performed.

```powershell
.\gradlew.bat :TeamCode:testDebugUnitTest :TeamCode:assembleDebug :TeamCode:lintDebug
```
