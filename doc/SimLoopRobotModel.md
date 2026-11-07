# BioBuzz SimLoop model and measurement provenance

Updated November 2, 2026. Source: `BioBuzzSimHardware`, `BioBuzzSimModel`, production subsystem constants.
This is an SDK/lifecycle and coordination model, not a calibrated digital twin. Configuration values
below are installed-code settings unless explicitly described as physical measurements. Do not tune
real mechanisms from simulated speed, current, distances or shot outcomes.

| Hardware / relationship | Fixture configuration | Evidence / limit |
| --- | --- | --- |
| Drive | leftfront/rightfront/leftback/rightback; physical mounting signs -,+,-,+; 28 ticks/rev, 5550 RPM | Matches current code; mounting signs independent of configured motor directions |
| Chassis dynamics | 14-in track/wheelbase, 2-in radius, 64 effective ticks/in, lateral ratio .85, response 10/s | Synthetic estimates; wheel geometry, friction and acceleration not measured |
| Intake | intake_motor; own shaft encoder on Expansion Hub port 1 | Independent motor/encoder dynamics; bench-only jam recovery has controller tests, no ball-contact model |
| Transfer | transfer_motor_1 and _2; stage 1 REVERSE, stage 2 FORWARD; turret encoder on stage 1's port 2 | Operator-confirmed wiring November 2; turret removes SDK motor-direction sign; never reset shared feedback; motor dynamics synthetic |
| Queue sensor | distance_sensor; digital HIGH, LOW default | Operator or test controls signal; production owns 0.5-second HIGH confirmation, immediate LOW recovery and 0.5-second stage-1 run-on |
| Gate | gate_servo; REVERSE; PWM 500–2500 us, logical .5 closed/.2 open | Asserted physical .5/.8 and 1500/2100 us; .20-s gate delay is code timing, not loaded clearance proof |
| Depollinator | depollinator; 30/355 home, 165/355 deployed; 500–2500 us | Position/PWM commands checked; linkage collision/travel not modeled |
| Turret | turret_servo_1/_2; 355-deg span, 1:1; actual encoder 8192 CPR *108/40 =61.44 ticks/deg | Independent physical yaw approaches PWM-derived target at synthetic 180 deg/s; jam freezes physical yaw |
| Hood | turret_pitch; home .5+16/355; 2:1 servo/hood | Commands bounded by production code; no hood feedback sensor or loaded model |
| Flywheel | turret_motor; goBILDA 5203 Yellow Jacket, 28 ticks/rev, 5400 observed motor RPM at 14 V; 1:1 output ratio | Team observation November 1 at 14 V in PIDF/bang-bang; upstream response rate remains synthetic, no ball impulse or calibrated drag |
| Battery | Control Hub FakeVoltageSensor =14 V | Synthetic constant; not a battery-sag/electrical/thermal model |
| Pinpoint | pinpoint; physical offsets +32,-74 mm, both pod directions REVERSED, 2000/(32*pi) ticks/mm | Chassis motion drives fake SDK I2C device; production config/reset/calibration/fault handling executes |
| Camera | limelight; pivot forward50.68184865mm; camera forward from pivot -99.53073974mm, height335mm, pitch up30deg, yaw0 | Matches documented team mount; fixture translates/rotates the camera with physical turret yaw |
| Camera optics/timing | 54x41-deg FOV, 2–200-in range, 30Hz, 15+20ms latency | Synthetic settings; real pipeline, exposure, intrinsics and latency need verification |
| Raised tags | nominal TU06 audience tags34–41, 3.25in, hive centers +/-12.75in, production-derived row/height, face tilted30deg | Nominal drawing map; not surveyed field coordinates; far/lowered states absent from this layout |

The drive plant integrates in +X forward at heading zero, +Y left, CCW. The fixture deliberately uses
that numeric frame as BioBuzz's field X/Y frame for its own tag layout; it does not import SimLoop's
season field objects or their 90-degree transform. Inches become mm for Pinpoint; camera field/layout
inputs remain inches. Sensor updates use the preceding plant step: the runner ticks fake devices once,
then advances the plant and supplies the next pose. No device is double-stepped.

The fake external encoder is driven from **physical yaw**, never the controller's desired angle. Flywheel
assertions inspect physical shaft velocity. Gate position is commanded position, not measured travel.
Neither estimated homing nor hood tracking should be described as physical feedback.

## Camera convention boundary

SimLoop alpha.2 uses printed face normal **-Z**; current `TagObservation` uses **+Z**. The official legacy
Limelight coordinate guide describes +Z out of the target, while AprilRobotics/OpenCV uses +Z into it.
The raw-camera scenario records rejection. The named normalized scenario installs a test-only
`TargetNormalCameraAdapter` that composes Rx(pi) in target coordinates; translation, geometric visibility,
latency and staleness are unchanged. The adapter exposes only the camera fields consumed by current
robot code; it is not a general replacement for Limelight or proof of a physical-camera fix.

**Pending:** capture a full-3D result from this robot at a known pose, tag tilt and turret angle; compare
translation, Euler angles and outward normal to the raw and normalized fixture. Until then preserve the
production normal convention and label normalized localization results conditional. Do not silently
change the robot to match an alpha simulator, or remove the raw mismatch regression.

Production receipt age now uses SDK `getStaleness()` plus capture/target latency; it does not subtract a
fake device's elapsed timestamp from SimLoop's fixed 2026 wall-clock epoch. Both ordinary epoch-equivalence
and geometric acquisition regressions exercise that boundary.

## To calibrate this model

Record revision, hardware/config changes, battery voltage, measurement method and data files. Measure
drive dimensions/ticks-per-distance and response, turret rate/settling, flywheel loaded response, gate
clearance/feed timing, camera intrinsics/latency and known-pose localization. Replace only corresponding
synthetic parameters, then reproduce independent robot traces. Real shot rows/release timing remain
separate production measurements. A faster synthetic plant or perfect feedback is never a bug fix.
