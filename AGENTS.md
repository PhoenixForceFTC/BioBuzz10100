# AGENTS.md — FTC Robot Code Best Practices & Architecture Guide

This document defines the software engineering standards, FTC SDK conventions, and modular subsystem architecture for **BioBuzz 10100** (`V1` branch). Both human contributors and AI coding agents should follow these practices whenever adding or modifying code in `TeamCode`.

---

## 1. Core Architecture: Subsystems, Test OpModes, and Combined OpModes

A common beginner mistake in FTC is putting every motor, servo, sensor, and button check into a single giant OpMode file—or writing separate OpMode files for each mechanism and copy-pasting code between them when building the competition TeleOp or Autonomous.

Instead, we split the robot into **three clean layers**:

### Layer 1: Subsystem Classes (Plain Java Classes — No `@TeleOp` or `@Autonomous`)
Each physical mechanism on the robot gets its own **Subsystem class** (for example: `MecanumDrivetrain`, `PinpointOdometry`, `Intake`, `Shooter`, `Lift`).
- **What it is**: A regular Java class (does **not** extend `LinearOpMode` or `OpMode`).
- **What it owns**: Only the hardware devices (`DcMotorEx`, `Servo`, sensors) and constants for that single mechanism.
- **What it exposes**: Simple, high-level methods like `init(HardwareMap)`, `drive(axial, lateral, yaw)`, `setIntakePower(power)`, `update()`, and `addTelemetry(Telemetry)`.
- **Rule**: A subsystem should **never** contain `waitForStart()`, `while (opModeIsActive())`, or `sleep()`. That way, the exact same subsystem class works in both TeleOp and Autonomous.

### Layer 2: Standalone Mechanism Test OpModes (`@TeleOp(group = "Test")`)
Every subsystem should have its own small, dedicated **Test OpMode** file (for example: `DrivetrainTestOpMode.java`, `IntakeTestOpMode.java`, `PinpointTestOpMode.java`).
- **Why**: If the intake is broken or the shooter isn't wired yet, you can still test the drivetrain by itself on the Driver Station without crashing on missing hardware or risking another mechanism moving unexpectedly.
- **How**: The Test OpMode simply creates **one** subsystem instance (`MecanumDrivetrain drive = new MecanumDrivetrain(hardwareMap);`) and maps a gamepad to test it in isolation.

### Layer 3: Combined Competition OpModes (`@TeleOp` and `@Autonomous`)
The main match OpModes (`MecanumTeleOp.java`, `MainAutonomous.java`) combine all the subsystems together in one main loop.
- **Why**: Because the hardware logic lives inside the Subsystem classes, the main OpMode is short, easy to read, and just coordinates gamepad inputs and autonomous steps.

#### Example: How Subsystems + Test OpModes + Combined OpModes Work Together

```java
// 1. SUBSYSTEM CLASS (IntakeSubsystem.java) — Plain Java class, reusable everywhere
public class IntakeSubsystem {
    private final DcMotorEx motor;

    public IntakeSubsystem(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, "intake");
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void in()  { motor.setPower(1.0); }
    public void out() { motor.setPower(-1.0); }
    public void stop(){ motor.setPower(0.0); }
}

// 2. STANDALONE TEST OPMODE (IntakeTestOpMode.java) — Tests ONLY the intake on the Driver Station
@TeleOp(name = "Test: Intake Only", group = "Test")
public class IntakeTestOpMode extends LinearOpMode {
    @Override
    public void runOpMode() {
        IntakeSubsystem intake = new IntakeSubsystem(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.right_bumper) intake.in();
            else if (gamepad1.left_bumper) intake.out();
            else intake.stop();
        }
    }
}

// 3. COMBINED COMPETITION OPMODE (CompetitionTeleOp.java) — Combines all subsystems
@TeleOp(name = "Competition TeleOp", group = "Competition")
public class CompetitionTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() {
        MecanumDrivetrain drive = new MecanumDrivetrain(hardwareMap);
        PinpointOdometry odometry = new PinpointOdometry(hardwareMap);
        IntakeSubsystem intake = new IntakeSubsystem(hardwareMap);

        waitForStart();
        while (opModeIsActive()) {
            odometry.update();
            drive.driveRobotCentric(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

            if (gamepad1.right_trigger > 0.1) intake.in();
            else if (gamepad1.left_trigger > 0.1) intake.out();
            else intake.stop();

            odometry.addTelemetry(telemetry);
            drive.addTelemetry(telemetry);
            telemetry.update(); // Call telemetry.update() ONCE per loop at the very end
        }
    }
}
```

---

## 2. FTC Hardware & Loop Performance Best Practices

### Initialize All Hardware Before `waitForStart()`
- Retrieve every `DcMotorEx`, `Servo`, and I2C sensor (`GoBildaPinpointDriver`, `IMU`, etc.) from `hardwareMap` during the `INIT` phase (before `waitForStart()`).
- **Never** call `hardwareMap.get(...)` inside the `while (opModeIsActive())` loop—initializing I2C devices mid-loop blocks the thread and causes robot lag.
- Perform IMU calibration (`pinpoint.resetPosAndIMU()`) during `INIT` while the robot is completely stationary on the field.

### Keep Loop Times Fast (Non-Blocking Code)
- Control loop frequency directly impacts odometry accuracy, PID stability, and driver responsiveness. Aim for **50–100+ Hz** loop rates.
- **No `sleep()` in TeleOp**: Never use `sleep()` or nested `while` loops inside a TeleOp control loop. If an action takes time (like moving a servo, waiting 300 ms, then spinning a motor), use an `ElapsedTime` timer and a state variable (`enum State { IDLE, GRABBING, LIFTING }`) so the drivetrain never freezes while a mechanism moves.
- **Minimize I2C Reads**: Call `pinpoint.update()` **once** per loop iteration at the top of the loop, then read cached values (`getPosition()`, `getVelX()`, `getEncoderX()`, etc.) from that update.

### Motor & Encoder Configuration
- Use `DcMotorEx` instead of `DcMotor` to access velocity (`getVelocity()`, `setVelocity()`), PIDF coefficients, and current draw.
- When overriding motor specs (`ticksPerRev`, `maxRPM`, `gearing`), always **clone** the motor configuration first:
  ```java
  MotorConfigurationType motorType = motor.getMotorType().clone();
  motorType.setTicksPerRev(28.0);
  motorType.setMaxRPM(6000.0);
  motor.setMotorType(motorType);
  ```
  Mutating `motor.getMotorType()` directly without `.clone()` alters the shared SDK singleton for all motors of that type.
- Explicitly set `motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)` on drivetrain and lift motors, and `FLOAT` on high-inertia flywheels.

### Gamepad Input & Edge Detection
- **Y-Axis Sign**: Pushing a joystick forward produces a **negative** `left_stick_y` value on FTC gamepads. Always negate it (`double axial = -gamepad1.left_stick_y;`).
- **Button Edge Detection**: For toggle actions (e.g., resetting odometry pose or toggling slow mode), detect rising edges (`currentButton && !previousButton` or SDK methods like `gamepad1.aWasPressed()`) so holding the button for 10 loops doesn't trigger the action 10 times.

### Telemetry Hygiene
- Call `telemetry.update()` **only once** at the very end of the main OpMode loop. Subsystems should only call `telemetry.addData(...)` and let the main OpMode flush the telemetry packet.
- Report units explicitly in telemetry labels (`mm`, `in`, `deg`, `ticks/s`) so drivers and pit crew can diagnose issues quickly.

### Constants & Units
- Name every physical constant with its unit (`PINPOINT_X_POD_OFFSET_MM`, `BARE_MOTOR_TICKS_PER_REV`, `BARE_MOTOR_MAX_RPM`).
- Keep hardware map names, pod offsets, and gear ratios in `public static final` constants at the top of the class (or in a shared `RobotConstants` file) rather than scattering magic numbers through the code.

---

## 3. Build & Verification Workflow

Before committing or deploying code to the Control Hub, verify that `TeamCode` compiles and packages cleanly from the repository root:

```bash
./gradlew.bat :TeamCode:assembleDebug
```

### Optional TeleOp simulation

SimLoop is optional and intended for major feature additions or changes. Minor edits and routine tuning
do not require it; use proportionate existing tests and physical measurements. For a major change,
consider an observable scenario against the affected real OpModes and both competition TeleOps when
shared behavior is involved. If simulation is used, identify the baseline, reproduce the issue where
applicable, assert the intended behavior, and report results and model limits. Never relax assertions,
substitute perfect feedback, or change calibration flags to hide a failure. Do not claim unsupported or
unrun behavior was verified. Preserve existing build requirements and working robot behavior.

Run `.\tools\verify-teleop.ps1` for ordinary unit tests, debug packaging and lint. Add `-Sim -Scenario Gate`
or `-Sim -Scenario Drivetrain` for selected simulations; `-FullSim` runs all currently available simulations.
The opt-in Gradle task is `:TeamCode:simTeleOpTest`. The current milestone covers actual drivetrain/gate
test OpModes, not full red/blue competition simulation. See the
[implementation plan](doc/SimLoopTeleOpImplementationPlan.md),
[verification matrix](doc/TeleOpVerificationMatrix.md) and [local usage](simloop/README.md).
