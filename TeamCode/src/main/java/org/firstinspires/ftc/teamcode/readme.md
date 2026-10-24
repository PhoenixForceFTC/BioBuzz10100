## TeamCode Module — BioBuzz 10100 (`V1`)

See the repository `doc/HardwareReviewImplementationStatus.md` for the October 23 implementation, controls and calibration boundaries. `Test: Limelight Goal State 3D` initializes only the camera. Calibration REVIEW + right-stick click saves persistent session samples/rows without activating a competition profile.

Turret rotation, hood, and depollinator servos are **Axon MAX+**; the gate is a **goBILDA 2000 Series Dual Mode Super Speed (2000-0025-0004)** in positional mode. References to Axon MAX mean MAX+. The rated maximum speed at **6.0 V is 0.115 s/60°** (about **521.74°/s** at the shaft; **260.87°/s** at the hood through its 2:1 linkage), per the [manufacturer specification](https://docs.axon-robotics.com/archive/max). `AxonMaxPlus` centralizes this rating and the robot's programmed 355° / 500–2500 us settings. Travel estimates are not physical position feedback; verify programmed range and loaded timing on the robot. The gate uses 300° travel over 500–2500 us: closed `0.5`, open `0.2` (90° CCW viewed from the output shaft). Its [manufacturer specification](https://www.gobilda.com/2000-series-dual-mode-servo-25-4-super-speed/) rates 0.043 s/60° at 6.0 V (0.0645 s for 90°); the 0.25 s opening allowance remains for loaded settling.

Welcome! This module (`TeamCode`) holds our team's robot code built on **FTC Robot Controller SDK v12.0**.

**Pollen autonomous:** [PollenAuto.java](PollenAuto.java) adds `Auto: Pollen Collection`, using [PollenVision.java](PollenVision.java) for the Logitech C270 (confirmed lens mount: 195 mm high, 248 mm forward, 0 mm sideways, aimed 10 degrees below horizontal) and [PollenNavigator.java](PollenNavigator.java) for non-blocking search, alignment, approach, and bounded intake pickups. Pickup/color calibration and suitable intrinsics remain required. [PollenVisionTestOpMode.java](PollenVisionTestOpMode.java) tests only the camera. The turret's Limelight remains separate. Current scope is pollen detection and navigation; match paths are deferred. Read [camera setup, calibration, and pickup tuning](../../../../../../../../doc/PollenAuto.md) before driving.

- **Architecture & Best Practices**: Read [AGENTS.md](../../../../../../../AGENTS.md) for FTC software best practices, including how to organize code into **Subsystem classes**, individual **Test OpModes**, and **Combined Competition OpModes**.

### Confirmed Control Hub Connections

| Device | Hardware-map name | Control Hub connection | Encoder cable |
| --- | --- | --- | --- |
| Front-left drivetrain | `leftfront` | Motor port 0 | Connected |
| Front-right drivetrain | `rightfront` | Motor port 1 | Connected |
| Back-left drivetrain | `leftback` | Motor port 2 | Connected |
| Back-right drivetrain | `rightback` | Motor port 3 | Connected |
| Pinpoint odometry | `pinpoint` | I2C port 0 | Two odometry pods connect to Pinpoint |

These are wiring assignments for the active Robot Configuration; Java selects each device by its hardware-map name.

Servo power is supplied by the [goBILDA 6V Servo Power Injector 3125-0001-0001](https://www.gobilda.com/6v-servo-power-injector-6-channel-8-15v-input/). It regulates 8–15 V input to 6 V, with up to 24 A total shared across six channels; the Expansion Hub provides the control signals.

Both the Limelight (`limelight`) and USB webcam (`webcam_1`) connect to the **same powered USB hub**, which plugs into the Control Hub. Rename the webcam entry in the active Robot Configuration to `webcam_1`.

| Expansion Hub servo port | Configuration name | Mechanism |
| --- | --- | --- |
| 0 | `turret_pitch` | Hood/pitch |
| 1 | `turret_servo_1` | Turret rotation 1 |
| 2 | `turret_servo_2` | Turret rotation 2 |
| 3 | `gate_servo` | goBILDA Super Speed gate, 150 degrees closed / 60 degrees open (90 degrees CCW) |
| 4 | `depollinator` | Axon MAX+, 30 degrees home / 165 degrees deployed |

### Current Implemented Code

Choose **Mecanum TeleOp - Red** or **Mecanum TeleOp - Blue** in the Competition group. Pipeline 0 can detect all IDs 30–45; only tags mapped to the explicitly selected alliance, side and hive state are used. Visibility does not select a different opening or establish a scoring state. See [configuration and controls](../../../../../../../../doc/SoftwareConfiguration.md) for calibration dependencies, pose/vision timing, the moving-shot solver, shared firing and autonomous routes.

| Alliance | Far / scoring-table goal | Audience goal |
| --- | --- | --- |
| Red | 30–33 | 34–37 |
| Blue | 42–45 | 38–41 |

Tag assignments follow [FIRST's BIOBUZZ manual, AprilTags section](https://ftc-resources.firstinspires.org/ftc/archive/2027/game/cm-html/BIOBUZZ%20Competition%20Manual%20-%20V1.htm). Shot profiles, individual tag geometry and camera/pivot/muzzle offsets require measured configuration; individual tag centers are not the cell opening center.

1. **Layer 1 — Subsystems**:
   - **[DepollinatorSubsystem.java](DepollinatorSubsystem.java)**: Axon MAX+ on Expansion Hub servo port 4, named `depollinator`. Uses the existing Axon MAX+ assumption of 355-degree programmed travel and 500–2500 us PWM, with increasing PWM set to CW (servo must not be programmed inverted), viewed from the output shaft. Home/up is 30 degrees (`0.08451`); deploy is 165 degrees (`0.46479`), a 135-degree CW move. Homes during INIT. GP1 right-stick click deploys and left-stick click homes; commands are edge-triggered and hold their position after release. Home wins simultaneous presses. The servo retains its last commanded position when the OpMode ends.
   - **[TransferSubsystem.java](TransferSubsystem.java)**: Stage 1 is `transfer_motor_1` on Expansion Hub motor port 2, `REVERSE` (CW facing the shaft), contacting balls 2–4. Stage 2 is `transfer_motor_2` on Expansion Hub motor port 3, `FORWARD` (CCW facing the shaft), contacting only the first/top ball. Both use `RUN_WITHOUT_ENCODER` and `BRAKE`. Stage 2 remains stopped during loading and runs only while feeding after gate opening; both stages run at full power during shots. The goBILDA 3124-0001-0001 laser distance sensor is configured as a digital input named `distance_sensor` on Expansion Hub digital port 0. After START, the motor runs at full open-loop power until the digital signal stays HIGH continuously for 2 seconds, confirming full. Brief HIGH pulses do not stop it; every LOW resets confirmation and immediately restarts a stopped motor. Uses a non-blocking timer, reports raw state and confirmation time, and stops on OpMode shutdown. Sensor placement and its potentiometer threshold must be set to indicate four balls loaded; software does not count balls. No gamepad binding is needed.
   - **[IntakeSubsystem.java](IntakeSubsystem.java)**: 1620 RPM goBILDA YellowJacket on Expansion Hub motor port 1, named `intake_motor`, with external 10T motor / 8T intake gearing. Uses `RUN_WITHOUT_ENCODER` and `BRAKE`; no intake encoder cable is required. GP1 right bumper runs forward at full power (motor CCW viewed facing the shaft), left bumper reverses, and release or both held stops. The turret Through-Bore encoder can still use this port's separate encoder input via the same configured motor name.
   - **[GateSubsystem.java](GateSubsystem.java)**: `gate_servo` on Expansion Hub servo port 3. goBILDA 2000 Series Dual Mode Super Speed (2000-0025-0004) in positional mode, 300-degree travel with 500–2500 us PWM. Closed/home is centered at 150 degrees (`0.5`); open is 60 degrees (`0.2`), giving 90 degrees CCW viewed from the output shaft. Closes during INIT and shutdown; automatic/manual feeding opens it after readiness or override.
   - **[MecanumDrivetrain.java](MecanumDrivetrain.java)**: Plain Java subsystem for the 4-motor mecanum drive using **goBILDA 5202/5203 Bare Motors** (`28.0` ticks/rev, `6000.0 RPM`, `RUN_USING_ENCODER`, `BRAKE`). Supports config names `leftfront`/`rightfront`/`leftback`/`rightback` (as well as `front_left_drive`, etc.).
   - **[PinpointOdometry.java](PinpointOdometry.java)**: Plain Java subsystem for the **goBILDA Pinpoint Odometry Computer** and two **4-Bar Odometry Pods** (`goBILDA_4_BAR_POD`, `19.8944 ticks/mm`, both pods `EncoderDirection.REVERSED`):
     - Forward/Backward (X) pod: `24.80 + 14.40/2 = +32.00 mm` left of center (`xOffset = +32.00 mm`), `72.20 mm` back of center.
     - Left/Right Strafe (Y) pod: `81.20 - 14.40/2 = 74.00 mm` behind center (`yOffset = -74.00 mm`), `33.80 mm` right of center.
   - **[TurretSubsystem.java](TurretSubsystem.java)**: Plain Java subsystem for the Turret & Limelight 3A assembly:
     - **Rotation Servos (`"turret_servo_1"`, `"turret_servo_2"`)**: 2x Axon MAX+ in Servo Mode (`500–2500 us`), Expansion Hub Servo Ports 1 & 2 (via goBILDA 6V Servo Power Injector), `1:1` with turret rotation. Home (`0.5`) = robot heading `90°` (right). Headings increase clockwise: front `0°`, right `90°`, back `180°`, left `270°`; the left `260°..280°` zone is excluded. Internal servo-relative yaw is counterclockwise, `-170°..+170°` from home.
     - **Through-Bore Encoder (`"turret_encoder"`)**: REV Through-Bore Encoder V1 (`8192 CPR`) on Expansion Hub Quadrature Port 1 (`108T Turret : 40T Encoder` = `2.7x` faster = `22,118.4 ticks/rev`, `61.44 ticks/deg`), providing closed-loop position trim after servo homing.
     - **Pitch Servo (`"turret_pitch"`)**: 1x Axon MAX+ in Servo Mode on Expansion Hub Servo Port 0, linked `2:1` with hood pitch. Home position `0.5` gives the highest launch elevation, +15° above horizontal. Counter-clockwise shaft rotation lowers elevation to -15° over 30° of hood travel (60° shaft travel, servo position approximately `0.330986`).
     - **Shooter Motor (`"turret_motor"`)**: 6000 RPM goBILDA 5202/5203 Bare YellowJacket on Expansion Hub Motor & Quadrature Port 0 (`28.0 ticks/rev`, `6000.0 RPM`, `RUN_USING_ENCODER`, `FLOAT`).
     - **Limelight 3A (`"limelight"`)**: Connected to Control Hub USB 3.0 via powered USB hub, angled `30.0°` up from horizontal toward the goal, lens height `335 mm` (`13.189 in`), pipeline `0`.
2. **Layer 2 — Standalone Mechanism Test OpModes**:
   - **[DepollinatorTestOpMode.java](DepollinatorTestOpMode.java)** (`Test: Depollinator Only`, group `Test`): Tests only `depollinator` with GP1 right-stick click to deploy and left-stick click to home.
   - **[TransferTestOpMode.java](TransferTestOpMode.java)** (`Test: Transfer Only`, group `Test`): Tests stage 1 automatic loading and the digital full sensor; both transfer motors and gate must be configured. Hold RB for stage 2 alone with stage 1 stopped and the gate closed; release stops stage 2. LB pauses ordinary stage 1 loading. After START, continuous HIGH for 2 seconds stops and LOW runs at full power in REVERSE. Reports raw digital state, confirmation time and commanded power; stops on shutdown.
   - **[IntakeTestOpMode.java](IntakeTestOpMode.java)** (`Test: Intake Only`, group `Test`): Tests only `intake_motor` using GP1 right/left bumpers, without requiring other mechanisms.
   - **[GateTestOpMode.java](GateTestOpMode.java)** (`Test: Gate Only`, group `Test`): Tests only `gate_servo`; hold GP1 left trigger to open to 60 degrees (`0.2`, 90 degrees CCW from home), release to close to 150 degrees (`0.5`).
   - **[DrivetrainTestOpMode.java](DrivetrainTestOpMode.java)** (`@TeleOp(name = "Test: Drivetrain Only", group = "Test")`): Tests only `MecanumDrivetrain` (joystick driving + `RB + X/Y/A/B` single-wheel test).
   - **[PinpointTestOpMode.java](PinpointTestOpMode.java)** (`@TeleOp(name = "Test: Pinpoint Odometry Only", group = "Test")`): Tests only `PinpointOdometry` (push robot by hand to inspect pose and raw pod counts).
   - **[ServoZeroCalibrationOpMode.java](ServoZeroCalibrationOpMode.java)** (`Calibrate: DISCONNECTED Servo RAW`, group `Calibration`): Select ONE mechanically disconnected servo with D-pad Left/Right. After START, both bumpers toggle arming; hold RT plus A/X/Y for midpoint/0/1. No command is sent at INIT or START. Installed turret/hood, gate and depollinator use their bounded subsystem tests; coupled turret servos are tested together.
   - **[TurretTestOpMode.java](TurretTestOpMode.java)** (`@TeleOp(name = "Test: Turret", group = "Test")`): Manual turret rotation, homing, Through-Bore encoder trim, `0°–30°` downward hood travel (+15° to -15° launch elevation), and flywheel testing. Limelight and odometry are not initialized; no automatic aiming is performed.
   - **[AutoAimTestOpMode.java](AutoAimTestOpMode.java)** (`@TeleOp(name = "Test: Auto Aim", group = "Test")`): Limelight + Pinpoint + turret automatic aiming using the competition `ShotCoordinator`. GP1 `RT` aims, `RT + RB` also spins the flywheel, `A` rehomes, `B`/`X` selects Red/Blue, and D-pad Left/Right selects Audience/Far (also during INIT). Push the chassis by hand; no drivetrain, intake, transfer, or gate initialization and no feeding. Requires measured `ShootingConfiguration` mount, target, shot profile, timing, and starting field pose; existing unmeasured defaults block aiming and are reported in telemetry.
3. **Layer 3 — Combined TeleOp**:
   - **Full-queue intake reversal**: A newly confirmed full queue (sensor HIGH continuously for 2 seconds) latches automatic intake reversal at full power. Either controller's intake bumpers always take priority: right runs forward, left reverses, and opposing bumper commands stop intake. Releasing all bumpers resumes latched reversal. The latch stays active when the fourth-ball sensor clears during shooting. Releasing GP1 right trigger declares the shooting cycle complete and clears the latch, restoring bumper control. Manual left-trigger feeding does not clear the latch. The same still-HIGH sensor reading cannot immediately re-latch it; another LOW followed by a fresh 2-second HIGH is required. RT release is the driver's completion signal, not a measured count of four shots.
   - **Automatic shot feeding**: GP1 RT requests the shared `ShotCoordinator`/`FiringCoordinator`. Estimated home, calibrated geometry/profile/timing, a fresh stable observed RAISED cell, valid field or target-relative pose and fresh target, supported motion, compensated yaw/rate tracking, hood travel estimate and stable flywheel speed are required. Invalid solutions stop further feeding; an already committed ball may still exit. Automatic attempts use bounded gate/feed/recovery windows. Manual feed is continuous after gate travel; GP2 feed inhibit always wins. Telemetry reports attempts, not launched-ball counts. Default example calibration cannot enable automatic feeding.
   - **[MecanumTeleOp.java](MecanumTeleOp.java)** (shared abstract base for `Mecanum TeleOp - Red` and `Mecanum TeleOp - Blue`): Combines `MecanumDrivetrain`, `PinpointOdometry`, `IntakeSubsystem`, `GateSubsystem`, and `TurretSubsystem`. Either controller's right/left bumpers run intake forward/reverse; manual bumper commands always override automatic intake reversal, and opposing bumper commands always stop intake. The flywheel starts at 4200 RPM after START and stops on shutdown. Only GP1 right trigger activates Limelight yaw auto-aim, distance-based hood/RPM, and automatic firing when ready. Release restores the manual RPM setting and holds the hood position; unavailable automatic settings use manual RPM; a valid configured opening can be acquired using odometry while vision still inhibits automatic feed. GP1 left trigger or GP2 right trigger manually opens the gate and feeds; GP2 left trigger forces the gate closed and stops both transfer stages. GP2 B holds the flywheel stopped. GP1 sticks drive/turn the chassis; Only GP2 right stick X manually rotates the turret when not auto-aiming. Only GP2 D-pad up/down raises/lowers launch elevation by 0.5 degrees within -15 to +15 degrees. Only GP2 Y/A increases/decreases manual flywheel RPM by 50 per press, clamped to 0–6000 RPM. Manual hood/RPM adjustments are ignored while GP1 auto-aim is held. GP2 D-pad left/right selects audience/far; GP2 X starts a non-blocking rehome. GP1 face buttons/D-pad and odometry reset shortcuts are unassigned. The turret-only test retains manual mechanisms and observation diagnostics; combined compensated aiming requires the competition OpMode.

---

## Creating your own OpModes

The easiest way to create your own OpMode is to copy a Sample OpMode and make it your own.

Sample opmodes exist in the FtcRobotController module.
To locate these samples, find the FtcRobotController module in the "Project/Android" tab.

Expand the following tree elements:
 FtcRobotController/java/org.firstinspires.ftc.robotcontroller/external/samples

### Naming of Samples

To gain a better understanding of how the samples are organized, and how to interpret the
naming system, it will help to understand the conventions that were used during their creation.

These conventions are described (in detail) in the sample_conventions.md file in this folder.

To summarize: A range of different samples classes will reside in the java/external/samples.
The class names will follow a naming convention which indicates the purpose of each class.
The prefix of the name will be one of the following:

Basic:  	This is a minimally functional OpMode used to illustrate the skeleton/structure
            of a particular style of OpMode.  These are bare bones examples.

Sensor:    	This is a Sample OpMode that shows how to use a specific sensor.
            It is not intended to drive a functioning robot, it is simply showing the minimal code
            required to read and display the sensor values.

Robot:	    This is a Sample OpMode that assumes a simple two-motor (differential) drive base.
            It may be used to provide a common baseline driving OpMode, or
            to demonstrate how a particular sensor or concept can be used to navigate.

Concept:	This is a sample OpMode that illustrates performing a specific function or concept.
            These may be complex, but their operation should be explained clearly in the comments,
            or the comments should reference an external doc, guide or tutorial.
            Each OpMode should try to only demonstrate a single concept so they are easy to
            locate based on their name.  These OpModes may not produce a drivable robot.

After the prefix, other conventions will apply:

* Sensor class names are constructed as:    Sensor - Company - Type
* Robot class names are constructed as:     Robot - Mode - Action - OpModetype
* Concept class names are constructed as:   Concept - Topic - OpModetype

Once you are familiar with the range of samples available, you can choose one to be the
basis for your own robot.  In all cases, the desired sample(s) needs to be copied into
your TeamCode module to be used.

This is done inside Android Studio directly, using the following steps:

 1) Locate the desired sample class in the Project/Android tree.

 2) Right click on the sample class and select "Copy"

 3) Expand the  TeamCode/java folder

 4) Right click on the org.firstinspires.ftc.teamcode folder and select "Paste"

 5) You will be prompted for a class name for the copy.
    Choose something meaningful based on the purpose of this class.
    Start with a capital letter, and remember that there may be more similar classes later.

Once your copy has been created, you should prepare it for use on your robot.
This is done by adjusting the OpMode's name, and enabling it to be displayed on the
Driver Station's OpMode list.

Each OpMode sample class begins with several lines of code like the ones shown below:

```
 @TeleOp(name="Template: Linear OpMode", group="Linear Opmode")
 @Disabled
```

The name that will appear on the driver station's "opmode list" is defined by the code:
 ``name="Template: Linear OpMode"``
You can change what appears between the quotes to better describe your opmode.
The "group=" portion of the code can be used to help organize your list of OpModes.

As shown, the current OpMode will NOT appear on the driver station's OpMode list because of the
  ``@Disabled`` annotation which has been included.
This line can simply be deleted , or commented out, to make the OpMode visible.



## ADVANCED Multi-Team App management:  Cloning the TeamCode Module

In some situations, you have multiple teams in your club and you want them to all share
a common code organization, with each being able to *see* the others code but each having
their own team module with their own code that they maintain themselves.

In this situation, you might wish to clone the TeamCode module, once for each of these teams.
Each of the clones would then appear along side each other in the Android Studio module list,
together with the FtcRobotController module (and the original TeamCode module).

Selective Team phones can then be programmed by selecting the desired Module from the pulldown list
prior to clicking to the green Run arrow.

Warning:  This is not for the inexperienced Software developer.
You will need to be comfortable with File manipulations and managing Android Studio Modules.
These changes are performed OUTSIDE of Android Studios, so close Android Studios before you do this.
 
Also.. Make a full project backup before you start this :)

To clone TeamCode, do the following:

Note: Some names start with "Team" and others start with "team".  This is intentional.

1)  Using your operating system file management tools, copy the whole "TeamCode"
    folder to a sibling folder with a corresponding new name, eg: "Team0417".

2)  In the new Team0417 folder, delete the TeamCode.iml file.

3)  the new Team0417 folder, rename the "src/main/java/org/firstinspires/ftc/teamcode" folder
    to a matching name with a lowercase 'team' eg:  "team0417".

4)  In the new Team0417/src/main folder, edit the "AndroidManifest.xml" file, change the line that contains
         package="org.firstinspires.ftc.teamcode"
    to be
         package="org.firstinspires.ftc.team0417"

5)  Add:    include ':Team0417' to the "/settings.gradle" file.
    
6)  Open up Android Studios and clean out any old files by using the menu to "Build/Clean Project""

Competition autonomous adds **Auto: Red Match (Route Required)** and **Auto: Blue Match (Route Required)**. Supply approved `AutonomousStrategy` routes before use; collection remains available separately in `PollenAuto`.
