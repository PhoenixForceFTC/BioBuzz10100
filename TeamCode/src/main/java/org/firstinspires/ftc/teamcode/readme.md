## TeamCode Module — BioBuzz 10100 (`V1`)

Welcome! This module (`TeamCode`) holds our team's robot code built on **FTC Robot Controller SDK v12.0**.

- **Architecture & Best Practices**: Read [AGENTS.md](../../../../../../../AGENTS.md) for FTC software best practices, including how to organize code into **Subsystem classes**, individual **Test OpModes**, and **Combined Competition OpModes**.

### Current Implemented Code

1. **Layer 1 — Subsystems**:
   - **[MecanumDrivetrain.java](MecanumDrivetrain.java)**: Plain Java subsystem for the 4-motor mecanum drive using **goBILDA 5202/5203 Bare Motors** (`28.0` ticks/rev, `6000.0 RPM`, `RUN_USING_ENCODER`, `BRAKE`). Supports config names `leftfront`/`rightfront`/`leftback`/`rightback` (as well as `front_left_drive`, etc.).
   - **[PinpointOdometry.java](PinpointOdometry.java)**: Plain Java subsystem for the **goBILDA Pinpoint Odometry Computer** and two **4-Bar Odometry Pods** (`goBILDA_4_BAR_POD`, `19.8944 ticks/mm`, both pods `EncoderDirection.REVERSED`):
     - Forward/Backward (X) pod: `24.80 + 14.40/2 = +32.00 mm` left of center (`xOffset = +32.00 mm`), `72.20 mm` back of center.
     - Left/Right Strafe (Y) pod: `81.20 - 14.40/2 = 74.00 mm` behind center (`yOffset = -74.00 mm`), `33.80 mm` right of center.
   - **[TurretSubsystem.java](TurretSubsystem.java)**: Plain Java subsystem for the Turret & Limelight 3A assembly:
     - **Rotation Servos (`"turret_servo_1"`, `"turret_servo_2"`)**: 2x Axon MAX in Servo Mode (`500–2500 us`), Expansion Hub Servo Ports 1 & 2 (via goBILDA 6V Power Distribution Hub), `1:1` with turret rotation. Center (`0.5`) = `0°` (right), with a `±10°` deadzone on the left (`-170°..+170°`).
     - **Through-Bore Encoder (`"turret_encoder"`)**: REV Through-Bore Encoder V1 (`8192 CPR`) on Expansion Hub Quadrature Port 1 (`108T Turret : 40T Encoder` = `2.7x` faster = `22,118.4 ticks/rev`, `61.44 ticks/deg`), providing closed-loop position trim after servo homing.
     - **Pitch Servo (`"turret_pitch"`)**: 1x Axon MAX in Servo Mode on Expansion Hub Servo Port 0, linked `2:1` with hood pitch (`15.0°` at `0.0` servo pos, adjustable down to `0.0°` at `0.0845` servo pos).
     - **Shooter Motor (`"turret_motor"`)**: 6000 RPM goBILDA 5202/5203 Bare YellowJacket on Expansion Hub Motor & Quadrature Port 0 (`28.0 ticks/rev`, `6000.0 RPM`, `RUN_USING_ENCODER`, `FLOAT`).
     - **Limelight 3A (`"limelight"`)**: Connected to Control Hub USB 3.0 via powered USB hub, angled `30.0°` up from horizontal toward the goal.
2. **Layer 2 — Standalone Mechanism Test OpModes**:
   - **[DrivetrainTestOpMode.java](DrivetrainTestOpMode.java)** (`@TeleOp(name = "Test: Drivetrain Only", group = "Test")`): Tests only `MecanumDrivetrain` (joystick driving + `RB + X/Y/A/B` single-wheel test).
   - **[PinpointTestOpMode.java](PinpointTestOpMode.java)** (`@TeleOp(name = "Test: Pinpoint Odometry Only", group = "Test")`): Tests only `PinpointOdometry` (push robot by hand to inspect pose and raw pod counts).
   - **[ServoZeroCalibrationOpMode.java](ServoZeroCalibrationOpMode.java)** (`@TeleOp(name = "Test: Set All Servos to 0", group = "Test")`): Drives all turret servos (`turret_pitch`, `turret_servo_1`, `turret_servo_2`) to `0.0` on start, or `0.5` center home (`B`).
   - **[TurretTestOpMode.java](TurretTestOpMode.java)** (`@TeleOp(name = "Test: Turret & Limelight Only", group = "Test")`): Tests only `TurretSubsystem` (rotation, Through-Bore encoder trim, `15°–0°` pitch, 6000 RPM shooter motor, and Limelight 3A auto-aiming).
3. **Layer 3 — Combined TeleOp**:
   - **[MecanumTeleOp.java](MecanumTeleOp.java)** (`@TeleOp(name = "Mecanum TeleOp", group = "Drive")`): Combines `MecanumDrivetrain`, `PinpointOdometry`, and `TurretSubsystem`.

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