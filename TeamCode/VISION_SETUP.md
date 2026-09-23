# BIOBUZZ vision setup

The main `Mecanum TeleOp` uses the Limelight 3A for hive AprilTags and the
Logitech C270 for yellow POLLEN pickup. Configure them as `limelight` and
`webcam` in the Robot Controller hardware configuration. Both cameras face
forward. The current math assumes the Limelight is tilted about 75 degrees up
and the webcam about 30 degrees down, both at roughly 6 inches height.

## Limelight

1. In the Limelight web interface, make pipeline **0** a 36h11 AprilTag
   detector. Set the BIOBUZZ printed tag size to **3.25 inches**. This code
   changes to pipeline 0 during OpMode initialization.
2. Hold a printed tag in front of the mounted Limelight and check telemetry:
   IDs 30–33 are red opposite audience, 34–37 red audience, 38–41 blue
   audience, and 42–45 blue opposite audience. The OpMode reports every
   visible cell and the IDs that produced it.
3. Toggle hive assist while a cell is visible. The closest-to-center visible
   cell is selected and remains locked until the assist is toggled off. The
   assist changes **turning only**; the driver still controls translation.
   If the selected cell is lost or its result is over 250 ms old, turn output
   is zero. Moving the right stick deliberately cancels the assist.
4. Check that an off-center target on the camera's right causes clockwise
   turning and that the reported bearing falls toward zero. If the rotation
   is opposite, invert the sign of the assisted turn in `MecanumTeleOp`.

Limelight reports individual tags, whereas the FTC SDK groups them into
four-tag clusters. `HiveTagTracker` uses the SDK 12.0 member X offsets and
Limelight's individual tag angles and ranges to estimate the cluster's
horizontal origin. With multiple tags it estimates foreshortening; with a
single tag it uses an approximate offset. This is for horizontal turn
alignment only. It does not measure launch distance, calibrate shooter aim,
or decide whether a tipped cell is currently scorable. Test each of the four
cells on a real field before relying on it in a match.

## Logitech C270 and POLLEN

The C270 runs at 320×240 through VisionPortal. `PollenDetector` uses the
yellow BIOBUZZ color limits from the Hive Vision OpenCV configuration and
filters contours by size, fill, aspect, and circularity. The published
limits were fitted from a different camera and field footage. The detector
requires three consecutive detections near the same image position before
moving. It turns toward the blob, drives forward at limited speed, runs the
intake, and stops drive when the blob center reaches 86% of image height.
The intake runs briefly afterward to finish collection. Manual stick input
or intake reverse cancels pickup.

Watch the webcam preview on the Driver Station and adjust `HSV_MIN`,
`HSV_MAX`, `MIN_AREA`, `MAX_AREA`, and `POLLEN_STOP_Y` for your actual camera,
lighting, and intake geometry. If the intake is centered elsewhere in the
image, adjust the horizontal target in `MecanumTeleOp`. Test with wheels
raised first, then with one pollen ball on a clear field. The camera cannot
confirm that a ball entered the intake; the lower-image stop point is only a
visual proxy.

## Gamepad 1 controls

| Control | Function |
| --- | --- |
| Left stick | Drive forward/back and strafe |
| Right stick X | Turn; deliberate input cancels hive assist |
| Left stick button | 25% manual drive speed |
| Right/left triggers | Intake in/reverse |
| A/Y | Kicker forward/reverse |
| Right/left bumpers | Flywheel output RPM +100/−100 |
| B | Stop flywheels |
| X | Toggle visible-hive turn assist |
| D-pad down | Toggle automatic pollen pickup |

The two assists are mutually exclusive. Driver Station Stop always stops
the drive, intake, kicker, and flywheel motors in the OpMode cleanup block.

Sources: [FIRST 2026–27 game manual](https://ftc-resources.firstinspires.org/ftc/game),
[FIRST AprilTag cluster guide](https://ftc-docs.firstinspires.org/en/latest/tech_tips/tech-tips/tech-tip-apriltag-clusters/tech-tip-apriltag-clusters.html),
and [Hive Vision Control Hub detector](https://github.com/sidhuharjas/Hive-Vision/tree/main/cv).
