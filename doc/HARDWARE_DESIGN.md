# BioBuzz hardware design overview

This document describes what is currently recorded in the repository. It is a starting point for understanding the robot, **not a complete set of build instructions**. Confirm the actual robot and current design files before ordering parts, wiring devices, or running motors.

## Robot at a glance

BioBuzz's team code is written for an FTC robot with a four-wheel mecanum drivetrain, two fly motors, an intake, a kicker, a Limelight 3A, and a Logitech C270 webcam. See [the main TeleOp](../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/MecanumTeleOp.java) for the device names expected by the software and [the vision setup guide](../TeamCode/VISION_SETUP.md) for camera setup details.

The checked-in [bill of materials](../bom.csv) is a CSV parts list. Its current entries include mecanum wheels, odometry pods, gears, standoffs, beams, spacers, and pillow blocks. It is not a complete, confirmed list of everything needed to reproduce the robot.

## Early drivetrain concept

The original project notes describe a compact drivetrain that sandwiches mecanum wheels between two parallel plates. Those notes propose 6,000 RPM motors, a 1:4 gear reduction, then a 1:3 belt reduction driving 104 mm mecanum wheels. They also describe an HTD-3M, 9 mm-wide belt, an adjustable tensioner, and bearing stacks to constrain the belt path. The original project images are included below as concept illustrations.

> **Important: this concept does not match the current software assumptions.** [The drivetrain checks](../TeamCode/DRIVETRAIN_CHECKS.md) and [TeleOp code](../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/MecanumTeleOp.java) instead assume four 435 RPM motors with approximately 384.5 encoder ticks per wheel revolution and a 1:1 external bevel gear ratio. Do not assume both descriptions refer to the same final, assembled robot. The drivetrain ratio, motor model, wheel size, encoder values, and direction settings need to be reconciled with the actual hardware before driving or building.

| Existing concept image | Preview |
| --- | --- |
| Drivetrain concept | <img width="632" alt="Early drivetrain concept" src="https://github.com/user-attachments/assets/7dfbf4ac-fdf0-4a8e-b49c-2196934d119e" /> |
| Concept detail | <img width="710" alt="Early drivetrain detail" src="https://github.com/user-attachments/assets/0294889b-dea5-4750-bab0-5132535cc05c" /> |
| Concept detail | <img width="698" alt="Early drivetrain detail" src="https://github.com/user-attachments/assets/1e86831d-3705-45ca-8e06-f18124b60ac5" /> |
| Concept detail | <img width="364" alt="Early drivetrain detail" src="https://github.com/user-attachments/assets/caccd3be-4b9c-4168-903d-01186e672817" /> |
| Concept detail | <img width="365" alt="Early drivetrain detail" src="https://github.com/user-attachments/assets/51a01fc9-2302-4356-b718-53ae49aac4e5" /> |

## Files available and still needed

The repository contains software-side hardware notes and a parts-list starter, but the file inventory does **not** currently include the following design deliverables:

- Editable CAD source or a neutral-format export such as STEP for the complete robot assembly.
- Individual CAD files for custom plates, brackets, mounts, spacers, or other fabricated parts.
- A labeled wiring diagram showing hubs, motor and encoder ports, cameras, and power connections.
- Dimensioned drawings, belt length and pulley tooth-count details, or a final assembly view.
- A complete, verified BOM with quantities and all custom or off-the-shelf parts.
- Current photographs of the assembled robot that can be compared with the CAD and wiring plan.

The README can be updated with links and previews when those files are added. A useful layout would be `CAD/` for native files and STEP exports, `wiring/` for a labeled diagram, and this `doc/` folder for readable design notes. These folders are recommendations only; no CAD or wiring artifacts are being claimed as present today.

## Safe next steps

1. Identify which drivetrain description matches the robot that is actually assembled.
2. Add or link the current CAD assembly and its editable source files.
3. Add a wiring diagram and confirm every motor name and hub port against the robot configuration.
4. Reconcile the BOM with the CAD and the parts on the robot.
5. Complete the low-power, wheels-off-the-ground checks in [DRIVETRAIN_CHECKS.md](../TeamCode/DRIVETRAIN_CHECKS.md) before normal driving.
