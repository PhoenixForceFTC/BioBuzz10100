# BIOBUZZ FTC Robot Code

Welcome! This repository contains the Java code and build files for **BioBuzz**, a FIRST Tech Challenge (FTC) robot for the 2026–27 season. The code runs on the robot's Control Hub and connects the driver's gamepad to the robot's motors and cameras.

You do not need to know robotics to start exploring. A few terms you will see here:

- **FTC** is a student robotics competition run by FIRST.
- An **OpMode** is one robot program you select on the Driver Station. BioBuzz has a driving program and a safe wheel-test program.
- A **mecanum drive** uses special angled-roller wheels so the robot can move forward, sideways, and turn.

## What BioBuzz can do

The Java code includes mecanum driving, a flywheel, an intake and kicker, and camera-assisted features for finding game elements. The cameras and robot must be configured and tested on the physical robot; code alone cannot verify wiring, mounting, or safe motor directions.

The pictures and early parallel-plate drivetrain notes previously shown here are collected in the [hardware design notes](doc/HARDWARE_DESIGN.md). They are concept documentation, not a substitute for the robot's CAD or a verified assembly drawing. The repository currently does not include CAD source files or a wiring diagram, and the early drivetrain notes need to be checked against the hardware assumptions in the software before building from them.

## New to FTC or Java?

Start with the official [FTC Blocks tutorial](https://ftc-docs.firstinspires.org/programming_resources/blocks/Blocks-Tutorial.html) to learn how robot programs work. FTC's [OnBot Java guide](https://ftc-docs.firstinspires.org/programming_resources/onbot_java/OnBot-Java-Tutorial.html) is another way to write Java in a browser.

This repository is for the **Android Studio Java** workflow. To edit and build this code, install Android Studio Narwhal 3 Feature Drop or later. If your team programs entirely with Blocks or OnBot Java on the robot, you do not need to download this repository just to use those tools.

## Build the Android Studio project

1. Download this repository with Git, or use GitHub's **Code → Download ZIP** button.

   ```bash
   git clone https://github.com/PhoenixForceFTC/BioBuzz10100.git
   ```

2. Open the downloaded `BioBuzz10100` folder in Android Studio and let the project finish syncing.
3. From the project folder, build the team app and run its lint checks:

   ```bash
   ./gradlew :TeamCode:assembleDebug :TeamCode:lintDebug
   ```

   On Windows, use `gradlew.bat :TeamCode:assembleDebug :TeamCode:lintDebug` instead.

For help connecting or programming a Robot Controller, use the current [FTC documentation](https://ftc-docs.firstinspires.org/index.html). Building the app does not check that the robot is wired or mechanically safe.

## Where to look

| Folder or file | What it contains |
| --- | --- |
| [`TeamCode/src/main/java/org/firstinspires/ftc/teamcode/`](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/) | BioBuzz's Java robot programs and vision helpers |
| [`TeamCode/DRIVETRAIN_CHECKS.md`](TeamCode/DRIVETRAIN_CHECKS.md) | Safe, step-by-step drivetrain and encoder checks; read before driving |
| [`TeamCode/VISION_SETUP.md`](TeamCode/VISION_SETUP.md) | Camera setup, vision behavior, and driver controls |
| [`doc/HARDWARE_DESIGN.md`](doc/HARDWARE_DESIGN.md) | Hardware overview, early drivetrain concept, and a checklist of missing design files |
| [`bom.csv`](bom.csv) | Bill of materials (BOM): a spreadsheet-friendly list of some purchased parts |
| [`FtcRobotController/`](FtcRobotController/) | FTC Robot Controller app and the FTC SDK sample code |
| [`TeamCode/`](TeamCode/) | BioBuzz's team-specific Android app module |
| [`doc/FTC_SDK_RELEASE_NOTES.md`](doc/FTC_SDK_RELEASE_NOTES.md) | Upstream FTC SDK release history (kept separate from this getting-started guide) |

## Before testing on the robot

Read [Mecanum bring-up checks](TeamCode/DRIVETRAIN_CHECKS.md) before enabling the drive motors. Those checks require supporting the robot with its wheels off the ground and confirming encoder wiring and wheel direction at low power. Follow the [vision setup guide](TeamCode/VISION_SETUP.md) before relying on camera features.

The software documents the hardware configuration it expects, but the concept drivetrain notes and that configuration do not yet fully agree. Review [the hardware design notes](doc/HARDWARE_DESIGN.md) and verify the assembled robot before changing motor settings or ordering parts.

## Help and references

- [FTC documentation and tutorials](https://ftc-docs.firstinspires.org/index.html)
- [FTC community forum](https://ftc-community.firstinspires.org/)
- [FTC SDK Java API reference](https://javadoc.io/doc/org.firstinspires.ftc)
- [FIRST Tech Challenge game information](https://ftc.game/)

This project builds on the [FIRST Tech Challenge Robot Controller SDK](https://github.com/FIRST-Tech-Challenge/FtcRobotController). Its upstream release history is preserved in [`doc/FTC_SDK_RELEASE_NOTES.md`](doc/FTC_SDK_RELEASE_NOTES.md).
