# Zenith match project

Open the repository's zenith.json in Zenith. This directory is the source for match routines, robot command declarations and field/waypoint data.

There are no competition routes yet. Create red-match.auto.json and blue-match.auto.json using the team's actual start poses, paths and timing. Existing match OpModes load those names. Other names get their own Ivy OpMode from `npm run zenith:deploy`.

The pollen experiment lives directly in TeamCode/src/main/assets/autos/pollen-navigation-test.auto.json and runs only through the hand-written Test-group OpMode. It is not a match template.

Read [autonomous setup and command contracts](../doc/Autonomous.md) before robot motion. Dynamics and dimensions in robot.json are explicitly unmeasured examples, and PedroConfiguration.TUNING_VERIFIED remains false. The BIOBUZZ field is from Zenith v0.1.2; attribution and license are in that setup document.
