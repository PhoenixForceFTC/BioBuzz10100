# Zenith UI acceptance record

Completed October 31, 2026 against the actual BioBuzz10100 repository in the official portable **Zenith v0.1.2** desktop editor. The executable matched the release SHA256 recorded in [the integration audit](ZenithIntegrationAudit.md). Earlier browser-folder and desktop-approval attempts were unsuccessful; desktop access subsequently worked and the following checks were performed.

## Observed workflow

| Check | Evidence |
| --- | --- |
| Open the local project | Open folder selected the repository containing `zenith.json`; the editor showed BioBuzz10100 and the canonical routines. |
| Pollen remains discoverable | Opened `pollen-navigation-test.auto.json`; title **Test: Pollen Navigation (Pedro + Ivy)**, local start (0,0), `navigateToPollen` and `stop` steps. Hardware isolation/Test registration is verified by Java/deployment tests, not inferred from the editor title. |
| Complete catalog | The command selector listed all 43 names; the condition-wait menu listed all 23 named predicates and help. |
| Create/save | New auto created `autos/new-auto.auto.json`; Ctrl+S wrote canonical source, without Java edits. |
| Numbers/units | Inserted `intakeRun`, entered power 0.4 and seconds 0.5; inspected range/help and saved values. |
| Enum/Boolean | Inserted `aim`; default AUDIENCE/FIELD/spin true was visible. Changed frame to AUTHORED and spin to false, then saved. |
| Geometry/speed/start | Changed the generated line to a Bezier curve, control point (-6,-51) inches, path speed 15%, timeout 6 seconds and start X -13 inches. |
| Marker | Added an `intakeOn` marker at 50% along the curve. |
| Bounded wait/error | Added `queueFull` wait without timeout: UI displayed WAIT_NO_TIMEOUT and team validation rejected `new-auto.auto.json: steps.3: condition waits require timeoutS`. Entered 0.4 seconds; the warning cleared and corrected source passed validation. |
| Stop policy | Added a final `stopAll`. This is a resumable action, so its placement after the other steps matters. |
| Native groups/branches | In `integration-example`, changed parallel mode all to race and branch `shotReady` to `queueFull`, saved and reopened. A deep JSON comparison proved that only those two fields changed; paths, markers, arguments, branch arms and metadata survived. Restored the original example bytes afterward. |
| Reopen | Switched away from the UI-created routine and reopened it from disk. The saved start (-13,-63), curved path, marker and five-step tree persisted. |
| Generate/build | Team deployment generated **NewAutoAuto**, title `new-auto`, group Generated, using `BioBuzzAutoFromFile` and full `BioBuzzRobot`. It compiled and its source JSON was verified in the temporary nine-asset APK. |
| Rename/delete | Renamed the temporary source to `ui-roundtrip-check` with matching name/title using file tools; team deployment added its wrapper and removed the old wrapper/asset. Deleted that temporary source and redeployed; both temporary wrappers/assets were removed. No dedicated native UI rename/delete controls are claimed. |
| Final state | Only the original two canonical routines remain; the pollen Test wrapper is retained. Deployment/check requests zero writes/removals. Final rebuilt APK has eight matching Zenith JSON assets. |

The editor also displayed TIMEOUT_TIGHT when a four-second path timeout was shorter than its estimate. That estimate changed with curve geometry and speed; it remains based on unmeasured dynamics. Collection's unknown duration can appear as zero in the total timeline with a dash on the action. This does not mean collection finishes instantly.

## Retained regression artifact

The exact UI-saved JSON is retained at
`TeamCode/src/test/resources/zenith/new-auto.auto.json`, outside the deployable `autos/` directory.
SHA256: `96cd55cc2ee6a59670b92302d16fc145cb1fb1c1b2adcf7fd16abe7f8008b237`.

`AutonomousIntegrationTest.actualZenithUiSavedRoutineRunsThroughNativeFactoriesAndStops` loads that file through the real AutoBuilder/Ivy libraries and production command factories. It checks the curved path/speed, the false queue-full wait's TIMEOUT surviving final stop, and stopped intake/flywheel/gate/transfer outputs. The path world and independent mechanism feedback are test fixtures; this is not chassis physics or a robot run. The fixture remains intentionally generic and is not a match route.

Session screenshots showed the controls and results. Additional local working evidence lives in ignored `build/zenith-ui-audit/`: the pre-edit examples, changed example, rejected-wait and corrected validation logs, renamed manifest and build logs. Permanent evidence is the source fixture, regression, deployment tests and this record; those build files are disposable.

## Acceptance limits

Core local UI authoring/save/reopen and supported team deployment/build are verified. Named-waypoint library editing and every parameter field of every action were not individually exercised in this UI run; catalog coverage and argument/resource checks also rely on automated tests. Routine rename/delete was checked through source-file maintenance, not a promised editor button. The native Deploy button, cloud sharing/PR actions and a high-fidelity autonomous simulator were not tested or configured as part of the supported workflow.

No hardware was connected or commanded. Current robot dimensions/dynamics, deployed envelope, camera eligibility/conventions, shot profiles/presets and release timing retain their documented measurement gates. No physical calibration or match-route acceptance follows from this UI record.
