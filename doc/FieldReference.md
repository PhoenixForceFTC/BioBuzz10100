# BIOBUZZ TU06 field reference for robot software

Reviewed October 23, 2026 against the supplied [Section 9 ARENA PDF](C:/Users/preco/Downloads/BIOBUZZ_CM_Section9_TU06.pdf), version TU06, printed pages 63-79 (17 PDF pages). The dimensioned hive, cell and tag diagrams were inspected visually; text extraction alone omits dimensions and mangles several tag IDs. This note records field data, not instructions to perform field setup or physical checks. Match paths remain deferred.

Section 9.1, printed p.63, identifies the CAD model as the authoritative construction reference and describes manual dimensions as nominal, generally with +/- 1 in. (+/- 2.5 cm) tolerance unless otherwise specified. Numerical conversions below do not imply tighter accuracy. No dimensions were measured from image pixels.

## Confirmed nominal dimensions

| Item | Reference value | Source |
| --- | --- | --- |
| Field interior | Approximately 144 x 144 in. (3.6576 x 3.6576 m by inch conversion) | Section 9.2, p.64 |
| Hive pivot height above tiles | 43.95 in. (1.11633 m by inch conversion) | Figure 9-8, p.70 |
| Hive lateral center spacing | 25.5 in. (647.7 mm) | Figure 9-10, p.71 |
| Frame base width/depth | 49.46 / 38.95 in. | Figure 9-8, p.70 |
| Hive cell separation / depth | 18.84 / 12.04 in. | Figure 9-9, p.71 |
| Raised opening bottom / top above tiles | 53.5 / 65.6 in. (1.3589 / 1.66624 m) | Figure 9-10, p.71 |
| Opening width / peak height | 20 / 14 in. (508 / 355.6 mm) | Figure 9-11, p.72 |
| Opening shoulder height | 7.61 in. (193.294 mm) | Figure 9-11, p.72 |
| Hive tilt shown in side view | 30 degrees for the tilted cell floor relative to horizontal; the perpendicular opening plane is 60 degrees from horizontal | Figure 9-10, p.71; plane interpretation agrees with the team's clarification |
| Pollen | Yellow, approximately 2.8 in. (71 mm in the manual), not perfectly spherical | Section 9.8, p.74 |
| AprilTag family / individual size | 36h11, 3.25 in. square (82.55 mm by inch conversion) | Section 9.9 and Figure 9-15, pp.74-75 |

The code's 144-in. field, 25.5-in. hive spacing, 43.95-in. pivot height, 18.84/12.04-in. cell geometry and 20/14/7.61-in. opening dimensions agree with this reference. The 30-degree hive-tilt constant and the team's 60-degree opening-plane angle describe different planes; they are not competing values.

The midpoint of the manual's opening top/bottom heights is about **1.513 m**, consistent with the team's **1.50 m opening-center** value at the manual's stated dimensional precision. This midpoint is a geometric comparison, not a newly selected aim point or the centroid of the pentagonal opening. Preserve the team's confirmed aim height.

## AprilTag clusters and ordering

Figures 9-16 and 9-17, printed pp.76-77, confirm the existing alliance/side mapping:

| Cell | IDs | Order from left to right in the audience-oriented overhead drawing |
| --- | --- | --- |
| Red far / scoring side | 30-33 | 33, 32, 31, 30 |
| Red audience | 34-37 | 34, 35, 36, 37 |
| Blue audience | 38-41 | 38, 39, 40, 41 |
| Blue far / scoring side | 42-45 | 45, 44, 43, 42 |

Figure 9-15 gives tag-center offsets from the cluster center: **-6.5, -2.75, +2.75, +6.5 in.** across the sticker (**-165.1, -69.85, +69.85, +165.1 mm**). Each sticker's IDs increase from left to right when viewed facing its printed surface. When using the team's field +X toward blue convention, audience-side IDs increase with X, while far-side IDs decrease with X. Do not give all four IDs the cluster center or the opening center.

Additional dimensions in Figure 9-15:

- The two reference-hole centers are 7.0 in. (177.8 mm) either side of the cluster centerline.
- Tag centers are 2.75 in. (69.85 mm) from the reference-hole line toward the front of the cell, measured in the sticker plane.
- The reference-hole line is 9.938 in. (252.4252 mm) behind the front-of-cell reference, along the underside.
- Consequently, the tag-center line is **7.188 in. (182.5752 mm)** behind that reference in the cell plane. This is not a field-horizontal distance.
- The sticker sits on the bottom of the cell, with its bottom edge toward field center (p.75).

Under an ideal 30-degree raised-cell floor tilt, that 7.188-in. setback has a horizontal component of about **158.1 mm** and a vertical component of about **91.3 mm**. Using the nominal 53.5-in. front/bottom height gives a rough tag-height consistency estimate of **1.268 m**, close to the team's **1.25 m**. This is not a replacement measurement: the exact front-edge/surface references and construction offsets need the CAD transform. The team's confirmed heights remain unchanged.

Configure the Limelight AprilTag pipeline for **36h11 and 3.25-in. individual tags** when configuring the camera. Java currently selects pipeline 0; it does not set tag family/size. Camera configuration was not accessed or changed here. The current angle-based geometry does not obtain metric range from tag size, but any camera-reported metric pose depends on correct configuration.

## Moving hive state

Section 9.6, printed p.69, states that each hive pivots between two stable positions, with one cell facing upward. The AprilTag clusters are attached to those cells and move with them. A tag ID identifies a cell, not its current raised/lowered state. Low-goal shooting is outside the team's scope, but automatic shooting must still distinguish a lowered or transitioning selected cell from a raised one.

The PDF's linked [Event Field Setup Guide, section 11.1, p.23](https://ftc-resources.firstinspires.org/ftc/archive/2027/field/eventfieldguide) specifies the initial raised cells as **red audience** and **blue scoring/far**. This is an initial setup condition, not a state guarantee later in the match. No runtime state or default target was enabled from this fact.

## Remaining field-coordinate dependency

This dependency applies to the existing absolute field-localization path. Raised/low state can instead be estimated from an independent Limelight 3D tag pose and the known camera mounting, without first supplying a full field map. Target-relative aiming also needs the local tag-to-opening transform. See the [implementation plan](HardwareReviewImplementationPlan.md); the metric detector and target-relative software path are now implemented, with calibration boundaries recorded in [implementation status](HardwareReviewImplementationStatus.md).

This reference resolves tag IDs, ordering, cluster spacing, sticker-to-front dimensions, field orientation and nominal structural dimensions. It does **not directly dimension the raised cell's complete transform from the hive pivot to its front/underside reference**, including offsets introduced by the connecting assembly. The existing **17.73-in. opening Y offset is not independently established by these labeled dimensions** and must remain an unapproved example.

For a complete field-localization map, obtain a CAD-derived raised-cell opening center and tag-cluster center relative to the hive pivot. The confirmed tag spacing/order then gives the individual tag coordinates; there is no need to measure every sticker separately. Match paths are not required for this step. Keep the team's 1.50-m opening and 1.25-m tag heights unless explicitly superseded.

The PDF links to the [official field CAD](https://ftc-resources.firstinspires.org/ftc/archive/2027/field/field-cad), which redirects to an Onshape assembly. The web fetch could not inspect that assembly during this review. The linked event setup guide was readable, but did not supply the missing pivot-to-cell transform. Physical checks remain deferred.

The code therefore retains its incomplete target-map status and automatic-shooting inhibits. This review updates source provenance and narrows the missing coordinate data; it does not certify a field map, change autonomous routes or resolve the separate software findings in HardwareSoftwareReview.md.
