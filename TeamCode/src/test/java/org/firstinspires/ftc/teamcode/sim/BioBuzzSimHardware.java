package org.firstinspires.ftc.teamcode.sim;

import org.horizon36596.simloop.fakehardware.FakeHardwareMap;
import org.horizon36596.simloop.fakehardware.FakeMotor;
import org.horizon36596.simloop.fakehardware.FakeServo;
import org.horizon36596.simloop.fakehardware.FakeDigitalChannel;
import org.horizon36596.simloop.fakehardware.FakeVoltageSensor;
import org.horizon36596.simloop.fakehardware.FakeGoBildaPinpoint;
import org.horizon36596.simloop.fakehardware.FakeLimelight3A;
import org.horizon36596.simloop.vision.AprilTagLayout;
import org.horizon36596.simloop.vision.CameraMount;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.EncoderDirection;

/** Mechanism-only fixtures: an unrelated lookup must fail, rather than silently supplying hardware. */
final class BioBuzzSimHardware {
    final FakeHardwareMap map = new FakeHardwareMap();
    final FakeMotor[] wheels = new FakeMotor[4]; // FL, FR, BL, BR
    FakeServo gate;
    FakeMotor intake, transfer1, transfer2, flywheel, encoder;
    FakeServo yaw1, yaw2, hood, depollinator;
    FakeDigitalChannel distance;
    FakeGoBildaPinpoint pinpoint;
    FakeLimelight3A camera;
    TargetNormalCameraAdapter cameraAdapter;

    private FakeMotor motor(String name) {
        FakeMotor motor = new FakeMotor(BioBuzzSimModel.FREE_SPEED_TICKS_PER_SECOND, 10, 28);
        map.register(name, motor);
        return motor;
    }

    private FakeServo servo(String name) {
        FakeServo servo = new FakeServo();
        map.register(name, servo);
        return servo;
    }

    BioBuzzSimHardware withIntake() { intake = motor("intake_motor"); return this; }
    BioBuzzSimHardware withTransfer() {
        if (gate == null) gate = servo("gate_servo");
        transfer1 = motor("transfer_motor_1"); transfer2 = motor("transfer_motor_2");
        distance = new FakeDigitalChannel(); map.register("distance_sensor", distance);
        distance.setSimulatedInputState(false);
        return this;
    }
    BioBuzzSimHardware withDepollinator() {
        depollinator = servo("depollinator"); return this;
    }
    BioBuzzSimHardware withTurret() {
        yaw1 = servo("turret_servo_1"); yaw2 = servo("turret_servo_2"); hood = servo("turret_pitch");
        // Independent team observation: 5400 RPM at 14 V, 28 ticks/rev, 1:1 drive.
        // Response rate remains synthetic; no calibrated shot/load dynamics.
        flywheel = new FakeMotor(5400.0 / 60 * 28, 10, 28);
        map.register("turret_motor", flywheel);
        // Real through-bore quadrature is plugged into transfer motor 1's encoder port.
        // Register only the installed name, exercising the production fallback lookup.
        if (transfer1 == null) transfer1 = motor("transfer_motor_1");
        encoder = transfer1;
        map.register("Control Hub", new FakeVoltageSensor(14.0));
        return this;
    }
    BioBuzzSimHardware withPinpoint() {
        pinpoint = new FakeGoBildaPinpoint();
        pinpoint.setPhysicalPodMount(32, -74, EncoderDirection.REVERSED,
                EncoderDirection.REVERSED, 2000 / (32 * Math.PI));
        pinpoint.attachChassisAt(0, 0, 0);
        map.register("pinpoint", pinpoint);
        return this;
    }
    BioBuzzSimHardware withCamera(AprilTagLayout layout) {
        return withCamera(layout, false);
    }
    BioBuzzSimHardware withCamera(AprilTagLayout layout, boolean normalizeTargetNormal) {
        if (camera != null) map.remove(null, "limelight", camera);
        if (cameraAdapter != null) map.remove(null, "limelight", cameraAdapter);
        // Camera lives on the rotating turret. Model supplies the pivot pose, not the chassis pose.
        camera = FakeLimelight3A.builder(layout,
                new CameraMount(-99.53073974 / 25.4, 0, 335 / 25.4, 0, -Math.PI / 6), 54, 41)
                .rangeInches(2, 200).frameRateHz(30).latencyMs(15, 20).build();
        cameraAdapter = normalizeTargetNormal ? new TargetNormalCameraAdapter(camera) : null;
        if (cameraAdapter == null) map.register("limelight", camera);
        else map.register("limelight", cameraAdapter);
        return this;
    }
    static BioBuzzSimHardware fullRobot() {
        return drivetrain().withIntake().withTransfer().withDepollinator().withTurret()
                .withPinpoint().withCamera(AprilTagLayout.empty());
    }

    static AprilTagLayout raisedAudienceTags() {
        AprilTagLayout.Builder layout = AprilTagLayout.builder();
        // TU06 nominal map: one raised cell per alliance; never both states of the same hive.
        double[] offsets = {-6.5, -2.75, 2.75, 6.5};
        for (int side = 0; side < 2; side++) for (int i = 0; i < 4; i++) {
            layout.addTiltedTag(34 + 4 * side + i, (side == 0 ? -12.75 : 12.75) + offsets[i],
                    -org.firstinspires.ftc.teamcode.HiveTagGeometry.RAISED_TAG_ROW_Y_METERS / .0254,
                    org.firstinspires.ftc.teamcode.HiveTagGeometry.DRAWING_TAG_HEIGHT_METERS / .0254,
                    -Math.PI / 2, Math.PI / 6, 3.25);
        }
        return layout.build();
    }

    static BioBuzzSimHardware forOpMode(String name) {
        switch (name) {
            case "DrivetrainTestOpMode": return drivetrain();
            case "GateTestOpMode": return gate();
            case "IntakeTestOpMode": return new BioBuzzSimHardware().withIntake();
            case "TransferTestOpMode": return new BioBuzzSimHardware().withTransfer();
            case "DepollinatorTestOpMode": return new BioBuzzSimHardware().withDepollinator();
            case "TurretTestOpMode": return new BioBuzzSimHardware().withTurret();
            case "PinpointTestOpMode": return new BioBuzzSimHardware().withPinpoint();
            case "ShotCalibrationOpMode":
            case "IntakeShooterTestOpMode": return new BioBuzzSimHardware().withIntake().withTransfer().withTurret();
            case "RedMecanumTeleOp": case "BlueMecanumTeleOp":
            case "AutoAimTestOpMode": case "LimelightTagTestOpMode": case "GoalStateTestOpMode":
            case "PedroDriveTestOpMode": return fullRobot();
            default: throw new IllegalArgumentException("Unsupported review OpMode: " + name);
        }
    }

    static BioBuzzSimHardware drivetrain(String... names) {
        BioBuzzSimHardware hardware = new BioBuzzSimHardware();
        for (int i = 0; i < 4; i++) {
            hardware.wheels[i] = new FakeMotor(BioBuzzSimModel.FREE_SPEED_TICKS_PER_SECOND,
                    BioBuzzSimModel.RESPONSE_RATE_PER_SECOND, 28);
            hardware.map.register(names[i], hardware.wheels[i]);
        }
        return hardware;
    }

    static BioBuzzSimHardware drivetrain() {
        return drivetrain("leftfront", "rightfront", "leftback", "rightback");
    }

    static BioBuzzSimHardware gate() {
        BioBuzzSimHardware hardware = new BioBuzzSimHardware();
        hardware.gate = new FakeServo();
        hardware.map.register("gate_servo", hardware.gate);
        return hardware;
    }
}
