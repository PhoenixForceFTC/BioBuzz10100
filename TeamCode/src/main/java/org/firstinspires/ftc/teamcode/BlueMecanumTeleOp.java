package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/** Blue alliance shooting targets; field localization may use any mapped raised hive tag. */
@TeleOp(name = "Mecanum TeleOp - Blue", group = "Competition")
public class BlueMecanumTeleOp extends MecanumTeleOp {
    public static final double REHOME_X_METERS = TeleOpFieldReference.START_X_METERS;
    public static final double REHOME_Y_METERS = TeleOpFieldReference.START_Y_METERS;
    public static final double REHOME_HEADING_DEG = TeleOpFieldReference.START_HEADING_DEG;

    /** Audience-right corner, opposite the blue far-side pollen depot; face -X into the field. */
    @Override
    protected Pose2D getFieldRehomePose() {
        return new Pose2D(DistanceUnit.METER, REHOME_X_METERS, REHOME_Y_METERS,
                AngleUnit.DEGREES, REHOME_HEADING_DEG);
    }

    @Override
    protected TurretSubsystem.Alliance getAlliance() {
        return TurretSubsystem.Alliance.BLUE;
    }
}
