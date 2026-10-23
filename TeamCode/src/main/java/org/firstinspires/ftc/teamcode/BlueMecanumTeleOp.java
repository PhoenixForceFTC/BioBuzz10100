package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/** Blue alliance competition controls; pipeline 0 accepts only blue goal tags 38..45. */
@TeleOp(name = "Mecanum TeleOp - Blue", group = "Competition")
public class BlueMecanumTeleOp extends MecanumTeleOp {
    @Override
    protected TurretSubsystem.Alliance getAlliance() {
        return TurretSubsystem.Alliance.BLUE;
    }
}
