package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/** Red alliance competition controls; pipeline 0 accepts only red goal tags 30..37. */
@TeleOp(name = "Mecanum TeleOp - Red", group = "Competition")
public class RedMecanumTeleOp extends MecanumTeleOp {
    @Override
    protected TurretSubsystem.Alliance getAlliance() {
        return TurretSubsystem.Alliance.RED;
    }
}
