package org.firstinspires.ftc.teamcode.sim;

import org.horizon36596.simloop.fakehardware.FakeHardwareMap;
import org.horizon36596.simloop.fakehardware.FakeMotor;
import org.horizon36596.simloop.fakehardware.FakeServo;

/** Mechanism-only fixtures: an unrelated lookup must fail, rather than silently supplying hardware. */
final class BioBuzzSimHardware {
    final FakeHardwareMap map = new FakeHardwareMap();
    final FakeMotor[] wheels = new FakeMotor[4]; // FL, FR, BL, BR
    FakeServo gate;

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
