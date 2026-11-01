package org.firstinspires.ftc.teamcode;

import java.util.Arrays;
import java.util.HashSet;
import java.util.Set;

/** INIT hardware contract. Isolated diagnostics retain their existing standalone OpModes. */
public enum AutonomousProfile {
    FULL_ROBOT("drive", "intake", "transfer", "gate", "depollinator", "turretYaw", "hood", "flywheel", "pollenCamera", "limelight"),
    POLLEN_TEST("drive", "intake", "transfer", "gate", "pollenCamera");

    private final Set<String> capabilities;
    AutonomousProfile(String... capabilities) { this.capabilities = new HashSet<>(Arrays.asList(capabilities)); }
    public boolean supports(String capability) { return capabilities.contains(capability); }
    public void require(String command, String... required) {
        for (String capability : required) if (!supports(capability))
            throw new IllegalArgumentException(command + " requires " + capability + "; profile " + name() + " does not initialize it");
    }
}
