package org.firstinspires.ftc.teamcode;

import java.util.function.Function;
import java.util.function.Supplier;
import java.util.Arrays;

/** Name-only resolution; the inventory supplier is diagnostic data, never a device selector. */
public final class NamedHardware {
    private NamedHardware() { }
    public static <T> T require(Function<String,T> namedLookup, Supplier<String> diagnostics, String... supportedNames) {
        for (String name : supportedNames) {
            T device=namedLookup.apply(name);
            if (device != null) return device;
        }
        throw new IllegalArgumentException("Missing named hardware " + Arrays.toString(supportedNames) + ". Configured devices: " + diagnostics.get());
    }
}
