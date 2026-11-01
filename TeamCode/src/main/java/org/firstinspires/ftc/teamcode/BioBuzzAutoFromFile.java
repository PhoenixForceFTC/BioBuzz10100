package org.firstinspires.ftc.teamcode;

import org.horizon36596.zenith.ivy.AutoFromFile;

/** Zenith v0.1.2 lifecycle with the original exception preserved if shutdown also fails. */
public abstract class BioBuzzAutoFromFile extends AutoFromFile {
    @Override public void runOpMode() throws InterruptedException {
        Throwable failure = null;
        try {
            initialize();
            while (opModeInInit()) initialize_loop();
            if (opModeIsActive()) {
                startRoutine();
                while (opModeIsActive()) run();
            }
        } catch (RuntimeException | Error e) { failure = e; throw e; }
        finally {
            try { reset(); }
            catch (RuntimeException | Error cleanup) {
                if (failure != null) failure.addSuppressed(cleanup); else throw cleanup;
            }
        }
    }
}
