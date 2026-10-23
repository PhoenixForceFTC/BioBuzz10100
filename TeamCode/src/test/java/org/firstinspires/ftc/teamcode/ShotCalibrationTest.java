package org.firstinspires.ftc.teamcode;

import org.junit.Test;
import static org.junit.Assert.*;

public class ShotCalibrationTest {
    private void hits(ShotCalibration calibration) {
        for (int i = 0; i < 3; i++) calibration.record(true);
    }

    @Test public void unlimitedConfirmedShotsAverageAcrossSettingsAndMissesAreExcluded() {
        ShotCalibration c = new ShotCalibration();
        assertNull(c.currentAverage());
        c.record(true); c.record(false);
        assertEquals(1, c.hits());
        c.configure(24, 4000, 10);
        c.record(true);
        assertEquals(3600, c.currentAverage().rpm, 1e-8);
        assertEquals(5, c.currentAverage().hoodDeg, 1e-8);
        for (int i = 0; i < 98; i++) c.record(true);
        assertEquals(100, c.hits());
        assertEquals(3992, c.currentAverage().rpm, 1e-8);
        assertEquals(9.9, c.currentAverage().hoodDeg, 1e-8);
        c.configure(48, 5000, 20);
        c.record(false);
        assertEquals(0, c.hits()); assertNull(c.currentAverage());
        assertEquals(1, c.size());
        c.record(true);
        c.configure(24, 3000, 1);
        assertEquals(100, c.hits()); assertEquals(2, c.size());
    }
    @Test public void fullDownwardTravelCanBeCalibratedAndInterpolated() {
        ShotCalibration c = new ShotCalibration();
        c.configure(24, 3000, 0); hits(c);
        c.configure(48, 4000, 30); hits(c);
        assertEquals(15, c.profile("hood-range").sample(36 * .0254).hoodDeg, 1e-8);
        assertEquals(30, c.profile("hood-range").sample(48 * .0254).hoodDeg, 1e-8);
        assertThrows(IllegalArgumentException.class, () -> c.configure(48, 4000, 30.5));
        assertThrows(IllegalArgumentException.class, () -> c.configure(48, 4000, -.5));
    }

    @Test public void unsortedDistancesInterpolateBothOutputsWithoutExtrapolation() {
        ShotCalibration c = new ShotCalibration();
        c.configure(48, 4000, 10); hits(c);
        c.configure(24, 3000, 2); hits(c);
        ShotProfile p = c.profile("measured-hive");
        assertTrue(p.calibrated);
        ShotProfile.Row mid = p.sample(36 * ShotCalibration.METERS_PER_INCH);
        assertEquals(3500, mid.rpm, 1e-8); assertEquals(6, mid.hoodDeg, 1e-8);
        assertTrue(Double.isNaN(mid.flightSeconds));
        assertEquals(3000, p.sample(24 * ShotCalibration.METERS_PER_INCH).rpm, 0);
        assertEquals(10, p.sample(48 * ShotCalibration.METERS_PER_INCH).hoodDeg, 0);
        assertNull(p.sample(23 * ShotCalibration.METERS_PER_INCH));
        assertNull(p.sample(49 * ShotCalibration.METERS_PER_INCH));
        c.configure(24, 3100, 3); hits(c);
        assertEquals(2, c.size());
        assertEquals(3050, c.profile("retest").sample(24 * ShotCalibration.METERS_PER_INCH).rpm, 1e-8);
    }

    @Test public void telemetryRowsAreSortedSnapshotsAndANewRunStartsEmpty() {
        ShotCalibration c = new ShotCalibration();
        c.configure(48, 4000, 10); hits(c);
        c.configure(24, 3000, 2); hits(c);
        ShotProfile.Row[] snapshot = c.rows();
        assertEquals(2, snapshot.length);
        assertEquals(24 * .0254, snapshot[0].distance, 1e-8);
        assertEquals(48 * .0254, snapshot[1].distance, 1e-8);
        snapshot[0] = null;
        assertNotNull(c.rows()[0]);
        c.configure(24, 4000, 4); hits(c);
        assertEquals(3500, c.rows()[0].rpm, 1e-8);
        assertEquals(3750, c.profile("live-table").sample(36 * .0254).rpm, 1e-8);
        assertEquals(0, new ShotCalibration().rows().length);
    }
}
